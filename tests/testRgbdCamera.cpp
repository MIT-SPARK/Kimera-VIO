/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testRgbdCamera.cpp
 * @brief  test RgbdCamera
 * @author Antoni Rosinol
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <string>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/frontend/RgbdCamera.h"
#include "kimera-vio/frontend/RgbdFrame.h"
#include "kimera-vio/mesh/MeshUtils.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/utils/UtilsOpenCV.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/DisplayFactory.h"
#include "kimera-vio/visualizer/DisplayModule.h"
#include "kimera-vio/visualizer/OpenCvDisplayParams.h"
#include "kimera-vio/visualizer/OpenCvVisualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3DFactory.h"

DECLARE_string(test_data_path);
DECLARE_bool(display);

namespace VIO {

class RgbdCameraFixture : public ::testing::Test {
 public:
  RgbdCameraFixture()
      : vio_params_(FLAGS_test_data_path + "/EurocParams"),
        rgbd_camera_(nullptr),
        visualizer_3d_(nullptr),
        display_module_(nullptr),
        display_input_queue_("display_input_queue") {
    // Set sequential mode
    vio_params_.parallel_run_ = false;

    // Create RGB-D camera
    rgbd_camera_ =
        VIO::make_unique<RgbdCamera>(vio_params_.camera_params_.at(0));

    // Create visualizer
    VisualizationType viz_type = VisualizationType::kPointcloud;
    BackendType backend_type = BackendType::kStereoImu;
    visualizer_3d_ =
        VIO::make_unique<OpenCvVisualizer3D>(viz_type, backend_type);

    // Create Displayer
    if (FLAGS_display) {
      CHECK(vio_params_.display_params_);
      OpenCv3dDisplayParams modified_display_params =
          VIO::safeCast<DisplayParams, OpenCv3dDisplayParams>(
              *vio_params_.display_params_);
      modified_display_params.hold_3d_display_ = true;
      DisplayParams::Ptr new_display_params =
          std::make_shared<OpenCv3dDisplayParams>(modified_display_params);
      display_module_ = VIO::make_unique<DisplayModule>(
          &display_input_queue_,
          nullptr,
          vio_params_.parallel_run_,
          VIO::make_unique<OpenCv3dDisplay>(new_display_params, nullptr));
    }
  }
  ~RgbdCameraFixture() override = default;

 protected:
  void SetUp() override {}
  void TearDown() override {}

  void displayPcl(const cv::Mat& pcl) {
    CHECK(!pcl.empty());
    VisualizerOutput::UniquePtr output = VIO::make_unique<VisualizerOutput>();
    output->visualization_type_ = VisualizationType::kPointcloud;

    // Depth image contains INFs. We have to remove them:
    cv::Mat_<cv::Point3f> valid_depth = cv::Mat(1, 0, CV_32FC3);
    for (int32_t u = 0; u < pcl.rows; ++u) {
      for (int32_t v = 0; v < pcl.cols; ++v) {
        const cv::Point3f& xyz = pcl.at<cv::Point3f>(u, v);
        if (isValidPoint(xyz, 10000.0, 0.0, 10.0)) {
          valid_depth.push_back(xyz);
        }
      }
    }
    CHECK(!valid_depth.empty());
    CHECK(visualizer_3d_);
    visualizer_3d_->visualizePointCloud(valid_depth, &output->widgets_);
    CHECK_GT(output->widgets_.size(), 0u);
    CHECK(display_module_);
    display_module_->spinOnce(std::move(output));
  }

 protected:
  VioParams vio_params_;
  RgbdCamera::UniquePtr rgbd_camera_;

  //! For visualization only
  OpenCvVisualizer3D::Ptr visualizer_3d_;
  DisplayModule::UniquePtr display_module_;
  DisplayModule::InputQueue display_input_queue_;
};

TEST_F(RgbdCameraFixture, convertToPoincloud) {
  ASSERT_TRUE(rgbd_camera_);
  ASSERT_GT(vio_params_.camera_params_.size(), 0u);
  CameraParams cam_params = vio_params_.camera_params_.at(0);
  const auto& width = cam_params.image_size_.width;
  const auto& height = cam_params.image_size_.height;

  // Create fake sinusoidal depth map
  cv::Mat_<uint16_t> depth_map =
      cv::Mat(height, width, CV_16UC1, cv::Scalar(0u));
  VLOG(1) << "Creating depth map.";
  for (int v = 0u; v < depth_map.rows; v++) {
    for (int u = 0u; u < depth_map.cols; u++) {
      // 1unit of 16_t depth => 0.001m
      depth_map.at<uint16_t>(v, u) =
          1000.0f +
          200.0f * std::sin(static_cast<float>(u) /
                            static_cast<float>(depth_map.rows) * 2.0f * M_PI);
    }
  }

  // Get pointcloud of this depth map using unproject of mono camera with
  // same params
  VLOG(1) << "Creating mono cam.";
  // Make identity for simplicity, but should test as well for non-identity
  cam_params.body_Pose_cam_ = gtsam::Pose3();
  Camera mono_cam(cam_params);
  cv::Mat_<cv::Point3f> expected_cloud =
      cv::Mat(height, width, CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
  VLOG(1) << "Creating cloud.";
  for (int v = 0u; v < expected_cloud.rows; v++) {
    for (int u = 0u; u < expected_cloud.cols; u++) {
      // 0.001f bcs depth is uint16_t! Float => 1.0f
      double depth = depth_map.at<uint16_t>(v, u) * 0.001;
      LandmarkCV lmk;
      KeypointCV kp(u, v);
      mono_cam.backProject(kp, depth, &lmk);
      expected_cloud.at<cv::Point3f>(v, u) = lmk;
      VLOG(10) << "U,V: " << u << ' ' << v << '\n'
               << "Depth: " << depth << '\n'
               << "Lmk: " << lmk;
    }
  }

  // Create actual pointcloud using rgbd camera
  // Inventing intensity image since we don't use it.
  VLOG(1) << "Reconstructing cloud.";
  cv::Mat intensity_img = cv::Mat(height, width, CV_8UC1, cv::Scalar(0u));
  Frame::UniquePtr frame =
      VIO::make_unique<Frame>(0u, 0u, cam_params, intensity_img);
  DepthFrame::UniquePtr depth_frame =
      VIO::make_unique<DepthFrame>(0u, 0u, depth_map);
  RgbdFrame rgbd_frame(0u, 0u, *frame, *depth_frame);
  cv::Mat actual_cloud;
  cv::Mat colors;
  rgbd_camera_->convertRgbdToPointcloud(rgbd_frame, &actual_cloud, &colors);
  VLOG(5) << "Expected cloud: " << expected_cloud;
  VLOG(5) << "Actual cloud: " << actual_cloud;
  EXPECT_TRUE(
      UtilsOpenCV::compareCvMatsUpToTol(expected_cloud, actual_cloud, 0.00001));
  if (FLAGS_display) {
    LOG(WARNING) << "Visualizing Expected cloud";
    displayPcl(expected_cloud);
    LOG(WARNING) << "Visualizing Actual cloud";
    displayPcl(actual_cloud);
  }
}

}  // namespace VIO
