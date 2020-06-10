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

#include <math.h>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/frontend/rgbd/RgbdCamera.h"
#include "kimera-vio/frontend/rgbd/RgbdFrame.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"

DECLARE_string(test_data_path);
DECLARE_bool(display);

namespace VIO {

class RgbdCameraFixture : public ::testing::Test {
 public:
  RgbdCameraFixture()
      : vio_params_(FLAGS_test_data_path + "/EurocParams"),
        rgbd_camera_(nullptr) {
    rgbd_camera_ =
        VIO::make_unique<RgbdCamera>(vio_params_.camera_params_.at(0));
  }
  ~RgbdCameraFixture() override = default;

 protected:
  void SetUp() override {}
  void TearDown() override {}

 protected:
  // Default Parms
  //! Params
  VioParams vio_params_;
  RgbdCamera::UniquePtr rgbd_camera_;
};

TEST_F(RgbdCameraFixture, convertToPoincloud) {
  CHECK(rgbd_camera_);
  const auto& cam_params = vio_params_.camera_params_.at(0);
  const auto& width = cam_params.image_size_.width;
  const auto& height = cam_params.image_size_.height;

  // Create fake sinusoidal depth map
  cv::Mat_<uint16_t> depth_map = cv::Mat(height, width, CV_16UC1);
  for (int v = 0u; depth_map.rows; v++) {
    for (int u = 0u; depth_map.cols; u++) {
      depth_map.at<uint16_t>(v, u) = std::sin(v + u);
    }
  }

  // Get pointcloud of this depth map using unproject of mono camera with
  // same params
  Camera mono_cam(cam_params);
  cv::Mat_<cv::Point3f> expected_cloud = cv::Mat(height, width, CV_32FC3);
  for (int v = 0u; expected_cloud.rows; v++) {
    for (int u = 0u; expected_cloud.cols; u++) {
      KeypointCV kp(u, v);
      // 0.001f bcs depth is uint16_t! Float => 1.0f
      double depth = depth_map.at<uint16_t>(kp) * 0.001f;
      LandmarkCV lmk;
      mono_cam.backProject(kp, depth, &lmk);
      expected_cloud.at<cv::Point3f>(v, u) = lmk;
    }
  }

  // Create actual pointcloud using rgbd camera
  // Inventing intensity image since we don't use it.
  cv::Mat intensity_img = cv::Mat(height, width, CV_8UC1, cv::Scalar(0u));
  Frame::UniquePtr frame =
      VIO::make_unique<Frame>(0u, 0u, cam_params, intensity_img);
  DepthFrame::UniquePtr depth_frame =
      VIO::make_unique<DepthFrame>(0u, 0u, depth_map);
  RgbdFrame rgbd_frame(0u, 0u, std::move(frame), std::move(depth_frame));
  cv::Mat actual_cloud = rgbd_camera_->convertRgbdToPointcloud(rgbd_frame);
  cv::Mat diff = actual_cloud != expected_cloud;
  // Equal if no elements disagree
  bool equal = cv::countNonZero(diff) == 0;
  EXPECT_TRUE(equal);
}

}  // namespace VIO
