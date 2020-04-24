/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   EurocPlayground.h
 * @brief  Loads Euroc datasets with trajectories, images and even pointcloud if
 * requested. It allows one to play with realistic data, while having
 * ground-truth.
 * @author Antoni Rosinol
 */

#include <future>
#include <limits>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/dataprovider/DataProviderModule.h"
#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/DisplayFactory.h"
#include "kimera-vio/visualizer/DisplayModule.h"
#include "kimera-vio/visualizer/Visualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3DFactory.h"

namespace VIO {

class EurocPlayground {
 public:
  EurocPlayground(const std::string& dataset_path,
                  const std::string& params_path,
                  const int& initial_k = 20,
                  const int& final_k = 10000)
      : dataset_path_(dataset_path),
        vio_params_(params_path),
        feature_detector_(nullptr),
        euroc_data_provider_(nullptr),
        visualizer_3d_(nullptr),
        display_module_(nullptr),
        display_input_queue_("display_input_queue"),
        imu_data_(),
        left_frame_queue_("left_frame_queue"),
        right_frame_queue_("right_frame_queue") {
    // Set sequential mode
    vio_params_.parallel_run_ = false;

    // Create euroc data parser
    euroc_data_provider_ = VIO::make_unique<EurocDataProvider>(
        dataset_path, initial_k, final_k, vio_params_);

    // Register Callbacks
    euroc_data_provider_->registerImuSingleCallback(
        std::bind(&EurocPlayground::fillImuQueue, this, std::placeholders::_1));
    euroc_data_provider_->registerLeftFrameCallback(std::bind(
        &EurocPlayground::fillLeftFrameQueue, this, std::placeholders::_1));
    euroc_data_provider_->registerRightFrameCallback(std::bind(
        &EurocPlayground::fillRightFrameQueue, this, std::placeholders::_1));

    // Parse Euroc dataset.
    // Since we run in sequential mode, we need to spin it till it finishes.
    while (euroc_data_provider_->spin()) {
    };  // Fill queues.

    // Create 3D visualizer
    VisualizationType viz_type = VisualizationType::kPointcloud;
    BackendType backend_type = BackendType::kStereoImu;
    visualizer_3d_ = VisualizerFactory::createVisualizer(
        VisualizerType::OpenCV,
        static_cast<VisualizationType>(viz_type),
        backend_type);

    // Create Displayer
    OpenCv3dDisplayParams opencv_3d_display_params;
    opencv_3d_display_params.hold_display_ = true;
    display_module_ = VIO::make_unique<DisplayModule>(
        &display_input_queue_,
        nullptr,
        vio_params_.parallel_run_,
        VIO::make_unique<OpenCv3dDisplay>(nullptr, opencv_3d_display_params));

    // Create Feature detector
    FeatureDetectorParams feature_detector_params;
    feature_detector_params.feature_detector_type_ = FeatureDetectorType::FAST;
    feature_detector_ =
        VIO::make_unique<FeatureDetector>(feature_detector_params);
  }

  ~EurocPlayground() = default;

 public:
  /**
   * @brief visualizeGtData Spawns a 3D window where all ground-truth data is
   * visualized according to given flags
   * @param viz_traj Visualize ground-truth trajectory
   * @param viz_img_in_frustum Visualize actual images inside frustums
   * @param viz_pointcloud Visualize ground-truth 3D pointcloud
   */
  void visualizeGtData(const bool& viz_traj,
                       const bool& viz_img_in_frustum,
                       const bool& viz_pointcloud) {
    VisualizerOutput::UniquePtr output = VIO::make_unique<VisualizerOutput>();
    output->visualization_type_ = VisualizationType::kPointcloud;
    if (viz_traj) {
      CHECK_GT(vio_params_.camera_params_.size(), 0);
      const auto& K = vio_params_.camera_params_.at(0).K_;
      LOG_IF(ERROR, euroc_data_provider_->gt_data_.map_to_gt_.size() == 0)
          << "Empty ground-truth trajectory.";
      Frame::UniquePtr left_frame = nullptr;
      left_frame_queue_.pop(left_frame);
      CHECK(left_frame);
      for (const auto& kv : euroc_data_provider_->gt_data_.map_to_gt_) {
        const VioNavState& state = kv.second;
        const cv::Affine3d& left_cam_pose = UtilsOpenCV::gtsamPose3ToCvAffine3d(
            state.pose_.compose(euroc_data_provider_->gt_data_.body_Pose_cam_));
        visualizer_3d_->addPoseToTrajectory(left_cam_pose, left_frame->img_);

        if (viz_img_in_frustum &&
            left_frame->timestamp_ == kv.first) {  // This might not be true...
          // Add frame to frustum
          visualizer_3d_->visualizePoseWithImgInFrustum(
              left_frame->img_,
              left_cam_pose,
              &output->widgets_,
              "Camera id: " + std::to_string(left_frame->id_));

          // Get next frame
          Frame::UniquePtr left_frame = nullptr;
          left_frame_queue_.pop(left_frame);
          CHECK(left_frame);
        }
      }
      visualizer_3d_->visualizeTrajectory3D(&output->widgets_);
    }

    if (viz_pointcloud) {
      static bool visualize_ply_mesh_once = true;
      static const std::string pcl_ply_filename =
          dataset_path_ + "/mav0/pointcloud0/data.ply";
      if (visualize_ply_mesh_once) {
        visualizer_3d_->visualizePlyMesh(pcl_ply_filename, &output->widgets_);
        visualize_ply_mesh_once = false;
      }
    }
    display_module_->spinOnce(std::move(output));
  }

 protected:
  //! Fill one IMU measurement only
  inline void fillImuQueue(const ImuMeasurement& imu_measurement) {
    imu_data_.imu_buffer_.addMeasurement(imu_measurement.timestamp_,
                                         imu_measurement.acc_gyr_);
  }

  //! Callbacks to fill queues: they should be all lighting fast.
  inline void fillLeftFrameQueue(Frame::UniquePtr left_frame) {
    CHECK(left_frame);
    left_frame_queue_.push(std::move(left_frame));
  }

  //! Callbacks to fill queues: they should be all lighting fast.
  inline void fillRightFrameQueue(Frame::UniquePtr left_frame) {
    CHECK(left_frame);
    right_frame_queue_.push(std::move(left_frame));
  }

 protected:
  std::string dataset_path_;

  //! Params
  VioParams vio_params_;

  //! Feature Detector to extract features from the images.
  FeatureDetector::UniquePtr feature_detector_;

  //! Modules
  EurocDataProvider::UniquePtr euroc_data_provider_;
  Visualizer3D::UniquePtr visualizer_3d_;
  DisplayModule::UniquePtr display_module_;
  DisplayModule::InputQueue display_input_queue_;

  //! Data
  ImuData imu_data_;
  ThreadsafeQueue<Frame::UniquePtr> left_frame_queue_;
  ThreadsafeQueue<Frame::UniquePtr> right_frame_queue_;
};
}
