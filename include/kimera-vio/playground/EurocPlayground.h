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

#pragma once

#include <future>
#include <limits>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/dataprovider/DataProviderModule.h"
#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/DisplayFactory.h"
#include "kimera-vio/visualizer/DisplayModule.h"
#include "kimera-vio/visualizer/OpenCvVisualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3DFactory.h"

namespace VIO {

//! Data for mesh optimization
using DepthMap = cv::Mat_<cv::Point3f>;
using CamPose = cv::Affine3d;
/**
 * @brief The CamPoseDepthMaps struct
 * Stores with id timestamp the depth map and pose of a given camera.
 */
struct MeshPacket {
  Timestamp timestamp_;
  DepthMap depth_map_;
  gtsam::Pose3 world_pose_body_;
  CamPose left_cam_rect_pose_;
  CamPose right_cam_rect_pose_;
  cv::Mat left_image_rect_;
  cv::Mat right_image_rect;
};
using MeshPackets = std::map<Timestamp, MeshPacket>;

class EurocPlayground {
 public:
  EurocPlayground(const std::string& dataset_path,
                  const std::string& params_path,
                  const int& initial_k = 20,
                  const int& final_k = 10000,
                  const size_t& subsample_n = 100u);
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
                       const bool& viz_pointcloud);

  // Very naive!
  void projectVisibleLandmarksToCam(const StereoCamera& stereo_cam,
                                    const Landmarks& lmks);

public:
  //! Mesh Optimization stuff
  MeshPackets mesh_packets_;

  //! Params
  VioParams vio_params_;

  OpenCvVisualizer3D::Ptr visualizer_3d_;

  //! Stereo Camera to back/project and do stereo dense reconstruction.
  StereoCamera::ConstPtr stereo_camera_;
  StereoMatcher::UniquePtr stereo_matcher_;

  DisplayModule::UniquePtr display_module_;
  DisplayModule::InputQueue display_input_queue_;


 protected:
  //! Fill one IMU measurement only
  void fillImuQueue(const ImuMeasurement& imu_measurement);

  //! Callbacks to fill queues: they should be all lighting fast.
  void fillLeftFrameQueue(Frame::UniquePtr left_frame);

  //! Callbacks to fill queues: they should be all lighting fast.
  void fillRightFrameQueue(Frame::UniquePtr left_frame);

 protected:
  std::string dataset_path_;

  //! N subsampled frames which we use
  const FrameId subsample_n = 50u;

  //! Feature Detector to extract features from the images.
  FeatureDetector::UniquePtr feature_detector_;


  //! Modules
  EurocDataProvider::UniquePtr euroc_data_provider_;


  //! Data
  ImuData imu_data_;
  // These are empty after visualizing because we pop...
  ThreadsafeQueue<Frame::UniquePtr> left_frame_queue_;
  ThreadsafeQueue<Frame::UniquePtr> right_frame_queue_;
};
}
