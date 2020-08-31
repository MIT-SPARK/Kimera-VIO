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
#include "kimera-vio/frontend/StereoCamera.h"
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
using DepthMaps = std::map<Timestamp, DepthMap>;
using CamPoses = std::map<Timestamp, cv::Affine3d>;
/**
 * @brief The CamPoseDepthMaps struct
 * Stores with id timestamp the depth map and pose of a given camera.
 */
struct CamPoseDepthMaps {
  DepthMaps depth_maps_;
  CamPoses cam_poses_;
};

class EurocPlayground {
 public:
  EurocPlayground(const std::string& dataset_path,
                  const std::string& params_path,
                  const int& initial_k = 20,
                  const int& final_k = 10000);
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

 protected:
  //! Fill one IMU measurement only
  void fillImuQueue(const ImuMeasurement& imu_measurement);

  //! Callbacks to fill queues: they should be all lighting fast.
  void fillLeftFrameQueue(Frame::UniquePtr left_frame);

  //! Callbacks to fill queues: they should be all lighting fast.
  void fillRightFrameQueue(Frame::UniquePtr left_frame);

 protected:
  std::string dataset_path_;

  //! Mesh Optimization stuff
  CamPoseDepthMaps cam_pose_depth_maps_;

  //! Params
  VioParams vio_params_;

  //! Feature Detector to extract features from the images.
  FeatureDetector::UniquePtr feature_detector_;

  //! Stereo Camera to back/project and do stereo dense reconstruction.
  StereoCamera::UniquePtr stereo_camera_;

  //! Modules
  EurocDataProvider::UniquePtr euroc_data_provider_;
  OpenCvVisualizer3D::UniquePtr visualizer_3d_;
  DisplayModule::UniquePtr display_module_;
  DisplayModule::InputQueue display_input_queue_;

  //! Data
  ImuData imu_data_;
  ThreadsafeQueue<Frame::UniquePtr> left_frame_queue_;
  ThreadsafeQueue<Frame::UniquePtr> right_frame_queue_;
};
}
