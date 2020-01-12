/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DataSource-definitions.h
 * @brief  Definitions for data providers for the VIO pipeline.
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <map>
#include <string>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include "kimera-vio/common/vio_types.h"

namespace VIO {

// TODO (Toni): this should be in its own...
/*
 * Compact storage of state.
 */
class VioNavState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VioNavState() : pose_(), velocity_(gtsam::Vector3::Zero()), imu_bias_() {}

  VioNavState(const gtsam::Pose3& pose,
              const gtsam::Vector3& velocity,
              const gtsam::imuBias::ConstantBias& imu_bias);

  VioNavState(const gtsam::NavState& nav_state,
              const gtsam::imuBias::ConstantBias& imu_bias);

  gtsam::Pose3 pose_;
  gtsam::Vector3 velocity_;
  gtsam::imuBias::ConstantBias imu_bias_;

  void print(const std::string& message = " ") const;

  bool equals(const VioNavState& rhs) const;
};

// Struct for performance in initialization
struct InitializationPerformance {
 public:
  // Default constructor
  InitializationPerformance(const Timestamp& init_timestamp,
                            const int& init_n_frames,
                            const double& avg_rotationErrorBA,
                            const double& avg_tranErrorBA,
                            const VioNavState& init_nav_state,
                            const gtsam::Vector3& init_gravity,
                            const VioNavState& gt_nav_state,
                            const gtsam::Vector3& gt_gravity);

  void print() const;

 public:
  const Timestamp init_timestamp_;
  const int init_n_frames_;
  const double avg_rotationErrorBA_;
  const double avg_tranErrorBA_;
  const VioNavState init_nav_state_;
  const gtsam::Vector3 init_gravity_;
  const VioNavState gt_nav_state_;
  const gtsam::Vector3 gt_gravity_;
};

/*
 * Store GT poses and GT info.
 */
class GroundTruthData {
 public:
  // Display all params.
  void print() const;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Sensor extrinsics wrt. the body-frame
  gtsam::Pose3 body_Pose_cam_;

  // Data rate in seconds, for debug.
  double gt_rate_;

  // Map from timestamp to VioNavState.
  std::map<Timestamp, VioNavState> map_to_gt_;
};

/*
 * Store a list of image names and provide functionalities to parse them.
 */
class CameraImageLists {
 public:
  bool parseCamImgList(const std::string& folderpath,
                       const std::string& filename);
  //bool parseZedImgs(cv::Mat img,
  //                  const std::string& folderpath,
  //                  const std::string& filename);
  inline size_t getNumImages() const { return img_lists_.size(); }
  void print() const;

 public:
  std::string image_folder_path_;
  typedef std::pair<Timestamp, std::string> TimestampToFilename;
  typedef std::vector<TimestampToFilename> ImgLists;
  ImgLists img_lists_;
};

}  // namespace VIO
