/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DataProviderInterface-definitions.cpp
 * @brief  Base implementation of a data provider for the VIO pipeline.
 * @author Antoni Rosinol
 */
#include "kimera-vio/dataprovider/DataProviderInterface-definitions.h"

#include <fstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace VIO {

VioNavState::VioNavState(const gtsam::Pose3& pose,
                         const gtsam::Vector3& velocity,
                         const gtsam::imuBias::ConstantBias& imu_bias)
    : pose_(pose), velocity_(velocity), imu_bias_(imu_bias) {}

VioNavState::VioNavState(const gtsam::NavState& nav_state,
                         const gtsam::imuBias::ConstantBias& imu_bias)
    : pose_(nav_state.pose()),
      velocity_(nav_state.velocity()),
      imu_bias_(imu_bias) {}

void VioNavState::print(const std::string& message) const {
  LOG(INFO) << "--- " << message << "--- ";
  pose_.print("\n pose: \n");
  LOG(INFO) << "\n velocity: \n" << velocity_.transpose();
  imu_bias_.print("\n imuBias: \n");
}

void VioNavStateTimestamped::print(const std::string& message) const {
  LOG(INFO) << "--- " << message << "--- \n"
            << "Timestamp: " << timestamp_;
  VioNavState::print();
}

bool VioNavStateTimestamped::equals(const VioNavStateTimestamped& rhs) const {
  return timestamp_ == rhs.timestamp_ && VioNavState::equals(rhs);
}

bool VioNavState::equals(const VioNavState& rhs) const {
  return pose_.equals(rhs.pose_) && imu_bias_.equals(rhs.imu_bias_) &&
         velocity_.isApprox(rhs.velocity_);
}

bool CameraImageLists::parseCamImgList(const std::string& folderpath,
                                       const std::string& filename) {
  CHECK(!folderpath.empty());
  CHECK(!filename.empty());
  image_folder_path_ = folderpath;  // stored, only for debug
  const std::string fullname = folderpath + "/" + filename;
  std::ifstream fin(fullname.c_str());
  CHECK(fin.is_open()) << "Cannot open file: " << fullname;

  // Skip the first line, containing the header.
  std::string item;
  std::getline(fin, item);

  // Read/store list of image names.
  while (std::getline(fin, item)) {
    // Print the item!
    int idx = item.find_first_of(',');
    Timestamp timestamp = std::stoll(item.substr(0, idx));
    std::string image_filename =
        folderpath + "/data/" + item.substr(0, idx) + ".png";
    // Strangely, on mac, it does not work if we use: item.substr(idx + 1);
    // maybe different string termination characters???
    img_lists_.push_back(make_pair(timestamp, image_filename));
  }
  fin.close();
  return true;
}

/* -------------------------------------------------------------------------- */
void CameraImageLists::print() const {
  LOG(INFO) << "------------ CameraImageLists::print -------------\n"
            << "image_folder_path: " << image_folder_path_ << '\n'
            << "img_lists size: " << img_lists_.size();
}

void GroundTruthData::print() const {
  LOG(INFO) << "------------ GroundTruthData::print -------------";
  body_Pose_cam_.print("body_Pose_cam_: \n");
  LOG(INFO) << "\n gt_rate: " << gt_rate_ << '\n'
            << "nr of gtStates: " << map_to_gt_.size();
}

InitializationPerformance::InitializationPerformance(
    const Timestamp& init_timestamp,
    const int& init_n_frames,
    const double& avg_rotationErrorBA,
    const double& avg_tranErrorBA,
    const VioNavState& init_nav_state,
    const gtsam::Vector3& init_gravity,
    const VioNavState& gt_nav_state,
    const gtsam::Vector3& gt_gravity)
    : init_timestamp_(init_timestamp),
      init_n_frames_(init_n_frames),
      avg_rotationErrorBA_(avg_rotationErrorBA),
      avg_tranErrorBA_(avg_tranErrorBA),
      init_nav_state_(init_nav_state),
      init_gravity_(init_gravity),
      gt_nav_state_(gt_nav_state),
      gt_gravity_(gt_gravity) {}

void InitializationPerformance::print() const {
  // Log everything
  LOG(INFO) << "BUNDLE ADJUSTMENT\n"
            << "Average relative errors: (BA vs. GT)\n"
            << "(avg. relative rot error)\n"
            << avg_rotationErrorBA_ << "\n(avg. relative tran error)\n"
            << avg_tranErrorBA_ << "\nONLINE GRAVITY ALIGNMENT\n"
            << "Initialization state:\n"
            << "(timestamp)\n"
            << init_timestamp_ << "\n(pitch estimate)\n"
            << init_nav_state_.pose_.rotation().pitch() * 180.0 / M_PI
            << "\n(pitch GT)\n"
            << gt_nav_state_.pose_.rotation().pitch() * 180.0 / M_PI
            << "\n(roll estimate)\n"
            << init_nav_state_.pose_.rotation().roll() * 180.0 / M_PI
            << "\n(roll GT)\n"
            << gt_nav_state_.pose_.rotation().roll() * 180.0 / M_PI
            << "\n(gyroscope bias estimate)\n"
            << init_nav_state_.imu_bias_.gyroscope()
            << "\n(gyroscope bias GT)\n"
            << gt_nav_state_.imu_bias_.gyroscope()
            << "\n(initial body frame velocity estimate)\n"
            << init_nav_state_.velocity_
            << "\n(initial body frame velocity GT)\n"
            << gt_nav_state_.velocity_
            << "\n(initial body frame gravity estimate)\n"
            << init_gravity_ << "\n(initial body frame gravity GT)\n"
            << gt_gravity_;
}

}  // namespace VIO
