/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioNavState.cpp
 * @brief  Concept of Vio Navigation State: contains all information for a
 * robot state as used in the factor-graph.
 * @author Antoni Rosinol
 */
#include "kimera-vio/common/VioNavState.h"

#include <glog/logging.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

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

}  // namespace VIO
