/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioNavState.h
 * @brief  Concept of Vio Navigation State: contains all information for a
 * robot state as used in the factor-graph.
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
#include "kimera-vio/utils/Macros.h"

namespace VIO {

/*
 * Compact storage of state.
 */
class VioNavState {
 public:
  KIMERA_POINTER_TYPEDEFS(VioNavState);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VioNavState() : pose_(), velocity_(gtsam::Vector3::Zero()), imu_bias_() {}
  virtual ~VioNavState() = default;

  VioNavState(const gtsam::Pose3& pose,
              const gtsam::Vector3& velocity,
              const gtsam::imuBias::ConstantBias& imu_bias);

  VioNavState(const gtsam::NavState& nav_state,
              const gtsam::imuBias::ConstantBias& imu_bias);

  gtsam::Pose3 pose_;
  gtsam::Vector3 velocity_;
  gtsam::imuBias::ConstantBias imu_bias_;

  virtual void print(const std::string& message = " ") const;
  virtual bool equals(const VioNavState& rhs) const;
};

class VioNavStateTimestamped : public VioNavState {
 public:
  KIMERA_POINTER_TYPEDEFS(VioNavState);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VioNavStateTimestamped(const Timestamp& timestamp,
                         const VioNavState& vio_nav_state)
      : VioNavState(vio_nav_state), timestamp_(timestamp) {}

  VioNavStateTimestamped(const Timestamp& timestamp,
                         const gtsam::Pose3& pose,
                         const gtsam::Vector3& velocity,
                         const gtsam::imuBias::ConstantBias& imu_bias)
      : VioNavState(pose, velocity, imu_bias), timestamp_(timestamp) {}

  Timestamp timestamp_;

  virtual void print(const std::string& message = " ") const override;
  virtual bool equals(const VioNavStateTimestamped& rhs) const;
};

}  // namespace VIO
