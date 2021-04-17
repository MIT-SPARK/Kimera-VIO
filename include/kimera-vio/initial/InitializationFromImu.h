/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   InitializationFromImu.h
 * @brief  Class to initialize VIO pipeline from IMU measurements only.
 * @author Antoni Rosinol
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"

namespace VIO {

/**
 * @brief The InitializationFromImu class
 * Guess State (pose, velocity, imu bias) from IMU only:
 *  - Guesses initial state **assuming zero velocity**.
 *  - Guesses IMU bias **assuming upright vehicle**.
 */
class InitializationFromImu {
 public:
  InitializationFromImu() = default;
  ~InitializationFromImu() = default;

 public:
  static VioNavState getInitialStateEstimate(
      const ImuAccGyrS& imu_accgyr,
      const gtsam::Vector3& global_gravity,
      const bool& round);

 private:
  static inline ImuAccGyr computeAverageImuMeasurements(
      const ImuAccGyrS& imu_accgyr) {
    return imu_accgyr.rowwise().mean();
  }

  static gtsam::Pose3 guessPoseFromImuMeasurements(
      const ImuAcc& mean_acc,
      const gtsam::Vector3& global_gravity,
      const bool& round);

  static ImuBias guessImuBias(const ImuAccGyr& mean_accgyr,
                              const gtsam::Vector3& local_gravity);
};

}  // namespace VIO
