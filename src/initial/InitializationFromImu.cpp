/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   InitializationFromImu.cpp
 * @brief  Class to initialize VIO pipeline from IMU measurements only.
 * @author Antoni Rosinol
 */

#include "kimera-vio/initial/InitializationFromImu.h"

#include <gtsam/base/Vector.h>

namespace VIO {

VioNavState InitializationFromImu::getInitialStateEstimate(
    const ImuAccGyrS& imu_accgyr,
    const gtsam::Vector3& global_gravity,
    const bool& round) {
  LOG(WARNING) << "InitializationFromImu: assumes that the "
                  "vehicle is stationary and upright along some axis,"
                  "and gravity vector is along a single axis!";

  // Compute mean acceleration and angular velocity.
  ImuAccGyr mean_accgyr = computeAverageImuMeasurements(imu_accgyr);

  // Guess initial pose assuming vehicle is stationary (zero acceleration
  // besides negative of gravity), and that gravity is aligned with one IMU
  // axis (vehicle upright).
  gtsam::Pose3 initial_pose_guess =
      guessPoseFromImuMeasurements(mean_accgyr.head(3),  // Mean Acc values.
                                   global_gravity,
                                   round);

  // Zero Velocity assumption!
  gtsam::Vector3 velocity_guess = gtsam::Vector3::Zero();

  // Convert global gravity to local frame of reference.
  // TODO(Toni): this guy should be the same as -1 * mean_acc (aka measured
  // local gravity)...
  gtsam::Vector3 local_gravity =
      initial_pose_guess.rotation().inverse().matrix() * global_gravity;

  // Guess IMU bias. Assumes static vehicle!
  ImuBias imu_bias_guess = guessImuBias(mean_accgyr, local_gravity);

  // Return estimated state.
  return VioNavState(initial_pose_guess, velocity_guess, imu_bias_guess);
}

gtsam::Pose3 InitializationFromImu::guessPoseFromImuMeasurements(
    const ImuAcc& mean_acc,
    const gtsam::Vector3& global_gravity,
    const bool& round) {
  // We measure the negative of gravity. Assumes static vehicle.
  gtsam::Vector3 measured_gravity = -1.0 * mean_acc;
  // Align measured gravity with real gravity to figure out our attitude.
  // Assumes gravity aligned along an axis.
  gtsam::Rot3 attitude_wrt_gravity =
      UtilsOpenCV::AlignGravityVectors(measured_gravity, global_gravity, round);
  // Absolute translation is unobservable, so return [0, 0, 0].
  return gtsam::Pose3(attitude_wrt_gravity, gtsam::Point3::Zero());
}

ImuBias InitializationFromImu::guessImuBias(
    const ImuAccGyr& mean_accgyr,
    const gtsam::Vector3& local_gravity) {
  // Assumes static vehicle.
  return ImuBias(mean_accgyr.head(3) + local_gravity,  // Acceleration
                 mean_accgyr.tail(3));                 // Gyro
}

}  // namespace VIO
