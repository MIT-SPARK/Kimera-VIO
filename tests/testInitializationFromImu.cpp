/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testInitializationFromImu.cpp
 * @brief  test InitializationFromImu
 * @author Antoni Rosinol
 */

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/initial/InitializationFromImu.h"

namespace VIO {

TEST(testVio, getInitialStateEstimate) {
  static const double tol = 1e-7;

  for (size_t test = 0; test < 5; test++) {
    gtsam::Vector3 n_gravity;
    gtsam::Vector3 acc;
    gtsam::Vector3 gyro;
    switch (test) {
      case 0:  // generic vectors
        acc = gtsam::Vector3(9.8, 1, 0);
        gyro = gtsam::Vector3(0.8, -1, 0);
        n_gravity = gtsam::Vector3(0, 0, -9.8);
        break;
      case 1:  // already aligned vectors
        acc = gtsam::Vector3(0, -9.8, 0);
        gyro = gtsam::Vector3(0.2, -1, 0);
        n_gravity = gtsam::Vector3(0, -9.8, 0);
        break;
      case 2:  // opposite vectors
        acc = gtsam::Vector3(0, 0, -9.8);
        gyro = gtsam::Vector3(0.2, -1.2, 0.3);
        n_gravity = gtsam::Vector3(0, 0, +9.8);
        break;
      case 3:
        acc = gtsam::Vector3(9.8, 0, 0);
        gyro = gtsam::Vector3(0.9, 0.1, -1);
        n_gravity = gtsam::Vector3(0, -9.8, 0);
        break;
      case 4:
        acc = gtsam::Vector3(9.8, -1, 0);
        gyro = gtsam::Vector3(2, 1, 0);
        n_gravity = Rot3::Expmap(gtsam::Vector3(0.1, 1, 0.5)).matrix() * acc;
        break;
    }

    size_t num_measurements = 10;
    ImuAccGyrS accGyroRaw;
    accGyroRaw.resize(6, num_measurements);  // n identical measurements
    for (size_t i = 0; i < num_measurements; i++) {
      // we measure the opposite of gravity
      accGyroRaw.col(i) << -acc, gyro;
    }

    bool round = false;
    VioNavState initial_state_guess =
        InitializationFromImu::getInitialStateEstimate(
            accGyroRaw, n_gravity, round);
    Pose3 poseActual = initial_state_guess.pose_;

    // Check translation
    gtsam::Vector3 tExpected = gtsam::Vector3::Zero();
    gtsam::Vector3 tActual = poseActual.translation();
    EXPECT_TRUE(gtsam::assert_equal(tActual, tExpected, tol));

    // Check rotation
    gtsam::Unit3 n_gravityDir_actual =
        poseActual.rotation().rotate(gtsam::Unit3(acc));
    gtsam::Unit3 n_gravityDir_expected = gtsam::Unit3(n_gravity);
    EXPECT_TRUE(
        gtsam::assert_equal(n_gravityDir_expected, n_gravityDir_actual, tol));

    // Check velocity
    gtsam::Vector3 vExpected = initial_state_guess.velocity_;
    gtsam::Vector3 vActual = gtsam::Vector3::Zero();
    EXPECT_TRUE(gtsam::assert_equal(vActual, vExpected, tol));

    // Check IMU bias
    Vector6 imu_mean = Vector6::Zero();
    for (int i = 0; i < num_measurements; i++) imu_mean += accGyroRaw.col(i);
    imu_mean = imu_mean / static_cast<double>(num_measurements);
    imu_mean.head(3) =
        imu_mean.head(3) + poseActual.rotation().rotate(n_gravity);
    ImuBias imu_bias_expected(imu_mean.head(3), imu_mean.tail(3));

    if (test > 0 &&
        test < 4) {  // case in which true gravity is along a single axis
      round = true;
      // check that rounding does not mess up with the previous cases
      // by rounding we should filter out perturbation
      VioNavState initial_state_guess_2 =
          InitializationFromImu::getInitialStateEstimate(
              accGyroRaw, n_gravity, round);
      gtsam::Pose3 poseActual2 = initial_state_guess_2.pose_;
      EXPECT_TRUE(gtsam::assert_equal(poseActual, poseActual2, tol));

      // check that rounding filter out perturbation
      gtsam::Vector3 n_gravity_perturbed =
          n_gravity + gtsam::Vector3(-0.1, 0.1, 0.3);
      VioNavState initial_state_guess_round =
          InitializationFromImu::getInitialStateEstimate(
              accGyroRaw, n_gravity_perturbed, round);
      Pose3 poseActualRound = initial_state_guess_round.pose_;
      EXPECT_TRUE(gtsam::assert_equal(poseActual, poseActualRound, tol));
    }
  }
}

}  // namespace VIO
