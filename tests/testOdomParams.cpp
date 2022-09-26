
/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testOdomParams.cpp
 * @brief  test OdometryParams (based on testImuParams.cpp)
 * @author Toni Rosinol
 * @author Nathan Hughes
 */

#include <string>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <kimera-vio/frontend/OdometryParams.h>

DECLARE_string(test_data_path);

namespace VIO {

// check that param identities hold
TEST(TestOdometryParams, defaultEquality) {
  OdometryParams default_odom_params;
  // check equality identity
  EXPECT_EQ(default_odom_params, default_odom_params);

  OdometryParams default_odom_params_2;
  // check that defaults are deterministic
  EXPECT_EQ(default_odom_params, default_odom_params_2);

  OdometryParams modified_odom_params;
  modified_odom_params.betweenRotationPrecision_ = 123.4;

  // check that modifications don't preserve equality
  EXPECT_NE(modified_odom_params, default_odom_params);
}

// check that loading a file works
TEST(TestOdometryParams, ParsingCorrect) {
  OdometryParams expected_params;
  gtsam::Vector3 expected_position;
  expected_position << 1.0, 2.0, 3.0;
  expected_params.body_Pose_ext_odom_ =
      gtsam::Pose3(gtsam::Rot3(), expected_position);
  expected_params.betweenRotationPrecision_ = 1.0;
  expected_params.betweenTranslationPrecision_ = 2.0;
  expected_params.velocityPrecision_ = 3.0;
  expected_params.nominal_sampling_time_s_ = 1.0 / 4.0;

  OdometryParams parsed_params;
  parsePipelineParams(FLAGS_test_data_path + "/EurocParams/OdometryParams.yaml",
                      &parsed_params);

  EXPECT_EQ(expected_params, parsed_params);
}

}  // namespace VIO
