/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testVioParams.cpp
 * @brief  test VioParams
 * @author Antoni Rosinol
 */

#include <string>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "kimera-vio/imu-frontend/ImuFrontendParams.h"

DECLARE_string(test_data_path);

namespace VIO {

class ImuParamsFixture : public ::testing::Test {
 public:
  ImuParamsFixture() = default;

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  // Helper function
  void parseParamsManually() {
    ImuParams imu_params;
    parsePipelineParams(FLAGS_test_data_path + "/EurocParams/ImuParams.yaml",
                        &imu_params);
  }

  // Default Parms
  ImuParams imu_params_;
};

TEST_F(ImuParamsFixture, defaultEquality) {
  // Build default params
  ImuParams default_imu_params;
  // Compare
  EXPECT_EQ(default_imu_params, default_imu_params);

  // Build default params 2
  ImuParams default_imu_params_2;
  // Compare
  EXPECT_EQ(default_imu_params_2, default_imu_params);

  // Modify default
  ImuParams modified_imu_params;
  modified_imu_params.gyro_noise_density_ = 123.4;
  // Compare
  EXPECT_NE(modified_imu_params, default_imu_params);

  // Parse params, expect different from default.
  parseParamsManually();
  EXPECT_EQ(imu_params_, default_imu_params);
}

TEST_F(ImuParamsFixture, defaultConstructorWithParsing) {
  // Use vio params parser
  ImuParams imu_params;

  // Fill manually (looking at yaml file)

  // Parse from file
  parseParamsManually();

  // Compare
  EXPECT_EQ(imu_params, imu_params_);
}

}  // namespace VIO
