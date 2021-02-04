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

#include "kimera-vio/pipeline/Pipeline-definitions.h"

DECLARE_string(test_data_path);

namespace VIO {

class VioParamsFixture : public ::testing::Test {
 public:
  VioParamsFixture() = default;
  ~VioParamsFixture() override = default;

 protected:
  void SetUp() override {}
  void TearDown() override {}

  // Helper function
  void parseParamsManually() {
    parsePipelineParams(FLAGS_test_data_path + "/EurocParams/ImuParams.yaml",
                        &imu_params_);
    parsePipelineParams(
        FLAGS_test_data_path + "/EurocParams/LeftCameraParams.yaml",
        &left_cam_params_);
    parsePipelineParams(FLAGS_test_data_path +
                            "/EurocParams/RightCameraParams.yaml",
                        &right_cam_params_);
    parsePipelineParams(FLAGS_test_data_path +
                            "/EurocParams/BackendParams.yaml",
                        &backend_params_);
    parsePipelineParams(FLAGS_test_data_path +
                            "/EurocParams/BackendParams.yaml",
                        &regular_backend_params_);
    parsePipelineParams(FLAGS_test_data_path +
                            "/EurocParams/FrontendParams.yaml",
                        &frontend_params_);
    parsePipelineParams(FLAGS_test_data_path +
                            "/EurocParams/LcdParams.yaml",
                        &lcd_params_);
  }

  // Default Parms
  ImuParams imu_params_;
  CameraParams left_cam_params_;
  CameraParams right_cam_params_;
  BackendParams backend_params_;
  RegularVioBackendParams regular_backend_params_;
  FrontendParams frontend_params_;
  LoopClosureDetectorParams lcd_params_;
};

TEST_F(VioParamsFixture, defaultConstructorWithoutParsing) {
  VioParams vio_params ("");

  // Check that it matches all default ctors!

  // Build default params
  ImuParams default_imu_params;
  BackendParams default_backend_params;
  FrontendParams default_frontend_params;
  LoopClosureDetectorParams default_lcd_params;

  // Compare
  EXPECT_EQ(default_imu_params, default_imu_params);
  EXPECT_EQ(vio_params.imu_params_, default_imu_params);
  EXPECT_EQ(vio_params.camera_params_.size(), 0u);
  EXPECT_EQ(*vio_params.backend_params_, default_backend_params);
  EXPECT_EQ(vio_params.frontend_params_, default_frontend_params);
  EXPECT_EQ(vio_params.lcd_params_, default_lcd_params);
}

TEST_F(VioParamsFixture, defaultConstructorWithParsing) {
  // Use vio params parser
  VioParams vio_params(FLAGS_test_data_path + "/EurocParams");

  // Parse yourself
  parseParamsManually();

  // Compare
  EXPECT_EQ(vio_params.frontend_type_, FrontendType::kStereoImu);
  EXPECT_EQ(vio_params.backend_type_, BackendType::kStructuralRegularities);
  EXPECT_EQ(vio_params.parallel_run_, 1);
  EXPECT_EQ(*vio_params.backend_params_, regular_backend_params_);
  EXPECT_NE(*vio_params.backend_params_, backend_params_);
  EXPECT_EQ(vio_params.frontend_params_, frontend_params_);
  EXPECT_EQ(vio_params.imu_params_, imu_params_);
  ASSERT_EQ(vio_params.camera_params_.size(), 2u);
  EXPECT_EQ(vio_params.camera_params_.at(0), left_cam_params_);
  EXPECT_EQ(vio_params.camera_params_.at(1), right_cam_params_);
  EXPECT_EQ(vio_params.lcd_params_, lcd_params_);
}

}  // namespace VIO
