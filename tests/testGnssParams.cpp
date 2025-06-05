/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testGnssParams.cpp
 * @brief  test GnssParams
 * @author Elizaveta Karaseva
 */

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <string>

#include "kimera-vio/frontend/GnssParams.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"

DECLARE_string(test_data_path);

namespace VIO {

class GnssParamsFixture : public ::testing::Test {
 public:
  GnssParamsFixture() = default;

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  // Helper function
  void parseParamsManually() {
    GnssParams gnss_params;
    parsePipelineParams(
        FLAGS_test_data_path + "/AlternateTestParams/GnssParams.yaml",
        &gnss_params);
  }

  // Default Parms
  GnssParams gnss_params_;
};

TEST_F(GnssParamsFixture, defaultEquality) {
  // Build default params
  GnssParams default_gnss_params;
  // Compare
  EXPECT_EQ(default_gnss_params, default_gnss_params);

  // Build default params 2
  GnssParams default_gnss_params_2;
  // Compare
  EXPECT_EQ(default_gnss_params_2, default_gnss_params);

  // Modify default
  GnssParams modified_gnss_params;
  modified_gnss_params.timestamp_ = 123.4;
  // Compare
  EXPECT_NE(modified_gnss_params, default_gnss_params);

  // Parse params, expect different from default.
  parseParamsManually();
  EXPECT_EQ(gnss_params_, default_gnss_params);
}

TEST_F(GnssParamsFixture, defaultConstructorWithParsing) {
  // Use vio params parser
  GnssParams gnss_params;

  // Fill manually (looking at yaml file)

  // Parse from file
  parseParamsManually();

  // Compare
  EXPECT_EQ(gnss_params, gnss_params_);
}

TEST_F(GnssParamsFixture, defaultConstructorWithVioParamsParsing) {
  // Use vio params parser
  VioParams vio_params(
      FLAGS_test_data_path + "/EurocParams",
      FLAGS_test_data_path + "/AlternateTestParams/" +
          VioParams::kPipelineFilename,
      FLAGS_test_data_path + "/AlternateTestParams/" + VioParams::kImuFilename,
      FLAGS_test_data_path + "/EurocParams/" + VioParams::kLeftCameraFilename,
      FLAGS_test_data_path + "/EurocParams/" + VioParams::kRightCameraFilename,
      FLAGS_test_data_path + "/EurocParams/" + VioParams::kFrontendFilename,
      FLAGS_test_data_path + "/AlternateTestParams/" +
          VioParams::kBackendFilename,
      FLAGS_test_data_path + "/EurocParams/" + VioParams::kLcdFilename,
      FLAGS_test_data_path + "/EurocParams/" + VioParams::kDisplayFilename,
      FLAGS_test_data_path + "/AlternateTestParams/" +
          VioParams::kGnssFilename);

  // Parse yourself
  parseParamsManually();

  EXPECT_EQ(vio_params.gnss_params_, gnss_params_);
}

}  // namespace VIO
