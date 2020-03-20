/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testOpticalFlowPredictor.cpp
 * @brief  test OpticalFlowPredictor
 * @author Antoni Rosinol
 */

#include <string>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/OpticalFlowPredictor.h"

DECLARE_string(test_data_path);

namespace VIO {

class OpticalFlowPredictorFixture : public ::testing::Test {
 public:
  OpticalFlowPredictorFixture() {
   vio_params_(FLAGS_test_data_path + "/EurocParams");
  };
  ~OpticalFlowPredictorFixture() override = default;

 protected:
  void SetUp() override {}
  void TearDown() override {}

  // Helper functions

  // Default Parms
  VioParams vio_params_;
};

TEST_F(OpticalFlowPredictorFixture, SillyOpticalFlowPrediction) {
}

TEST_F(OpticalFlowPredictorFixture, RotationalOpticalFlowPrediction) {
}

}  // namespace VIO
