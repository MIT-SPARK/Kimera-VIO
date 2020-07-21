/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testVioBackEndParams.cp
 * @brief  test VioBackEndParams
 * @author Antoni Rosinol, Luca Carlone
 */

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <string>

#include "kimera-vio/backend/VioBackEndParams.h"

DECLARE_string(test_data_path);
DECLARE_bool(fast);
DECLARE_bool(faster);

namespace VIO {

/* ************************************************************************* */
TEST(testVioBackEndParams, VioParseYAML) {
  // Test parseYAML
  BackendParams vp;
  vp.parseYAML(std::string(FLAGS_test_data_path) +
               "/ForVIO/vioParameters.yaml");

  // Check the parsed values!
  // INITIALIZATION params
  EXPECT_EQ(vp.autoInitialize_, false);
  EXPECT_EQ(vp.roundOnAutoInitialize_, true);
  EXPECT_DOUBLE_EQ(1e-01, vp.initialPositionSigma_);
  EXPECT_DOUBLE_EQ(0.11, vp.initialRollPitchSigma_);
  EXPECT_DOUBLE_EQ(0.13, vp.initialYawSigma_);
  EXPECT_DOUBLE_EQ(0.15, vp.initialVelocitySigma_);
  EXPECT_DOUBLE_EQ(0.17, vp.initialAccBiasSigma_);
  EXPECT_DOUBLE_EQ(11, vp.initialGyroBiasSigma_);

  // VISION params
  EXPECT_EQ(vp.linearizationMode_, 3);
  EXPECT_EQ(vp.degeneracyMode_, 2);
  EXPECT_DOUBLE_EQ(5, vp.smartNoiseSigma_);
  EXPECT_DOUBLE_EQ(2.1, vp.rankTolerance_);
  EXPECT_DOUBLE_EQ(10.2, vp.landmarkDistanceThreshold_);
  EXPECT_DOUBLE_EQ(3.2, vp.outlierRejection_);
  EXPECT_DOUBLE_EQ(0.1, vp.retriangulationThreshold_);
  EXPECT_EQ(vp.addBetweenStereoFactors_, true);
  EXPECT_DOUBLE_EQ(1.11, vp.betweenRotationPrecision_);
  EXPECT_DOUBLE_EQ(2.22, vp.betweenTranslationPrecision_);

  // OPTIMIZATION params
  EXPECT_DOUBLE_EQ(0.0001, vp.relinearizeThreshold_);
  EXPECT_DOUBLE_EQ(12, vp.relinearizeSkip_);
  EXPECT_DOUBLE_EQ(1.1, vp.zeroVelocitySigma_);
  EXPECT_DOUBLE_EQ(1.2, vp.noMotionPositionSigma_);
  EXPECT_DOUBLE_EQ(1.3, vp.noMotionRotationSigma_);
  EXPECT_DOUBLE_EQ(1.4, vp.constantVelSigma_);
  EXPECT_DOUBLE_EQ(0, vp.numOptimize_);
  EXPECT_DOUBLE_EQ(2, vp.horizon_);
  EXPECT_DOUBLE_EQ(0.001, vp.wildfire_threshold_);
  EXPECT_DOUBLE_EQ(1, vp.useDogLeg_);
}

/* ************************************************************************* */
TEST(testVioBackEndParams, equals) {
  BackendParams vp = BackendParams();
  EXPECT_TRUE(vp.equals(vp));

  BackendParams vp2 = BackendParams();
  vp2.smartNoiseSigma_ += 1e-5;  // small perturbation

  EXPECT_TRUE(!vp.equals(vp2));
}

TEST(testVioBackEndParams, FastAndFasterModifications) {
  // check that fast and faster appropriately adjust the number of features
  double default_value = 0.0;
  {  // default parameter closure
    BackendParams vp;
    vp.parseYAML(std::string(FLAGS_test_data_path) +
                 "/ForVIO/vioParameters.yaml");
    default_value = vp.horizon_;
  }

  double fast_value = 0.0;
  {  // fast parameter closure
    google::FlagSaver saver;
    FLAGS_fast = true;
    BackendParams vp;
    vp.parseYAML(std::string(FLAGS_test_data_path) +
                 "/ForVIO/vioParameters.yaml");
    fast_value = vp.horizon_;
  }

  double faster_value = 0.0;
  {  // faster parameter closure
    google::FlagSaver saver;
    FLAGS_faster = true;
    BackendParams vp;
    vp.parseYAML(std::string(FLAGS_test_data_path) +
                 "/ForVIO/vioParameters.yaml");
    faster_value = vp.horizon_;
  }
  EXPECT_NE(default_value, 0.0);
  EXPECT_NE(fast_value, 0.0);
  EXPECT_NE(faster_value, 0.0);
  EXPECT_LE(fast_value, default_value);
  EXPECT_LE(faster_value, fast_value);
  double fast_to_default_ratio = fast_value / default_value;
  double faster_to_fast_ratio = faster_value / fast_value;
  EXPECT_NEAR(fast_to_default_ratio, faster_to_fast_ratio, 1.0e-6);
}

TEST(testVioBackEndParams, FastAndFasterConflict) {
  // check that fast and faster together results in faster configuration
  double faster_value = 0.0;
  {  // fast parameter closure
    google::FlagSaver saver;
    FLAGS_faster = true;
    BackendParams vp;
    vp.parseYAML(std::string(FLAGS_test_data_path) +
                 "/ForVIO/vioParameters.yaml");
    faster_value = vp.horizon_;
  }

  double combined_value = 0.0;
  {  // faster parameter closure
    google::FlagSaver saver;
    FLAGS_fast = true;
    FLAGS_faster = true;
    BackendParams vp;
    vp.parseYAML(std::string(FLAGS_test_data_path) +
                 "/ForVIO/vioParameters.yaml");
    combined_value = vp.horizon_;
  }
  EXPECT_EQ(faster_value, combined_value);
}

}  // namespace VIO
