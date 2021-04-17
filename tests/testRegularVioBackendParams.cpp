/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testRegularVioBackendParams.cp
 * @brief  test RegularVioBackendParams
 * @author Antoni Rosinol, Luca Carlone
 */

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "kimera-vio/backend/RegularVioBackendParams.h"

DECLARE_string(test_data_path);

namespace VIO {

TEST(testRegularVioBackendParams, VioParseYAML) {
  // Test parseYAML
  RegularVioBackendParams vp;
  vp.parseYAML(FLAGS_test_data_path + "/ForVIO/regularVioParameters.yaml");

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
  EXPECT_DOUBLE_EQ(6, vp.monoNoiseSigma_);
  EXPECT_EQ(0, vp.monoNormType_);
  EXPECT_DOUBLE_EQ(0, vp.monoNormParam_);
  EXPECT_DOUBLE_EQ(3, vp.stereoNoiseSigma_);
  EXPECT_EQ(0, vp.stereoNormType_);
  EXPECT_DOUBLE_EQ(0, vp.stereoNormParam_);
  EXPECT_DOUBLE_EQ(0.3, vp.regularityNoiseSigma_);
  EXPECT_EQ(1, vp.regularityNormType_);
  EXPECT_DOUBLE_EQ(5.1, vp.regularityNormParam_);
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

TEST(testRegularVioBackendParams, equals) {
  RegularVioBackendParams vp = RegularVioBackendParams();
  EXPECT_TRUE(vp.equals(vp));

  RegularVioBackendParams vp2 = RegularVioBackendParams();
  vp2.smartNoiseSigma_ += 1e-5;  // small perturbation

  EXPECT_TRUE(!vp.equals(vp2));
}

}  // namespace VIO
