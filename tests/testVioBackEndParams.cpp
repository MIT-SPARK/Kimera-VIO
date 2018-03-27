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
 * @author Luca Carlone
 */

#include <cstdlib>
#include <iostream>
#include <random>
#include <algorithm>
#include <CppUnitLite/TestHarness.h>
#include "VioBackEndParams.h"
#include "test_config.h"

using namespace gtsam;
using namespace std;
using namespace VIO;
using namespace cv;

static const double tol = 1e-7;

/* ************************************************************************* */
TEST(testVioBackEndParams, VioParseYAML) {
  // Test parseYAML
  VioBackEndParams vp;
  vp.parseYAML(string(DATASET_PATH) + "/ForVIO/vioParameters.yaml");

  // Check the parsed values!
  // IMU params
  EXPECT_DOUBLES_EQUAL(0.00013, vp.gyroNoiseDensity_, tol);
  EXPECT_DOUBLES_EQUAL(0.001, vp.accNoiseDensity_, tol);
  EXPECT_DOUBLES_EQUAL(1e-05, vp.imuIntegrationSigma_, tol);
  EXPECT_DOUBLES_EQUAL(1.92e-05, vp.gyroBiasSigma_, tol);
  EXPECT_DOUBLES_EQUAL(0.001, vp.accBiasSigma_, tol);
  EXPECT(assert_equal(Vector3(-10, 2, -7.81), vp.n_gravity_));
  EXPECT_DOUBLES_EQUAL(1e-04, vp.nominalImuRate_, tol);

  // INITIALIZATION params
  EXPECT(vp.autoInitialize_ == false);
  EXPECT(vp.roundOnAutoInitialize_ == true);
  EXPECT_DOUBLES_EQUAL(1e-01, vp.initialPositionSigma_, tol);
  EXPECT_DOUBLES_EQUAL(0.11, vp.initialRollPitchSigma_, tol);
  EXPECT_DOUBLES_EQUAL( 0.13, vp.initialYawSigma_, tol);
  EXPECT_DOUBLES_EQUAL(0.15, vp.initialVelocitySigma_, tol);
  EXPECT_DOUBLES_EQUAL(0.17, vp.initialAccBiasSigma_, tol);
  EXPECT_DOUBLES_EQUAL(11, vp.initialGyroBiasSigma_, tol);

  // VISION params
  EXPECT(vp.linearizationMode_ == 3);
  EXPECT(vp.degeneracyMode_ == 2);
  EXPECT_DOUBLES_EQUAL(5, vp.smartNoiseSigma_, tol);
  EXPECT_DOUBLES_EQUAL(2.1, vp.rankTolerance_, tol);
  EXPECT_DOUBLES_EQUAL(10.2, vp.landmarkDistanceThreshold_, tol);
  EXPECT_DOUBLES_EQUAL(3.2, vp.outlierRejection_, tol);
  EXPECT_DOUBLES_EQUAL(0.1, vp.retriangulationThreshold_, tol);
  EXPECT(vp.addBetweenStereoFactors_ == true);
  EXPECT_DOUBLES_EQUAL(1.11, vp.betweenRotationPrecision_, tol);
  EXPECT_DOUBLES_EQUAL(2.22, vp.betweenTranslationPrecision_, tol);

  // OPTIMIZATION params
  EXPECT_DOUBLES_EQUAL(0.0001, vp.relinearizeThreshold_, tol);
  EXPECT_DOUBLES_EQUAL(12, vp.relinearizeSkip_, tol);
  EXPECT_DOUBLES_EQUAL(1.1, vp.zeroVelocitySigma_, tol);
  EXPECT_DOUBLES_EQUAL(1.2, vp.noMotionPositionSigma_, tol);
  EXPECT_DOUBLES_EQUAL(1.3, vp.noMotionRotationSigma_, tol);
  EXPECT_DOUBLES_EQUAL(1.4, vp.constantVelSigma_, tol);
  EXPECT_DOUBLES_EQUAL(0, vp.numOptimize_, tol);
  EXPECT_DOUBLES_EQUAL(2, vp.horizon_, tol);
  EXPECT_DOUBLES_EQUAL(1, vp.useDogLeg_, tol);
}

/* ************************************************************************* */
TEST(testVioBackEndParams, equals) {
  VioBackEndParams vp = VioBackEndParams();
  EXPECT(vp.equals(vp));

  VioBackEndParams vp2 = VioBackEndParams();
  vp2.smartNoiseSigma_ += 1e-5; // small perturbation

  EXPECT(!vp.equals(vp2));
}

/* ************************************************************************* */
TEST(testVioBackEndParams, cppVSmatlabTrackerParams) {
  // check that the cpp default params match the matlab ones.
  // before running, make sure that you run "writeDefaultParams" in matlab
  VioBackEndParams cppDefault_vp = VioBackEndParams();

  VioBackEndParams matlabDefault_vp;
  matlabDefault_vp.parseYAML(string(DATASET_PATH) + "/../../matlab/myLib/defaultVioParams.yaml");

  EXPECT(matlabDefault_vp.equals(cppDefault_vp,1e-5));

  if(!matlabDefault_vp.equals(cppDefault_vp)){
    matlabDefault_vp.print();
    cppDefault_vp.print();
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
