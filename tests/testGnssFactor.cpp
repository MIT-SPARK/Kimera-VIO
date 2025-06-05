/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testGnssFactor.cpp
 * @brief  test GnssFactor
 * @author Elizaveta Karaseva
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>

#include "kimera-vio/backend/VioBackendParams.h"
#include "kimera-vio/factors/GnssFactor.h"
#include "kimera-vio/frontend/Gnss.h"
#include "kimera-vio/test/EvaluateFactor.h"

using gtsam::assert_equal;
using gtsam::GaussNewtonOptimizer;
using gtsam::GaussNewtonParams;
using gtsam::GnssFactor;
using gtsam::Matrix;
using gtsam::NonlinearFactorGraph;
using gtsam::numericalDerivative11;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::PriorFactor;
using gtsam::Rot3;
using gtsam::Symbol;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector1;
using gtsam::Vector3;

/// Test tolerance
static constexpr double tol = 1e-5;
/// Delta increment for numerical derivative
static constexpr double delta_value = 1e-5;

/**
 * Test that error does give the right result when it is zero.
 */
TEST(testGnssFactor, ErrorIsZero) {
  gtsam::Key poseKey = Symbol('x', 1);
  Point3 point1(2.4, 1.2, 1.9);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(Vector3::Constant(0.1));
  GnssFactor factor(poseKey, point1, noise);
  Pose3 pose(Rot3(), point1);
  Vector1 error = factor.evaluateError(pose);
  Vector1 expected_error = Vector1::Constant(0.0);
  EXPECT_TRUE(assert_equal(expected_error, error, tol));
}

/**
 * Test that error does give the right result when it is not zero.
 */
TEST(testGnssFactor, ErrorOtherThanZero) {
  gtsam::Key poseKey = Symbol('x', 1);
  Point3 point1(2.4, 1.2, 1.9);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(Vector3::Constant(0.1));
  GnssFactor factor(poseKey, point1, noise);
  Point3 point2(-2.3, 2.1, 0.0);
  Pose3 pose(Rot3(), point2);
  Vector1 error = factor.evaluateError(pose);
  Vector1 expected_error = Vector1::Constant(-4.7);
  EXPECT_TRUE(assert_equal(expected_error, error, tol));
}

TEST(testGnssFactor, Jacobians) {
  gtsam::Key poseKey = Symbol('x', 1);
  Point3 point1(1.0, 2.0, 3.0);
  auto model = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
  GnssFactor factor(poseKey, point1, model);

  Pose3 pose(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(10.0, 20.0, 30.0));

  Matrix numerical = numericalDerivative11<Vector, Point3>(
      [&](const Point3& t) {
        Pose3 p(Rot3(), t);  // Rotation is identity
        return factor.evaluateError(p);
      },
      pose.translation());

  Matrix H_actual;
  factor.evaluateError(pose, H_actual);
  EXPECT_TRUE(assert_equal(numerical, H_actual.block<3, 3>(0, 3), 1e-4));
}

TEST(testGnssFactor, PoseOptimizationToGNSSPosition) {
  NonlinearFactorGraph graph;

  Point3 gnssPoint1(1.0, 2.0, 3.0);
  Point3 gnssPoint2(1.0, 2.0, 3.0);
  Point3 gnssPoint3(1.0, 2.0, 3.0);

  gtsam::Key poseKey = Symbol('x', 0);

  auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

  graph.emplace_shared<GnssFactor>(poseKey, gnssPoint1, noise);
  graph.emplace_shared<GnssFactor>(poseKey, gnssPoint2, noise);
  graph.emplace_shared<GnssFactor>(poseKey, gnssPoint3, noise);

  Values initial;
  Pose3 initialPose(Rot3::RzRyRx(0.3, 0.2, -0.1), Point3(10.0, -5.0, 2.0));
  auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(1000), Vector3::Constant(1000))
          .finished());
  graph.emplace_shared<PriorFactor<Pose3>>(poseKey, initialPose, priorNoise);
  initial.insert(poseKey, initialPose);

  GaussNewtonParams params;
  params.setMaxIterations(100);
  params.setRelativeErrorTol(1e-9);
  params.setAbsoluteErrorTol(1e-9);

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();

  Pose3 expectedPose(Rot3(), Point3(1.0, 2.0, 3.0));
  EXPECT_TRUE(assert_equal(expectedPose.translation(),
                           result.at<Pose3>(poseKey).translation(),
                           1e-3));
}
