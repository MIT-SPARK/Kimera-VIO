/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testParallelPlaneRegularBasicFactor.cpp
 * @brief  test ParallelPlaneRegularBasicFactor
 * @author Antoni Rosinol Vidal
 */

#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>

#include "kimera-vio/factors/ParallelPlaneRegularFactor.h"
#include "kimera-vio/factors/PointPlaneFactor.h"
#include "kimera-vio/test/EvaluateFactor.h"

using namespace std;
using namespace gtsam;

static const double tol = 1e-5;
static const double der_tol = 1e-5;

/**
 * Test that error does give the right result when it is zero.
 */
TEST(testParallelPlaneRegularBasicFactor, ErrorIsZero) {
  /// Plane keys.
  Key plane_key_1(1);
  Key plane_key_2(2);

  /// Noise model for cosntraint between the two planes.
  noiseModel::Diagonal::shared_ptr parallel_plane_noise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  /// Parallelism constraint between Plane 1 and Plane 2.
  ParallelPlaneRegularBasicFactor factor(
      plane_key_1, plane_key_2, parallel_plane_noise);

  /// Planes.
  OrientedPlane3 plane_1(0.1, 0.1, 0.9, 0.9);
  OrientedPlane3 plane_2(0.1, 0.1, 0.9, 0.1);

  /// Calculate error.
  Vector error = factor.evaluateError(plane_1, plane_2);

  /// Expected error.
  Vector3 expected_error = Vector3::Constant(0.0);

  ASSERT_TRUE(assert_equal(expected_error, error, tol));
}

/**
 * Test that error does give the right result when it is not zero.
 */
TEST(testParallelPlaneRegularBasicFactor, ErrorOtherThanZero) {
  /// Plane keys.
  Key plane_key_1(1);
  Key plane_key_2(2);

  /// Noise model for cosntraint between the two planes.
  noiseModel::Diagonal::shared_ptr parallel_plane_noise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  /// Parallelism constraint between Plane 1 and Plane 2.
  ParallelPlaneRegularBasicFactor factor(
      plane_key_1, plane_key_2, parallel_plane_noise);

  /// Planes.
  OrientedPlane3 plane_1(0.3, 0.2, 1.9, 0.9);
  OrientedPlane3 plane_2(0.1, 0.1, 0.9, 0.1);

  /// Calculate error.
  Vector error = factor.evaluateError(plane_1, plane_2);

  /// Expected error.
  Vector3 expected_error;
  expected_error << 0.045362, -0.00634672, -0.00541173;

  ASSERT_TRUE(assert_equal(expected_error, error, tol));
}

/**
 * Test that analytical jacobians equal numerical ones.
 *
 */
TEST(testParallelPlaneRegularFactor, Jacobians) {
  /// Plane keys.
  Key plane_key_1(1);
  Key plane_key_2(2);

  /// Noise model for cosntraint between the two planes.
  noiseModel::Diagonal::shared_ptr parallel_plane_noise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  /// Parallelism constraint between Plane 1 and Plane 2.
  ParallelPlaneRegularBasicFactor factor(
      plane_key_1, plane_key_2, parallel_plane_noise);

  /// Planes.
  OrientedPlane3 plane_1(0.3, 0.2, 1.9, 0.9);
  OrientedPlane3 plane_2(0.1, 0.1, 0.9, 0.1);
  VIO::test::evaluateFactor(factor, plane_1, plane_2, tol, der_tol);
}

/* ************************************************************************* */
TEST(testParallelPlaneRegularBasicFactor, PlanePrior) {
  /// Three landmarks, with prior factors, and a plane constrained together
  /// using the landmark-plane factor.
  NonlinearFactorGraph graph;

  /// Keys
  Key plane_key_1 = 1;

  /// Shared noise for all landmarks.
  noiseModel::Diagonal::shared_ptr prior_noise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  OrientedPlane3 priorMeanPlane(0.0, 0.0, 1.0, 0.0);
  graph.emplace_shared<PriorFactor<OrientedPlane3> >(
      plane_key_1, priorMeanPlane, prior_noise);

  // graph.print("\nFactor Graph:\n");

  Values initial;
  initial.insert(plane_key_1, OrientedPlane3(0.1, 0.2, 0.9, 0.8));

  GaussNewtonParams params;
  params.setVerbosity("ERROR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  // params.setErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();
  // Values result = LevenbergMarquardtOptimizer(graph, initial,
  // params).optimize();

  Values expected;
  expected.insert(plane_key_1, OrientedPlane3(0.0, 0.0, 1.0, 0.0));

  ASSERT_TRUE(assert_equal(expected, result, tol));
}

/**
 * Test that optimization works.
 * A plane and a landmark with prior factors, and a second plane constrained
 * together with the first plane using the ParallelPlaneRegularBasic factor.
 *
 *              Prior                      +-------+    +-+
 *               +-+                       | Lmk 1 +----+ | Prior
 *               +-+        Parallelism    +---+---+    +-+
 *                |           factor           |
 *            +---+---+        +-+         +---+---+
 *            |Plane 1+--------+ +---------+Plane 2|
 *            +-------+        +-+         +-------+
 *
 */
TEST(testParallelPlaneRegularBasicFactor, PlaneOptimization) {
  NonlinearFactorGraph graph;

  /// Keys
  Key landmark_key = 1;
  Key plane_key_1 = 2;
  Key plane_key_2 = 3;

  /// Shared noise for all landmarks.
  noiseModel::Diagonal::shared_ptr prior_noise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  Point3 priorMeanLandmark1(0.0, 0.0, 0.0);
  graph.emplace_shared<PriorFactor<Point3> >(
      landmark_key, priorMeanLandmark1, prior_noise);

  OrientedPlane3 priorMeanPlane1(0.0, 0.0, 1.0, 1.0);
  graph.emplace_shared<PriorFactor<OrientedPlane3> >(
      plane_key_1, priorMeanPlane1, prior_noise);

  /// Shared noise for all constraints between landmarks and planes.
  noiseModel::Isotropic::shared_ptr regularity_noise =
      noiseModel::Isotropic::Sigma(1, 0.5);

  /// Plane 2 to landmark.
  graph.emplace_shared<PointPlaneFactor>(
      landmark_key, plane_key_2, regularity_noise);

  /// Noise model for cosntraint between the two planes.
  noiseModel::Diagonal::shared_ptr parallel_plane_noise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  /// Parallelism constraint between Plane 1 and Plane 2.
  graph.emplace_shared<ParallelPlaneRegularBasicFactor>(
      plane_key_1, plane_key_2, parallel_plane_noise);

  // graph.print("\nFactor Graph:\n");

  Values initial;
  initial.insert(landmark_key, Point3(0.0, 0.2, 0.1));
  initial.insert(plane_key_1, OrientedPlane3(0.1, 0.1, 0.9, 0.9));
  initial.insert(plane_key_2, OrientedPlane3(0.1, 0.1, 0.8, 0.1));

  // GaussianFactorGraph gfg = *graph.linearize(initial);
  // gfg.print("\nFactor Graph:\n");

  GaussNewtonParams params;
  // params.setVerbosity("LINEAR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  // params.setErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();
  // Values result = LevenbergMarquardtOptimizer(graph, initial,
  // params).optimize();

  Values expected;
  expected.insert(landmark_key, priorMeanLandmark1);
  expected.insert(plane_key_1, priorMeanPlane1);
  expected.insert(plane_key_2, OrientedPlane3(0.0, 0.0, 1.0, 0.0));

  ASSERT_TRUE(assert_equal(expected, result, tol));
}
