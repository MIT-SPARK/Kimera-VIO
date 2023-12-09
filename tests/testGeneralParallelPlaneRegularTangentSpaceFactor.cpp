/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testGeneralParallelPlaneRegularTangentSpaceFactor.cpp
 * @brief  test GeneralParallelPlaneRegularTangentSpaceFactor
 * @author Antoni Rosinol
 */

#include <glog/logging.h>
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
TEST(testGeneralParallelPlaneRegularTangentSpaceFactor, ErrorIsZero) {
  /// Plane keys.
  Key plane_key_1(1);
  Key plane_key_2(2);

  /// Noise model for cosntraint between the two planes.
  noiseModel::Diagonal::shared_ptr parallel_plane_noise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  /// Measured distance between plane.
  double measured_distance_from_plane2_to_plane1 = 0.8;

  /// GeneralParallelism constraint between Plane 1 and Plane 2.
  GeneralParallelPlaneRegularTangentSpaceFactor factor(
      plane_key_1,
      plane_key_2,
      parallel_plane_noise,
      measured_distance_from_plane2_to_plane1);

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
TEST(testGeneralParallelPlaneRegularTangentSpaceFactor, ErrorOtherThanZero) {
  /// Plane keys.
  Key plane_key_1(1);
  Key plane_key_2(2);

  /// Noise model for cosntraint between the two planes.
  noiseModel::Diagonal::shared_ptr parallel_plane_noise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  /// Measured distance between plane.
  double measured_distance_from_plane2_to_plane1 = 0.7;

  /// GeneralParallelism constraint between Plane 1 and Plane 2.
  GeneralParallelPlaneRegularTangentSpaceFactor factor(
      plane_key_1,
      plane_key_2,
      parallel_plane_noise,
      measured_distance_from_plane2_to_plane1);

  /// Planes.
  OrientedPlane3 plane_1(0.0, 0.0, 1.0, 0.9);
  OrientedPlane3 plane_2(0.1, 0.1, 0.9, 0.1);

  /// Calculate error.
  Vector error = factor.evaluateError(plane_1, plane_2);

  /// Expected error.
  Vector3 expected_error;
  expected_error << 0.109764, -0.109764, 0.1;

  ASSERT_TRUE(gtsam::assert_equal(expected_error, error, tol));
}

/**
 * Test that analytical jacobians equal numerical ones.
 *
 */
TEST(testGeneralParallelPlaneRegularTangentSpaceFactor, Jacobians) {
  /// Plane keys.
  Key plane_key_1(1);
  Key plane_key_2(2);

  /// Noise model for cosntraint between the two planes.
  noiseModel::Diagonal::shared_ptr parallel_plane_noise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  /// GeneralParallelism constraint between Plane 1 and Plane 2.
  GeneralParallelPlaneRegularTangentSpaceFactor factor(
      plane_key_1, plane_key_2, parallel_plane_noise);

  /// Planes.
  OrientedPlane3 plane_1(0.3, 0.2, 1.9, 0.9);
  OrientedPlane3 plane_2(0.1, 0.1, 0.9, 0.1);
  VIO::test::evaluateFactor(factor, plane_1, plane_2, tol, der_tol);
}

/**
 * Test that optimization works.
 * Two planes constrained together, using distance + parallelism factor,
 *  in tangent space. With one of the planes having a prior.
 *
 *                  Prior
 *                   +-+        Parallelism +
 *                   +-+        Distance
 *                    |           factor
 *                +---+---+        +-+         +-------+
 *                |Plane 1+--------+ +---------+Plane 2|
 *                +-------+        +-+         +-------+
 *
 */
TEST(testGeneralParallelPlaneRegularTangentSpaceFactor, PlaneOptimization) {
  NonlinearFactorGraph graph;

  Key planeKey1 = 1;
  Key planeKey2 = 2;

  /// Shared noise for all landmarks.
  noiseModel::Diagonal::shared_ptr priorNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  /// Prior mean for plane 1.
  OrientedPlane3 priorMean1(0.0, 0.0, 1.0, 0.0);
  graph.emplace_shared<PriorFactor<OrientedPlane3> >(
      planeKey1, priorMean1, priorNoise);

  /// Shared noise for constraints between planes.
  noiseModel::Diagonal::shared_ptr regularityNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  /// Factor between planes for co-planarity.
  const double measured_distance_from_plane2_to_plane1(-2.0);
  graph.emplace_shared<GeneralParallelPlaneRegularTangentSpaceFactor>(
      planeKey1,
      planeKey2,
      regularityNoise,
      measured_distance_from_plane2_to_plane1);

  Values initial;
  initial.insert(planeKey1, OrientedPlane3(0.1, 0.2, 0.9, 0.0));
  initial.insert(planeKey2, OrientedPlane3(0.1, 0.2, 0.3, -1.0));

  GaussNewtonParams params;
  // params.setVerbosity("ERROR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  // params.setErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();
  // Values result = LevenbergMarquardtOptimizer(graph, initial,
  // params).optimize();

  Values expected;
  expected.insert(planeKey1, OrientedPlane3(0.0, 0.0, 1.0, 0.0));
  expected.insert(planeKey2, OrientedPlane3(0.0, 0.0, 1.0, 2.0));

  ASSERT_TRUE(assert_equal(expected, result, tol));
}
