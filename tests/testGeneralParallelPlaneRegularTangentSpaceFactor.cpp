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
 * @author Antoni Rosinol Vidal
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <random>
#include <algorithm>

#include <gtsam/base/numericalDerivative.h>
#include <boost/bind.hpp>
#include <boost/assign/std/vector.hpp>

#include <CppUnitLite/TestHarness.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include "factors/PointPlaneFactor.h"
#include "factors/ParallelPlaneRegularFactor.h"

using namespace std;
using namespace gtsam;

static const double tol = 1e-5;
static const double der_tol = 1e-5;

/* ************************************************************************* */
//TEST(testParallelPlaneRegularFactor, Jacobians) {
//  // Create the factor with a measurement that is 3 pixels off in x
//  Key pointKey(1);
//  Key planeKey(2);
//  noiseModel::Diagonal::shared_ptr regularityNoise =
//          noiseModel::Diagonal::Sigmas(Vector1(0.1));
//  BasicRegularPlane3Factor factor(pointKey, planeKey, regularityNoise);
//
//  double plane_normal [3] = {2.4, 1.2, 1.9};
//  double distance(2.3);
//
//  // Set the linearization point
//  Point3 point(4.3, 3.2, 1.9);
//  OrientedPlane3 plane(Unit3(plane_normal[0], plane_normal[1], plane_normal[2]),
//                       distance);
//
//  // Use the factor to calculate the Jacobians
//  gtsam::Matrix H1Actual, H2Actual;
//  factor.evaluateError(point, plane, H1Actual, H2Actual);
//
//  // Calculate numerical derivatives
//  Matrix H1Expected = numericalDerivative21<Vector, Point3, OrientedPlane3>(
//      boost::bind(&BasicRegularPlane3Factor::evaluateError, &factor, _1, _2,
//          boost::none, boost::none), point, plane, der_tol);
//
//  Matrix H2Expected = numericalDerivative22<Vector, Point3, OrientedPlane3>(
//      boost::bind(&BasicRegularPlane3Factor::evaluateError, &factor, _1, _2,
//          boost::none, boost::none), point, plane, der_tol);
//
//  // Verify the Jacobians are correct
//  CHECK(assert_equal(H1Expected, H1Actual, tol));
//  CHECK(assert_equal(H2Expected, H2Actual, tol));
//}
//
//TEST(testBasicRegularPlane3Factor, JacobiansNegative) {
//  // Create the factor with a measurement that is 3 pixels off in x
//  Key pointKey(1);
//  Key planeKey(2);
//  noiseModel::Diagonal::shared_ptr regularityNoise =
//          noiseModel::Diagonal::Sigmas(Vector1(0.1));
//  BasicRegularPlane3Factor factor(pointKey, planeKey, regularityNoise);
//
//  double plane_normal [3] = {2.4, 1.2, 1.9};
//  double distance(-2.3);
//
//  // Set the linearization point
//  Point3 point(4.3, 3.2, 1.9);
//  OrientedPlane3 plane(Unit3(plane_normal[0], plane_normal[1], plane_normal[2]),
//                       distance);
//
//  // Use the factor to calculate the Jacobians
//  gtsam::Matrix H1Actual, H2Actual;
//  factor.evaluateError(point, plane, H1Actual, H2Actual);
//
//  // Calculate numerical derivatives
//  Matrix H1Expected = numericalDerivative21<Vector, Point3, OrientedPlane3>(
//      boost::bind(&BasicRegularPlane3Factor::evaluateError, &factor, _1, _2,
//          boost::none, boost::none), point, plane, der_tol);
//
//  Matrix H2Expected = numericalDerivative22<Vector, Point3, OrientedPlane3>(
//      boost::bind(&BasicRegularPlane3Factor::evaluateError, &factor, _1, _2,
//          boost::none, boost::none), point, plane, der_tol);
//
//  // Verify the Jacobians are correct
//  CHECK(assert_equal(H1Expected, H1Actual, tol));
//  CHECK(assert_equal(H2Expected, H2Actual, tol));
//}

/* ************************************************************************* */
TEST(testGeneralParallelPlaneRegularTangentSpaceFactor, PlaneOptimization) {
  /// Two planes constrained together, using distance + parallelism factor,
  ///  in tangent space. With one of the planes having a prior.
  NonlinearFactorGraph graph;
  Key planeKey1 = 7;
  Key planeKey2 = 8;

  /// Shared noise for all landmarks.
  noiseModel::Diagonal::shared_ptr priorNoise =
          noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  /// Prior mean for plane 1.
  OrientedPlane3 priorMean1(0.0, 0.0, 1.0, 0.0);
  graph.emplace_shared<PriorFactor<OrientedPlane3> >(planeKey1, priorMean1, priorNoise);

  /// Shared noise for constraints between planes.
  noiseModel::Diagonal::shared_ptr regularityNoise =
          noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  /// Factor between planes for co-planarity.
  const double measured_distance_from_plane2_to_plane1(-2.0);
  graph.emplace_shared<GeneralParallelPlaneRegularTangentSpaceFactor>(
              planeKey1, planeKey2, regularityNoise,
              measured_distance_from_plane2_to_plane1);

  Values initial;
  initial.insert(planeKey1, OrientedPlane3(0.1, 0.2, 0.9, 0.0));
  initial.insert(planeKey2, OrientedPlane3(0.1, 0.2, 0.3, -2.0));

  GaussNewtonParams params;
  //params.setVerbosity("ERROR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  //params.setErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();
  //Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();

  Values expected;
  expected.insert(planeKey1, OrientedPlane3(0.0, 0.0, 1.0, 0.0));
  expected.insert(planeKey2, OrientedPlane3(0.0, 0.0, 1.0, 2.0));

  EXPECT(assert_equal(expected, result, tol))
}

/* ************************************************************************* */
int main() {
  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
