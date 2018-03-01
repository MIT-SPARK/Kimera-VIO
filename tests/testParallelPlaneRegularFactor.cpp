/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testParallelPlaneRegularFactor.cpp
 * @brief  test ParallelPlaneRegularFactor
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
#include "RegularPlane3Factor.h"
#include "ParallelPlaneRegularFactor.h"

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

TEST(testParallelPlaneRegularFactor, PlaneOptimization) {
  /// Three landmarks, with prior factors, and a plane constrained together
  /// using the landmark-plane factor.
  NonlinearFactorGraph graph;
  Key planeKey1 = 7;
  Key planeKey2 = 8;

  /// Shared noise for all landmarks.
  noiseModel::Diagonal::shared_ptr priorNoise =
          noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  /// Landmarks at same z=1.0
  Point3 priorMean1(0.0, 0.0, 1.0);
  graph.emplace_shared<PriorFactor<Point3> >(1, priorMean1, priorNoise);
  Point3 priorMean2(1.0, 0.0, 1.0);
  graph.emplace_shared<PriorFactor<Point3> >(2, priorMean2, priorNoise);
  Point3 priorMean3(0.0, 1.0, 1.0);
  graph.emplace_shared<PriorFactor<Point3> >(3, priorMean3, priorNoise);

  /// Landmarks at same z=0.0
  Point3 priorMean4(1.0, 0.0, 0.0);
  graph.emplace_shared<PriorFactor<Point3> >(4, priorMean4, priorNoise);
  Point3 priorMean5(1.0, 0.0, 0.0);
  graph.emplace_shared<PriorFactor<Point3> >(5, priorMean5, priorNoise);
  Point3 priorMean6(0.0, 1.0, 0.0);
  graph.emplace_shared<PriorFactor<Point3> >(6, priorMean6, priorNoise);

  OrientedPlane3 priorMean7(0.0, 0.0, 1.0, 1.0);
  graph.emplace_shared<PriorFactor<OrientedPlane3> >(7, priorMean7, priorNoise);

  /// Shared noise for all constraints between landmarks and planes.
  noiseModel::Isotropic::shared_ptr regularityNoise =
          noiseModel::Isotropic::Sigma(1, 0.5);

  /// Plane 1 to landmarks 1,2,3.
  graph.emplace_shared<BasicRegularPlane3Factor>(1, planeKey1, regularityNoise);
  graph.emplace_shared<BasicRegularPlane3Factor>(2, planeKey1, regularityNoise);
  graph.emplace_shared<BasicRegularPlane3Factor>(3, planeKey1, regularityNoise);
  /// Plane 2 to landmarks 4,5,6.
  graph.emplace_shared<BasicRegularPlane3Factor>(4, planeKey2, regularityNoise);
  graph.emplace_shared<BasicRegularPlane3Factor>(5, planeKey2, regularityNoise);
  graph.emplace_shared<BasicRegularPlane3Factor>(6, planeKey2, regularityNoise);

  if (true) {
    /// Noise model for cosntraint between the two planes.
    noiseModel::Diagonal::shared_ptr parallelPlaneNoise =
        noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

    /// Parallelism constraint between Plane 1 and Plane 2.
    graph.emplace_shared<ParallelPlaneRegularBasicFactor>(
          planeKey1,
          planeKey2,
          parallelPlaneNoise);
  } else {
      /// Noise model for constraint between the two planes, in tangent space.
      noiseModel::Diagonal::shared_ptr parallelPlaneNoise =
          noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));

      /// Parallelism constraint between Plane 1 and Plane 2, in tangent space.
      graph.emplace_shared<ParallelPlaneRegularTangentSpaceFactor>(
            planeKey1,
            planeKey2,
            parallelPlaneNoise);
  }

  graph.print("\nFactor Graph:\n");

  Values initial;
  initial.insert(1, Point3(0.0, 19.0, 3.0));
  initial.insert(2, Point3(-1.0, 2.0, 2.0));
  initial.insert(3, Point3(0.3, -1.0, 8.0));
  initial.insert(4, Point3(0.0, 1.0, -1.0));
  initial.insert(5, Point3(3.0, 2.0, 2.0));
  initial.insert(6, Point3(0.3, -1.0, 1.0));
  initial.insert(planeKey1, OrientedPlane3(0.1, 0.2, 0.9, 0.0));
  initial.insert(planeKey2, OrientedPlane3(0.1, 0.2, 0.9, 2.0));

  GaussNewtonParams params;
  params.setVerbosity("ERROR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  //params.setErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();
  //Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();

  Values expected;
  expected.insert(1, priorMean1);
  expected.insert(2, priorMean2);
  expected.insert(3, priorMean3);
  expected.insert(4, priorMean4);
  expected.insert(5, priorMean5);
  expected.insert(6, priorMean6);
  expected.insert(planeKey1, OrientedPlane3(0.0, 0.0, 1.0, 1.0));
  expected.insert(planeKey2, OrientedPlane3(0.0, 0.0, 1.0, 0.0));

  CHECK(assert_equal(expected, result, tol))
}

TEST(testCoplanarPlaneTangentSpaceRegularFactor, PlaneOptimization) {
  /// Two planes constrained together, using co-planarity factor, in tangent space.
  /// With one of the planes having a prior.
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
  graph.emplace_shared<CoplanarPlaneRegularTangentSpaceFactor>(
              planeKey1, planeKey2, regularityNoise);

  Values initial;
  initial.insert(planeKey1, OrientedPlane3(0.1, 0.2, 0.9, 0.0));
  initial.insert(planeKey2, OrientedPlane3(0.1, 0.2, 0.3, 2.0));

  GaussNewtonParams params;
  params.setVerbosity("ERROR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  //params.setErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();
  //Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();

  Values expected;
  expected.insert(planeKey1, OrientedPlane3(0.0, 0.0, 1.0, 0.0));
  expected.insert(planeKey2, OrientedPlane3(0.0, 0.0, 1.0, 0.0));

  CHECK(assert_equal(expected, result, tol))
}

TEST(testCoplanarPlaneBasicRegularFactor, PlaneOptimization) {
  /// Two planes constrained together, using co-planarity basic factor.
  /// With one of the planes having a prior.
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
          noiseModel::Diagonal::Sigmas(Vector4(0.1, 0.1, 0.1, 0.1));

  /// Factor between planes for co-planarity.
  graph.emplace_shared<CoplanarPlaneRegularBasicFactor>(
              planeKey1, planeKey2, regularityNoise);

  Values initial;
  initial.insert(planeKey1, OrientedPlane3(0.1, 0.2, 0.9, 0.0));
  initial.insert(planeKey2, OrientedPlane3(0.1, 0.2, 0.3, 2.0));

  GaussNewtonParams params;
  params.setVerbosity("ERROR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  //params.setErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();
  //Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();

  Values expected;
  expected.insert(planeKey1, OrientedPlane3(0.0, 0.0, 1.0, 0.0));
  expected.insert(planeKey2, OrientedPlane3(0.0, 0.0, 1.0, 0.0));

  CHECK(assert_equal(expected, result, tol))
}

TEST(testGeneralParallelPlaneBasicRegularFactor, PlaneOptimization) {
  /// Two planes constrained together, using co-planarity basic factor.
  /// With one of the planes having a prior.
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
          noiseModel::Diagonal::Sigmas(Vector4(0.1, 0.1, 0.1, 0.1));

  /// Factor between planes for co-planarity.
  const double measured_distance_from_plane2_to_plane1(-2.0);
  graph.emplace_shared<GeneralParallelPlaneRegularBasicFactor>(
              planeKey1, planeKey2, regularityNoise,
              measured_distance_from_plane2_to_plane1);

  Values initial;
  initial.insert(planeKey1, OrientedPlane3(0.1, 0.2, 0.9, 0.0));
  initial.insert(planeKey2, OrientedPlane3(0.1, 0.2, 0.3, 2.0));

  GaussNewtonParams params;
  params.setVerbosity("ERROR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  //params.setErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();
  //Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();

  Values expected;
  expected.insert(planeKey1, OrientedPlane3(0.0, 0.0, 1.0, 0.0));
  expected.insert(planeKey2, OrientedPlane3(0.0, 0.0, 1.0, 2.0));

  CHECK(assert_equal(expected, result, tol))
}


/* ************************************************************************* */
int main() {
  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
