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
#include "factors/RegularPlane3Factor.h"
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
TEST(testParallelPlaneRegularFactor, PlanePrior) {
  /// Three landmarks, with prior factors, and a plane constrained together
  /// using the landmark-plane factor.
  NonlinearFactorGraph graph;

  /// Keys
  Key plane_key_1 = 1;

  /// Shared noise for all landmarks.
  noiseModel::Diagonal::shared_ptr prior_noise =
          noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  OrientedPlane3 priorMeanPlane(0.0, 0.0, 1.0, 0.0);
  graph.emplace_shared<PriorFactor<OrientedPlane3> >(plane_key_1,
                                                     priorMeanPlane,
                                                     prior_noise);

  graph.print("\nFactor Graph:\n");

  Values initial;
  initial.insert(plane_key_1, OrientedPlane3(0.1, 0.2, 0.9, 0.8));

  GaussNewtonParams params;
  params.setVerbosity("ERROR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  //params.setErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();
  //Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();

  Values expected;
  expected.insert(plane_key_1, OrientedPlane3(0.0, 0.0, 1.0, 0.0));

  EXPECT(assert_equal(expected, result, tol))
}

/* ************************************************************************* */
TEST(testParallelPlaneRegularFactor, PlaneOptimization) {
  /// Three landmarks, with prior factors, and a plane constrained together
  /// using the landmark-plane factor.
  NonlinearFactorGraph graph;

  /// Keys
  Key landmark_key_1 = 1;
  Key landmark_key_2 = 2;
  Key landmark_key_3 = 3;
  Key plane_key_1 = 4;
  Key plane_key_2 = 5;

  /// Shared noise for all landmarks.
  noiseModel::Diagonal::shared_ptr prior_noise =
          noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  /// Landmarks at same z=1.0
  Point3 prior_mean_1(0.0, 0.0, 1.0);
  Point3 prior_mean_2(1.0, 0.0, 1.0);
  Point3 prior_mean_3(0.0, 1.0, 1.0);

  graph.emplace_shared<PriorFactor<Point3> >(landmark_key_1, prior_mean_1, prior_noise);
  graph.emplace_shared<PriorFactor<Point3> >(landmark_key_2, prior_mean_2, prior_noise);
  graph.emplace_shared<PriorFactor<Point3> >(landmark_key_3, prior_mean_3, prior_noise);

  OrientedPlane3 priorMeanPlane2(0.0, 0.0, 1.0, 0.0);
  graph.emplace_shared<PriorFactor<OrientedPlane3> >(plane_key_2,
                                                     priorMeanPlane2,
                                                     prior_noise);

  OrientedPlane3 priorMeanPlane1(0.0, 0.0, 1.0, 1.0);
  graph.emplace_shared<PriorFactor<OrientedPlane3> >(plane_key_1,
                                                     priorMeanPlane1,
                                                     prior_noise);

  /// Shared noise for all constraints between landmarks and planes.
  noiseModel::Isotropic::shared_ptr regularity_noise =
          noiseModel::Isotropic::Sigma(1, 0.5);

  /// Plane 1 to landmarks 1,2,3.
  graph.emplace_shared<BasicRegularPlane3Factor>(landmark_key_1, plane_key_1, regularity_noise);
  graph.emplace_shared<BasicRegularPlane3Factor>(landmark_key_2, plane_key_1, regularity_noise);
  graph.emplace_shared<BasicRegularPlane3Factor>(landmark_key_3, plane_key_1, regularity_noise);

  if (true) {
    /// Noise model for cosntraint between the two planes.
    noiseModel::Diagonal::shared_ptr parallel_plane_noise =
        noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

    /// Parallelism constraint between Plane 1 and Plane 2.
    graph.emplace_shared<ParallelPlaneRegularBasicFactor>(
          plane_key_1,
          plane_key_2,
          parallel_plane_noise);
  } else {
      /// Noise model for constraint between the two planes, in tangent space.
      noiseModel::Diagonal::shared_ptr parallelPlaneNoise =
          noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));

      /// Parallelism constraint between Plane 1 and Plane 2, in tangent space.
      graph.emplace_shared<ParallelPlaneRegularTangentSpaceFactor>(
            plane_key_1,
            plane_key_2,
            parallelPlaneNoise);
  }

  //graph.print("\nFactor Graph:\n");

  Values initial;
  initial.insert(landmark_key_1, Point3(0.0, 0.2, 0.9));
  initial.insert(landmark_key_2, Point3(1.0, 0.0, 1.0));
  initial.insert(landmark_key_3, Point3(0.1, 1.1, 1.1));
  initial.insert(plane_key_1, OrientedPlane3(0.1, 0.1, 0.9, 0.9));
  initial.insert(plane_key_2, OrientedPlane3(0.1, 0.1, 0.9, 0.1));

  GaussianFactorGraph gfg = *graph.linearize(initial);
  gfg.print("\nFactor Graph:\n");

  //GaussNewtonParams params;
  //params.setVerbosity("LINEAR");
  //params.setMaxIterations(1);
  //params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  ////params.setErrorTol(-std::numeric_limits<double>::max());
  //params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  //Values result = GaussNewtonOptimizer(graph, initial, params).optimize();
  //Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();

  Values expected;
  expected.insert(landmark_key_1, prior_mean_1);
  expected.insert(landmark_key_2, prior_mean_2);
  expected.insert(landmark_key_3, prior_mean_3);
  expected.insert(plane_key_1, priorMeanPlane1);
  expected.insert(plane_key_2, priorMeanPlane2);

  //EXPECT(assert_equal(expected, result, tol))
}

/* ************************************************************************* */
TEST(testGeneralParallelPlaneTangentSpaceRegularFactor, PlaneOptimization) {
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
TEST(testGeneralParallelPlaneBasicRegularFactor, PlaneOptimization) {
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
          noiseModel::Diagonal::Sigmas(Vector4(0.1, 0.1, 0.1, 0.1));

  /// Factor between planes for co-planarity.
  const double measured_distance_from_plane2_to_plane1(-2.0);
  graph.emplace_shared<GeneralParallelPlaneRegularBasicFactor>(
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


/* ************************************************************************* */
int main() {
  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
