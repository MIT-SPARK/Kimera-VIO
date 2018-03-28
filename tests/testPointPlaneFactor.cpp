/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testBasicRegularPlane3Factor.cpp
 * @brief  test BasicRegularPlane3Factor
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
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include "factors/PointPlaneFactor.h"

using namespace std;
using namespace gtsam;

/// Test tolerance
static constexpr double tol = 1e-5;
/// Delta increment for numerical derivative
static constexpr double delta_value = 1e-5;

/**
  * Test that error does give the right result when it is zero.
/* ************************************************************************* */
TEST(testPointPlaneFactor, ErrorIsZero) {
  Key pointKey(1);
  Key planeKey(2);
  noiseModel::Diagonal::shared_ptr regularityNoise =
          noiseModel::Diagonal::Sigmas(Vector1(0.1));
  PointPlaneFactor factor(pointKey, planeKey, regularityNoise);

  Unit3 plane_normal(2.4, 1.2, 1.9);
  double distance(2.3);

  // Set the linearization point
  Point3 point(distance * plane_normal.unitVector()[0],
               distance * plane_normal.unitVector()[1],
               distance * plane_normal.unitVector()[2]);
  OrientedPlane3 plane(plane_normal, distance);

  // Use the factor to calculate the Jacobians
  Vector error = factor.evaluateError(point, plane);
  Vector1 expected_error = Vector1::Constant(0.0);

  CHECK(assert_equal(expected_error, error, tol));
}

/**
  * Test that error does give the right result when it is not zero.
/* ************************************************************************* */
TEST(testPointPlaneFactor, ErrorOtherThanZero) {
  Key pointKey(1);
  Key planeKey(2);
  noiseModel::Diagonal::shared_ptr regularityNoise =
          noiseModel::Diagonal::Sigmas(Vector1(0.1));
  PointPlaneFactor factor(pointKey, planeKey, regularityNoise);

  Unit3 plane_normal(2.4, 1.2, 1.9);
  double distance(2.3);

  // Set the linearization point
  Point3 point(-2.3, 2.1, 0.0);
  OrientedPlane3 plane(plane_normal, distance);

  // Use the factor to calculate the Jacobians
  Vector error = factor.evaluateError(point, plane);
  Vector1 expected_error = Vector1::Constant(-3.2124498);

  CHECK(assert_equal(expected_error, error, tol));
}

/**
  * Test that analytical jacobians equal numerical ones.
  *
/* ************************************************************************* */
TEST(testPointPlaneFactor, Jacobians) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key pointKey(1);
  Key planeKey(2);
  noiseModel::Diagonal::shared_ptr regularityNoise =
          noiseModel::Diagonal::Sigmas(Vector1(0.1));
  PointPlaneFactor factor(pointKey, planeKey, regularityNoise);

  double plane_normal [3] = {2.4, 1.2, 1.9};
  double distance(2.3);

  // Set the linearization point
  Point3 point(4.3, 3.2, 1.9);
  OrientedPlane3 plane(Unit3(plane_normal[0], plane_normal[1], plane_normal[2]),
                       distance);

  // Use the factor to calculate the Jacobians
  gtsam::Matrix H1Actual, H2Actual;
  factor.evaluateError(point, plane, H1Actual, H2Actual);

  // Calculate numerical derivatives
  Matrix H1Expected = numericalDerivative21<Vector, Point3, OrientedPlane3>(
      boost::bind(&PointPlaneFactor::evaluateError, &factor, _1, _2,
          boost::none, boost::none), point, plane, delta_value);

  Matrix H2Expected = numericalDerivative22<Vector, Point3, OrientedPlane3>(
      boost::bind(&PointPlaneFactor::evaluateError, &factor, _1, _2,
          boost::none, boost::none), point, plane, delta_value);

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1Actual, tol));
  CHECK(assert_equal(H2Expected, H2Actual, tol));
}

/**
  * Test that analytical jacobians equal numerical ones, with negative values.
  *
/* ************************************************************************* */
TEST(testPointPlaneFactor, JacobiansNegative) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key pointKey(1);
  Key planeKey(2);
  noiseModel::Diagonal::shared_ptr regularityNoise =
          noiseModel::Diagonal::Sigmas(Vector1(0.1));
  PointPlaneFactor factor(pointKey, planeKey, regularityNoise);

  double plane_normal [3] = {2.4, 1.2, 1.9};
  double distance(-2.3);

  // Set the linearization point
  Point3 point(4.3, 3.2, 1.9);
  OrientedPlane3 plane(Unit3(plane_normal[0], plane_normal[1], plane_normal[2]),
                       distance);

  // Use the factor to calculate the Jacobians
  gtsam::Matrix H1Actual, H2Actual;
  factor.evaluateError(point, plane, H1Actual, H2Actual);

  // Calculate numerical derivatives
  Matrix H1Expected = numericalDerivative21<Vector, Point3, OrientedPlane3>(
      boost::bind(&PointPlaneFactor::evaluateError, &factor, _1, _2,
          boost::none, boost::none), point, plane, delta_value);

  Matrix H2Expected = numericalDerivative22<Vector, Point3, OrientedPlane3>(
      boost::bind(&PointPlaneFactor::evaluateError, &factor, _1, _2,
          boost::none, boost::none), point, plane, delta_value);

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1Actual, tol));
  CHECK(assert_equal(H2Expected, H2Actual, tol));
}

/**
  * Test that optimization works for plane parameters.
  * Three landmarks, with prior factors, and a plane constrained together
  * using the landmark-plane factor.
  *
/* ************************************************************************* */
TEST(testPointPlaneFactor, PlaneOptimization) {
  NonlinearFactorGraph graph;

  Point3 priorMean1(0.0, 0.0, 1.0); // prior at origin
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
  graph.emplace_shared<PriorFactor<Point3> >(1, priorMean1, priorNoise);
  Point3 priorMean2(1.0, 0.0, 1.0); // prior at origin
  graph.emplace_shared<PriorFactor<Point3> >(2, priorMean2, priorNoise);
  Point3 priorMean3(0.0, 1.0, 1.0); // prior at origin
  graph.emplace_shared<PriorFactor<Point3> >(3, priorMean3, priorNoise);

  noiseModel::Isotropic::shared_ptr regularityNoise = noiseModel::Isotropic::Sigma(1, 0.5);
  graph.emplace_shared<PointPlaneFactor>(1, 4, regularityNoise);
  graph.emplace_shared<PointPlaneFactor>(2, 4, regularityNoise);
  graph.emplace_shared<PointPlaneFactor>(3, 4, regularityNoise);

  Values initial;
  initial.insert(1, Point3(0.0, 19.0, 3.0));
  initial.insert(2, Point3(-1.0, 2.0, 2.0));
  initial.insert(3, Point3(0.3, -1.0, 8.0));
  initial.insert(4, OrientedPlane3(0.1, 0.2, 0.9, 0.0));

  GaussNewtonParams params;
  //params.setVerbosity("ERROR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  //params.setErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();

  Values expected;
  expected.insert(1, priorMean1);
  expected.insert(2, priorMean2);
  expected.insert(3, priorMean3);
  expected.insert(4, OrientedPlane3(0.0, 0.0, 1.0, 1.0));

  CHECK(assert_equal(expected, result, tol))
}

/**
  * Test that optimization works for landmark parameters.
  * Three landmarks, with prior factor with high uncertainty in z, and a plane,
  * with a confident prior factor, constrained together using the
  * landmark-plane factor.
  *
/* ************************************************************************* */
TEST(testBasicRegularPlane3Factor, LandmarkOptimization) {
  NonlinearFactorGraph graph;

  // Set landmark priors, with high uncertainty on z.
  Point3 priorMean1(0.0, 0.0, 2.0);
  noiseModel::Diagonal::shared_ptr priorNoise =
          noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 100.0));
  graph.emplace_shared<PriorFactor<Point3> >(1, priorMean1, priorNoise);
  Point3 priorMean2(1.0, 0.0, 0.0);
  graph.emplace_shared<PriorFactor<Point3> >(2, priorMean2, priorNoise);
  Point3 priorMean3(0.0, 1.0, -2.0);
  graph.emplace_shared<PriorFactor<Point3> >(3, priorMean3, priorNoise);

  // Horizontal Plane with prior.
  Key planeKey = 4;
  OrientedPlane3 priorMean(0.0, 0.0, 1.0, 1.0);
  noiseModel::Diagonal::shared_ptr priorNoisePlane =
          noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
  graph.emplace_shared<PriorFactor<OrientedPlane3> >(planeKey, priorMean, priorNoisePlane);

  noiseModel::Isotropic::shared_ptr regularityNoise = noiseModel::Isotropic::Sigma(1, 0.1);
  graph.emplace_shared<PointPlaneFactor>(1, planeKey, regularityNoise);
  graph.emplace_shared<PointPlaneFactor>(2, planeKey, regularityNoise);
  graph.emplace_shared<PointPlaneFactor>(3, planeKey, regularityNoise);

  Values initial;
  initial.insert(1, Point3(0.0, 0.0, -3.0));
  initial.insert(2, Point3(1.0, 2.0, 2.0));
  initial.insert(3, Point3(0.3, 1.0, 8.0));
  initial.insert(4, OrientedPlane3(0.1, 0.2, 0.9, 0.0));

  GaussNewtonParams params;
  //params.setVerbosity("ERROR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();

  Values expected;
  expected.insert(1, Point3(0.0, 0.0, 1.0));
  expected.insert(2, Point3(1.0, 0.0, 1.0));
  expected.insert(3, Point3(0.0, 1.0, 1.0));
  expected.insert(4, OrientedPlane3(0.0, 0.0, 1.0, 1.0));

  CHECK(assert_equal(expected, result, tol))
}

/* ************************************************************************* */
int main() {
  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
