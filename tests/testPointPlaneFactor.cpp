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
#include "kimera-vio/factors/PointPlaneFactor.h"
#include "kimera-vio/test/EvaluateFactor.h"

using namespace std;
using namespace gtsam;
using namespace VIO;

/// Test tolerance
static constexpr double tol = 1e-5;
/// Delta increment for numerical derivative
static constexpr double delta_value = 1e-5;

/* -------------------------------------------------------------------------- */
// Set parameters for ISAM 2 incremental smoother.
void setIsam2Params(const BackendParams& vio_params,
                    gtsam::ISAM2Params* isam_param) {
  CHECK_NOTNULL(isam_param);
  // iSAM2 SETTINGS
  gtsam::ISAM2GaussNewtonParams gauss_newton_params;
  // TODO remove this hardcoded value...
  gauss_newton_params.wildfireThreshold = -1.0;
  // gauss_newton_params.setWildfireThreshold(0.001);

  gtsam::ISAM2DoglegParams dogleg_params;
  // dogleg_params.setVerbose(false); // only for debugging.

  if (vio_params.useDogLeg_) {
    isam_param->optimizationParams = dogleg_params;
  } else {
    isam_param->optimizationParams = gauss_newton_params;
  }

  // Here there was commented code about setRelinearizeThreshold.
  isam_param->cacheLinearizedFactors = false;
  isam_param->evaluateNonlinearError = true;
  isam_param->relinearizeThreshold = vio_params.relinearizeThreshold_;
  isam_param->relinearizeSkip = vio_params.relinearizeSkip_;
  // isam_param->enablePartialRelinearizationCheck = true;
  isam_param->findUnusedFactorSlots = true;
  // isam_param->cacheLinearizedFactors = true;
  // isam_param->enableDetailedResults = true;   // only for debugging.
  isam_param->factorization = gtsam::ISAM2Params::CHOLESKY;  // QR
  // isam_param->print("isam_param");
  // isam_param.evaluateNonlinearError = true;  // only for debugging.
}

/**
 * Test that error does give the right result when it is zero.
 */
TEST(testPointPlaneFactor, ErrorIsZero) {
  Key pointKey(1);
  Key planeKey(2);
  noiseModel::Diagonal::shared_ptr regularityNoise =
      noiseModel::Diagonal::Sigmas(Vector1::Constant(0.1));
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

  ASSERT_TRUE(assert_equal(expected_error, error, tol));
}

/**
 * Test that error does give the right result when it is not zero.
 */
TEST(testPointPlaneFactor, ErrorOtherThanZero) {
  Key pointKey(1);
  Key planeKey(2);
  noiseModel::Diagonal::shared_ptr regularityNoise =
      noiseModel::Diagonal::Sigmas(Vector1::Constant(0.1));
  PointPlaneFactor factor(pointKey, planeKey, regularityNoise);

  Unit3 plane_normal(2.4, 1.2, 1.9);
  double distance(2.3);

  // Set the linearization point
  Point3 point(-2.3, 2.1, 0.0);
  OrientedPlane3 plane(plane_normal, distance);

  // Use the factor to calculate the Jacobians
  Vector error = factor.evaluateError(point, plane);
  Vector1 expected_error = Vector1::Constant(-3.2124498);

  ASSERT_TRUE(assert_equal(expected_error, error, tol));
}

/**
 * Test that analytical jacobians equal numerical ones.
 *
 */
TEST(testPointPlaneFactor, Jacobians) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key pointKey(1);
  Key planeKey(2);
  noiseModel::Diagonal::shared_ptr regularityNoise =
      noiseModel::Diagonal::Sigmas(Vector1::Constant(0.1));
  PointPlaneFactor factor(pointKey, planeKey, regularityNoise);

  double plane_normal[3] = {2.4, 1.2, 1.9};
  double distance(2.3);

  // Set the linearization point
  Point3 point(4.3, 3.2, 1.9);
  OrientedPlane3 plane(Unit3(plane_normal[0], plane_normal[1], plane_normal[2]),
                       distance);

  VIO::test::evaluateFactor(factor, point, plane, tol, delta_value);
}

/**
 * Test that analytical jacobians equal numerical ones, with negative values.
 *
 */
TEST(testPointPlaneFactor, JacobiansNegative) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key pointKey(1);
  Key planeKey(2);
  noiseModel::Diagonal::shared_ptr regularityNoise =
      noiseModel::Diagonal::Sigmas(Vector1::Constant(0.1));
  PointPlaneFactor factor(pointKey, planeKey, regularityNoise);

  double plane_normal[3] = {2.4, 1.2, 1.9};
  double distance(-2.3);

  // Set the linearization point
  Point3 point(4.3, 3.2, 1.9);
  OrientedPlane3 plane(Unit3(plane_normal[0], plane_normal[1], plane_normal[2]),
                       distance);

  VIO::test::evaluateFactor(factor, point, plane, delta_value, tol);
}

/**
 * Test that optimization works for plane parameters.
 * Three landmarks, with prior factors, and a plane constrained together
 * using the landmark-plane factor.
 *
 */
TEST(testPointPlaneFactor, PlaneOptimization) {
  NonlinearFactorGraph graph;

  Point3 priorMean1(0.0, 0.0, 1.0);  // prior at origin
  noiseModel::Diagonal::shared_ptr priorNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
  graph.emplace_shared<PriorFactor<Point3>>(1, priorMean1, priorNoise);
  Point3 priorMean2(1.0, 0.0, 1.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(2, priorMean2, priorNoise);
  Point3 priorMean3(0.0, 1.0, 1.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(3, priorMean3, priorNoise);

  noiseModel::Isotropic::shared_ptr regularityNoise =
      noiseModel::Isotropic::Sigma(1, 0.5);
  graph.emplace_shared<PointPlaneFactor>(1, 4, regularityNoise);
  graph.emplace_shared<PointPlaneFactor>(2, 4, regularityNoise);
  graph.emplace_shared<PointPlaneFactor>(3, 4, regularityNoise);

  Values initial;
  initial.insert(1, Point3(0.0, 19.0, 3.0));
  initial.insert(2, Point3(-1.0, 2.0, 2.0));
  initial.insert(3, Point3(0.3, -1.0, 8.0));
  initial.insert(4, OrientedPlane3(0.1, 0.2, 0.9, 0.0));

  GaussNewtonParams params;
  // params.setVerbosity("ERROR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  // params.setErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();

  Values expected;
  expected.insert(1, priorMean1);
  expected.insert(2, priorMean2);
  expected.insert(3, priorMean3);
  expected.insert(4, OrientedPlane3(0.0, 0.0, 1.0, 1.0));

  ASSERT_TRUE(assert_equal(expected, result, tol));
}

/**
 * Test that optimization works for landmark parameters.
 * Three landmarks, with prior factor with high uncertainty in z, and a plane,
 * with a confident prior factor, constrained together using the
 * landmark-plane factor.
 *
 */
TEST(testBasicRegularPlane3Factor, LandmarkOptimization) {
  NonlinearFactorGraph graph;

  // Set landmark priors, with high uncertainty on z.
  Point3 priorMean1(0.0, 0.0, 2.0);
  noiseModel::Diagonal::shared_ptr priorNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 100.0));
  graph.emplace_shared<PriorFactor<Point3>>(1, priorMean1, priorNoise);
  Point3 priorMean2(1.0, 0.0, 0.0);
  graph.emplace_shared<PriorFactor<Point3>>(2, priorMean2, priorNoise);
  Point3 priorMean3(0.0, 1.0, -2.0);
  graph.emplace_shared<PriorFactor<Point3>>(3, priorMean3, priorNoise);

  // Horizontal Plane with prior.
  Key planeKey = 4;
  OrientedPlane3 priorMean(0.0, 0.0, 1.0, 1.0);
  noiseModel::Diagonal::shared_ptr priorNoisePlane =
      noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
  graph.emplace_shared<PriorFactor<OrientedPlane3>>(
      planeKey, priorMean, priorNoisePlane);

  noiseModel::Isotropic::shared_ptr regularityNoise =
      noiseModel::Isotropic::Sigma(1, 0.1);
  graph.emplace_shared<PointPlaneFactor>(1, planeKey, regularityNoise);
  graph.emplace_shared<PointPlaneFactor>(2, planeKey, regularityNoise);
  graph.emplace_shared<PointPlaneFactor>(3, planeKey, regularityNoise);

  Values initial;
  initial.insert(1, Point3(0.0, 0.0, -3.0));
  initial.insert(2, Point3(1.0, 2.0, 2.0));
  initial.insert(3, Point3(0.3, 1.0, 8.0));
  initial.insert(4, OrientedPlane3(0.1, 0.2, 0.9, 0.0));

  GaussNewtonParams params;
  // params.setVerbosity("ERROR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();

  Values expected;
  expected.insert(1, Point3(0.0, 0.0, 1.0));
  expected.insert(2, Point3(1.0, 0.0, 1.0));
  expected.insert(3, Point3(0.0, 1.0, 1.0));
  expected.insert(4, OrientedPlane3(0.0, 0.0, 1.0, 1.0));

  ASSERT_TRUE(assert_equal(expected, result, tol));
}

TEST(testPointPlaneFactor, MultiplePlanesOptimization) {
  NonlinearFactorGraph graph;

  noiseModel::Diagonal::shared_ptr priorNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
  Point3 priorMean1(0.0, 0.0, 1.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(1, priorMean1, priorNoise);
  Point3 priorMean2(1.0, 0.0, 1.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(2, priorMean2, priorNoise);
  Point3 priorMean3(0.0, 1.0, 1.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(3, priorMean3, priorNoise);

  noiseModel::Isotropic::shared_ptr regularityNoise =
      noiseModel::Isotropic::Sigma(1, 0.5);
  graph.emplace_shared<PointPlaneFactor>(1, 4, regularityNoise);
  graph.emplace_shared<PointPlaneFactor>(2, 4, regularityNoise);
  graph.emplace_shared<PointPlaneFactor>(3, 4, regularityNoise);

  // Add new plane.
  Point3 priorMeanA(0.0, 0.0, 2.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(5, priorMeanA, priorNoise);
  Point3 priorMeanB(1.0, 0.0, 2.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(6, priorMeanB, priorNoise);
  Point3 priorMeanC(0.0, 1.0, 2.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(7, priorMeanC, priorNoise);

  graph.emplace_shared<PointPlaneFactor>(5, 8, regularityNoise);
  graph.emplace_shared<PointPlaneFactor>(6, 8, regularityNoise);
  graph.emplace_shared<PointPlaneFactor>(7, 8, regularityNoise);

  Values initial;
  initial.insert(1, Point3(0.0, 19.0, 3.0));
  initial.insert(2, Point3(-1.0, 2.0, 2.0));
  initial.insert(3, Point3(0.3, -1.0, 8.0));
  initial.insert(4, OrientedPlane3(0.1, 0.2, 0.9, 0.0));
  initial.insert(5, Point3(0.0, 19.0, 2.0));
  initial.insert(6, Point3(-1.0, 2.0, 2.0));
  initial.insert(7, Point3(0.3, -1.0, 2.0));
  initial.insert(8, OrientedPlane3(0.1, 0.2, 0.9, 2.0));

  GaussNewtonParams params;
  // params.setVerbosity("ERROR");
  params.setMaxIterations(20);
  params.setRelativeErrorTol(-std::numeric_limits<double>::max());
  // params.setErrorTol(-std::numeric_limits<double>::max());
  params.setAbsoluteErrorTol(-std::numeric_limits<double>::max());

  Values result = GaussNewtonOptimizer(graph, initial, params).optimize();

  Values expected;
  expected.insert(1, priorMean1);
  expected.insert(2, priorMean2);
  expected.insert(3, priorMean3);
  expected.insert(4, OrientedPlane3(0.0, 0.0, 1.0, 1.0));
  expected.insert(5, priorMeanA);
  expected.insert(6, priorMeanB);
  expected.insert(7, priorMeanC);
  expected.insert(8, OrientedPlane3(0.0, 0.0, 1.0, 2.0));

  ASSERT_TRUE(assert_equal(expected, result, tol));
}

TEST(testPointPlaneFactor, MultiplePlanesIncrementalOptimization) {
  NonlinearFactorGraph graph;

  noiseModel::Diagonal::shared_ptr priorNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
  Point3 priorMean1(0.0, 0.0, 1.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(1, priorMean1, priorNoise);
  Point3 priorMean2(1.0, 0.0, 1.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(2, priorMean2, priorNoise);
  Point3 priorMean3(0.0, 1.0, 1.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(3, priorMean3, priorNoise);

  noiseModel::Isotropic::shared_ptr regularityNoise =
      noiseModel::Isotropic::Sigma(1, 0.5);
  graph.emplace_shared<gtsam::PointPlaneFactor>(1, 4, regularityNoise);
  graph.emplace_shared<gtsam::PointPlaneFactor>(2, 4, regularityNoise);
  graph.emplace_shared<gtsam::PointPlaneFactor>(3, 4, regularityNoise);
  Values initial;
  initial.insert(1, Point3(0.0, 19.0, 3.0));
  initial.insert(2, Point3(-1.0, 2.0, 2.0));
  initial.insert(3, Point3(0.3, -1.0, 8.0));
  initial.insert(4, OrientedPlane3(0.1, 0.2, 0.9, 0.0));

  std::map<Key, double> timestamps;
  for (const auto& keyValue : initial) {
    timestamps[keyValue.key] = 0;
  }

  gtsam::ISAM2Params isam_param;
  BackendParams vioParams = BackendParams();
  setIsam2Params(vioParams, &isam_param);
  gtsam::IncrementalFixedLagSmoother smoother(vioParams.nr_states_, isam_param);
  try {
    // Update smoother.
    gtsam::FactorIndices delete_slots;
    smoother.update(graph, initial, timestamps, delete_slots);
    // Another extra iteration.
    for (size_t i = 0; i < 3; i++) {
      smoother.update(gtsam::NonlinearFactorGraph(),
                      gtsam::Values(),
                      gtsam::FixedLagSmoother::KeyTimestampMap(),
                      gtsam::FastVector<gtsam::FactorIndex>());
    }
  } catch (const gtsam::IndeterminantLinearSystemException& e) {
    LOG(ERROR) << e.what();

    const gtsam::Key& var = e.nearbyVariable();
    gtsam::Symbol symb(var);

    LOG(ERROR) << "ERROR: Variable has type '" << symb.chr() << "' "
               << "and index " << symb.index();

    smoother.getFactors().print("Smoother's factors:\n[\n\t");
    LOG(ERROR) << " ]";

    throw;
  }

  // Get result.
  Values result;
  result = smoother.calculateEstimate();

  Values expected;
  expected.insert(1, priorMean1);
  expected.insert(2, priorMean2);
  expected.insert(3, priorMean3);
  expected.insert(4, OrientedPlane3(0.0, 0.0, 1.0, 1.0));

  ASSERT_TRUE(assert_equal(expected, result, tol));

  // Clean old factors.
  graph.resize(0);

  // Add new plane.
  Point3 priorMeanA(0.0, 0.0, 2.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(5, priorMeanA, priorNoise);
  Point3 priorMeanB(1.0, 0.0, 2.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(6, priorMeanB, priorNoise);
  Point3 priorMeanC(0.0, 1.0, 2.0);  // prior at origin
  graph.emplace_shared<PriorFactor<Point3>>(7, priorMeanC, priorNoise);

  graph.emplace_shared<gtsam::PointPlaneFactor>(5, 8, regularityNoise);
  graph.emplace_shared<gtsam::PointPlaneFactor>(6, 8, regularityNoise);
  graph.emplace_shared<gtsam::PointPlaneFactor>(7, 8, regularityNoise);

  initial.clear();
  initial.insert(5, Point3(0.0, 19.0, 2.0));
  initial.insert(6, Point3(-1.0, 2.0, 2.0));
  initial.insert(7, Point3(0.3, -1.0, 2.0));
  initial.insert(8, OrientedPlane3(0.1, 0.2, 0.9, 2.0));

  timestamps.clear();
  for (const auto& keyValue : initial) {
    timestamps[keyValue.key] = 0;
  }

  try {
    // Update smoother.
    gtsam::FactorIndices delete_slots;
    smoother.update(graph, initial, timestamps, delete_slots);
    // Another extra iteration.
    for (size_t i = 0; i < 3; i++) {
      smoother.update(gtsam::NonlinearFactorGraph(),
                      gtsam::Values(),
                      gtsam::FixedLagSmoother::KeyTimestampMap(),
                      gtsam::FastVector<gtsam::FactorIndex>());
    }
  } catch (const gtsam::IndeterminantLinearSystemException& e) {
    LOG(ERROR) << e.what();

    const gtsam::Key& var = e.nearbyVariable();
    gtsam::Symbol symb(var);

    LOG(ERROR) << "ERROR: Variable has type '" << symb.chr() << "' "
               << "and index " << symb.index();

    smoother.getFactors().print("Smoother's factors:\n[\n\t");
    LOG(ERROR) << " ]";

    throw;
  }

  // Get result.
  result = smoother.calculateEstimate();

  expected.insert(5, priorMeanA);
  expected.insert(6, priorMeanB);
  expected.insert(7, priorMeanC);
  expected.insert(8, OrientedPlane3(0.0, 0.0, 1.0, 2.0));

  ASSERT_TRUE(assert_equal(expected, result, tol));
}
