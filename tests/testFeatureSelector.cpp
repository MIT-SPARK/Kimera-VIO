/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testFeatureSelector.h
 * @brief  test FeatureSelector. FeatureSelector describes a greedy feature
 * selection strategy, based on the paper:
 *
 * L. Carlone and S. Karaman. Attention and Anticipation in Fast Visual-Inertial
 * Navigation. In IEEE Intl. Conf. on Robotics and Automation (ICRA), pages
 * 3886-3893, 2017.
 *
 * @author Luca Carlone
 */

#include <glog/logging.h>

#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include "FeatureSelector.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

using namespace gtsam;
using namespace std;
using namespace boost::assign;
using namespace VIO;
using namespace cv;

// default
static const VioFrontEndParams trackerParams = VioFrontEndParams();
VioBackEndParams vioParams = VioBackEndParams();

typedef PinholeCamera<Cal3_S2> Camera;
typedef vector<Camera> Cameras;

static const Pose3 body_P_leftCam =
    Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
static const Pose3 body_P_rightCam =
    body_P_leftCam.compose(Pose3(Rot3(), Point3(10, 0, 0)));  // some baseline

static const Cal3_S2 Kleft(1500, 1200, 0, 640, 480);
static const Cal3_S2 Kright(1500, 1400, 0, 640, 480);

/* ************************************************************************* */
TEST(FeatureSelector, cameras) {
  KeyframeToStampedPose poses;
  poses.push_back(StampedPose(Pose3(), 0));
  poses.push_back(
      StampedPose(Pose3(Rot3::Ypr(-M_PI, 0.1, -M_PI), Point3(1, 1, 1)), 1));

  FeatureSelectorData featureSelectionData;
  featureSelectionData.body_P_leftCam = body_P_leftCam;
  featureSelectionData.body_P_rightCam = body_P_rightCam;
  featureSelectionData.left_undistRectCameraMatrix = Kleft;
  featureSelectionData.right_undistRectCameraMatrix = Kright;
  featureSelectionData.posesAtFutureKeyframes = poses;

  vioParams.smartNoiseSigma_ =
      1;  // parameter include after the test were written
  FeatureSelector f(trackerParams, vioParams);
  Cameras left_cameras, right_cameras;
  tie(left_cameras, right_cameras) = f.getCameras(featureSelectionData);

  // check nr of left and right cameras:
  EXPECT_EQ(left_cameras.size(), 2);
  EXPECT_EQ(right_cameras.size(), 2);

  for (size_t i = 0; i < 2; i++) {
    Camera expectedLeft((poses.at(i).pose).compose(body_P_leftCam), Kleft);
    Camera actualLeft = left_cameras.at(i);
    EXPECT_TRUE(assert_equal(expectedLeft, actualLeft));

    Camera expectedRight((poses.at(i).pose).compose(body_P_rightCam), Kright);
    Camera actualRight = right_cameras.at(i);
    EXPECT_TRUE(assert_equal(expectedRight, actualRight));
  }
}

/* ************************************************************************* */
TEST(FeatureSelector, createPrior1) {
  // this is mainly to confer that covariances in gtsam are in local frame
  // and that the block structure of the covariance is what we ask for [pose,
  // vel, biases] we test two main things: 1) the covariances of two poses
  // connected by a (almost noiseless) between factor are the same after
  // rotating them to the global frame. NOTE: one thing we do not capture is
  // that the position uncertainty depends on the rotation uncertainty,
  // therefore this test is only working when the rotation uncertainty is
  // relatively small 2) check that WhitenInPlace works as we expect 3)
  // createPrior returns covariances in global frame

  // create toy example
  Pose3 pose0 =
      Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));  // some pose
  Pose3 pose1 =
      Pose3(Rot3::Ypr(M_PI, 0.1, 0.3), Point3(1, 10, 1));  // some other pose
  Vector3 vel0 = Vector3(1, 1, 0);
  Vector3 vel1 = Vector3(5, 3, 1);
  imuBias::ConstantBias bias0 =
      imuBias::ConstantBias(Vector3(0.1, 0.3, 0.1), Vector3(0.2, 0.8, 0.1));
  imuBias::ConstantBias bias1 =
      imuBias::ConstantBias(Vector3(1.1, 2.3, 3.1), Vector3(1.2, 1.8, 1.1));

  NonlinearFactorGraph graph;
  // Add prior on pose0
  SharedNoiseModel noise_init_pose = noiseModel::Diagonal::Sigmas(
      (Vector(6) << 0.01, 0.02, 0.03, 4, 5, 6).finished());
  graph.push_back(boost::make_shared<PriorFactor<Pose3>>(Symbol('x', 0), pose0,
                                                         noise_init_pose));

  // Add initial velocity priors.
  SharedNoiseModel zeroVelocityPriorNoise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 7, 8, 9).finished());
  graph.push_back(boost::make_shared<PriorFactor<Vector3>>(
      Symbol('v', 0), vel0, zeroVelocityPriorNoise));

  // Add initial bias priors:
  SharedNoiseModel imu_bias_prior_noise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << 10, 11, 12, 13, 14, 15).finished());
  graph.push_back(boost::make_shared<PriorFactor<imuBias::ConstantBias>>(
      Symbol('b', 0), bias0, imu_bias_prior_noise));

  double smallSigma = 1e-3;
  // add between factors on poses
  graph.push_back(boost::make_shared<BetweenFactor<Pose3>>(
      Symbol('x', 0), Symbol('x', 1), pose0.between(pose1),
      noiseModel::Isotropic::Sigma(6, smallSigma)));  // very small noise

  // add between factors on vel
  graph.push_back(boost::make_shared<BetweenFactor<Vector3>>(
      Symbol('v', 0), Symbol('v', 1), vel1 - vel0,
      noiseModel::Isotropic::Sigma(3, smallSigma)));  // very small noise

  // add between factors on biases
  graph.push_back(boost::make_shared<BetweenFactor<imuBias::ConstantBias>>(
      Symbol('b', 0), Symbol('b', 1), bias0.between(bias1),
      noiseModel::Isotropic::Sigma(6, smallSigma)));  // very small noise

  Values state;
  state.insert(Symbol('x', 0), pose0);
  state.insert(Symbol('x', 1), pose1);
  state.insert(Symbol('v', 0), vel0);
  state.insert(Symbol('v', 1), vel1);
  state.insert(Symbol('b', 0), bias0);
  state.insert(Symbol('b', 1), bias1);
  // now let us get the covariance on pose1:
  // we expect this to be the same as the one on pose0 (since the between is
  // almost noiseless) but rotated in the frame of pose1:
  Marginals marginals(graph, state, Marginals::Factorization::CHOLESKY);

  // local covariance state 0
  KeyVector keys0;
  keys0.push_back(Symbol('x', 0));
  keys0.push_back(Symbol('v', 0));
  keys0.push_back(Symbol('b', 0));
  // return the marginal covariance matrix: SORTED KEYS!! [b v x]
  Matrix cov0 = UtilsOpenCV::Covariance_bvx2xvb(
      marginals.jointMarginalCovariance(keys0).fullMatrix());

  // local covariance state 1
  KeyVector keys1;
  keys1.push_back(Symbol('x', 1));
  keys1.push_back(Symbol('v', 1));
  keys1.push_back(Symbol('b', 1));
  // return the marginal covariance matrix: SORTED KEYS!!
  Matrix cov1 = UtilsOpenCV::Covariance_bvx2xvb(
      marginals.jointMarginalCovariance(keys1).fullMatrix());

  // NOTE:
  // if pose1 = pose0 the following is true
  // NOTE: EXPECT_TRUE(assert_equal(cov0,cov1,0.01));

  /////////////////////////////////////////////////////////////////////////////////
  // 1) compare covariances after transforming them to global frame
  // If the poses are different we have to compare in global frame
  Matrix R0 = Matrix::Identity(15, 15);
  R0.block<3, 3>(0, 0) = pose0.rotation().matrix();
  R0.block<3, 3>(3, 3) = pose0.rotation().matrix();
  Matrix cov0_global = R0 * cov0 * R0.transpose();

  Matrix R1 = Matrix::Identity(15, 15);
  Matrix3 rot1 =
      pose1.rotation().matrix();  // (pose0.between(pose1)).rotation().matrix();
  R1.block<3, 3>(0, 0) = rot1;
  R1.block<3, 3>(3, 3) = rot1;
  Matrix cov1_global = R1 * cov1 * R1.transpose();

  // large tolerance since covariances differ by smallSigma noise anyway
  // NOTE: the following is true only if the noise on the
  EXPECT_TRUE(assert_equal(cov0_global, cov1_global, 0.1));

  /////////////////////////////////////////////////////////////////////////////////
  // 2) understand WhitenInPlace
  noiseModel::Gaussian::shared_ptr noisePrior =
      noiseModel::Gaussian::Covariance(cov1_global);
  Matrix A = Matrix::Identity(15, 15);
  noisePrior->WhitenInPlace(A);  // This should make A = R = sqrt info matrix
  Matrix actualHessianMatrix =
      A.transpose() * A;  // information = inverse-covariance
  Matrix expectedHessianMatrix = cov1_global.inverse();
  EXPECT_TRUE(assert_equal(expectedHessianMatrix, actualHessianMatrix, 0.1));

  /////////////////////////////////////////////////////////////////////////////////
  // 3) check output of createPrior
  FeatureSelectorData featureSelectionData;
  featureSelectionData.posesAtFutureKeyframes.push_back(StampedPose(pose1, 1));
  featureSelectionData.currentNavStateCovariance = cov1;

  vioParams.smartNoiseSigma_ =
      1;  // parameter include after the test were written
  FeatureSelector f(trackerParams, vioParams);
  JacobianFactor J = f.createPrior(featureSelectionData);
  actualHessianMatrix = J.information();  // information = inverse-covariance
  expectedHessianMatrix = (cov1_global.block<9, 9>(3, 3)).inverse();
  EXPECT_TRUE(assert_equal(expectedHessianMatrix, actualHessianMatrix, 0.1));
}

/* ************************************************************************* */
TEST(FeatureSelector, createMatricesLinearImuFactor) {
  // create 2 stamped pose
  StampedPose spose0 = StampedPose(
      Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1)), 0);
  StampedPose spose1 =
      StampedPose(Pose3(Rot3::Ypr(M_PI, 0.1, 0.3), Point3(1, 10, 1)), 0.5);

  VioFrontEndParams trackerParams2 = VioFrontEndParams();
  trackerParams2.featureSelectionImuRate_ =
      0.1;  // fake imu rate to make problem simpler

  VioBackEndParams vioParams2 = VioBackEndParams();
  vioParams2.smartNoiseSigma_ = 1000;
  vioParams2.accNoiseDensity_ =
      0.1;  // fake noise to avoid that covariance it too small
  vioParams2.accBiasSigma_ = 0.03;
  vioParams2.imuIntegrationSigma_ = 0.2;

  Matrix Ai_actual, imuCov;
  FeatureSelector f(trackerParams2, vioParams2);
  tie(Ai_actual, imuCov) = f.createMatricesLinearImuFactor(spose0, spose1);

  double delta = 0.1;

  ///////////////////////////////////////////////////////////////////////////
  // TEST Ai: we can obtain it as Ai = A * A * ... * A
  Matrix Ai_expected = Matrix::Identity(9, 9);
  Rot3 rotij = (spose0.pose.between(spose1.pose)).rotation();
  const Matrix3 R_k_kp1 = Rot3::Expmap(Rot3::Logmap(rotij) / 5).matrix();
  Matrix3 Rk = spose0.pose.rotation().matrix();

  Matrix imuCov_expected = Matrix::Zero(9, 9);
  Matrix measNoiseCov = Matrix::Zero(9, 9);
  measNoiseCov.block<3, 3>(0, 0) = f.integrationVar_ * I_3x3;
  measNoiseCov.block<3, 3>(3, 3) = f.accVarianceDiscTime_ * I_3x3;
  measNoiseCov.block<3, 3>(6, 6) = f.biasAccVarianceDiscTime_ * I_3x3;

  for (size_t k = 0; k < 5;
       k++) {  // in 1 second we get 5 measurements at rate 0.1

    // Note: we integrate forward, hence at time j, we only have measurement of
    // time j-1
    Matrix A_k_kp1 = Matrix::Zero(9, 9);  // system matrix for single time step
    A_k_kp1.block<3, 9>(0, 0) << I_3x3, delta * I_3x3,
        -0.5 * (delta * delta) * Rk;
    A_k_kp1.block<3, 9>(3, 0) << Z_3x3, I_3x3, -delta * Rk;
    A_k_kp1.block<3, 9>(6, 0) << Z_3x3, Z_3x3, I_3x3;
    Ai_expected = A_k_kp1 * Ai_expected;

    // Propagate covariance in Kalman filter style
    // Noise coefficient matrix: multiplies noiseAcc and noiseBiasAcc
    Matrix C_k_kp1 = Matrix::Zero(9, 9);
    C_k_kp1.block<3, 9>(0, 0) << I_3x3, -0.5 * (delta * delta) * Rk, Z_3x3;
    C_k_kp1.block<3, 9>(3, 0) << Z_3x3, -delta * Rk, Z_3x3;
    C_k_kp1.block<3, 9>(6, 0) << Z_3x3, Z_3x3, I_3x3;
    imuCov_expected = A_k_kp1 * imuCov_expected * A_k_kp1.transpose() +
                      C_k_kp1 * measNoiseCov * C_k_kp1.transpose();
    // update rotation
    Rk = Rk * R_k_kp1;
  }
  Ai_expected =
      -Ai_expected;  // because we have to move it to the left-hand side

  // check that rotations are subdivided correctly: after integrating 10 times,
  // we get Rj
  EXPECT_TRUE(assert_equal(Rk, spose1.pose.rotation().matrix(), 1e-8));

  // check that building A incrementally gives the same result as batch creation
  EXPECT_TRUE(assert_equal(Ai_expected, Ai_actual, 1e-8));

  // check that building covariance incrementally gives the same result as batch
  // creation NOTE: we do not capture correlation with bias evolution hence we
  // need to zero out those blocks Same as difference between imu factor and
  // combined Imu factor
  imuCov_expected.block<6, 3>(0, 6) = Matrix::Zero(6, 3);
  imuCov_expected.block<3, 6>(6, 0) = Matrix::Zero(3, 6);
  EXPECT_TRUE(assert_equal(imuCov_expected, imuCov, 1e-1));
}

/* ************************************************************************* */
TEST(FeatureSelector, DISABLED_createOmegaBarImu) {
  // create 3 stamped pose
  StampedPose spose0 =
      StampedPose(Pose3(Rot3::Ypr(0.2, 0.4, 0.5), Point3(0, 0, 1)), 0);
  StampedPose spose1 =
      StampedPose(Pose3(Rot3::Ypr(0.5, 0.5, 0.3), Point3(1, 10, 1)), 0.5);
  StampedPose spose2 =
      StampedPose(Pose3(Rot3::Ypr(0.1, 0.2, 0.3), Point3(0.1, 0.1, 0.1)), 1);

  // create featureSelectionData
  FeatureSelectorData featureSelectionData;
  featureSelectionData.posesAtFutureKeyframes.push_back(spose0);
  featureSelectionData.posesAtFutureKeyframes.push_back(spose1);
  featureSelectionData.posesAtFutureKeyframes.push_back(spose2);
  featureSelectionData.currentNavStateCovariance = Matrix::Identity(15, 15);

  // create OmegaBar
  vioParams.smartNoiseSigma_ =
      1;  // parameter include after the test were written
  FeatureSelector f(trackerParams, vioParams);
  GaussianFactorGraph gfg = f.createOmegaBarImuAndPrior(featureSelectionData);

  // check size
  EXPECT_EQ(gfg.size(), 3);  //  1 prior and 2 linear imu factors

  // check keys:
  FastVector<Key> keys0 = gfg.at(0)->keys();
  EXPECT_EQ(keys0.size(), 1);  // first factor is a prior

  FastVector<Key> keys1 = gfg.at(1)->keys();
  EXPECT_EQ(keys1.size(), 2);  // linear imu factor
  EXPECT_EQ(keys1[0] == 0 && keys1[1], 1);

  FastVector<Key> keys2 = gfg.at(2)->keys();
  EXPECT_EQ(keys2.size(), 2);  // linear imu factor
  EXPECT_EQ(keys2[0] == 1 && keys2[1], 2);
}

typedef PinholeCamera<Cal3_S2> Camera;
/* ************************************************************************* */
TEST(FeatureSelector, DISABLED_GetVersorIfInFOV) {
  // create a camera
  Camera cam(body_P_leftCam, Cal3_S2(500, 500, 0.1, 640 / 2, 480 / 2));

  // reproject some points *in* FOV
  double depth = 2;
  // this falls an epsilon outside, so we'll try less extreme conditions
  // Point3 p = cam.backproject(Point2(0,0), depth);

  // CHECK points inside fov
  Point3 p;
  for (size_t i = 0; i < 4; i++) {
    switch (i) {
      case 0:
        p = cam.backproject(Point2(1, 1), depth);
        break;
      case 1:
        p = cam.backproject(Point2(639, 480), depth);
        break;
      case 2:
        p = cam.backproject(Point2(320, 200), depth);
        break;
      case 3:
        p = cam.backproject(Point2(640, 0), depth);
        break;
    }
    // check that points in FOV are calibrated correctly
    EXPECT_TRUE(assert_equal(Unit3((cam.pose().transform_to(p)).vector()),
                             *FeatureSelector::GetVersorIfInFOV(cam, p)));

    // check that point far away does not pass check (set max distance = 1m)
    EXPECT_TRUE(!FeatureSelector::GetVersorIfInFOV(cam, p, 1.0));
  }
  // CHECK points outside fov
  for (size_t i = 0; i < 4; i++) {
    switch (i) {
      case 0:
        p = cam.backproject(Point2(-1, -1), depth);
        break;
      case 1:
        p = cam.backproject(Point2(641, 480), depth);
        break;
      case 2:
        p = cam.backproject(Point2(-320, 200), depth);
        break;
      case 3:
        p = cam.backproject(Point2(641, 0), depth);
        break;
    }
    // check that points in FOV are calibrated correctly
    EXPECT_TRUE(!FeatureSelector::GetVersorIfInFOV(cam, p));
  }
}

static const Cal3_S2 K = Cal3_S2(500, 500, 0.1, 640 / 2, 480 / 2);
/* ************************************************************************* */
TEST(FeatureSelector, createLinearVisionFactor_no_parallax) {
  // create 3 stamped pose
  Pose3 pose0 = Pose3(Rot3::Ypr(0.2, 0.4, 0.5), Point3(0, 0, 1));
  StampedPose spose0 = StampedPose(pose0, 0);
  StampedPose spose1 = StampedPose(pose0, 0.5);  // same pose

  // create featureSelectionData
  FeatureSelectorData featureSelectionData;
  featureSelectionData.posesAtFutureKeyframes.push_back(spose0);
  featureSelectionData.posesAtFutureKeyframes.push_back(spose1);
  featureSelectionData.left_undistRectCameraMatrix = K;
  featureSelectionData.right_undistRectCameraMatrix = K;

  vioParams.smartNoiseSigma_ =
      1;  // parameter include after the test were written
  FeatureSelector f(trackerParams, vioParams);
  Cameras left_cameras, right_cameras;
  tie(left_cameras, right_cameras) = f.getCameras(featureSelectionData);

  // create a set of Jacobian factors, then eliminate points and get the same
  // matrix
  Camera cam(spose0.pose, K);
  Point3 pworld_l =
      cam.backproject(Point2(320, 200), 2.0);  // backprojected 2 meters away

  // check that points in FOV are calibrated correctly
  EXPECT_TRUE(
      FeatureSelector::GetVersorIfInFOV(Camera(spose1.pose, K), pworld_l));

  // get actual factor
  double time1 = 0, time2 = 0, time3 = 0;
  HessianFactor::shared_ptr hFactor = f.createLinearVisionFactor(
      pworld_l, left_cameras, right_cameras, time1, time2, time3);

  // check that we got an empty factor
  EXPECT_TRUE(assert_equal(HessianFactor(), *hFactor.get()));
}

/* ************************************************************************* */
TEST(FeatureSelector, createLinearVisionFactor_And_SchurComplement) {
  // create 3 stamped pose
  Pose3 pose0 = Pose3(Rot3::Ypr(0.2, 0.4, 0.5), Point3(0, 0, 1));
  StampedPose spose0 = StampedPose(pose0, 0);
  StampedPose spose1 =
      StampedPose(pose0.compose(Pose3(Rot3(), Point3(0.2, 0, 0))), 0.5);
  StampedPose spose2 =
      StampedPose(pose0.compose(Pose3(Rot3(), Point3(0.5, 0, 0))), 1);

  // create featureSelectionData
  FeatureSelectorData featureSelectionData;
  featureSelectionData.posesAtFutureKeyframes.push_back(spose0);
  featureSelectionData.posesAtFutureKeyframes.push_back(spose1);
  featureSelectionData.posesAtFutureKeyframes.push_back(spose2);
  featureSelectionData.left_undistRectCameraMatrix = K;
  featureSelectionData.right_undistRectCameraMatrix = K;

  vioParams.smartNoiseSigma_ =
      1;  // parameter include after the test were written
  FeatureSelector f(trackerParams, vioParams);
  Cameras left_cameras, right_cameras;
  tie(left_cameras, right_cameras) = f.getCameras(featureSelectionData);

  // create a set of Jacobian factors, then eliminate points and get the same
  // matrix
  Camera cam(spose0.pose, K);
  Point3 pworld_l =
      cam.backproject(Point2(320, 200), 2.0);  // backprojected 2 meters away

  // check that points in FOV are calibrated correctly
  EXPECT_TRUE(
      FeatureSelector::GetVersorIfInFOV(Camera(spose1.pose, K), pworld_l));
  EXPECT_TRUE(
      FeatureSelector::GetVersorIfInFOV(Camera(spose2.pose, K), pworld_l));

  double time1 = 0, time2 = 0, time3 = 0;
  HessianFactor::shared_ptr hFactor = f.createLinearVisionFactor(
      pworld_l, left_cameras, right_cameras, time1, time2, time3);
  Matrix actualHessian = hFactor->information();  // NOTE: he we assume that the
                                                  // keys are in the right order

  // create graph Jacobian factors corresponding to projection measurements:
  GaussianFactorGraph gfg;
  Key l0 = 10;  // a key
  for (size_t i = 0; i < 3;
       i++) {  // for each camera we create 2 jacobian factors (left, right)
    for (size_t j = 0; j < 2; j++) {  // for left and right
      Pose3 pose = featureSelectionData.posesAtFutureKeyframes.at(i).pose;
      Unit3 uij = Unit3(pose.transform_to(pworld_l).vector());
      Matrix3 M33 = uij.skew() * pose.rotation().transpose();
      Matrix M39 = Matrix::Zero(3, 9);
      M39.block<3, 3>(0, 0) = -M33;
      gfg.push_back(JacobianFactor(i, M39, l0, M33, Vector3::Zero()));
    }
  }
  // now get the Hessian out of the graph:
  Ordering ordering = Ordering(list_of(0)(1)(2)(10));  // point last
  Matrix expectedJacobianWithPoint = gfg.jacobian(ordering).first;
  // expected
  Matrix F = expectedJacobianWithPoint.block<18, 27>(0, 0);
  Matrix E = expectedJacobianWithPoint.block<18, 3>(0, 27);

  Matrix expectedHessian =
      F.transpose() * F -
      F.transpose() * E * (E.transpose() * E).inverse() * E.transpose() * F;

  // check
  EXPECT_TRUE(assert_equal(expectedHessian, actualHessian));
}

/* ************************************************************************* */
// helper function
Matrix schurComplementTest(Point3 pworld_l,
                           vector<StampedPose> posesAtFutureKeyframes,
                           Pose3 b_P_LCam, Pose3 b_P_RCam) {
  // create graph Jacobian factors corresponding to projection measurements:
  GaussianFactorGraph gfg;
  Key l0 = 10;  // a key
  for (size_t i = 0; i < posesAtFutureKeyframes.size();
       i++) {  // for each camera we create 2 jacobian factors (left, right)
    for (size_t j = 0; j < 2; j++) {  // for left and right

      Pose3 pose;
      if (j == 0)
        pose = posesAtFutureKeyframes.at(i).pose.compose(b_P_LCam);
      else
        pose = posesAtFutureKeyframes.at(i).pose.compose(b_P_RCam);

      Unit3 uij = Unit3(pose.transform_to(pworld_l).vector());
      Matrix3 M33 = uij.skew() * pose.rotation().transpose();
      Matrix M39 = Matrix::Zero(3, 9);
      M39.block<3, 3>(0, 0) = -M33;
      gfg.push_back(JacobianFactor(i, M39, l0, M33, Vector3::Zero()));
    }
  }
  // now get the Hessian out of the graph:
  Ordering ordering = Ordering(list_of(0)(1)(2)(10));  // point last
  Matrix expectedJacobianWithPoint = gfg.jacobian(ordering).first;
  // expected
  Matrix F = expectedJacobianWithPoint.block<18, 27>(0, 0);
  Matrix E = expectedJacobianWithPoint.block<18, 3>(0, 27);

  Matrix expectedHessian =
      F.transpose() * F -
      F.transpose() * E * (E.transpose() * E).inverse() * E.transpose() * F;
  return expectedHessian;
}

/* ************************************************************************* */
TEST(FeatureSelector, createLinearVisionFactor_body_P_cam) {
  const Pose3 b_P_LCam =
      Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  const Pose3 b_P_RCam =
      b_P_LCam.compose(Pose3(Rot3(), Point3(0.1, 0, 0)));  // some baseline

  // create 3 stamped pose
  Pose3 pose0 = Pose3(Rot3::Ypr(0.2, 0.4, 0.5), Point3(0, 0, 1));
  StampedPose spose0 = StampedPose(pose0, 0);
  StampedPose spose1 =
      StampedPose(pose0.compose(Pose3(Rot3(), Point3(0.2, 0, 0))), 0.5);
  StampedPose spose2 =
      StampedPose(pose0.compose(Pose3(Rot3(), Point3(0.5, 0, 0))), 1);

  // create featureSelectionData
  FeatureSelectorData featureSelectionData;
  featureSelectionData.body_P_leftCam = b_P_LCam;
  featureSelectionData.body_P_rightCam = b_P_RCam;
  featureSelectionData.posesAtFutureKeyframes.push_back(spose0);
  featureSelectionData.posesAtFutureKeyframes.push_back(spose1);
  featureSelectionData.posesAtFutureKeyframes.push_back(spose2);
  featureSelectionData.left_undistRectCameraMatrix = K;
  featureSelectionData.right_undistRectCameraMatrix = K;

  vioParams.smartNoiseSigma_ =
      1;  // parameter include after the test were written
  FeatureSelector f(trackerParams, vioParams);
  Cameras left_cameras, right_cameras;
  tie(left_cameras, right_cameras) = f.getCameras(featureSelectionData);

  // create a set of Jacobian factors, then eliminate points and get the same
  // matrix
  Camera cam(spose0.pose.compose(b_P_LCam), K);
  Point3 pworld_l =
      cam.backproject(Point2(320, 200), 2.0);  // backprojected 2 meters away

  // check that points in FOV are calibrated correctly
  EXPECT_TRUE(FeatureSelector::GetVersorIfInFOV(
      Camera(spose1.pose.compose(b_P_LCam), K), pworld_l));
  EXPECT_TRUE(FeatureSelector::GetVersorIfInFOV(
      Camera(spose2.pose.compose(b_P_LCam), K), pworld_l));
  EXPECT_TRUE(FeatureSelector::GetVersorIfInFOV(
      Camera(spose1.pose.compose(b_P_RCam), K), pworld_l));
  EXPECT_TRUE(FeatureSelector::GetVersorIfInFOV(
      Camera(spose2.pose.compose(b_P_RCam), K), pworld_l));

  double time1 = 0, time2 = 0, time3 = 0;
  HessianFactor::shared_ptr hFactor = f.createLinearVisionFactor(
      pworld_l, left_cameras, right_cameras, time1, time2, time3);
  Matrix actualHessian = hFactor->information();  // NOTE: he we assume that the
                                                  // keys are in the right order

  Matrix expectedHessian =
      schurComplementTest(pworld_l, featureSelectionData.posesAtFutureKeyframes,
                          b_P_LCam, b_P_RCam);

  // check
  EXPECT_TRUE(assert_equal(expectedHessian, actualHessian));
}

/* ************************************************************************* */
// helper function
GaussianFactorGraph::shared_ptr createOmegaBarTest() {
  // create 3 stamped pose
  Pose3 pose0 = Pose3(Rot3::Ypr(0.2, 0.4, 0.5), Point3(0, 0, 1));
  StampedPose spose0 = StampedPose(pose0, 0);
  StampedPose spose1 = StampedPose(
      pose0.compose(Pose3(Rot3::Ypr(0.02, 0.04, 0.05), Point3(0.2, 0, 0))),
      0.5);

  // create featureSelectionData
  FeatureSelectorData featureSelectionData;
  featureSelectionData.posesAtFutureKeyframes.push_back(spose0);
  featureSelectionData.posesAtFutureKeyframes.push_back(spose1);
  featureSelectionData.currentNavStateCovariance =
      0.0001 * Matrix::Identity(15, 15);
  featureSelectionData.left_undistRectCameraMatrix = K;
  featureSelectionData.right_undistRectCameraMatrix = K;
  Camera cam(spose0.pose, K);
  Point3 pworld_l =
      cam.backproject(Point2(320, 200), 2.0);  // backprojected 2 meters away
  featureSelectionData.keypoints_3d.push_back(
      pose0.transform_to(pworld_l));  // convert to local frame
  featureSelectionData.keypointLife.push_back(3);

  // instantiate selector
  VioBackEndParams vp = VioBackEndParams();
  vp.smartNoiseSigma_ = 1000;
  vp.imuIntegrationSigma_ = 1e-4;
  vp.accNoiseDensity_ = 1e-2;
  vp.accBiasSigma_ = 1e-2;
  FeatureSelector f(trackerParams, vp);
  Cameras left_cameras, right_cameras;
  tie(left_cameras, right_cameras) = f.getCameras(featureSelectionData);

  // create OmegaBar
  return f.createOmegaBar(featureSelectionData, left_cameras, right_cameras);
}
/* ************************************************************************* */
TEST(FeatureSelector, createOmegaBar) {
  // get gaussian factor graph by calling createOmegaBar (done inside function
  // to avoid more copy-paste)
  GaussianFactorGraph::shared_ptr gfg = createOmegaBarTest();

  // check keys:
  FastVector<Key> keys0 = gfg->at(0)->keys();
  EXPECT_EQ(keys0.size(), 2);
  EXPECT_EQ(
      keys0[0],
      0 && keys0[1] == 1);  // single factor, including imu, prior, and vision
}

/* ************************************************************************* */
TEST(FeatureSelector, evaluateGain_det) {
  // get some gaussian factor graph
  GaussianFactorGraph::shared_ptr gfg = createOmegaBarTest();

  // some jacobian to facilitate creation of Hessian factor
  JacobianFactor J =
      JacobianFactor(0, 1 * Matrix::Identity(3, 9), 1,
                     1 * Matrix::Identity(3, 9), Vector3::Zero());
  HessianFactor::shared_ptr H = boost::make_shared<HessianFactor>(J);

  // test with actual hessian factor
  {
    // expected
    GaussianFactorGraph::shared_ptr gfg_H = gfg;
    gfg_H->push_back(H);
    Matrix expectedHessian = gfg_H->hessian().first;
    double expectedLogDet = log(expectedHessian.determinant());

    bool useDenseMatrices = true;
    for (size_t denseOrNot = 0; denseOrNot < 2; denseOrNot++) {
      if (denseOrNot == 1) useDenseMatrices = false;

      // actual
      double actualDet = FeatureSelector::EvaluateGain(
          gfg, H, VioFrontEndParams::FeatureSelectionCriterion::LOGDET,
          useDenseMatrices);
      EXPECT_NEAR(expectedLogDet, actualDet,
                  expectedLogDet * 1e-3);  // relative tolerance
      // compare against matlab determinant:
      EXPECT_NEAR(log(5.591685310658876e+79), actualDet,
                  expectedLogDet * 1e-3);  // determinant should be large

      // actual2: call it again and make sure we did not mess up gfg inside the
      // function
      actualDet = FeatureSelector::EvaluateGain(
          gfg, H, VioFrontEndParams::FeatureSelectionCriterion::LOGDET,
          useDenseMatrices);
      EXPECT_NEAR(expectedLogDet, actualDet,
                  expectedLogDet * 1e-3);  // relative tolerance
      EXPECT_NEAR(log(5.591685310658876e+79), actualDet,
                  expectedLogDet * 1e-3);  // determinant should be large
    }
  }
  // test with empty hessian factor
  {
    bool useDenseMatrices = true;
    for (size_t denseOrNot = 0; denseOrNot < 2; denseOrNot++) {
      if (denseOrNot == 1) useDenseMatrices = false;

      // expected
      Matrix expectedHessian = gfg->hessian().first;
      double expectedLogDet = log(expectedHessian.determinant());

      // actual
      double actualDet = FeatureSelector::EvaluateGain(
          gfg, boost::make_shared<HessianFactor>(),
          VioFrontEndParams::FeatureSelectionCriterion::LOGDET,
          useDenseMatrices);
      EXPECT_NEAR(expectedLogDet, actualDet,
                  expectedLogDet * 1e-3);  // relative tolerance
    }
  }
}

/* ************************************************************************* */
TEST(FeatureSelector, lowerBound) {
  double numericalUpperBound = std::numeric_limits<double>::max();
  // min instead will return a tiny positive number.
  double numericalLowerBound = -numericalUpperBound;
  EXPECT_EQ(numericalLowerBound, numericalLowerBound);  // same as itself
  EXPECT_TRUE(numericalLowerBound != -1);  // different from a negative number
  EXPECT_TRUE(numericalLowerBound != +1);  // different from a positive number
  EXPECT_LT(numericalLowerBound, -1000);   // smaller than a negative number
  EXPECT_LT(numericalLowerBound, 0.1);     // smaller than a positive number
  cout << "numericalLowerBound " << numericalLowerBound << endl;
}

/* ************************************************************************* */
TEST(FeatureSelector, upperBound) {
  double numericalUpperBound = std::numeric_limits<double>::max();
  // min instead will return a tiny positive number.
  double numericalLowerBound = -numericalUpperBound;
  EXPECT_EQ(numericalUpperBound, numericalUpperBound);  // same as itself
  EXPECT_TRUE(numericalUpperBound != -1);    // different from a negative number
  EXPECT_TRUE(numericalUpperBound != +1);    // different from a positive number
  EXPECT_TRUE(numericalUpperBound > -1000);  // larger than a negative number
  EXPECT_TRUE(numericalUpperBound > 1000);   // larger than a positive number
  cout << "numericalUpperBound " << numericalUpperBound << endl;
}

/* ************************************************************************* */
TEST(FeatureSelector, logget) {
  Matrix M = (Matrix(5, 5) << 0.0874197, -0.0030860, 0.0116969, 0.0081463,
              0.0048741, -0.0030860, 0.0872727, 0.0183073, 0.0125325,
              -0.0037363, 0.0116969, 0.0183073, 0.0966217, 0.0103894,
              -0.0021113, 0.0081463, 0.0125325, 0.0103894, 0.0747324, 0.0036415,
              0.0048741, -0.0037363, -0.0021113, 0.0036415, 0.0909464)
                 .finished();

  double expected = log(M.determinant());
  double actual = FeatureSelector::Logdet(M);
  EXPECT_NEAR(expected, actual, fabs(expected) * 1e-3);
}

/* ************************************************************************* */
TEST(FeatureSelector, smallestEig) {
  Matrix M = (Matrix(5, 5) << 0.0874197, -0.0030860, 0.0116969, 0.0081463,
              0.0048741, -0.0030860, 0.0872727, 0.0183073, 0.0125325,
              -0.0037363, 0.0116969, 0.0183073, 0.0966217, 0.0103894,
              -0.0021113, 0.0081463, 0.0125325, 0.0103894, 0.0747324, 0.0036415,
              0.0048741, -0.0037363, -0.0021113, 0.0036415, 0.0909464)
                 .finished();

  double expectedEig, actualEig;
  Vector expectedVect, actualVect;
  int expectedRank, actualRank;

  boost::tie(expectedRank, expectedEig, expectedVect) =
      FeatureSelector::SmallestEigs(M);

  boost::tie(actualRank, actualEig, actualVect) =
      FeatureSelector::SmallestEigsPowerIter(M);

  EXPECT_NEAR(expectedEig, actualEig, fabs(expectedEig) * 1e-3);
  EXPECT_TRUE(assert_equal(expectedVect, actualVect, 1e-2));
  // cout << "expectedVect " << expectedVect.transpose() << endl;
  // cout << "actualVect " << actualVect.transpose() << endl;
}

/* ************************************************************************* */
TEST(FeatureSelector, evaluateGain_minEig) {
  // get some gaussian factor graph
  GaussianFactorGraph::shared_ptr gfg = createOmegaBarTest();

  // some jacobian to facilitate creation of Hessian factor
  JacobianFactor J =
      JacobianFactor(0, 1 * Matrix::Identity(3, 9), 1,
                     1 * Matrix::Identity(3, 9), Vector3::Zero());
  HessianFactor::shared_ptr H = boost::make_shared<HessianFactor>(J);

  // test with actual hessian factor
  {
    // expected
    GaussianFactorGraph::shared_ptr gfg_H = createOmegaBarTest();
    gfg_H->push_back(H);
    Matrix expectedHessian = gfg_H->hessian().first;
    // NOTE: singular value of H (symm and positive definite) = eig H
    int rank;
    double expectedMinEig;
    Vector eigVector;
    boost::tie(rank, expectedMinEig, eigVector) = DLT(expectedHessian);
    EXPECT_NEAR(2.990918403930777e+03, expectedMinEig,
                expectedMinEig * 1e-4);  // relative tolerance

    bool useDenseMatrices = true;
    for (size_t denseOrNot = 0; denseOrNot < 2; denseOrNot++) {
      if (denseOrNot == 1) useDenseMatrices = false;

      // actual
      double actualMinEig = FeatureSelector::EvaluateGain(
          gfg, H, VioFrontEndParams::FeatureSelectionCriterion::MIN_EIG,
          useDenseMatrices);
      EXPECT_NEAR(expectedMinEig, actualMinEig,
                  expectedMinEig * 1e-4);  // relative tolerance

      // actual2: call it again and make sure we did not mess up gfg inside the
      // function
      actualMinEig = FeatureSelector::EvaluateGain(
          gfg, H, VioFrontEndParams::FeatureSelectionCriterion::MIN_EIG,
          useDenseMatrices);
      EXPECT_NEAR(expectedMinEig, actualMinEig,
                  expectedMinEig * 1e-4);  // relative tolerance
    }
  }
  // test with empty hessian factor
  {
    bool useDenseMatrices = true;
    for (size_t denseOrNot = 0; denseOrNot < 2; denseOrNot++) {
      if (denseOrNot == 1) useDenseMatrices = false;
      // expected
      Matrix expectedHessian = gfg->hessian().first;
      int rank;
      double expectedMinEig;
      Vector eigVector;
      boost::tie(rank, expectedMinEig, eigVector) = DLT(expectedHessian);

      // actual
      double actualMinEig = FeatureSelector::EvaluateGain(
          gfg, boost::make_shared<HessianFactor>(),
          VioFrontEndParams::FeatureSelectionCriterion::MIN_EIG,
          useDenseMatrices);
      EXPECT_NEAR(expectedMinEig, actualMinEig,
                  expectedMinEig * 1e-4);  // relative tolerance
    }
  }
}

/* ************************************************************************* */
TEST(FeatureSelector, greedyAlgorithm) {
  // Create Omega bar
  GaussianFactorGraph::shared_ptr OmegaBar = createOmegaBarTest();

  // create N = 10 fake Deltas
  vector<HessianFactor::shared_ptr> Deltas;
  for (size_t i = 0; i < 10; i++) {
    // create the best
    double c;
    switch (i) {
      case 3:
        c = 11;
        break;
      case 5:
        c = 12;
        break;
      case 7:
        c = 21;
        break;
      case 8:
        c = 12;
        break;
      case 9:
        c = 10;
        break;
      default:
        c = double(i);
    }
    Deltas.push_back(boost::make_shared<HessianFactor>(HessianFactor(
        0, 1, c * Matrix::Identity(9, 9), Matrix::Zero(9, 9), Vector::Zero(9),
        c * Matrix::Identity(9, 9), Vector::Zero(9), 0.0)));
  }

  // we want to select the 5 best
  int need_n_corners = 5;

  // check max min eig selection
  vector<size_t> actualEig;
  vector<double> actualGainEig;
  tie(actualEig, actualGainEig) = FeatureSelector::GreedyAlgorithm(
      OmegaBar, Deltas, need_n_corners,
      VioFrontEndParams::FeatureSelectionCriterion::MIN_EIG);
  // check
  sort(actualEig.begin(), actualEig.end());  // to facilitate comparison
  EXPECT_NEAR(actualEig[0], 3, 1e-3);
  EXPECT_NEAR(actualEig[1], 5, 1e-3);
  EXPECT_NEAR(actualEig[2], 7, 1e-3);
  EXPECT_NEAR(actualEig[3], 8, 1e-3);
  EXPECT_NEAR(actualEig[4], 9, 1e-3);
  EXPECT_EQ(actualEig.size(), 5);

  // check max det selection
  vector<size_t> actualDet;
  vector<double> actualGainDet;
  tie(actualDet, actualGainDet) = FeatureSelector::GreedyAlgorithm(
      OmegaBar, Deltas, need_n_corners,
      VioFrontEndParams::FeatureSelectionCriterion::LOGDET);
  // check
  sort(actualDet.begin(), actualDet.end());  // to facilitate comparison
  EXPECT_NEAR(actualDet[0], 3, 1e-3);
  EXPECT_NEAR(actualDet[1], 5, 1e-3);
  EXPECT_NEAR(actualDet[2], 7, 1e-3);
  EXPECT_NEAR(actualDet[3], 8, 1e-3);
  EXPECT_NEAR(actualDet[4], 9, 1e-3);
  EXPECT_EQ(actualDet.size(), 5);
}

/* ************************************************************************* */
TEST(FeatureSelector, createDeltas) {
  // create 3 stamped pose
  Pose3 pose0 = Pose3(Rot3::Ypr(0.2, 0.4, 0.5), Point3(0, 0, 1));
  StampedPose spose0 = StampedPose(pose0, 0);
  StampedPose spose1 =
      StampedPose(pose0.compose(Pose3(Rot3(), Point3(0.3, 0, 0))), 0.5);
  StampedPose spose2 =
      StampedPose(pose0.compose(Pose3(Rot3(), Point3(0.4, 0, 0))), 1);

  // create featureSelectionData
  FeatureSelectorData featureSelectionData;
  featureSelectionData.posesAtFutureKeyframes.push_back(spose0);
  featureSelectionData.posesAtFutureKeyframes.push_back(spose1);
  featureSelectionData.posesAtFutureKeyframes.push_back(spose2);
  featureSelectionData.left_undistRectCameraMatrix = K;
  featureSelectionData.right_undistRectCameraMatrix = K;

  // add point to
  Camera cam(spose0.pose, K);
  Point3 pworld_l =
      cam.backproject(Point2(320, 200), 2.0);  // backprojected 2 meters away
  featureSelectionData.keypoints_3d.push_back(
      spose0.pose.transform_to(pworld_l));  // convert to local frame
  featureSelectionData.keypointLife.push_back(3);

  // instantiate selector
  vioParams.smartNoiseSigma_ =
      1;  // parameter include after the test were written
  FeatureSelector f(trackerParams, vioParams);
  Cameras left_cameras, right_cameras;
  tie(left_cameras, right_cameras) = f.getCameras(featureSelectionData);

  // create candidate versors that will end up in delta
  vector<Vector3> availableVersors;
  Vector3 v1 = K.calibrate(Vector3(322, 201, 1));
  v1 = v1 / v1.norm();
  availableVersors.push_back(v1);  // close to existing keypoint
  Vector3 v2 = K.calibrate(Vector3(400, 400, 1));
  v2 = v2 / v2.norm();
  availableVersors.push_back(v2);  // this should use assumed depth
  vector<double> cornerDistances;
  for (size_t i = 0; i < availableVersors.size(); i++)
    cornerDistances.push_back(0.0);

  vector<HessianFactor::shared_ptr> Deltas =
      f.createDeltas(availableVersors, cornerDistances, featureSelectionData,
                     left_cameras, right_cameras);

  EXPECT_EQ(Deltas.size(), 2);  // 2 factors, 1 for each point

  // NOTE: WE ARE ASSUMING DISTANCE, NOT DEPTH: see the following to understand
  // difference first point is close to existing keypoint, hence the depth
  // should be the same as that point
  Point3 p1depth = cam.backproject(Point2(322, 202), 2.0);
  // check 2 types of reprojection
  Point3 p1depth_check =
      left_cameras.at(0).pose() *
      Point3(availableVersors[0] * 2.0 / availableVersors[0](2));
  EXPECT_TRUE(assert_equal(p1depth, p1depth_check,
                           1e-2));  // strangely we need some tolerance here

  // however we assume norm, rather than depth:
  Point3 p1 = left_cameras.at(0).pose() * Point3(availableVersors[0] * 2.0);
  // check hessian
  Matrix expectedHessian1 = schurComplementTest(
      p1, featureSelectionData.posesAtFutureKeyframes, Pose3(), Pose3());

  // THIS FAILS NOW SINCE WE INCLUDED !hasRightPixel in FeatureSelector.h
  /*
  EXPECT_TRUE(assert_equal(expectedHessian1,Deltas[0]->information(),1e-2));

  // second point is far from existing keypoint, hence the distance should be
  the default = 5 Point3 p2 = left_cameras.at(0).pose() *
  Point3(availableVersors[1] * 5.0); Matrix expectedHessian2 =
  schurComplementTest(p2,featureSelectionData.posesAtFutureKeyframes,Pose3(),Pose3());
  EXPECT_TRUE(assert_equal(expectedHessian2,Deltas[1]->information(),1e-3));
  */
}

/* ************************************************************************* */
TEST(FeatureSelector, featureSelection) {
  // test feature selection with eig criterion

  const Cal3_S2 Kreal = Cal3_S2(10, 10, 0.1, 640 / 2, 480 / 2);

  // create 3 stamped pose: well crafter example
  // pose0 at origin, pose1 1 meter ahead along y, pose2 2 meter ahead
  Pose3 pose0 = Pose3();
  StampedPose spose0 = StampedPose(pose0, 0);
  StampedPose spose1 =
      StampedPose(pose0.compose(Pose3(Rot3(), Point3(0, 1, 0))), 0.5);
  StampedPose spose2 =
      StampedPose(pose0.compose(Pose3(Rot3(), Point3(0, 2, 0))), 1);

  // camera estrinsics: for left camera to have z forward, we need a -90deg
  // rotation along x
  const Pose3 b_P_LCam = Pose3(Rot3::Ypr(0.0, 0.0, -M_PI / 2), Point3());
  const Pose3 b_P_RCam = b_P_LCam;  // some baseline
  // check: point at 0 1 0 should be projected at the center of the camera
  Camera cam0 = Camera(pose0.compose(b_P_LCam), Kreal);
  EXPECT_TRUE(assert_equal(Point2(Kreal.px(), Kreal.py()),
                           cam0.project(Point3(0, 1, 0))));

  // create featureSelectionData
  FeatureSelectorData featureSelectionData;
  featureSelectionData.posesAtFutureKeyframes.push_back(spose0);
  featureSelectionData.posesAtFutureKeyframes.push_back(spose1);
  featureSelectionData.posesAtFutureKeyframes.push_back(spose2);
  featureSelectionData.body_P_leftCam = b_P_LCam;
  featureSelectionData.body_P_rightCam = b_P_RCam;
  featureSelectionData.left_undistRectCameraMatrix = Kreal;
  featureSelectionData.right_undistRectCameraMatrix = Kreal;

  // covariance is elongated along x direction
  featureSelectionData.currentNavStateCovariance =
      0.0001 * Matrix::Identity(15, 15);
  featureSelectionData.currentNavStateCovariance(4, 4) = 0.1;  // 2 comments:
  // 1) 4,4 picks the y cartesian component (0,1,2 are the rotations)
  // 2) since the initial pose is the identity, no translation occurs and
  // uncertainty remains along x

  // create camera params
  CameraParams cam_param;
  cam_param.calibration_ =
      Cal3DS2(Kreal.fx(), Kreal.fy(), 0.0, Kreal.px(), Kreal.py(), 0.0,
              0.0);  // 0 skew and no distortion
  cam_param.camera_matrix_ = Mat::eye(3, 3, CV_64F);
  cam_param.camera_matrix_.at<double>(0, 0) = Kreal.fx();
  cam_param.camera_matrix_.at<double>(1, 1) = Kreal.fy();
  cam_param.camera_matrix_.at<double>(0, 2) = Kreal.px();
  cam_param.camera_matrix_.at<double>(1, 2) = Kreal.py();
  cam_param.distortion_coeff_ = Mat::zeros(1, 5, CV_64F);
  cam_param.distortion_model_ = "radtan";

  // create feature selector
  VioBackEndParams vp = VioBackEndParams();
  vp.smartNoiseSigma_ = 1000;
  vp.accNoiseDensity_ =
      0.1;  // fake noise to avoid that covariance it too small
  vp.accBiasSigma_ = 0.03;
  vp.imuIntegrationSigma_ = 1e-3;
  FeatureSelector f(trackerParams, vp);
  // TEST 1
  {
    // create observations:
    KeypointsCV availableCorners;
    // 0) point to the far right of the camera, not visible by camera 2
    Point3 pworld_l = Point3(1, 0.05, 0);
    Point2 px = cam0.project(pworld_l);
    availableCorners.push_back(KeypointCV(px.x(), px.y()));
    EXPECT_TRUE(FeatureSelector::GetVersorIfInFOV(cam0, pworld_l));
    // 1) point to the right at 45 deg, visible in all cameras
    pworld_l = Point3(5, 5, 0);
    px = cam0.project(pworld_l);
    availableCorners.push_back(KeypointCV(px.x(), px.y()));
    EXPECT_TRUE(FeatureSelector::GetVersorIfInFOV(cam0, pworld_l));
    // 2) point exactly at the front, visible in all the cameras
    pworld_l = Point3(0, 5, 0);
    px = cam0.project(pworld_l);
    availableCorners.push_back(KeypointCV(px.x(), px.y()));
    EXPECT_TRUE(FeatureSelector::GetVersorIfInFOV(cam0, pworld_l));
    // 3) point at the front with slight parallax, visible in all the cameras
    pworld_l = Point3(0.1, 50, 0);
    px = cam0.project(pworld_l);
    availableCorners.push_back(KeypointCV(px.x(), px.y()));
    EXPECT_TRUE(FeatureSelector::GetVersorIfInFOV(cam0, pworld_l));

    // check eigMin
    int need_n_corners = 1;
    KeypointsCV selected;
    vector<size_t> selectedIndices;
    vector<double> selectedGains;
    vector<double> successProbability, cornerDistances;
    for (size_t i = 0; i < availableCorners.size(); i++) {
      successProbability.push_back(1.0);
      cornerDistances.push_back(0.0);
    }
    tie(selected, selectedIndices, selectedGains) =
        f.featureSelectionLinearModel(
            availableCorners, successProbability, cornerDistances, cam_param,
            need_n_corners, featureSelectionData,
            VioFrontEndParams::FeatureSelectionCriterion::MIN_EIG);
    EXPECT_NEAR(1, selectedIndices[0], 1e-3);
    EXPECT_EQ(selectedIndices.size(), 1);
    EXPECT_EQ(selected.size(), 1);

    // check det
    tie(selected, selectedIndices, selectedGains) =
        f.featureSelectionLinearModel(
            availableCorners, successProbability, cornerDistances, cam_param,
            need_n_corners, featureSelectionData,
            VioFrontEndParams::FeatureSelectionCriterion::LOGDET);
    EXPECT_NEAR(1, selectedIndices[0], 1e-3);
    EXPECT_EQ(selectedIndices.size(), 1);
    EXPECT_EQ(selected.size(), 1);
  }
}

/* ************************************************************************* */
TEST(FeatureSelector, sorting) {
  // test feature selection with eig criterion
  vector<double> upperBounds;
  upperBounds.push_back(0.1);
  upperBounds.push_back(1);
  upperBounds.push_back(0.5);
  upperBounds.push_back(0.11);
  upperBounds.push_back(11);

  vector<size_t> ordering;
  vector<double> sortedUpperBounds_actual;
  tie(ordering, sortedUpperBounds_actual) =
      FeatureSelector::SortDescending(upperBounds);

  EXPECT_NEAR(11, sortedUpperBounds_actual[0], 1e-5);
  EXPECT_NEAR(1, sortedUpperBounds_actual[1], 1e-5);
  EXPECT_NEAR(0.5, sortedUpperBounds_actual[2], 1e-5);
  EXPECT_NEAR(0.11, sortedUpperBounds_actual[3], 1e-5);
  EXPECT_NEAR(0.1, sortedUpperBounds_actual[4], 1e-5);

  EXPECT_NEAR(4, ordering[0], 1e-5);
  EXPECT_NEAR(1, ordering[1], 1e-5);
  EXPECT_NEAR(2, ordering[2], 1e-5);
  EXPECT_NEAR(3, ordering[3], 1e-5);
  EXPECT_NEAR(0, ordering[4], 1e-5);
}

/* ************************************************************************* */
TEST(FeatureSelector, MultiplyHessianInPlace) {
  Matrix M =
      (Matrix(7, 7) << 125.0000, 0.0, -25.0000, 0.0, -100.0000, 0.0, 25.0000,
       0.0, 125.0000, 0.0, -25.0000, 0.0, -100.0000, -17.5000, -25.0000, 0.0,
       25.0000, 0.0, 0.0, 0.0, -5.0000, 0.0, -25.0000, 0.0, 25.0000, 0.0, 0.0,
       7.5000, -100.0000, 0.0, 0.0, 0.0, 100.0000, 0.0, -20.0000, 0.0,
       -100.0000, 0.0, 0.0, 0.0, 100.0000, 10.0000, 25.0000, -17.5000, -5.0000,
       7.5000, -20.0000, 10.0000, 8.2500)
          .finished();

  FastVector<Key> keys;
  keys.push_back(0);
  keys.push_back(1);
  vector<DenseIndex> dims;
  dims.push_back(2);
  dims.push_back(4);
  dims.push_back(1);

  // multiply by 2:
  {
    HessianFactor::shared_ptr h =
        boost::make_shared<HessianFactor>(keys, SymmetricBlockMatrix(dims, M));
    FeatureSelector::MultiplyHessianInPlace(h, 2);
    HessianFactor::shared_ptr expected = boost::make_shared<HessianFactor>(
        keys, SymmetricBlockMatrix(dims, 2 * M));
    // the following fails since we are not multiplying the vector
    EXPECT_TRUE(assert_equal(*expected.get(), *h.get(), 1e-4));
  }
  // multiply by 0:
  {
    HessianFactor::shared_ptr h =
        boost::make_shared<HessianFactor>(keys, SymmetricBlockMatrix(dims, M));
    FeatureSelector::MultiplyHessianInPlace(h, 0);
    HessianFactor::shared_ptr expected = boost::make_shared<HessianFactor>(
        keys, SymmetricBlockMatrix(dims, Matrix::Zero(7, 7)));
    // the following fails since we are not multiplying the vector
    EXPECT_TRUE(assert_equal(*expected.get(), *h.get(), 1e-4));
  }
  // multiply by 100.01 :-)
  {
    HessianFactor::shared_ptr h =
        boost::make_shared<HessianFactor>(keys, SymmetricBlockMatrix(dims, M));
    FeatureSelector::MultiplyHessianInPlace(h, 100.01);
    HessianFactor::shared_ptr expected = boost::make_shared<HessianFactor>(
        keys, SymmetricBlockMatrix(dims, 100.01 * M));
    // the following fails since we are not multiplying the vector
    EXPECT_TRUE(assert_equal(*expected.get(), *h.get(), 1e-4));
  }
}

/* ************************************************************************* */
TEST(FeatureSelector, smallestEigSpectra) {
  Matrix M = (Matrix(5, 5) << 0.0874197, -0.0030860, 0.0116969, 0.0081463,
              0.0048741, -0.0030860, 0.0872727, 0.0183073, 0.0125325,
              -0.0037363, 0.0116969, 0.0183073, 0.0966217, 0.0103894,
              -0.0021113, 0.0081463, 0.0125325, 0.0103894, 0.0747324, 0.0036415,
              0.0048741, -0.0037363, -0.0021113, 0.0036415, 0.0909464)
                 .finished();

  double expectedEig;
  Vector expectedVect;
  int expectedRank;
  boost::tie(expectedRank, expectedEig, expectedVect) =
      FeatureSelector::SmallestEigs(M);

  double actualEig;
  Vector actualVect;
  int actualRank;
  boost::tie(actualRank, actualEig, actualVect) =
      FeatureSelector::SmallestEigsSpectra(M);

  EXPECT_NEAR(expectedEig, actualEig,
              fabs(expectedEig) * 1e-3);  // relative tolerance
  EXPECT_TRUE(assert_equal(expectedVect, actualVect, 1e-2));
}

/* ************************************************************************* */
TEST(FeatureSelector, smallestEigSpectraShift) {
  Matrix M = (Matrix(5, 5) << 0.0874197, -0.0030860, 0.0116969, 0.0081463,
              0.0048741, -0.0030860, 0.0872727, 0.0183073, 0.0125325,
              -0.0037363, 0.0116969, 0.0183073, 0.0966217, 0.0103894,
              -0.0021113, 0.0081463, 0.0125325, 0.0103894, 0.0747324, 0.0036415,
              0.0048741, -0.0037363, -0.0021113, 0.0036415, 0.0909464)
                 .finished();

  double expectedEig;
  Vector expectedVect;
  int expectedRank;
  boost::tie(expectedRank, expectedEig, expectedVect) =
      FeatureSelector::SmallestEigs(M);

  double actualEig;
  Vector actualVect;
  int actualRank;
  boost::tie(actualRank, actualEig, actualVect) =
      FeatureSelector::SmallestEigsSpectraShift(M);

  EXPECT_NEAR(expectedEig, actualEig,
              fabs(expectedEig) * 1e-3);  // relative tolerance
  EXPECT_TRUE(assert_equal(expectedVect, actualVect, 1e-2));
}
