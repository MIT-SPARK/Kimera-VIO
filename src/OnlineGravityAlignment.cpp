/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OnlineGravityAlignment.cpp
 * @brief  Contains initial Online Gravity Alignment functions.
 * @author Sandro Berchier
 * @author Luca Carlone
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vector>

#include "utils/Timer.h"

#include "OnlineGravityAlignment.h"

namespace VIO {

// TODO(Sandro): Retract velocities and poses from last optimization
// TODO(Sandro): Check conventions
// TODO(Sandro): Why are we not iterating also the tangent basis?
// TODO(Sandro): Either retrieve all velocities and poses to insert
  // in the backend optimization or retrieve last pose and velocity
  // so that there is no lag for the backend optimization

/* -------------------------------------------------------------------------- */
// Constructor for visual inertial alignment.
// [in] estimated camera poses in initial body frame.
// [in] delta timestamps of camera frames.
// [in] pre-integrations performed in visual frontend.
// [in] global gravity vector in world frame.
OnlineGravityAlignment::OnlineGravityAlignment(
    const AlignmentPoses &estimated_body_poses,
    const std::vector<double> &delta_t_camera, const AlignmentPims &pims,
    const gtsam::Vector3 &g_world)
    : estimated_body_poses_(estimated_body_poses),
      delta_t_camera_(delta_t_camera), pims_(pims), g_world_(g_world) {}

/* -------------------------------------------------------------------------- */
// Performs visual-inertial alignment and gravity estimate.
// [out] initial gyro bias estimate.
// [out] estimate of gravity vector in initial body pose (g_b0).
// [out] initial nav state for initialization.
bool OnlineGravityAlignment::alignVisualInertialEstimates(
    gtsam::Vector3 *gyro_bias, gtsam::Vector3 *g_iter,
    gtsam::NavState *init_navstate) {
  VLOG(10) << "Online gravity alignment called.";
  VisualInertialFrames vi_frames;
  gtsam::Velocity3 init_velocity;
  CHECK_DOUBLE_EQ(gyro_bias->norm(), 0.0);

  // Construct set of frames for linear alignment
  constructVisualInertialFrames(estimated_body_poses_, delta_t_camera_, pims_,
                                &vi_frames);

  // Estimate gyroscope bias
  CHECK(estimateGyroscopeBias(vi_frames, gyro_bias));
  // Update delta states with corrected bias
  updateDeltaStates(pims_, *gyro_bias, &vi_frames);

  CHECK_GT(5e-2, estimateGyroscopeResiduals(vi_frames).norm());

  // Align visual and inertial estimates
  if (alignEstimatesLinearly(vi_frames, g_world_, g_iter, &init_velocity)) {
    // Align gravity vectors and estimate initial pose
    gtsam::Pose3 w0_T_b0 =
        UtilsOpenCV::AlignGravityVectors(*g_iter, g_world_, false);
    // Create initial navstate and rotate velocity in world frame
    *init_navstate = gtsam::NavState(w0_T_b0 * vi_frames.at(0).b0_T_bk(),
                                     w0_T_b0.rotation() * init_velocity);
    // Log and return
    LOG(INFO) << "Online gravity alignment successful with:"
              << "\npose: " << init_navstate->pose()
              << "\nvelocity: " << init_navstate->velocity()
              << "\ngravity: " << *g_iter << "\n";
    return true;
  } else {
    LOG(ERROR) << "Online gravity alignment failed!\n";
    return false;
  }
}

/* -------------------------------------------------------------------------- */
// This function creates the frame objects used in the alignment.
// It populates the camera poses, delta pre-integrations and jacobian.
// [in] vector of estimated camera poses from Bundle-Adjustment.
// [in] vector of delta_t in camera frames.
// [in] vector of pre-integrations from visual-frontend.
// [out] vector of frames used for initial alignment.
void OnlineGravityAlignment::constructVisualInertialFrames(
    const AlignmentPoses &estimated_body_poses,
    const std::vector<double> &delta_t_camera, const AlignmentPims &pims,
    VisualInertialFrames *vi_frames) {
  CHECK_EQ(estimated_body_poses.size()-1, delta_t_camera.size());
  CHECK_EQ(delta_t_camera.size(), pims.size());
  vi_frames->clear();

  // Create initialization frames
  for (int i = 0; i < pims.size(); i++) {
    // Get bk_gamma_bkp1, bk_alpha_bkp1, bk_beta_bkp1
    // pim.deltaXij() corresponds to bodyLkf_X_bodyK_imu
    gtsam::NavState delta_state(pims.at(i).deltaXij());
    // Get delta_time_pim = t_kp1-t_k
    const double delta_t_pim = pims.at(i).deltaTij();
    // Get pre-integration Jacobian wrt. gyro_bias (dPIM = J * dbg)
    gtsam::Matrix dbg_J_dPIM = pims.at(i).preintegrated_H_biasOmega();
    // Get rotation Jacobian wrt. gyro_bias (dR_bkp1 = J * dbg_bkp1)
    gtsam::Matrix3 dbg_Jacobian_dR = gtsam::sub(dbg_J_dPIM, 0, 3, 0, 3);

    CHECK_GT(5e-3, abs(delta_t_pim - delta_t_camera.at(i)));

    // Create frame with b0_T_bkp1, b0_T_bk, dt_bk_cam,
    // dbg_Jacobian_dR_bk, dt_bk_imu
    VisualInertialFrame frame_i(
        estimated_body_poses.at(i + 1), estimated_body_poses.at(i),
        delta_t_camera.at(i), delta_state, dbg_Jacobian_dR, delta_t_pim);
    vi_frames->push_back(frame_i);
  }
}

/* -------------------------------------------------------------------------- */
// Estimate gyroscope bias by minimizing square of rotation error
// between the pre-integrated rotation constraint between frames
// and the estimated rotation between frames from Bundle-Adjustment.
// [in] frames containing pim constraints and body poses.
// [out] new estimated value for gyroscope bias.
// Unit tested.
bool OnlineGravityAlignment::estimateGyroscopeBias(
          const VisualInertialFrames &vi_frames,
          gtsam::Vector3 *gyro_bias) {
  // Create Gaussian Graph with unit noise
  gtsam::GaussianFactorGraph gaussian_graph;
  auto noise = gtsam::noiseModel::Unit::Create(3);

  // Loop through all initialization frame
  for (int i = 0; i < vi_frames.size(); i++) {
    auto frame_i = std::next(vi_frames.begin(), i);

    // Compute rotation error between pre-integrated and visual estimates
    gtsam::Rot3 bkp1_error_bkp1(frame_i->bk_gamma_bkp1().transpose() *
                                frame_i->bk_R_bkp1());
    // Compute rotation error in canonical coordinates (dR_bkp1)
    gtsam::Vector3 dR = gtsam::Rot3::Logmap(bkp1_error_bkp1);
    // Get rotation Jacobian wrt. gyro_bias (dR_bkp1 = J * dbg_bkp1)
    gtsam::Matrix3 dbg_J_dR = frame_i->dbg_jacobian_dR();

    // Insert Jacobian Factor in Gaussian Graph
    gaussian_graph.add(gtsam::Symbol('dbg', 0), dbg_J_dR, dR, noise);
  }

  // Optimize Gaussian Graph and get solution
  gtsam::VectorValues solution = gaussian_graph.optimize();
  gtsam::Vector3 delta_bg  = solution.at(gtsam::Symbol('dbg', 0));

  // Adapt gyroscope bias
  *gyro_bias += delta_bg;

  // Logging of solution
  if (VLOG_IS_ON(5)) { gaussian_graph.print("\nGaussian Factor graph:\n"); }
  LOG(INFO) << "\nGyro bias estimation:\n" << delta_bg;

  // TODO(Sandro): Implement check on quality of estimate
  return true;
}

/* -------------------------------------------------------------------------- */
// Estimate gyroscope bias by minimizing square of rotation error
// between the pre-integrated rotation constraint between frames
// and the estimated rotation between frames from Bundle-Adjustment.
// [in] frames containing pim constraints and body poses.
// [out] new estimated value for gyroscope bias.
// Unit tested.
gtsam::Vector3 OnlineGravityAlignment::estimateGyroscopeResiduals(
          const VisualInertialFrames &vi_frames) {
  gtsam::Vector3 dR(0.0, 0.0, 0.0);
  // Loop through all initialization frame
  for (int i = 0; i < vi_frames.size(); i++) {
    auto frame_i = std::next(vi_frames.begin(), i);
    // Compute rotation error between pre-integrated and visual estimates
    gtsam::Rot3 bkp1_error_bkp1(frame_i->bk_gamma_bkp1().transpose() *
                                frame_i->bk_R_bkp1());
    // Compute rotation error in canonical coordinates (dR_bkp1)
    dR += gtsam::Rot3::Logmap(bkp1_error_bkp1);
  }
  LOG(INFO) << "Residuals after bias correction: \n" << dR;
  return dR;
}

/* -------------------------------------------------------------------------- */
// Corrects pre-integrated delta states with the gyro bias estimate.
// It uses a first-order approximation around the gyro bias estimate.
// [in] vector of pre-integrations from visual-frontend.
// [in] new estimated value for gyroscope bias.
// [out] updated vector of frames used for initial alignment.
void OnlineGravityAlignment::updateDeltaStates(
    const AlignmentPims &pims, const gtsam::Vector3 &gyro_bias,
    VisualInertialFrames *vi_frames) {
  CHECK_EQ(vi_frames->size(), pims.size());

  // Repropagate measurements with first order approximation
  for (int i = 0; i < vi_frames->size(); i++) {
    // Update pre-integration with first-order approximation
    gtsam::Vector9 correct_preintegrated = pims.at(i).biasCorrectedDelta(
        gtsam::imuBias::ConstantBias(Vector3::Zero(), gyro_bias));
    // Retract delta state
    const gtsam::NavState bk_delta_state_bkp1 =
        gtsam::NavState().retract(correct_preintegrated);
    // Update value for delta_state in frames
    vi_frames->at(i).updateDeltaState(bk_delta_state_bkp1);
  }
}

/* -------------------------------------------------------------------------- */
// Align visual-inertial estimates and compute gravity vector
// in initial body frame. The initial body pose is set to identity (by def).
// The gravity estimate returned wrt. to the initial body frame and not pose!
// The subtle difference is important for unit testing (e.g.,
// init_pose!=eye(4)). [in] set of visual inertial frames for alignment. [in]
// global gravity value for alignment. [out] gravity vector expressed in initial
// body frame. [out] initial velocity expressed in initial body frame.
bool OnlineGravityAlignment::alignEstimatesLinearly(
    const VisualInertialFrames &vi_frames, const gtsam::Vector3 &g_world,
    gtsam::Vector3 *g_iter, gtsam::Velocity3 *init_vel) {

  // Create Gaussian Graph with unit noise
  gtsam::GaussianFactorGraph gaussian_graph;
  auto noise = gtsam::noiseModel::Unit::Create(3);

  // Loop through all frames
  for (int i = 0; i < vi_frames.size(); i++) {
    auto frame_i = std::next(vi_frames.begin(), i);

    // Add binary factor for position constraint
    gaussian_graph.add(gtsam::Symbol('b0_V_bk', i), frame_i->A_11(),
                       gtsam::Symbol('g_b0', 0), frame_i->A_13(),
                       frame_i->b_1(), noise);

    // Add ternary factor for velocity constraint
    gaussian_graph.add(gtsam::Symbol('b0_V_bk', i), frame_i->A_21(),
                       gtsam::Symbol('b0_V_bk', i + 1), frame_i->A_22(),
                       gtsam::Symbol('g_b0', 0), frame_i->A_23(),
                       frame_i->b_2(), noise);
  }

  // Optimize Gaussian Graph and get solution
  gtsam::VectorValues solution = gaussian_graph.optimize();
  gtsam::Vector3 g_b0  = solution.at(gtsam::Symbol('g_b0', 0));

  // Logging of solution
  if (VLOG_IS_ON(5)) { gaussian_graph.print("\nGaussian Factor graph:\n"); }

  // Refine gravity alignment if necessary
  // TODO(Sandro): Load tolerance in yaml file
  if (abs(g_b0.norm() - g_world.norm()) > 1e-6) {
    refineGravity(vi_frames, g_world, &g_b0, init_vel);
  } else {
    // Retrieve initial velocity from graph optimization
    *init_vel = solution.at(gtsam::Symbol('b0_V_bk', 0));
  }

  // We want the gravity vector and not the measured acceleration
  // by the IMU, hence multiply estimated value by -1 (the IMU measures
  // the reaction forces of gravity in the accelerometer).
  *g_iter = - g_b0;

  // Logging of solution
  LOG(INFO) << "Final gravity estimate:\n" << *g_iter
            << " with norm: " << g_iter->norm();

  // Return true if successful
  // TODO(Sandro): Load tolerance in yaml file
  if (abs(g_iter->norm() - g_world.norm()) > 1e-5) {
    return false;
  }
  return true;
}

/* -------------------------------------------------------------------------- */
// Creates tangent basis to input vector.
// [in] Vector for which tangent basis is desired.
// [return] Tangent basis spanned by orthogonal basis to input vector.
gtsam::Matrix
OnlineGravityAlignment::createTangentBasis(const gtsam::Vector3 &g0) {

  // Vectors b, c
  gtsam::Vector3 b, c;

  // Set a as g0 normalized
  gtsam::Vector3 a(g0.normalized());

  // Look for b orthogonal to a
  gtsam::Vector3 tmp(0, 0, 1);
  if (a == tmp)
    tmp << (1, 0, 0);
  b = (tmp - a * (a.transpose() * tmp)).normalized();

  // C orthogonal to b and a
  c = a.cross(b);

  // Tangent basis spanned by b, c
  gtsam::Matrix bc(3, 2);
  bc.block<3, 1>(0, 0) = b;
  bc.block<3, 1>(0, 1) = c;

  // Return tangent basis
  return bc;
}

/* -------------------------------------------------------------------------- */
// Refines gravity alignment by imposing norm of gravity vector iteratively.
// [in] set of visual inertial frames for alignment.
// [in] global gravity value for alignment.
// [out] gravity vector expressed in initial body frame.
// [out] initial velocity estimate in initial body frame.
void OnlineGravityAlignment::refineGravity(
    const VisualInertialFrames &vi_frames, const gtsam::Vector3 &g_world,
    gtsam::Vector3 *g_iter, gtsam::Velocity3 *init_vel) {
  // Define current gravity estimate (normalized)
  gtsam::Vector3 g0 = g_iter->normalized() * g_world.norm();

  // Create tangent basis to g (g = g0 + txty*dxdy)
  gtsam::Matrix txty = createTangentBasis(g0);

  // Multiple iterations till convergence
  // TODO(Sandro): Read this parameter in from yaml files.
  int n_iterations = 4;
  for (int l=0; l < n_iterations; l++) {
    // Create Gaussian Graph with unit noise
    gtsam::GaussianFactorGraph gaussian_graph;
    auto noise = gtsam::noiseModel::Unit::Create(3);

    // Loop through all frames
    for (int i = 0; i < vi_frames.size(); i++) {
      auto frame_i = std::next(vi_frames.begin(), i);

      // Apply tangent basis to g (g = g0 + txty*dxdy)
      gtsam::Matrix A_13_tangent = frame_i->A_13()*txty;
      gtsam::Vector3 b_1_tangent = frame_i->b_1() - frame_i->A_13()*g0;

      // Add binary factor for position constraint
      gaussian_graph.add(gtsam::Symbol('b0_V_bk', i), frame_i->A_11(),
                         gtsam::Symbol('dxdy', 0), A_13_tangent, b_1_tangent,
                         noise);

      // Apply tangent basis to g (g = g0 + txty*dxdy)
      gtsam::Matrix A_23_tangent = frame_i->A_23()*txty;
      gtsam::Vector3 b_2_tangent = frame_i->b_2() - frame_i->A_23()*g0;

      // Add ternary factor for velocity constraint
      gaussian_graph.add(gtsam::Symbol('b0_V_bk', i), frame_i->A_21(), 
                        gtsam::Symbol('b0_V_bk', i+1), frame_i->A_22(),
                        gtsam::Symbol('dxdy', 0), A_23_tangent,
                        b_2_tangent, noise);
    }

    // Optimize Gaussian Graph and get solution
    gtsam::VectorValues solution = gaussian_graph.optimize();
    auto dxdy  = solution.at(gtsam::Symbol('dxdy', 0));

    // Retrieve velocity from graph optimization
    *init_vel = solution.at(gtsam::Symbol('b0_V_bk', 0));

    // Compute new g estimate
    g0 = (g0 + txty*dxdy).normalized()*g_world.norm();
  }
}

/* -------------------------------------------------------------------------- */
// Performs only the gyro bias estimate.
// [out] initial gyro bias estimate.
bool OnlineGravityAlignment::estimateGyroscopeBiasOnly(
    gtsam::Vector3 *gyro_bias) {
  VLOG(10) << "Gyroscope bias only called.";
  VisualInertialFrames vi_frames;
  CHECK_DOUBLE_EQ(gyro_bias->norm(), 0.0);

  // Construct set of frames for linear alignment
  constructVisualInertialFrames(estimated_body_poses_, delta_t_camera_, 
                                pims_, &vi_frames);

  // Estimate gyroscope bias
  if (estimateGyroscopeBias(vi_frames, gyro_bias)) {
    // Log and return
    LOG(INFO) << "Gyroscope bias estimate only successful.\n";
    return true;
  } else {
    LOG(ERROR) << "Gyroscope bias estimate only failed!\n";
    return false;
  }
}

} // namespace VIO
