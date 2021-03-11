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
 *
 * Qin, Tong, and Shaojie Shen.
 * Robust initialization of monocular visual-inertial estimation on aerial
 * robots. International Conference on Intelligent Robots and Systems (IROS).
 * IEEE, 2017.
 *
 * @author Antoni Rosinol
 * @author Sandro Berchier
 * @author Luca Carlone
 */

#include <memory>
#include <unordered_map>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "kimera-vio/initial/OnlineGravityAlignment.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

// TODO(Sandro): Create YAML file for initialization and read in!
DEFINE_double(gyroscope_residuals,
              5e-2,
              "Maximum allowed gyroscope residual after"
              " pre-integrationcorrection.");
DEFINE_bool(use_ahrs_estimator,
            false,
            "Use AHRS gyroscope bias estimator"
            " instead of linear 1st order approximation.");
DEFINE_double(rotation_noise_prior,
              1e-2,
              "Rotation noise for Rot3 priors"
              " in AHRS gyroscope estimation.");
DEFINE_double(gravity_tolerance_linear,
              1e-6,
              "Maximum gravity tolerance for linear alignment.");
DEFINE_int32(num_iterations_gravity_refinement,
             4,
             "Number of iterations for gravity refinement.");
DEFINE_double(gravity_tolerance_refinement,
              1e-1,
              "Maximum gravity tolerance for refinement.");
DEFINE_double(camera_pim_delta_difference,
              5e-3,
              "Maximum tolerable difference in time interval.");

namespace VIO {

/* -------------------------------------------------------------------------- */
void VisualInertialFrame::updateDeltaState(const gtsam::NavState &delta_state) {
  bk_alpha_bkp1_ = gtsam::Vector3(delta_state.pose().translation());
  bk_beta_bkp1_ = delta_state.velocity();
  bk_gamma_bkp1_ = delta_state.pose().rotation().matrix();
}

// TODO(Sandro): Retract velocities and poses from last optimization
// TODO(Sandro): Check conventions
// TODO(Sandro): Why are we not iterating also the tangent basis?
// TODO(Sandro): Either retrieve all velocities and poses to insert
// in the Backend optimization or retrieve last pose and velocity
// so that there is no lag for the Backend optimization

/* -------------------------------------------------------------------------- */
// Constructor for visual inertial alignment.
// [in] estimated camera poses in initial body frame.
// [in] delta timestamps of camera frames.
// [in] pre-integrations performed in visual Frontend.
// [in] global gravity vector in world frame.
OnlineGravityAlignment::OnlineGravityAlignment(
    const AlignmentPoses &estimated_body_poses,
    const std::vector<double> &delta_t_camera,
    const AlignmentPims &pims,
    const gtsam::Vector3 &g_world,
    const InitialAHRSPims &ahrs_pims)
    : estimated_body_poses_(estimated_body_poses),
      delta_t_camera_(delta_t_camera),
      pims_(pims),
      g_world_(g_world),
      ahrs_pims_(ahrs_pims) {}

/* -------------------------------------------------------------------------- */
// Performs visual-inertial alignment and gravity estimate.
// [out] initial gyro bias estimate.
// [out] estimate of gravity vector in initial body pose (g_b0).
// [out] initial nav state for initialization.
// [optional] flag to estimate gyroscope bias.
bool OnlineGravityAlignment::alignVisualInertialEstimates(
    gtsam::Vector3 *gyro_bias,
    gtsam::Vector3 *g_iter,
    gtsam::NavState *init_navstate,
    const bool &estimate_bias) {
  CHECK_NOTNULL(gyro_bias);
  CHECK_NOTNULL(g_iter);
  VLOG(10) << "Online gravity alignment called.";
  VisualInertialFrames vi_frames;
  gtsam::Velocity3 init_velocity;

  // Construct set of frames for linear alignment
  constructVisualInertialFrames(
      estimated_body_poses_, delta_t_camera_, pims_, &vi_frames);

  // Estimate gyroscope bias if requested
  if (estimate_bias) {
    if (!estimateBiasAndUpdateStates(pims_,
                                     ahrs_pims_,
                                     gyro_bias,
                                     &vi_frames,
                                     FLAGS_use_ahrs_estimator)) {
      LOG(ERROR) << "Gyroscope bias estimation failed!";
      return false;
    }
  } else {
    LOG(WARNING) << "Gyroscope bias estimation skipped!";
  }

  // Align visual and inertial estimates
  if (alignEstimatesLinearly(vi_frames, g_world_, g_iter, &init_velocity)) {
    // Align gravity vectors and estimate initial pose
    gtsam::Rot3 w0_R_b0 =
        UtilsOpenCV::AlignGravityVectors(*g_iter, g_world_, false);
    gtsam::Pose3 w0_T_b0(w0_R_b0, gtsam::Point3::Zero());
    // Create initial navstate and rotate velocity in world frame
    *init_navstate = gtsam::NavState(w0_T_b0 * vi_frames.at(0).b0Tbk(),
                                     w0_T_b0.rotation() * init_velocity);
    // Log and return
    LOG(INFO) << "Online gravity alignment successful with:\n"
              << "pose: " << init_navstate->pose() << '\n'
              << "velocity: " << init_navstate->velocity() << '\n'
              << "gravity: " << *g_iter << '\n'
              << "with norm: " << g_iter->norm() << '\n'
              << "gyroscope bias: " << *gyro_bias << '\n';
    return true;
  } else {
    LOG(ERROR) << "Online gravity alignment failed!";
    return false;
  }
}

/* -------------------------------------------------------------------------- */
// This function creates the frame objects used in the alignment.
// It populates the camera poses, delta pre-integrations and jacobian.
// [in] vector of estimated camera poses from Bundle-Adjustment.
// [in] vector of delta_t in camera frames.
// [in] vector of pre-integrations from visual-Frontend.
// [out] vector of frames used for initial alignment.
void OnlineGravityAlignment::constructVisualInertialFrames(
    const AlignmentPoses &estimated_body_poses,
    const std::vector<double> &delta_t_camera,
    const AlignmentPims &pims,
    VisualInertialFrames *vi_frames) {
  CHECK_NOTNULL(vi_frames);
  CHECK_EQ(estimated_body_poses.size() - 1, delta_t_camera.size());
  CHECK_EQ(delta_t_camera.size(), pims.size());
  vi_frames->clear();

  // Create initialization frames
  for (int i = 0; i < pims.size(); i++) {
    // Get bk_gamma_bkp1, bk_alpha_bkp1, bk_beta_bkp1
    // pim.deltaXij() corresponds to bodyLkf_X_bodyK_imu
    gtsam::NavState delta_state(pims.at(i)->deltaXij());
    // Get delta_time_pim = t_kp1-t_k
    const double delta_t_pim = pims.at(i)->deltaTij();
#ifdef GTSAM_TANGENT_PREINTEGRATION
    // Get pre-integration Jacobian wrt. gyro_bias (dPIM = J * dbg)
    gtsam::Matrix dbg_J_dPIM = pims.at(i)->preintegrated_H_biasOmega();
    // Get rotation Jacobian wrt. gyro_bias (dR_bkp1 = J * dbg_bkp1)
    gtsam::Matrix3 dbg_Jacobian_dR = gtsam::sub(dbg_J_dPIM, 0, 3, 0, 3);
#else
    // Get rotation Jacobian wrt. gyro_bias (dR_bkp1 = J * dbg_bkp1)
    gtsam::Matrix3 dbg_Jacobian_dR = pims.at(i)->delRdelBiasOmega();
#endif

    CHECK_GT(FLAGS_camera_pim_delta_difference,
             abs(delta_t_pim - delta_t_camera.at(i)));

    // Create frame with b0_T_bkp1, b0_T_bk, dt_bk_cam,
    // dbg_Jacobian_dR_bk, dt_bk_imu
    VisualInertialFrame frame_i(estimated_body_poses.at(i + 1),
                                estimated_body_poses.at(i),
                                delta_t_camera.at(i),
                                delta_state,
                                dbg_Jacobian_dR,
                                delta_t_pim);
    vi_frames->push_back(frame_i);
  }
}

/* -------------------------------------------------------------------------- */
// Estimate gyroscope bias and update delta states in VI frames
// [in] vector of pre-integrations from visual-Frontend.
// [out] new estimated value for gyroscope bias.
// [out] frames containing pim constraints and body poses.
bool OnlineGravityAlignment::estimateBiasAndUpdateStates(
    const AlignmentPims &pims,
    const InitialAHRSPims &ahrs_pims,
    gtsam::Vector3 *gyro_bias,
    VisualInertialFrames *vi_frames,
    const bool &use_ahrs_estimator) {
  CHECK_NOTNULL(gyro_bias);
  CHECK_NOTNULL(vi_frames);
  if (gyro_bias->norm() != 0.0) {
    LOG(ERROR) << "Non-zero PIM gyro bias:\n" << *gyro_bias;
    return false;
  }
  // Estimate gyroscope bias using either AHRS estimator
  // (if available and requested) or linear 1st approximator.
  if (use_ahrs_estimator && (vi_frames->size() == ahrs_pims.size())) {
    estimateGyroscopeBiasAHRS(*vi_frames, ahrs_pims, gyro_bias);
    LOG(INFO) << "AHRS Bias Estimator used.";
  } else {
    estimateGyroscopeBias(*vi_frames, gyro_bias);
    LOG(INFO) << "Linear Bias Estimator used.";
  }
  // Update delta states with corrected bias
  updateDeltaStates(pims_, *gyro_bias, vi_frames);
  if (estimateGyroscopeResiduals(*vi_frames).norm() >
      FLAGS_gyroscope_residuals) {
    LOG(ERROR) << "High residuals after bias update.";
    return false;
  }
  return true;
}

/* -------------------------------------------------------------------------- */
// Estimate gyroscope bias by minimizing square of rotation error
// between the pre-integrated rotation constraint between frames
// and the estimated rotation between frames from Bundle-Adjustment.
// [in] frames containing pim constraints and body poses.
// [out] new estimated value for gyroscope bias.
// Unit tested.
void OnlineGravityAlignment::estimateGyroscopeBias(
    const VisualInertialFrames &vi_frames,
    gtsam::Vector3 *gyro_bias) {
  CHECK_NOTNULL(gyro_bias);
  // Create Gaussian Graph with unit noise
  gtsam::GaussianFactorGraph gaussian_graph;
  auto noise = gtsam::noiseModel::Unit::Create(3);

  // Loop through all initialization frame
  for (int i = 0; i < vi_frames.size(); i++) {
    auto frame_i = std::next(vi_frames.begin(), i);

    // Compute rotation error between pre-integrated and visual estimates
    gtsam::Rot3 bkp1_error_bkp1(frame_i->bkGammaBkp1().transpose() *
                                frame_i->bkRbkp1());
    // Compute rotation error in canonical coordinates (dR_bkp1)
    gtsam::Vector3 dR = gtsam::Rot3::Logmap(bkp1_error_bkp1);
    // Get rotation Jacobian wrt. gyro_bias (dR_bkp1 = J * dbg_bkp1)
    gtsam::Matrix3 dbg_J_dR = frame_i->dbgJacobianDr();

    // Insert Jacobian Factor in Gaussian Graph
    gaussian_graph.add(gtsam::Symbol('d', 0), dbg_J_dR, dR, noise);
  }
  // Optimize Gaussian Graph and get solution
  gtsam::VectorValues solution = gaussian_graph.optimize();
  gtsam::Vector3 delta_bg = solution.at(gtsam::Symbol('d', 0));

  // Adapt gyroscope bias
  *gyro_bias += delta_bg;

  // Logging of solution
  if (VLOG_IS_ON(5)) {
    gaussian_graph.print("\nGaussian Factor graph:\n");
  }
  VLOG(5) << "Gyro bias estimation:\n" << delta_bg;

  // TODO(Sandro): Implement check on quality of estimate
  return;
}

/* -------------------------------------------------------------------------- */
// Estimate gyroscope bias using AHRS factors.
// [in] frames containing pim constraints and body poses.
// [in] frames containing ahrs pim constraints.
// [out] new estimated value for gyroscope bias.
// Not yet unit tested!
void OnlineGravityAlignment::estimateGyroscopeBiasAHRS(
    const VisualInertialFrames &vi_frames,
    const InitialAHRSPims &ahrs_pims,
    gtsam::Vector3 *gyro_bias) {
  CHECK_NOTNULL(gyro_bias);
  CHECK_EQ(vi_frames.size(), ahrs_pims.size());

  // Create initial values and factor graph
  gtsam::Values initial;
  gtsam::NonlinearFactorGraph nfg;
  // Add gyroscope bias initial value
  initial.insert(gtsam::Symbol('b', 0), gtsam::Vector3());

  // Create noise model (Gaussian) in body frame
  Matrix3 B_Rot_W = vi_frames.at(0).b0Rbk().transpose();
  gtsam::SharedNoiseModel noise_init_rot =
      gtsam::noiseModel::Gaussian::Covariance(
          B_Rot_W * (FLAGS_rotation_noise_prior * gtsam::Matrix3::Identity()) *
          B_Rot_W.transpose());

  // Insert initial value with tight rotation prior
  initial.insert(gtsam::Symbol('R', 0), gtsam::Rot3(vi_frames.at(0).b0Rbk()));
  nfg.push_back(
      gtsam::PriorFactor<gtsam::Rot3>(gtsam::Symbol('R', 0),
                                      gtsam::Rot3(vi_frames.at(0).b0Rbk()),
                                      noise_init_rot));

  // Loop through all initialization frames
  for (int i = 0; i < vi_frames.size(); i++) {
    auto frame_id = i + 1;
    auto frame_i = std::next(vi_frames.begin(), i);

    // Insert initial value with tight rotation prior
    initial.insert(gtsam::Symbol('R', frame_id),
                   gtsam::Rot3(frame_i->b0Rbkp1()));
    nfg.push_back(
        gtsam::PriorFactor<gtsam::Rot3>(gtsam::Symbol('R', frame_id),
                                        gtsam::Rot3(frame_i->b0Rbkp1()),
                                        noise_init_rot));
    // Add AHRS factor between rotations
    nfg.push_back(gtsam::AHRSFactor(gtsam::Symbol('R', i),
                                    gtsam::Symbol('R', frame_id),
                                    gtsam::Symbol('b', 0),
                                    ahrs_pims.at(i),
                                    gtsam::Vector3(0, 0, 0)));
  }

  // Levenberg-Marquardt optimization
  gtsam::LevenbergMarquardtParams lmParams;
  gtsam::LevenbergMarquardtOptimizer bias_estimator(nfg, initial, lmParams);

  // Optimize and get values
  gtsam::Values initial_values = bias_estimator.optimize();
  VLOG(10) << "LM Bias estimator done.";
  *gyro_bias = initial_values.at<gtsam::Vector3>(gtsam::Symbol('b', 0));

  // Logging of solution
  // std::stringstream ss;
  // auto old_buf = std::cout.rdbuf(ss.rdbuf());
  if (VLOG_IS_ON(5)) {
    nfg.print("\nNon-linear factor graph:\n");
  }
  // std::cout.rdbuf(old_buf); //reset
  // LOG(ERROR) << ss.str();
  VLOG(5) << "Gyro bias estimation:\n" << *gyro_bias;
  LOG(ERROR) << *gyro_bias;

  // TODO(Sandro): Implement check on quality of estimate
  return;
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
    gtsam::Rot3 bkp1_error_bkp1(frame_i->bkGammaBkp1().transpose() *
                                frame_i->bkRbkp1());
    // Compute rotation error in canonical coordinates (dR_bkp1)
    dR += gtsam::Rot3::Logmap(bkp1_error_bkp1);
  }
  VLOG(5) << "Residuals after bias correction: \n" << dR;
  return dR;
}

/* -------------------------------------------------------------------------- */
// Corrects pre-integrated delta states with the gyro bias estimate.
// It uses a first-order approximation around the gyro bias estimate.
// [in] vector of pre-integrations from visual-Frontend.
// [in] new estimated value for gyroscope bias.
// [out] updated vector of frames used for initial alignment.
void OnlineGravityAlignment::updateDeltaStates(
    const AlignmentPims &pims,
    const gtsam::Vector3 &gyro_bias,
    VisualInertialFrames *vi_frames) {
  CHECK_NOTNULL(vi_frames);
  CHECK_EQ(vi_frames->size(), pims.size());

  // Repropagate measurements with first order approximation
  for (int i = 0; i < vi_frames->size(); i++) {
    // Update pre-integration with first-order approximation
    gtsam::Vector9 correct_preintegrated = pims.at(i)->biasCorrectedDelta(
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
    const VisualInertialFrames &vi_frames,
    const gtsam::Vector3 &g_world,
    gtsam::Vector3 *g_iter,
    gtsam::Velocity3 *init_vel) {
  CHECK_NOTNULL(g_iter);
  CHECK_NOTNULL(init_vel);

  // Create Gaussian Graph with unit noise
  gtsam::GaussianFactorGraph gaussian_graph;
  auto noise = gtsam::noiseModel::Unit::Create(3);

  // Loop through all frames
  for (int i = 0; i < vi_frames.size(); i++) {
    auto frame_i = std::next(vi_frames.begin(), i);

    // Add binary factor for position constraint
    gaussian_graph.add(gtsam::Symbol('V', i),
                       frame_i->A11(),
                       gtsam::Symbol('g', 0),
                       frame_i->A13(),
                       frame_i->b1(),
                       noise);

    // Add ternary factor for velocity constraint
    gaussian_graph.add(gtsam::Symbol('V', i),
                       frame_i->A21(),
                       gtsam::Symbol('V', i + 1),
                       frame_i->A22(),
                       gtsam::Symbol('g', 0),
                       frame_i->A23(),
                       frame_i->b2(),
                       noise);
  }

  // Optimize Gaussian Graph and get solution
  gtsam::VectorValues solution = gaussian_graph.optimize();
  gtsam::Vector3 g_b0 = solution.at(gtsam::Symbol('g', 0));

  // Logging of solution
  if (VLOG_IS_ON(5)) {
    gaussian_graph.print("\nGaussian Factor graph:\n");
  }

  // Refine gravity alignment if necessary
  // TODO(Sandro): Load tolerance in yaml file
  if (abs(g_b0.norm() - g_world.norm()) > FLAGS_gravity_tolerance_linear) {
    refineGravity(vi_frames, g_world, &g_b0, init_vel);
  } else {
    // Retrieve initial velocity from graph optimization
    *init_vel = solution.at(gtsam::Symbol('V', 0));
  }

  // We want the gravity vector and not the measured acceleration
  // by the IMU, hence multiply estimated value by -1 (the IMU measures
  // the reaction forces of gravity in the accelerometer).
  *g_iter = -g_b0;

  // Logging of solution
  VLOG(5) << "Final gravity estimate:\n"
          << *g_iter << " with norm: " << g_iter->norm();

  // Return true if successful
  // TODO(Sandro): Load tolerance in yaml file
  if (abs(g_iter->norm() - g_world.norm()) >
      FLAGS_gravity_tolerance_refinement) {
    return false;
  }
  return true;
}

/* -------------------------------------------------------------------------- */
// Creates tangent basis to input vector.
// [in] Vector for which tangent basis is desired.
// [return] Tangent basis spanned by orthogonal basis to input vector.
gtsam::Matrix OnlineGravityAlignment::createTangentBasis(
    const gtsam::Vector3 &g0) {
  // Vectors b, c
  gtsam::Vector3 b, c;

  // Set a as g0 normalized
  gtsam::Vector3 a(g0.normalized());

  // Look for b orthogonal to a
  gtsam::Vector3 tmp(0, 0, 1);
  if (a == tmp) tmp << (1, 0, 0);
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
    const VisualInertialFrames &vi_frames,
    const gtsam::Vector3 &g_world,
    gtsam::Vector3 *g_iter,
    gtsam::Velocity3 *init_vel) {
  CHECK_NOTNULL(g_iter);
  CHECK_NOTNULL(init_vel);
  // Define current gravity estimate (normalized)
  gtsam::Vector3 g0 = g_iter->normalized() * g_world.norm();

  // Create tangent basis to g (g = g0 + txty*dxdy)
  gtsam::Matrix txty = createTangentBasis(g0);

  // Multiple iterations till convergence
  for (int l = 0; l < FLAGS_num_iterations_gravity_refinement; l++) {
    // Create Gaussian Graph with unit noise
    gtsam::GaussianFactorGraph gaussian_graph;
    auto noise = gtsam::noiseModel::Unit::Create(3);

    // Loop through all frames
    for (int i = 0; i < vi_frames.size(); i++) {
      auto frame_i = std::next(vi_frames.begin(), i);

      // Apply tangent basis to g (g = g0 + txty*dxdy)
      gtsam::Matrix A_13_tangent = frame_i->A13() * txty;
      gtsam::Vector3 b_1_tangent = frame_i->b1() - frame_i->A13() * g0;

      // Add binary factor for position constraint
      gaussian_graph.add(gtsam::Symbol('V', i),
                         frame_i->A11(),
                         gtsam::Symbol('d', 0),
                         A_13_tangent,
                         b_1_tangent,
                         noise);

      // Apply tangent basis to g (g = g0 + txty*dxdy)
      gtsam::Matrix A_23_tangent = frame_i->A23() * txty;
      gtsam::Vector3 b_2_tangent = frame_i->b2() - frame_i->A23() * g0;

      // Add ternary factor for velocity constraint
      gaussian_graph.add(gtsam::Symbol('V', i),
                         frame_i->A21(),
                         gtsam::Symbol('V', i + 1),
                         frame_i->A22(),
                         gtsam::Symbol('d', 0),
                         A_23_tangent,
                         b_2_tangent,
                         noise);
    }

    // Optimize Gaussian Graph and get solution
    gtsam::VectorValues solution = gaussian_graph.optimize();
    auto dxdy = solution.at(gtsam::Symbol('d', 0));

    // Retrieve velocity from graph optimization
    *init_vel = solution.at(gtsam::Symbol('V', 0));

    // Compute new g estimate
    g0 = (g0 + txty * dxdy).normalized() * g_world.norm();
  }
}

/* -------------------------------------------------------------------------- */
// Performs only the gyro bias estimate.
// [out] initial gyro bias estimate.
bool OnlineGravityAlignment::estimateGyroscopeBiasOnly(
    gtsam::Vector3 *gyro_bias,
    const bool &use_ahrs_estimator) {
  CHECK_NOTNULL(gyro_bias);
  VLOG(10) << "Gyroscope bias only called.";
  VisualInertialFrames vi_frames;

  // Construct set of frames for linear alignment
  constructVisualInertialFrames(
      estimated_body_poses_, delta_t_camera_, pims_, &vi_frames);

  // Estimate gyro bias and check residuals
  if (!estimateBiasAndUpdateStates(
          pims_, ahrs_pims_, gyro_bias, &vi_frames, use_ahrs_estimator)) {
    return false;
  }
  return true;
}

}  // namespace VIO
