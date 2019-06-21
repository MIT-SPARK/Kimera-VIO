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

#include "utils/Timer.h"

#include "OnlineGravityAlignment.h"

namespace VIO {

/* -------------------------------------------------------------------------- */
// TODO(Sandro): Adapt description
// Adding of states for bundle adjustment used in initialization.
// [in] timestamp_kf_nsec, keyframe timestamp.
// [in] status_smart_stereo_measurements_kf, vision data.
// [in] stereo_ransac_body_pose, inertial data.
OnlineGravityAlignment::OnlineGravityAlignment(
    const AlignmentPoses &estimated_body_poses, 
    const std::vector<double> &delta_t_camera,
    const AlignmentPims &pims,
    const gtsam::Vector3 &g_world, gtsam::Vector3 *gyro_bias)
    : estimated_body_poses_(estimated_body_poses),
      delta_t_camera_(delta_t_camera), pims_(pims),
      g_world_(g_world), gyro_bias_(*gyro_bias) {
        CHECK_DOUBLE_EQ(gyro_bias->norm(),0.0);
      }

/* -------------------------------------------------------------------------- */
// TODO(Sandro): Adapt description
// Adding of states for bundle adjustment used in initialization.
bool OnlineGravityAlignment::alignVisualInertialEstimates() {
  VLOG(5) << "Online gravity alignment called.";

  // Construct set of frames for linear alignment
  VisualInertialFrames vi_frames;
  constructVisualInertialFrames(estimated_body_poses_, delta_t_camera_, 
                                pims_, &vi_frames);

  // Estimate gyroscope bias
  CHECK(estimateGyroscopeBias(vi_frames, &gyro_bias_));

  // Update delta states with corrected bias
  CHECK(updateDeltaStates(pims_, gyro_bias_, &vi_frames));

  // Align visual and inertial estimates
  gtsam::Vector3 g_iter(1, 0, 0);
  if (alignEstimatesLinearly(vi_frames, g_world_, &g_iter)) {
    LOG(INFO) << "Online gravity alignment successful.";
    return true;
  } else {
    LOG(WARNING) << "Online gravity alignment successful.";
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
    const std::vector<double> &delta_t_camera,
    const AlignmentPims &pims,
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
// Estimate gyroscope bias by minimizing square of rotation error
// between the pre-integrated rotation constraint between frames
// and the estimated rotation between frames from Bundle-Adjustment.
// [in] frames containing pim constraints and body poses.
// [out] new estimated value for gyroscope bias.
bool OnlineGravityAlignment::estimateGyroscopeBias(
          const VisualInertialFrames &vi_frames,
          gtsam::Vector3 *gyro_bias) {

  // Create Gaussian Graph with unit noise
  gtsam::GaussianFactorGraph gaussian_graph;
  auto noise = gtsam::noiseModel::Unit::Create(3);
  
  // Loop through all initialization frame
  for (int i = 0; i < vi_frames.size(); i++) {
    // Compute relative rotation matrix from visual estimate
    gtsam::Rot3 bk_R_bkp1(vi_frames.at(i).bk_R_bkp1());
    // Get relative rotation matrix from pre-integrated estimate
    gtsam::Rot3 bk_gamma_bkp1(vi_frames.at(i).bk_gamma_bkp1());
    // Compute rotation error between pre-integrated and visual estimates
    gtsam::Rot3 bkp1_error_bkp1(bk_gamma_bkp1.transpose()*bk_R_bkp1.matrix());
    // Compute rotation error in canonical coordinates (dR_bkp1)
    gtsam::Vector3 dR = gtsam::Rot3::Logmap(bkp1_error_bkp1);
    // Get rotation Jacobian wrt. gyro_bias (dR_bkp1 = J * dbg_bkp1)
    gtsam::Matrix3 dbg_J_dR = vi_frames.at(i).dbg_jacobian_dR();
    
    // Insert Jacobian Factor in Gaussian Graph
    gaussian_graph.add(gtsam::Symbol('dbg', 0), dbg_J_dR, dR, noise);

    // Logging of variables inserted in the graph
    VLOG(10) << "Frame: " << (i + 1) << "\ndt_cam: (s)\n" << vi_frames.at(i).cam_dt()
            << "\ndt_pim: (s)\n" << vi_frames.at(i).pim_dt() << "\ncam bk_R_bkp1:\n"
            << bk_R_bkp1 << "\npim bk_gamma_bkp1:\n" << bk_gamma_bkp1 
            << "\nbk_error_bk:\n" << bkp1_error_bkp1;
  }

  // Optimize Gaussian Graph and get solution
  gtsam::VectorValues solution = gaussian_graph.optimize();
  gtsam::Vector3 delta_bg  = solution.at(gtsam::Symbol('dbg', 0));

  // Adapt gyroscope bias
  *gyro_bias += delta_bg;

  // Logging of solution
  if (VLOG_IS_ON(5)) { gaussian_graph.print("\nGaussia Factor graph:\n"); }
  LOG(INFO) << "\nGyro bias estimation:\n" << delta_bg;

  // TODO(Sandro): Implement check on quality of estimate
  return true;
}

/* -------------------------------------------------------------------------- */
// Corrects pre-integrated delta states with the gyro bias estimate.
// [in] vector of pre-integrations from visual-frontend.
// [in] new estimated value for gyroscope bias.
// [out] updated vector of frames used for initial alignment.
bool OnlineGravityAlignment::updateDeltaStates(const AlignmentPims &pims,
                                               const gtsam::Vector3 &gyro_bias,
                                               VisualInertialFrames *vi_frames) {
  CHECK_EQ(vi_frames->size(), pims.size());

  // Instead of repropagating, update the measurements
  // with a first-order approximation around the gyro bias estimate.
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
  // TODO(Sandro): Implement check on quality of estimate
  return true;
}

/* -------------------------------------------------------------------------- */
// TODO(Sandro): Adapt description
// Adding of states for bundle adjustment used in initialization.
// [in] timestamp_kf_nsec, keyframe timestamp.
// [in] status_smart_stereo_measurements_kf, vision data.
// [in] stereo_ransac_body_pose, inertial data.
bool OnlineGravityAlignment::alignEstimatesLinearly(
    const VisualInertialFrames &vi_frames, const gtsam::Vector3 &g_world,
    gtsam::Vector3 *g_iter) {

  // Create Gaussian Graph with unit noise
  gtsam::GaussianFactorGraph gaussian_graph;
  auto noise = gtsam::noiseModel::Unit::Create(3);

  // Total states in Linear Equation System
  // n_frames*3 for velocities v_x, v_y, v_z
  // + 3 for gravity vector in visual frame
  int n_states = vi_frames.size() * 3 + 3;

  // Linear Equation System: A*x = b
  gtsam::Matrix A = gtsam::Matrix::Zero(n_states, n_states);
  gtsam::Vector b = gtsam::Vector::Zero(n_states);
  gtsam::Matrix inv_cov = gtsam::Matrix::Identity(6, 6);

  // Loop through all frames
  for (int i = 0; i < vi_frames.size(); i++) {
    // Bookkeeping variables
    gtsam::Matrix tmp_A = gtsam::Matrix::Zero(6, 9);
    gtsam::Vector tmp_b = gtsam::Vector::Zero(6);

    // TODO(Sandro): Check conventions!!!

    ///////////// RELATIVE POSITION CONSTRAINT /////////////
    
    // Relative position constraint
    double dt_bk = vi_frames.at(i).pim_dt();
    gtsam::Matrix b0_R_bk = vi_frames.at(i).b0_R_bk();
    gtsam::Vector bk_alpha_bkp1 = vi_frames.at(i).delta_p();
    gtsam::Vector b0_p_bk = vi_frames.at(i).prev_p();
    gtsam::Vector b0_p_bkp1 = vi_frames.at(i).curr_p();

    // Matrix factors
    auto A_11 = -dt_bk * b0_R_bk;
    auto A_13 = 0.5 * dt_bk * dt_bk * b0_R_bk;
    auto b_1 = bk_alpha_bkp1 - b0_R_bk * (b0_p_bkp1 - b0_p_bk);

    ///////////// RELATIVE VELOCITY CONSTRAINT /////////////    

    // Relative velocity constraint
    gtsam::Vector bk_beta_bkp1 = vi_frames.at(i).delta_v();

    // Matrix factors
    auto A_21 = -b0_R_bk;
    auto A_22 = b0_R_bk;
    auto A_23 = -dt_bk * b0_R_bk;
    auto b_2 = bk_beta_bkp1;

    ///////////// FACTOR GRAPH /////////////

    // Add binary factor for position constraint
    gaussian_graph.add(gtsam::Symbol('b0_V_bk', i), A_11, 
                      gtsam::Symbol('g_b0', 0), A_13,
                      b_1, noise);

    // Add ternary factor for velocity constraint
    gaussian_graph.add(gtsam::Symbol('b0_V_bk', i), A_21, 
                      gtsam::Symbol('b0_V_bk', i+1), A_22,
                      gtsam::Symbol('g_b0', 0), A_23,
                      b_2, noise);

    ///////////// LINEAR EQUATION SYSTEM /////////////

    // Relative position constraint
    tmp_A.block<3, 3>(0, 0) = A_11;
    tmp_A.block<3, 3>(0, 6) = A_13;
    tmp_b.block<3, 1>(0, 0) = b_1;

    // Relative velocity constraint
    tmp_A.block<3, 3>(3, 0) = A_21;
    tmp_A.block<3, 3>(3, 3) = A_22;
    tmp_A.block<3, 3>(3, 6) = A_23;
    tmp_b.block<3, 1>(3, 0) = b_2;
    
    // Unweighted Least Squares
    gtsam::Matrix r_A = tmp_A.transpose() * inv_cov * tmp_A;
    gtsam::Vector r_b = tmp_A.transpose() * inv_cov * tmp_b;

    // Velocities block
    A.block<6, 6>(3 * i, 3 * i) += r_A.topLeftCorner<6, 6>();
    b.segment<6>(3 * i) += r_b.head<6>();

    // Gravity block
    A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
    b.tail<3>() += r_b.tail<3>();

    // Cross-term block
    A.block<6, 3>(3 * i, n_states - 3) += r_A.topRightCorner<6, 3>();
    A.block<3, 6>(n_states - 3, 3 * i) += r_A.bottomLeftCorner<3, 6>();
  }

  // Increase numerical stability
  A = A * 1000;
  b = b * 1000;

  // Solve linear equation
  gtsam::Vector x = A.ldlt().solve(b);
  *g_iter = x.tail<3>();

  // Log initial gravity guess
  LOG(INFO) << "Initial gravity estimate:\n" << *g_iter
            << " with norm: " << g_iter->norm();

  // Optimize Gaussian Graph and get solution
  gtsam::VectorValues solution = gaussian_graph.optimize();
  gtsam::Vector3 g_b0  = solution.at(gtsam::Symbol('g_b0', 0));

  // Logging of solution
  if (VLOG_IS_ON(5)) { gaussian_graph.print("\nGaussia Factor graph:\n"); }
  LOG(INFO) << "Initial gravity estimate:\n" << g_b0
            << " with norm: " << g_b0.norm();

  // TODO(Sandro): Insert check before gravity refinement
  // Return false in visual-inertial alignment if not successful

  // Refine gravity alignment
  refineGravity(vi_frames, g_world, g_iter);

  // Return true if successful
  return true;
}

/* -------------------------------------------------------------------------- */
// TODO(Sandro): Adapt description
// Adding of states for bundle adjustment used in initialization.
// [in] timestamp_kf_nsec, keyframe timestamp.
// [in] status_smart_stereo_measurements_kf, vision data.
// [in] stereo_ransac_body_pose, inertial data.
gtsam::Matrix createTangentBasis(gtsam::Vector3 &g0) {
  // TODO(Sandro): Implement computation here
  VLOG(5) << "Tangent basis created.";
  return gtsam::Matrix::Identity(3, 3);
}

/* -------------------------------------------------------------------------- */
// TODO(Sandro): Adapt description
// Adding of states for bundle adjustment used in initialization.
// [in] timestamp_kf_nsec, keyframe timestamp.
// [in] status_smart_stereo_measurements_kf, vision data.
// [in] stereo_ransac_body_pose, inertial data.
void OnlineGravityAlignment::refineGravity(const VisualInertialFrames &vi_frames,
                                           const gtsam::Vector3 &g_world,
                                           gtsam::Vector3 *g_iter) {
  // TODO(Sandro): Implement computation here

  // gtsam::Matrix tangent_basis = createTangentBasis(g_iter);
  VLOG(5) << "Gravity vector refined.";
}

} // namespace VIO
