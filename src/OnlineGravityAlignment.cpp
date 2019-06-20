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
    const gtsam::Vector3 &g_world, GyroBias *gyro_bias)
    : estimated_body_poses_(estimated_body_poses),
      delta_t_camera_(delta_t_camera), pims_(pims),
      g_world_(g_world), gyro_bias_(*gyro_bias) {
  // Initial log
  LOG(INFO) << "Online Gravity Alignment constructor called.\n";
}

/* -------------------------------------------------------------------------- */
// TODO(Sandro): Adapt description
// Adding of states for bundle adjustment used in initialization.
bool OnlineGravityAlignment::alignVisualInertialEstimates() {
  VLOG(5) << "Online gravity alignment called.";

  // TODO(Sandro): Need to get pre-integration without gravity!!!!!!
  AlignmentFrames frames;
  constructFrames(estimated_body_poses_, delta_t_camera_, pims_, &frames);

  // Estimate gyroscope bias
  CHECK(estimateGyroscopeBias(frames, pims_, &gyro_bias_));

  // Update delta states with corrected bias
  CHECK(updateDeltaStates(pims_, gyro_bias_, &frames));

  // Align visual and inertial estimates
  gtsam::Vector3 g_iter(1, 0, 0);
  if (alignEstimatesLinearly(frames, g_world_, &g_iter)) {
    LOG(INFO) << "Online gravity alignment successful.";
    return true;
  } else {
    LOG(WARNING) << "Online gravity alignment successful.";
    return false;
  }
}

/* -------------------------------------------------------------------------- */
// This function creates the frame objects used in the alignment.
// It populates the camera poses and delta pre-integrations.
// [in] vector of estimated camera poses from Bundle-Adjustment.
// [in] vector of delta_t in camera frames.
// [in] vector of pre-integrations from visual-frontend.
// [out] vector of frames used for initial alignment.
void OnlineGravityAlignment::constructFrames(
    const AlignmentPoses &estimated_body_poses, 
    const std::vector<double> &delta_t_camera,
    const AlignmentPims &pims, AlignmentFrames *frames) {

  CHECK_EQ(estimated_body_poses.size()-1, delta_t_camera.size());
  CHECK_EQ(delta_t_camera.size(), pims.size());
  frames->clear();

  // Create initialization frames
  for (int i = 0; i < pims.size(); i++) {
    // Get bk_gamma_bkp1, bk_alpha_bkp1, bk_beta_bkp1
    // pim.deltaXij() corresponds to bodyLkf_X_bodyK_imu
    gtsam::NavState delta_state(pims.at(i).deltaXij());
    // Get delta_time = t_kp1-t_k
    const double delta_t_pim = pims.at(i).deltaTij();

    VLOG(5) << "Delta t pim:\n"
            << delta_t_pim
            << "Delta R pim:\n"
            << delta_state.pose().rotation();

    // Create frame with current pose: v0_T_bkp1
    // and previous frame pose: v0_T_bk
    AlignmentFrame frame_i(estimated_body_poses.at(i + 1),
                           estimated_body_poses.at(i),
                           delta_t_camera.at(i),
                           delta_state, delta_t_pim);
    frames->push_back(frame_i);
  }
}

/* -------------------------------------------------------------------------- */
// Estimate gyroscope bias by minimizing square of rotation error
// between the pre-integrated rotation constraint between frames
// and the estimated rotation between frames from Bundle-Adjustment
// [in] frames containing pim constraints and body poses
// [in] pre-integrations for all frames
// [out] estimated delta for gyroscope bias
bool OnlineGravityAlignment::estimateGyroscopeBias(
    const AlignmentFrames &frames, const AlignmentPims &pims,
    GyroBias *gyro_bias) {

  CHECK_EQ(frames.size(), pims.size());

  /////////////////// LINEAR EQUATION SYSTEM ////////////////////////////

  // Matrices for linear equation system
  gtsam::Matrix3 A = gtsam::Matrix::Zero(3, 3);
  gtsam::Vector3 b = gtsam::Vector3::Zero();

  // Loop through all initialization frames
  for (int i = 0; i < frames.size(); i++) {
    // Compute relative rotation matrix from visual estimate
    gtsam::Matrix v_R_bk = frames.at(i).prev_R_mat();
    gtsam::Matrix v_R_bkp1 = frames.at(i).curr_R_mat();
    gtsam::Rot3 bk_R_bkp1(v_R_bk.transpose() * v_R_bkp1);

    // TODO(Sandro): Fix issue with frames accessing
    // Get relative rotation matrix from pre-integrated estimate
    gtsam::Rot3 bk_gamma_bkp1(pims.at(i).deltaRij());

    // Compute rotation error between pre-integrated and visual estimates
    gtsam::Rot3 bkp1_error_bkp1(bk_gamma_bkp1.transpose()*bk_R_bkp1.matrix());

    // Compute rotation error in canonical coordinates (dR_bkp1)
    gtsam::Vector3 dR = gtsam::Rot3::Logmap(bkp1_error_bkp1);

    // Get pre-integration Jacobian wrt. gyro_bias (dPIM = J * dbg)
    gtsam::Matrix dbg_J_dPIM = pims.at(i).preintegrated_H_biasOmega();

    // Get rotation Jacobian wrt. gyro_bias (dR_bkp1 = J * dbg_bkp1)
    gtsam::Matrix3 dbg_J_dR = gtsam::sub(dbg_J_dPIM, 0, 3, 0, 3);

    // Logging of variables
    VLOG(5) << "Gyro bias estimation: frame " << (i + 1)
            << "\ndelta t camera: (s)\n" << frames.at(i).camera_dt()
            << "\ndelta t pim: (s)\n" << frames.at(i).pim_dt()
            << "\ndelta t pim: (s)\n" << pims.at(i).deltaTij()
            << "\ncamera bk_R_bkp1:\n"
            << bk_R_bkp1 << "\npim bk_gamma_bkp1:\n"
            << bk_gamma_bkp1 << "\nbk_error_bk (rotation):\n"
            << bkp1_error_bkp1 << "\ndR_bkp1:\n"
            << dR << "\ndR_bkp1/dbg_bkp1 Jacobian (tmp_A):\n"
            << dbg_J_dR;
    
    // Formulate Least Squares problem for all measurements
    A += dbg_J_dR.transpose() * dbg_J_dR;
    b += dbg_J_dR.transpose() * dR;
  }

  /////////////////// SOLVE EQUATION SYSTEM /////////////////////////////

  // Increase numerical stability
  A = A * 1000;
  b = b * 1000;

  // Solve linear equation system for gyro bias
  gtsam::Vector3 delta_bg = A.ldlt().solve(b);

  // Logging of linear equation system
  LOG(INFO) << "Linear equation for gyro bias: "
            << "\nmatrix A: \n"
            << A << "\nmatrix b: \n"
            << b << "\nGyroscope bias initial calibration: \n"
            << delta_bg;

  // Adapt gyroscope bias
  *gyro_bias += delta_bg;

  // Return if successful
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
    const AlignmentFrames &frames, const gtsam::Vector3 &g_world,
    gtsam::Vector3 *g_iter) {

  // Total states in Linear Equation System
  // n_frames*3 for velocities v_x, v_y, v_z
  // + 3 for gravity vector in visual frame
  int n_states = frames.size() * 3 + 3;

  // Linear Equation System: A*x = b
  gtsam::Matrix A = gtsam::Matrix::Zero(n_states, n_states);
  gtsam::Vector b = gtsam::Vector::Zero(n_states);
  gtsam::Matrix inv_cov = gtsam::Matrix::Identity(6, 6);

  // Loop through all frames
  for (int i = 0; i < frames.size(); i++) {
    // Bookkeeping variables
    gtsam::Matrix tmp_A = gtsam::Matrix::Zero(6, 9);
    gtsam::Vector tmp_b = gtsam::Vector::Zero(6);

    // Variables from pre-integration and bundle-adjustment
    // TODO(Sandro): Check conventions!!!
    double dt = frames.at(i).pim_dt();
    gtsam::Matrix bk_R_v = frames.at(i).prev_R_mat().transpose();
    gtsam::Vector bk_alpha_bkp1 =
        frames.at(i).delta_p();                   // TODO(Sandro): Inverse???
    gtsam::Vector v_p_bk = frames.at(i).prev_p(); // TODO(Sandro): Inverse???
    gtsam::Vector v_p_bkp1 = frames.at(i).curr_p();    // TODO(Sandro): Inverse???
    gtsam::Vector bk_beta_bkp1 =
        frames.at(i).delta_v(); // TODO(Sandro): Inverse???

    // Relative position constraint
    tmp_A.block<3, 3>(0, 0) = -dt * bk_R_v;
    tmp_A.block<3, 3>(0, 6) = 0.5 * dt * dt * bk_R_v;
    tmp_b.block<3, 1>(0, 0) = bk_alpha_bkp1 - bk_R_v * (v_p_bkp1 - v_p_bk);

    // Relative velocity constraint
    tmp_A.block<3, 3>(3, 0) = -bk_R_v;
    tmp_A.block<3, 3>(3, 3) = bk_R_v;
    tmp_A.block<3, 3>(3, 6) = dt * bk_R_v;
    tmp_b.block<3, 1>(3, 0) = bk_beta_bkp1;

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
  LOG(INFO) << "Initial gravity estimate: " << *g_iter
            << " with norm: " << g_iter->norm();

  // TODO(Sandro): Insert check before gravity refinement
  // Return false in visual-inertial alignment if not successful

  // Refine gravity alignment
  refineGravity(frames, g_world, g_iter);

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
void OnlineGravityAlignment::refineGravity(const AlignmentFrames &frames,
                                           const gtsam::Vector3 &g_world,
                                           gtsam::Vector3 *g_iter) {
  // TODO(Sandro): Implement computation here

  // gtsam::Matrix tangent_basis = createTangentBasis(g_iter);
  VLOG(5) << "Gravity vector refined.";
}

/* -------------------------------------------------------------------------- */
// TODO(Sandro): Adapt description
// Adding of states for bundle adjustment used in initialization.
// [in] timestamp_kf_nsec, keyframe timestamp.
// [in] status_smart_stereo_measurements_kf, vision data.
// [in] stereo_ransac_body_pose, inertial data.
bool OnlineGravityAlignment::updateDeltaStates(const AlignmentPims &pims,
                                               const GyroBias &gyro_bias,
                                               AlignmentFrames *frames) {
  CHECK_EQ(frames->size(), pims.size());

  // Instead of repropagating, update the measurements
  // with a first-order approximation
  for (int i = 0; i < frames->size(); i++) {
    // Update pre-integration with first-order approximation
    gtsam::Vector9 updated_preintegrated = pims.at(i).biasCorrectedDelta(
        gtsam::imuBias::ConstantBias(Vector3::Zero(), gyro_bias));
    // Retract delta state
    const gtsam::NavState delta_state =
        gtsam::NavState().retract(updated_preintegrated);
    frames->at(i).updateDeltaState(delta_state);
  }

  // Return true if successful
  return true;
}

} // namespace VIO
