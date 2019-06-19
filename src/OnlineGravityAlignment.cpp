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

#include "src/OnlineGravityAlignment.h"

namespace VIO {

/* -------------------------------------------------------------------------- */
OnlineGravityAlignment::OnlineGravityAlignment(
    const AlignmentPoses &estimated_body_poses, const AlignmentPims &pims,
    const gtsam::Vector3 &g_world, GyroBias *gyro_bias)
    : estimated_body_poses_(estimated_body_poses), pims_(pims),
      g_world_(g_world), gyro_bias_(*gyro_bias) {
  // Initial log
  LOG(INFO) << "Online Gravity Alignment constructed.\n";
}

/* -------------------------------------------------------------------------- */
// TODO(Sandro): Adapt description
// Adding of states for bundle adjustment used in initialization.
// [in] timestamp_kf_nsec, keyframe timestamp.
// [in] status_smart_stereo_measurements_kf, vision data.
// [in] stereo_ransac_body_pose, inertial data.
bool OnlineGravityAlignment::alignVisualInertialEstimates() {
  VLOG(5) << "Online gravity alignment called.";

  // TODO(Sandro): Need to get pre-integration without gravity!!!!!!
  AlignmentFrames frames;
  constructFrames(estimated_body_poses_, pims_, &frames);

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
// TODO(Sandro): Adapt description
// Adding of states for bundle adjustment used in initialization.
// [in] timestamp_kf_nsec, keyframe timestamp.
// [in] status_smart_stereo_measurements_kf, vision data.
// [in] stereo_ransac_body_pose, inertial data.
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
    // Compute relative quaternion rotation from visual
    gtsam::Matrix v_R_bkm1 = frames.at(i).prev_R_mat();
    gtsam::Matrix v_R_bk = frames.at(i).R_mat();
    gtsam::Rot3 bkm1_R_bk(v_R_bkm1.transpose() * v_R_bk);
    gtsam::Quaternion bkm1_q_bk(bkm1_R_bk.matrix());

    // Compute relative quaternion rotation from pim
    // pim.deltaRij() corresponds to bodyLkf_R_bodyK_imu
    // gtsam::Rot3 bkm1_gamma_bk(frames.at(i).delta_R_mat());
    // TODO(Sandro): Fix issue with frames accessing
    gtsam::Rot3 bkm1_gamma_bk(pims.at(i).deltaXij().pose().rotation());
    gtsam::Quaternion bkm1_delta_q_bk(bkm1_gamma_bk.matrix());

    // Get pre-integration Jacobian wrt. gyro_bias
    gtsam::Matrix pims_jacobian = pims.at(i).preintegrated_H_biasOmega();

    // Setup linear equation system for one observation of delta_bg
    // tmp_A*delta_bg = tmp_b for all measurements
    gtsam::Matrix3 tmp_A = gtsam::sub(pims_jacobian, 0, 3, 0, 3);
    gtsam::Quaternion rot_error(bkm1_delta_q_bk.inverse() * bkm1_q_bk);
    // TODO(Sandro): Do we need this?
    rot_error.normalized();
    // TODO(Sandro): Is vec() correct? Check with Luca
    // Option 1:
    // gtsam::Vector3 tmp_b = 2 * rot_error.vec();
    // Option 2:
    gtsam::Vector3 tmp_b =
        gtsam::Rot3::Logmap(gtsam::Rot3(rot_error.toRotationMatrix()));

    // Average through all measurements
    A += tmp_A.transpose() * tmp_A;
    b += tmp_A.transpose() * tmp_b;

    // Logging of variables
    VLOG(5) << "Gyro bias estimation: frame " << (i + 1)
            << "\ncamera delta_R_ij: \n"
            << bkm1_R_bk << "\npim delta_R_ij: \n"
            << bkm1_gamma_bk << "\nrelative rotation error: \n"
            << rot_error.toRotationMatrix() << "\ndR rotatio error: \n"
            << tmp_b << "\nsub-jacobian (tmp_A): \n"
            << tmp_A << "\nrotation error (temp_b): \n"
            << tmp_b;

    VLOG(5) << "NavState: frame " << (i + 1) << frames.at(i).navstate();
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
    double dt = frames.at(i).dt();
    gtsam::Matrix bk_R_v = frames.at(i).prev_R_mat().transpose();
    gtsam::Vector bk_alpha_bkp1 =
        frames.at(i).delta_p();                   // TODO(Sandro): Inverse???
    gtsam::Vector v_p_bk = frames.at(i).prev_p(); // TODO(Sandro): Inverse???
    gtsam::Vector v_p_bkp1 = frames.at(i).p();    // TODO(Sandro): Inverse???
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
void OnlineGravityAlignment::constructFrames(
    const AlignmentPoses &estimated_body_poses, const AlignmentPims &pims,
    AlignmentFrames *frames) {

  frames->clear();

  // Create initialization frames
  for (int i = 0; i < pims.size(); i++) {
    gtsam::NavState delta_state(pims.at(i).deltaXij());
    const double delta_time = pims.at(i).deltaTij();
    AlignmentFrame frame_i(estimated_body_poses.at(i + 1),
                           estimated_body_poses.at(i), delta_state, delta_time);
    frames->push_back(frame_i);
  }
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
