/*
 * Vio_sw_plain.cpp
 *
 *  Created on: Feb 17, 2017
 *      Author: zhangzd
 */

#include "Vio_sw_plain.h"

///////////////////////////////////////////////////////////////////////////////
void VioPlain::initializeStateAndSetPriors(MatrixSW initial_pose,
    MatrixSW initial_velocity, MatrixSW initial_imu_bias) {


  // Initialize
  cur_id_ = 0;

  // Reset it to maximum number of features
  vision_measurements_.resize(params_.max_num_feature_tracks);
  vision_measurements_active.resize(params.max_num_feature_tracks, false);
  pt_begin_ = -1;
  pt_end_ = -1;

  // Set the initial value (for the initial prior)
  initial_pose_ = initial_pose;
  initial_velocity_ = initial_velocity;
  initial_bias_ = initial_imu_bias;

  // initialize the priors
  no_motion_factor_.reshape(params_.horizon, false);
  zero_velocity_prior_.reshape(params_.horizon, false);

  // updatable keys?
  is_updatable_.reshape(params_.horizon, true);
  is_updatable_[0] = true;

  // states
  poses_.reshape(params_.horizon);
  velocities_.reshape(params_.horizon);
  biases_.reshape(params_.horizon);

  poses_[0] = initial_pose;
  velocities_[0] = initial_velocity;
  biases_[0] = initial_imu_bias;
}

///////////////////////////////////////////////////////////////////////////////
void VioPlain::addVisualInertialStateAndOptimize(bool tracker_valid,
    vector<std::pair<int, MatrixSW>> measurements, ImuInfo& imu) {

  // Increase the current id!
  cur_id_++;

  // There should always be a slot available!

  // Index of the empty slot
  int id_in_track;
  if (cur_id_ >= params_.horizon) {
    id_in_track = params_.horizon - 1;
  } else {
    id_in_track = cur_id_;
  }

  // Add the imu to the factor
  imu_factors_[id_in_track - 1] = imu;

  // Add no motion factor, and zero velocity prior depending on the state
  if (!tracker_valid) {
    no_motion_factor_[id_in_track - 1] = true;
    zero_velocity_prior_[id_in_track - 1] = true;
  }

  // I will ALWAYS add the landmarks to the measurement banks!
  for (int i = 0; i < measurements.size(); i++) {
    int id = measurements[i].first;
    MatrixSW& m = measurements[i].second;

    // new management
    if (vision_measurements_active_[id]) {
      // old landmark!
      vision_measurements_[id].second.push_back(m);

    } else {
      // new landmark!
      vision_measurements_active_[id] = true;
      vision_measurements_[id].first = cur_id_;
      vision_measurements_[id].second.empty();
      vision_measurements_[id].second.push_back(m);

      if (pt_end_ < 0) {
        // first measurement
        pt_begin_ = 0;
        pt_end_ = id;
      } else {
        pt_end_ = max(pt_end_, id);
        // TODO: Add the wrapped features!!!
      }
    }
  }

  // Call the optimization engine
  optimize();

  // Slide and marginalize
  marginalize();

}

///////////////////////////////////////////////////////////////////////////////
void optimize() {
  int matrix_dim = params_.horizon * 15;
  MatrixSW H_all(matrix_dim, matrix_dim);
  MatrixSW b_all(matrix_dim, 1);

  // Linearize all vision factors
  // TODO: Add the wrapped feature!
  for (int pt = pt_begin_; pt <= pt_end_; pt++) {
    if (vision_measurements_active_[pt]) {
      // Call the linearize and accumulate!
      int track_start_t = vision_measurements_[id].first;
      MatrixSW AH_sub = vision_linearize(vision_measurements_[pt].second, poses_,
          track_start_t, cur_id, params_);
      // Update the main matrix
      update_hessian_vision(H_all, b_all, AH_sub, vision_measurements_active_[pt].first);
    }
  }

  // Linearize all of the IMU factors
  for (int pt = 0; pt < min(params_.horizon, cur_id_) - 1; pt++) {
    // TODO!
    MatrixSW AH_sub = IMU_linearize(imu_factors_[pt], biases_[pt]);
    update_hessian_IMU(H_all, b_all, AH_sub, pt);
  }

  // Linearize no motion factor
  for (int pt = 0; pt < min(params_.horizon, cur_id_) - 1; pt++) {
    if (no_motion_factor_[pt]) {
      // TODO!
      MatrixSW AH_sub = no_motion_linearize(poses_[pt], poses_[pt + 1], params_);
      update_hessian_no_motion(H_all, b_all, AH_sub, pt);
    }
  }

  // Linearize zero velocity prior
  for (int pt = 0; pt < min(params_.horizon, cur_id_) - 1; pt++) {
    if (zero_velocity_prior_factor_[pt]) {
      // TODO!
      MatrixSW AH_sub = zero_velocity_linearize(velocities_[pt], params_);
      update_hessian_zero_velocity(H_all, b_all, AH_sub, pt);
    }
  }

  // Linearize the initial prior
  if (cur_id_ < params_.horizon) {
    MatrixSW AH_pose = initial_pose_linearize(poses_[0],
        initial_pose_, params_);
    MatrixSW AH_vel = initial_velocity_linearize(velocities_[0],
        initial_velocity_, params_);
    MatrixSW AH_bias = initial_bias_linearize(biases_[0], initial_bias_, params_);
    update_hessian_initial(H_all, b_all, AH_pose, AH_vel, AH_bias, params_);
  }

  // TODO: Linearize marginal factors
  if (cur_id_ >= params_.horizon) {
    // Linearize the marginal factors!!!
  }

  // Call the solver!
  MatrixSW state_update = linear_solve(H_all, b_all);

  // Retract!
  retract(state_update);

  // Marginalize
  // This function will marginalize the unneeded factors away,
  // and update the factor graph!

  // marginalize();
}
