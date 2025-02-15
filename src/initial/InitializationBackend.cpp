/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   InitializationBackend.cpp
 * @brief  Derived class from VioBackend for bundle adjustment and alignment
 * for the online initialization
 * @author Antoni Rosinol
 * @author Sandro Berchier
 * @author Luca Carlone
 */

#include "kimera-vio/initial/InitializationBackend.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/initial/OnlineGravityAlignment.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsNumerical.h"

namespace VIO {

/* -------------------------------------------------------------------------- */
InitializationBackend::InitializationBackend(
    const gtsam::Pose3& B_Pose_leftCam,
    const StereoCalibPtr& stereo_calibration,
    const BackendParams& backend_params,
    const ImuParams& imu_params,
    const BackendOutputParams& backend_output_params,
    const bool log_output)
    : VioBackend(B_Pose_leftCam,
                 stereo_calibration,
                 backend_params,
                 imu_params,
                 backend_output_params,
                 log_output) {}

/* ------------------------------------------------------------------------ */
// Perform Bundle-Adjustment and initial gravity alignment
bool InitializationBackend::bundleAdjustmentAndGravityAlignment(
    InitializationQueue& output_frontend,
    gtsam::Vector3* gyro_bias,
    gtsam::Vector3* g_iter_b0,
    gtsam::NavState* init_navstate) {
  // Logging
  VLOG(10) << "N frames for initial alignment: " << output_frontend.size();
  // Create inputs for Backend
  std::vector<BackendInput::UniquePtr> inputs_backend;

  // Create inputs for online gravity alignment
  std::vector<ImuFrontend::PimPtr> pims;
  std::vector<double> delta_t_camera;
  // Iterate and fill Backend input vector
  while (!output_frontend.empty()) {
    // Create input for Backend
    const InitializationInputPayload& init_input_payload =
        *(*output_frontend.front());
    inputs_backend.push_back(std::make_unique<BackendInput>(
        init_input_payload.stereo_frame_lkf_.timestamp_,
        init_input_payload.status_stereo_measurements_,
        init_input_payload.pim_,
        init_input_payload.imu_acc_gyrs_));
    pims.push_back(init_input_payload.pim_);
    // Bookkeeping for timestamps
    Timestamp timestamp_kf = init_input_payload.stereo_frame_lkf_.timestamp_;
    delta_t_camera.push_back(
        UtilsNumerical::NsecToSec(timestamp_kf - timestamp_lkf_));
    timestamp_lkf_ = timestamp_kf;

    // Check that all frames are keyframes (required)
    CHECK(init_input_payload.is_keyframe_);
    // Pop from queue
    output_frontend.pop();
  }

  // TODO(Sandro): Bundle-Adjustment is not super robust and accurate!!!
  // Run initial Bundle Adjustment and retrieve body poses
  // wrt. to initial body frame (b0_T_bk, for k in 0:N).
  // The first pim and ransac poses are lost, as in the Bundle
  // Adjustment we need observations of landmarks intra-frames.
  auto tic_ba = utils::Timer::tic();
  const std::vector<gtsam::Pose3>& estimated_poses =
      addInitialVisualStatesAndOptimize(inputs_backend);
  auto ba_duration =
      utils::Timer::toc<std::chrono::nanoseconds>(tic_ba).count() * 1e-9;
  LOG(WARNING) << "Current bundle-adjustment duration: (" << ba_duration
               << " s).";
  // Remove initial delta time and pims from input vector to online
  // alignment due to the disregarded init values in bundle adjustment
  delta_t_camera.erase(delta_t_camera.begin());
  pims.erase(pims.begin());
  // Logging
  LOG(INFO) << "Initial bundle adjustment terminated.";

  // Run initial visual-inertial alignment(OGA)
  OnlineGravityAlignment initial_alignment(
      estimated_poses, delta_t_camera, pims, imu_params_.n_gravity_);
  auto tic_oga = utils::Timer::tic();
  bool is_success = initial_alignment.alignVisualInertialEstimates(
      gyro_bias, g_iter_b0, init_navstate, true);
  auto alignment_duration =
      utils::Timer::toc<std::chrono::nanoseconds>(tic_oga).count() * 1e-9;
  LOG(WARNING) << "Current alignment duration: (" << alignment_duration
               << " s).";

  // TODO(Sandro): Check initialization against GT
  // Compute performance and Log output if requested
  /*if (FLAGS_log_output && is_success) {
    std::string reason = "BA performance comparison";
    ETHDatasetParser gt_dataset(reason);
    std::vector<Timestamp> timestamps;
    for (int i = 0; i < output_frontend.size(); i++) {
      timestamps.push_back(
          output_frontend.at(i)->stereo_frame_lkf_.timestamp_);
    }
    gt_dataset.parseGTdata("/home/sb/Dataset/EuRoC/V1_01_gt", "gt");
    const gtsam::NavState init_navstate_pass = *init_navstate;
    const gtsam::Vector3 gravity_iter_pass = *g_iter_b0;
    const gtsam::Vector3 gyro_bias_pass = *gyro_bias;
    logger_.logInitializationResultsCSV(
          gt_dataset.getInitializationPerformance(
            timestamps,
            estimated_poses,
            gtNavState(init_navstate_pass.pose(),
              init_navstate_pass.pose().rotation().transpose() *
                init_navstate_pass.velocity(),
              gtsam::imuBias::ConstantBias(gtsam::Vector3(),
                                          gyro_bias_pass)),
            gravity_iter_pass),
            ba_duration,
            alignment_duration,
            is_success);
  } */
  ////////////////// (Remove)

  return is_success;
}

/* -------------------------------------------------------------------------- */
std::vector<gtsam::Pose3>
InitializationBackend::addInitialVisualStatesAndOptimize(
    const std::vector<BackendInput::UniquePtr>& input) {
  CHECK(input.front());

  // Initial clear values.
  new_values_.clear();
  // Initialize to trivial pose for initial bundle adjustment
  W_Pose_B_lkf_from_state_ = gtsam::Pose3();
  W_Pose_B_lkf_from_increments_ = gtsam::Pose3();

  // Insert relative poses for bundle adjustment
  for (size_t i = 0; i < input.size(); i++) {
    const BackendInput& input_iter = *input[i];
    VLOG(5) << "Adding initial visual state.";
    // Features and IMU line up --> do iSAM update
    CHECK(input_iter.status_stereo_measurements_kf_);
    addInitialVisualState(
        input_iter.timestamp_,  // Current time for fixed lag smoother.
        *input_iter.status_stereo_measurements_kf_,  // Vision data.
        0);
    last_kf_id_ = curr_kf_id_;
    ++curr_kf_id_;
  }

  VLOG(10) << "Initialisation states added.";

  // Add all landmarks to factor graph
  LandmarkIds landmarks_all_keyframes;
  for (const auto& keyTrack_j : feature_tracks_) {
    landmarks_all_keyframes.push_back(keyTrack_j.first);
  }

  addLandmarksToGraph(landmarks_all_keyframes);

  VLOG(10) << "Initialisation landmarks added.";

  // Perform Bundle Adjustment and retrieve body poses (b0_T_bk)
  // b0 is the initial body frame
  std::vector<gtsam::Pose3> estimated_poses = optimizeInitialVisualStates(
      input.front()->timestamp_, curr_kf_id_, backend_params_.numOptimize_);

  VLOG(10) << "Initial bundle adjustment completed.";

  // All relative to initial pose, as we need to fix x0 from the BA.
  gtsam::Pose3 initial_pose;
  for (size_t j = 0; j < estimated_poses.size(); j++) {
    if (j == 0) {
      initial_pose = estimated_poses.at(0);
      estimated_poses.at(0) = gtsam::Pose3();
    } else {
      estimated_poses.at(j) = initial_pose.between(estimated_poses.at(j));
    }
    if (VLOG_IS_ON(10)) estimated_poses.at(j).print();
  }
  // Return poses (b0_T_bk, for k in 0:N).
  // Since we need to optimize for poses with observed landmarks, the
  // ransac estimate for the first pose is not used, as there are no
  // observations for the previous keyframe (which doesn't exist).
  CHECK_EQ(input.size(), estimated_poses.size());
  return estimated_poses;
}

/* -------------------------------------------------------------------------- */
// Adding of states for bundle adjustment used in initialization.
// [in] timestamp_kf_nsec, keyframe timestamp.
// [in] status_smart_stereo_measurements_kf, vision data.
// [in] stereo_ransac_body_pose, inertial data.
void InitializationBackend::addInitialVisualState(
    const Timestamp& timestamp_kf_nsec,
    const StatusStereoMeasurements& status_smart_stereo_measurements_kf,
    const int verbosity_ = 0) {
  debug_info_.resetAddedFactorsStatistics();

  VLOG(10) << "Initialization: adding keyframe " << curr_kf_id_
           << " at timestamp:" << UtilsNumerical::NsecToSec(timestamp_kf_nsec)
           << " (nsec).";

  /////////////////// MANAGE IMU MEASUREMENTS ///////////////////////////
  // Predict next step, add initial guess
  if (status_smart_stereo_measurements_kf.first.kfTrackingStatus_stereo_ ==
          TrackingStatus::VALID &&
      curr_kf_id_ != 0) {
    // We need to keep adding the relative poses, since we process a
    // whole batch. Otherwise we start with wrong initial guesses.
    gtsam::Pose3 B_lkf_Pose_B_k =
        B_Pose_leftCamRect_ *
        status_smart_stereo_measurements_kf.first.lkf_T_k_stereo_ *
        B_Pose_leftCamRect_.inverse();
    W_Pose_B_lkf_from_state_ = W_Pose_B_lkf_from_state_.compose(B_lkf_Pose_B_k);
    new_values_.insert(gtsam::Symbol('x', curr_kf_id_),
                       W_Pose_B_lkf_from_state_);

    if (backend_params_.addBetweenStereoFactors_ &&
        status_smart_stereo_measurements_kf.first.kfTrackingStatus_stereo_ ==
            TrackingStatus::VALID) {
      addBetweenFactor(
          last_kf_id_,
          curr_kf_id_,
          // I think this should be B_Pose_leftCamRect_...
          B_Pose_leftCamRect_ *
              status_smart_stereo_measurements_kf.first.lkf_T_k_stereo_ *
              B_Pose_leftCamRect_.inverse(),
          backend_params_.betweenRotationPrecision_,
          backend_params_.betweenTranslationPrecision_);
    }
  } else {
    new_values_.insert(gtsam::Symbol('x', curr_kf_id_),
                       W_Pose_B_lkf_from_state_);
  }

  /////////////////// MANAGE VISION MEASUREMENTS ///////////////////////////
  const StereoMeasurements& smartStereoMeasurements_kf =
      status_smart_stereo_measurements_kf.second;

  // extract relevant information from stereo frame
  LandmarkIds landmarks_kf;
  addStereoMeasurementsToFeatureTracks(
      curr_kf_id_, smartStereoMeasurements_kf, &landmarks_kf);

  // Add zero velocity update if no-motion detected
  TrackingStatus kfTrackingStatus_mono =
      status_smart_stereo_measurements_kf.first.kfTrackingStatus_mono_;
  if (kfTrackingStatus_mono == TrackingStatus::LOW_DISPARITY &&
      curr_kf_id_ != 0) {
    VLOG(10) << "No-motion factor added in Bundle-Adjustment.\n";
    LOG(WARNING) << "No-motion factor added in Bundle-Adjustment.\n";
    addNoMotionFactor(last_kf_id_, curr_kf_id_);
  }

  if (verbosity_ >= 8) {
    printFeatureTracks();
  }
}

/* -------------------------------------------------------------------------- */
// TODO(Toni): do not return vectors...
std::vector<gtsam::Pose3> InitializationBackend::optimizeInitialVisualStates(
    const Timestamp& timestamp_kf_nsec,
    const FrameId& cur_id,
    const size_t& max_extra_iterations,
    const std::vector<size_t>& extra_factor_slots_to_delete,
    const int verbosity_) {  // TODO: Remove verbosity and use VLOG

  // Reset all timing info.
  debug_info_.resetTimes();

  // Create and fill non-linear graph
  gtsam::NonlinearFactorGraph new_factors_tmp;
  for (const auto& new_smart_factor : new_smart_factors_) {
    // Push back the smart factor to the list of new factors to add to the
    // graph.
    new_factors_tmp.push_back(
        new_smart_factor.second);  // Smart factor, so same address right?
    // VLOG(10) << "Iteration: " << new_smart_factor.first;
  }

  // Add also other factors (imu, priors).
  // SMART FACTORS MUST BE FIRST, otherwise when recovering the slots
  // for the smart factors we will mess up.
  new_factors_tmp.push_back(new_imu_prior_and_other_factors_.begin(),
                            new_imu_prior_and_other_factors_.end());

  // Print graph before optimization
  // TODO(Sandro): Change back verbosity level!
  if (VLOG_IS_ON(2)) new_factors_tmp.print();

  // Levenberg-Marquardt optimization
  gtsam::LevenbergMarquardtParams lmParams;
  gtsam::LevenbergMarquardtOptimizer initial_bundle_adjustment(
      new_factors_tmp, new_values_, lmParams);
  VLOG(10) << "LM optimizer created with error: "
           << initial_bundle_adjustment.error();

  // Optimize and get values
  gtsam::Values initial_values = initial_bundle_adjustment.optimize();
  VLOG(10) << "Levenberg Marquardt optimizer done.";
  // Query optimized poses in body frame (b0_T_bk)
  std::vector<gtsam::Pose3> initial_states;
  for (const auto& key_value : initial_values) {
    initial_states.push_back(initial_values.at<gtsam::Pose3>(key_value.key));
  }
  VLOG(10) << "Initialization values retrieved.";

  // TODO(Sandro): Implement check on initial and final covariance
  // Quality check on Bundle-Adjustment
  LOG(INFO) << "Initial states retrieved.\n";
  /*std::vector<gtsam::Matrix> initial_covariances;
  gtsam::Marginals marginals(new_factors_tmp, initial_values,
      gtsam::Marginals::Factorization::QR);
  //  gtsam::Marginals::Factorization::CHOLESKY);
  initial_covariances.push_back(
          marginals.marginalCovariance(gtsam::Symbol('x', 0)));
  //initial_covariances.push_back(
  //        marginals.marginalCovariance(gtsam::Symbol('x', curr_kf_id_)));
  LOG(INFO) << "Initial state covariance: (x, y, z)\n"
            << gtsam::sub(initial_covariances.at(0), 3, 6, 3, 6);
  //CHECK(gtsam::assert_equal(initial_covariance, final_covariance, 1e-2));
  */

  /////////////////////////// BOOKKEEPING //////////////////////////////////////

  // Reset everything for next round.
  VLOG(10) << "Clearing new_smart_factors_!";
  new_smart_factors_.clear();
  old_smart_factors_.clear();
  // Reset list of new imu, prior and other factors to be added.
  // TODO could this be used to check whether we are repeating factors?
  new_imu_prior_and_other_factors_.resize(0);
  // Clear values.
  new_values_.clear();

  return initial_states;
}

}  // namespace VIO
