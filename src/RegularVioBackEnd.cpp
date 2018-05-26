/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RegularVioBackEnd.h
 * @brief  Derived class from VioBackEnd which enforces regularity constraints
 * on the factor graph.
 * @author Toni Rosinol
 */

#include "RegularVioBackEnd.h"

#include <gtsam/slam/ProjectionFactor.h>

#include "factors/PointPlaneFactor.h"

#include <gtsam/slam/PriorFactor.h>

namespace VIO {

/* -------------------------------------------------------------------------- */
RegularVioBackEnd::RegularVioBackEnd(
    const Pose3& leftCamPose,
    const Cal3_S2& leftCameraCalRectified,
    const double& baseline,
    const VioBackEndParams& vioParams) :
  VioBackEnd(leftCamPose,
             leftCameraCalRectified,
             baseline,
             vioParams) {
  LOG(INFO) << "Using Regular VIO backend.\n";

  // Set type of mono_noise_ for generic projection factors.
  gtsam::SharedNoiseModel gaussian_dim_2 =
      gtsam::noiseModel::Isotropic::Sigma(2, vio_params_.monoNoiseSigma_);

  mono_noise_ = gtsam::noiseModel::Robust::Create(
                  gtsam::noiseModel::mEstimator::Huber::Create(
                    vio_params_.huberParam_,
                    gtsam::noiseModel::mEstimator::Huber::Scalar), // Default is Block
                  gaussian_dim_2);

  // Set type of stereo_noise_ for generic stereo projection factors.
  gtsam::SharedNoiseModel gaussian_dim_3 =
      gtsam::noiseModel::Isotropic::Sigma(3, vio_params_.stereoNoiseSigma_);

  stereo_noise_ = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(
                      vio_params_.huberParam_,
                      gtsam::noiseModel::mEstimator::Huber::Scalar), // Default is Block
                    gaussian_dim_3);

  // Set type of regularity noise for point plane factors.
  gtsam::SharedNoiseModel gaussian_dim_1 =
      gtsam::noiseModel::Isotropic::Sigma(1, vio_params_.regularityNoiseSigma_);

  switch (vio_params_.normType_) {
    case 0: {
      LOG(INFO) << "Using square norm.";
      point_plane_regularity_noise_ = gaussian_dim_1;
      break;
    }
    case 1: {
      LOG(INFO) << "Using Huber norm, with parameter value: " << vio_params_.huberParam_;
      point_plane_regularity_noise_ =
          gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(
              vio_params_.huberParam_,
              gtsam::noiseModel::mEstimator::Huber::Scalar), // Default is Block
            gaussian_dim_1);
      break;
    }
    case 2: {
      LOG(INFO) << "Using Tukey norm, with parameter value: " << vio_params_.tukeyParam_;
      point_plane_regularity_noise_ =
          gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Tukey::Create(
              vio_params_.tukeyParam_,
              gtsam::noiseModel::mEstimator::Tukey::Scalar), // Default is Block
            gaussian_dim_1); //robust
      break;
    }
    default: {
      LOG(INFO) << "Using square norm.";
      point_plane_regularity_noise_ = gtsam::noiseModel::Isotropic::Sigma(
                                        1, vio_params_.regularityNoiseSigma_);

      break;
    }
  }

  mono_cal_ = boost::make_shared<Cal3_S2>(stereo_cal_->calibration());
  CHECK(mono_cal_->equals(stereo_cal_->calibration()))
      << "Monocular calibration should match Stereo calibration";
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::addVisualInertialStateAndOptimize(
    const Timestamp& timestamp_kf_nsec,
    const StatusSmartStereoMeasurements& status_smart_stereo_measurements_kf,
    const ImuStamps& imu_stamps, const ImuAccGyr& imu_accgyr,
    const LandmarkIds& mesh_lmk_ids_ground_cluster,
    boost::optional<gtsam::Pose3> stereo_ransac_body_pose) {

  debug_info_.resetAddedFactorsStatistics();

  if (VLOG_IS_ON(20)) {
    StereoVisionFrontEnd::PrintStatusStereoMeasurements(
                                          status_smart_stereo_measurements_kf);
  }

  // Features and IMU line up --> do iSAM update.
  last_kf_id_ = cur_kf_id_;
  ++cur_kf_id_;

  timestamp_kf_ = UtilsOpenCV::NsecToSec(timestamp_kf_nsec);

  VLOG(7) << "Processing keyframe " << cur_kf_id_
          << " at timestamp: " << timestamp_kf_ << " (sec)\n";

  /////////////////// IMU FACTORS //////////////////////////////////////////////
  // Predict next step, add initial guess.
  integrateImuMeasurements(imu_stamps, imu_accgyr);
  addImuValues(cur_kf_id_);

  // Add imu factors between consecutive keyframe states.
  VLOG(10) << "Adding IMU factor between pose id: " << last_kf_id_
          << " and pose id: " << cur_kf_id_;
  addImuFactor(last_kf_id_, cur_kf_id_);

  /////////////////// STEREO RANSAC FACTORS ////////////////////////////////////
  // Add between factor from RANSAC.
  if (stereo_ransac_body_pose) {
    VLOG(10) << "Adding RANSAC factor between pose id: " << last_kf_id_
            << " and pose id: " << cur_kf_id_;
    if (VLOG_IS_ON(20)) {
      stereo_ransac_body_pose->print();
    }
    addBetweenFactor(last_kf_id_, cur_kf_id_, *stereo_ransac_body_pose);
  }

  /////////////////// VISION MEASUREMENTS //////////////////////////////////////
  static constexpr bool convert_extra_smart_factors_to_proj_factors = true;
  static constexpr bool remove_old_reg_factors = true;
  const SmartStereoMeasurements& smart_stereo_measurements_kf =
                                    status_smart_stereo_measurements_kf.second;

  // Extract relevant information from stereo frame.
  // Get the landmarks visible in current keyframe. (These are not all the lmks
  // in time horizon used for the optimization!)
  LandmarkIds lmks_kf;
  addStereoMeasurementsToFeatureTracks(
        cur_kf_id_,
        smart_stereo_measurements_kf,
        &lmks_kf);

  if (VLOG_IS_ON(20)) {
    printFeatureTracks();
  }

  // Decide which factors to add.
  Tracker::TrackingStatus kfTrackingStatus_mono =
                status_smart_stereo_measurements_kf.first.kfTrackingStatus_mono_;

  std::vector<size_t> delete_old_regularity_factors;
  switch(kfTrackingStatus_mono) {
    case Tracker::TrackingStatus::LOW_DISPARITY : {
      // Vehicle is not moving.
      VLOG(10) << "Tracker has a LOW_DISPARITY status.";
      VLOG(10) << "Add zero velocity and no motion factors.";
      addZeroVelocityPrior(cur_kf_id_);
      addNoMotionFactor(last_kf_id_, cur_kf_id_);
      // TODO why are we not adding the regularities here as well...?
      break;
    }
    default: {
      kfTrackingStatus_mono == Tracker::TrackingStatus::VALID?
          VLOG(10) << "Tracker has a VALID status.":
            kfTrackingStatus_mono == Tracker::TrackingStatus::FEW_MATCHES?
          VLOG(10) << "Tracker has a FEW_MATCHES status.":
            kfTrackingStatus_mono == Tracker::TrackingStatus::INVALID?
          VLOG(10) << "Tracker has a INVALID status.":
            kfTrackingStatus_mono == Tracker::TrackingStatus::DISABLED?
          VLOG(10) << "Tracker has a DISABLED status.": VLOG(10) << "";

      if (kfTrackingStatus_mono == Tracker::TrackingStatus::VALID) {
        // We add features in VIO.
        VLOG(10) << "Starting adding/updating landmarks to graph...";
        addLandmarksToGraph(lmks_kf,
                            mesh_lmk_ids_ground_cluster);
        VLOG(10) << "Finished adding/updating landmarks to graph.";

        // Convert all smart factors of lmks in time horizon that have
        // regularities to projection factors.
        // Most conversions from smart to proj are done before,
        // in addLandmarksToGraph, but here we also make sure we have converted
        // the ones with regularities in time horizon.
        if (convert_extra_smart_factors_to_proj_factors) {
          VLOG(10) << "Starting converting extra smart factors to proj factors...";
          convertExtraSmartFactorToProjFactor(mesh_lmk_ids_ground_cluster);
          VLOG(10) << "Finished converting extra smart factors to proj factors...";
        }

        /////////////////// REGULARITY FACTORS ///////////////////////////////////////
        // Add regularity factor on vertices of the mesh.
        // TODO argument should be generalized to diff
        // type of cluster and regularities.
        gtsam::Symbol plane_symbol;
        std::vector<std::pair<Slot, LandmarkId>> idx_of_point_plane_factors_to_add;
        if (mesh_lmk_ids_ground_cluster.size() != 0) {
          // TODO what happens if mesh has same ids over and over, are we duplicating
          // factors?
          VLOG(10) << "Adding regularity factors.";
          addRegularityFactors(mesh_lmk_ids_ground_cluster,
                               &plane_symbol,
                               &idx_of_point_plane_factors_to_add);
          VLOG(10) << "Finished adding regularity factors.";
        } else {
          VLOG(10) << "The clustered mesh on the ground is empty, skipping "
                      "addition of regularity factors.";
        }

        if (remove_old_reg_factors) {
          VLOG(10) << "Removing old regularity factors.";
          removeOldRegularityFactors_Slow(plane_symbol,
                                          idx_of_point_plane_factors_to_add,
                                          mesh_lmk_ids_ground_cluster,
                                          &delete_old_regularity_factors);
          VLOG(10) << "Finished removing old regularity factors.";
        }
      }
      break;
    }
  }

  /////////////////// OPTIMIZE /////////////////////////////////////////////////
  // This lags 1 step behind to mimic hw.
  imu_bias_prev_kf_ = imu_bias_lkf_;

  std::vector<size_t> delete_slots (delete_slots_of_converted_smart_factors_);
  delete_slots.insert(delete_slots.end(),
                      delete_old_regularity_factors.begin(),
                      delete_old_regularity_factors.end());
  // TODO add conversion from Smart factor to regular.
  optimize(cur_kf_id_, vio_params_.numOptimize_,
           delete_slots);

  // Reset list of factors to delete.
  // These are the smart factors that have been converted to projection factors
  // and must be deleted from the factor graph.
  delete_slots_of_converted_smart_factors_.resize(0);
}

/* -------------------------------------------------------------------------- */
// TODO Virtualize this appropriately,
void RegularVioBackEnd::addLandmarksToGraph(
    const LandmarkIds& lmks_kf,
    const LandmarkIds& mesh_lmk_ids_ground_cluster) {
  // Add selected landmarks to graph:
  size_t n_new_landmarks = 0;
  size_t n_updated_landmarks = 0;
  debug_info_.numAddedSmartF_ += lmks_kf.size();

  // Iterate over all landmarks in current key frame.
  for (const LandmarkId& lmk_id: lmks_kf) {
    CHECK(feature_tracks_.find(lmk_id) != feature_tracks_.end());
    FeatureTrack& feature_track = feature_tracks_.at(lmk_id);

    // Only insert feature tracks of length at least 2
    // (otherwise uninformative)
    static constexpr size_t min_num_of_observations = 2;
    if (feature_track.obs_.size() >= min_num_of_observations) {
      // We have enough observations of the lmk.
      if (!feature_track.in_ba_graph_) {
        // The lmk has not yet been added to the graph.
        VLOG(20) << "Adding lmk " << lmk_id << " to graph.";
        addLandmarkToGraph(lmk_id, feature_track);
        // Acknowledge that we have added the landmark in the graph.
        feature_track.in_ba_graph_ = true;
        ++n_new_landmarks;
      } else {
        //The lmk has already been added to the graph.
        CHECK_GE(feature_track.obs_.size(), 1);
        const std::pair<FrameId, StereoPoint2>& obs_kf =
            feature_track.obs_.back();

        // Sanity check.
        CHECK_EQ(obs_kf.first, cur_kf_id_) << "Last obs is not from the current"
                                              " keyframe!";

        // For each landmark we decide if it's going to be a smart factor or not.
        // TODO here there is a timeline mismatch, while landmarks_kf are only currently
        // visible lmks, mesh_lmks_ids contains lmks in time_horizon!
        const bool& is_lmk_smart = isLandmarkSmart(lmk_id,
                                                   mesh_lmk_ids_ground_cluster,
                                                   &lmk_id_is_smart_);

        VLOG(20) << "Updating lmk " << lmk_id << " to graph.";
        updateLandmarkInGraph(lmk_id, is_lmk_smart, obs_kf);
        ++n_updated_landmarks;
      }
    } else {
      VLOG(20) << "Feature track is shorter (" << feature_track.obs_.size()
               << ") than min_num_of_observations (" << min_num_of_observations
               << ") for lmk with id: " << lmk_id;
    }
  }

  // Convert to projection factors those landmarks that are not in the current
  // key frame, but that have a regularity.
  VLOG(10) << "Added " << n_new_landmarks << " new landmarks.\n"
           << "Updated " << n_updated_landmarks << " landmarks in graph.";
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::addLandmarkToGraph(const LandmarkId& lmk_id,
                                           const FeatureTrack& ft) {
  // All landmarks should be smart the first time we add them to the graph.
  // Add as a smart factor.
  // We use a unit pinhole projection camera for the smart factors to be
  // more efficient.
  SmartStereoFactor::shared_ptr new_factor =
      boost::make_shared<SmartStereoFactor>(smart_noise_,
                                            smart_factors_params_,
                                            B_Pose_leftCam_);

  VLOG(20) << "Adding landmark with id: " << lmk_id
           << " for the first time to graph. \n"
           << "Nr of observations of the lmk: " << ft.obs_.size()
           << " observations.\n";
  if (VLOG_IS_ON(30)) {
    new_factor->print();
  }

  // Add observations to smart factor.
  VLOG(20) << "Creating smart factor involving lmk with id: " << lmk_id;
  for (const std::pair<FrameId, StereoPoint2>& obs: ft.obs_) {
    VLOG(20) << "SmartFactor: adding observation of lmk with id: " << lmk_id
            << " from frame with id: " << obs.first;

    new_factor->add(obs.second,
                    gtsam::Symbol('x', obs.first),
                    stereo_cal_);
  }

  /////////////////////// BOOK KEEPING /////////////////////////////////////////

  // Add new factor to suitable structures.
  // TODO why do we need to store the new_factor in both structures??
  new_smart_factors_.insert(std::make_pair(lmk_id,
                                           new_factor));
  old_smart_factors_.insert(std::make_pair(lmk_id,
                                           std::make_pair(new_factor, -1)));
  //////////////////////////////////////////////////////////////////////////////
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::updateLandmarkInGraph(
    const LandmarkId& lmk_id,
    const bool& is_lmk_smart,
    const std::pair<FrameId, StereoPoint2>& new_obs) {
  if (is_lmk_smart) {
    // Lmk is meant to be smart.
    VLOG(20) << "Lmk with id: " << lmk_id << " is set to be smart.\n";

    updateExistingSmartFactor(lmk_id,
                              new_obs,
                              &new_smart_factors_,
                              &old_smart_factors_);
  } else {
    VLOG(20) << "Lmk with id: " << lmk_id
              << " is set to be a projection factor.\n";

    // Update lmk_id as a projection factor.
    gtsam::Key lmk_key = gtsam::Symbol('l', lmk_id).key();
    if (state_.find(lmk_key) == state_.end()) {
      VLOG(20) << "Lmk with id: " << lmk_id << " is not found in state.\n";
      // We did not find the lmk in the state.
      // It was a smart factor before.
      CHECK(old_smart_factors_.exists(lmk_id));
      // Convert smart to projection.
      bool is_conversion_done = convertSmartToProjectionFactor(
                                  lmk_id,
                                  &old_smart_factors_,
                                  &new_values_,
                                  &new_imu_prior_and_other_factors_,
                                  &delete_slots_of_converted_smart_factors_);
      // Unless we could not convert the smart factor to a set of projection
      // factors, then do not add it because we do not have a right value
      // to use.
      if (is_conversion_done) {
        VLOG(20) << "Lmk with id: " << lmk_id
                 << " added as a new projection factor with pose with id: "
                 << new_obs.first << ".\n";
        addProjectionFactor(lmk_id,
                            new_obs,
                            &new_imu_prior_and_other_factors_);
        // Sanity check, if the conversion was successful, then we should
        // not be able to see the smart factor anymore.
        CHECK(!old_smart_factors_.exists(lmk_id));
      } else {
        LOG(ERROR) << "Not using new observation for lmk: " << lmk_id
                   << " because we do not have a good initial value for it.";
      }
    } else {
      VLOG(20) << "Lmk with id: " << lmk_id << " has been found in state: "
               << "it is being used in a projection factor.\n"
               << "Adding lmk as a new projection factor with pose with id: "
               << new_obs.first << ".\n";
      // If it is not smart, just add current measurement.
      // It was a projection factor before.
      addProjectionFactor(lmk_id,
                          new_obs,
                          &new_imu_prior_and_other_factors_);
    }
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::updateExistingSmartFactor(
    const LandmarkId& lmk_id,
    const std::pair<FrameId, StereoPoint2>& new_obs,
    LandmarkIdSmartFactorMap* new_smart_factors,
    SmartFactorMap* old_smart_factors) {
  CHECK_NOTNULL(new_smart_factors);
  CHECK_NOTNULL(old_smart_factors);

  // Update existing smart-factor.
  const SmartFactorMap::iterator& old_smart_factors_it =
      old_smart_factors->find(lmk_id);
  CHECK(old_smart_factors_it != old_smart_factors->end())
      << "Landmark with id: " << lmk_id << " not found in old_smart_factors_\n";

  // Get old factor.
  SmartStereoFactor::shared_ptr old_factor =
      old_smart_factors_it->second.first;

  // Clone old factor as a new factor.
  CHECK_NOTNULL(old_factor.get());
  SmartStereoFactor::shared_ptr new_factor =
      boost::make_shared<SmartStereoFactor>(*old_factor);

  // Add observation to new factor.
  VLOG(20) << "Added observation for smart factor of lmk with id: "
           << lmk_id;
  new_factor->add(new_obs.second,
                  gtsam::Symbol('x', new_obs.first),
                  stereo_cal_);

  // If slot is still -1, it means that the factor has not been inserted yet
  // in the graph.
  CHECK(old_smart_factors_it->second.second != -1)
      << "When calling update the slot should be already != -1";

  // Slot is different than -1
  // It means that the factor has already been inserted in the graph.
  VLOG(20) << "Insert new smart factor to new_smart_factors_ for lmk with"
           << " id: " << lmk_id;

  ///// Book Keeping, update factors ///////////////////////////////////////////
  // Update the set of new smart factors.
  new_smart_factors->insert(std::make_pair(lmk_id, new_factor));

  // TODO Why do we do this??
  // if we don't the 3d points seem to be off.
  // is there a way to viz 3d points?
  // BUT then we are not pointing to the factor in the graph anymore, and a
  // check such as old_smart_factor_it->second.first ==
  //                graph.at[old_smart_factor_it->second.second
  // will fail!
  old_smart_factors_it->second.first = new_factor;

  //////////////////////////////////////////////////////////////////////////////
}

/* -------------------------------------------------------------------------- */
// Converts a smart factor to a set of projection factors.
// Returns whether the conversion was possible or not.
bool RegularVioBackEnd::convertSmartToProjectionFactor(
    const LandmarkId& lmk_id,
    SmartFactorMap* old_smart_factors,
    gtsam::Values* new_values,
    gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors,
    std::vector<size_t>* delete_slots_of_converted_smart_factors) {
  CHECK_NOTNULL(old_smart_factors);
  CHECK_NOTNULL(new_values);
  CHECK_NOTNULL(new_imu_prior_and_other_factors);
  CHECK_NOTNULL(delete_slots_of_converted_smart_factors);

  const auto& old_smart_factors_it = old_smart_factors->find(lmk_id);
  CHECK(old_smart_factors_it != old_smart_factors->end())
      << "Landmark not found in old_smart_factors_.";

  SmartStereoFactor::shared_ptr old_factor =
      old_smart_factors_it->second.first;
  CHECK_NOTNULL(old_factor.get());

  // Add landmark value to graph.
  VLOG(30) << "Print old_factor of lmk_id: " << lmk_id;
  if (VLOG_IS_ON(30)) {
    old_factor->print();
  }

  // Check triangulation result is initialized.
  CHECK(old_factor->point().is_initialized());

  if (old_factor->point().valid()) {
    VLOG(20) << "Performing conversion for lmk_id: " << lmk_id;

    // TODO check all the * whatever, for segmentation fault!
    gtsam::Key lmk_key = gtsam::Symbol('l', lmk_id).key();
    new_values->insert(lmk_key, *(old_factor->point()));

    // DEBUG add prior to lmks.
    //LOG_EVERY_N(ERROR, 100) << "Do not forget to remove lmk prior!";
    //static const gtsam::noiseModel::Diagonal::shared_ptr prior_lmk_noise =
    //    gtsam::noiseModel::Diagonal::Sigmas(Vector3(1, 1, 1));
    //new_imu_prior_and_other_factors_.push_back(
    //      boost::make_shared<gtsam::PriorFactor<gtsam::Point3> >(
    //        lmk_key,
    //        *(old_factor->point()),
    //        prior_lmk_noise));

    // Convert smart factor to multiple projection factors.
    // There is no need to have a threshold for how many observations we want
    // , since it is done already in the isLandmarkSmart function.
    // Nevertheless, check that there are at least 2 observations.
    CHECK_GE(old_factor->measured().size(), 2);
    for (size_t i = 0; i < old_factor->keys().size(); i++) {
      const gtsam::Symbol& cam_sym = gtsam::Symbol(old_factor->keys().at(i));
      CHECK_LT(i, old_factor->measured().size());
      const StereoPoint2& sp2 = old_factor->measured().at(i);
      std::pair<FrameId, StereoPoint2> obs (std::make_pair(cam_sym.index(),
                                                           sp2));
      VLOG(20) << "Lmk with id: " << lmk_id
               << " added as a new projection factor with pose with id: "
               << static_cast<int>(cam_sym.index()) << ".\n";
      addProjectionFactor(lmk_id,
                          obs,
                          &new_imu_prior_and_other_factors_);
    }

    ////////////////// BOOKKEEPING /////////////////////////////////////////
    // Make sure that the smart factor that we converted to projection
    // gets deleted from the graph.
    if (old_smart_factors_it->second.second != -1) {
      // Get current slot (if factor is already there it must be deleted).
      delete_slots_of_converted_smart_factors->push_back(
            old_smart_factors_it->second.second);
    }

    // Erase from old_smart_factors_ list since this has been converted into
    // projection factors.
    // Check to avoid undefined behaviour.
    CHECK(old_smart_factors->find(lmk_id) != old_smart_factors->end());
    old_smart_factors->erase(lmk_id);
    ////////////////////////////////////////////////////////////////////////
    return true;
  } else {
    LOG(WARNING) << "Cannot convert smart factor to proj. factor.\n"
                 << "Smart factor does not have a valid 3D position for lmk: "
                 << lmk_id << "\n"
                 << "Smart factor point status: \n" << old_factor->point();
    return false;
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::convertExtraSmartFactorToProjFactor(
    const LandmarkIds& mesh_lmk_ids_ground_cluster) {
  for (const LandmarkId& lmk_id: mesh_lmk_ids_ground_cluster) {
    // Track this lmk, if it was not already...
    if (old_smart_factors_.exists(lmk_id) &&
        !isLandmarkSmart(lmk_id,
                         mesh_lmk_ids_ground_cluster,
                         &lmk_id_is_smart_)) {
      // We have found a smart factor that should be a projection factor.
      // Convert it to a projection factor, so that we can enforce
      // regularities on it.
      if (convertSmartToProjectionFactor(
            lmk_id,
            &old_smart_factors_,
            &new_values_,
            &new_imu_prior_and_other_factors_,
            &delete_slots_of_converted_smart_factors_)) {
        VLOG(30) << "Converting smart factor to proj factor for lmk"
                    " with id: " << lmk_id;
      } else {
        VLOG(30) << "NOT converting smart factor to proj factor for lmk"
                    " with id: " << lmk_id;
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::deleteLmkFromExtraStructures(const LandmarkId& lmk_id) {
  if (lmk_id_is_smart_.find(lmk_id) != lmk_id_is_smart_.end()) {
    LOG(WARNING) << "Delete entrance in lmk_id_is_smart_"
                  " for lmk with id: " << lmk_id;
    lmk_id_is_smart_.erase(lmk_id);
  }
  if (lmk_id_to_regularity_type_map_.find(lmk_id) !=
      lmk_id_to_regularity_type_map_.end()) {
  LOG(WARNING) << "Delete entrance in lmk_id_to_regularity_type_map_"
                " for lmk with id: " << lmk_id;
  lmk_id_to_regularity_type_map_.erase(lmk_id);
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::addProjectionFactor(
    const LandmarkId& lmk_id,
    const std::pair<FrameId, StereoPoint2>& new_obs,
    gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors) {
  CHECK_NOTNULL(new_imu_prior_and_other_factors);
  if (!std::isnan(new_obs.second.uR())) {
    double parallax = new_obs.second.uL() - new_obs.second.uR();
    static constexpr double max_parallax = 200;
    if (parallax < max_parallax) {
      CHECK_GT(parallax, 0);
      new_imu_prior_and_other_factors->push_back(
            boost::make_shared<
            gtsam::GenericStereoFactor<Pose3, Point3>>
            (new_obs.second, stereo_noise_,
             gtsam::Symbol('x', new_obs.first),
             gtsam::Symbol('l', lmk_id),
             stereo_cal_, true, true, B_Pose_leftCam_));
    } else {
      LOG(ERROR) << "Parallax for lmk_id: " << lmk_id << " is = "
                 << parallax;
    }
  } else {
    // Right pixel has a NAN value for u, use GenericProjectionFactor instead
    // of stereo.
    new_imu_prior_and_other_factors->push_back(
          boost::make_shared<
          gtsam::GenericProjectionFactor<Pose3, Point3>>
          (gtsam::Point2(new_obs.second.uL(),
                         new_obs.second.v()),
           mono_noise_,
           gtsam::Symbol('x', new_obs.first),
           gtsam::Symbol('l', lmk_id),
           mono_cal_, true, true, B_Pose_leftCam_)
          );
  }
}

/* -------------------------------------------------------------------------- */
bool RegularVioBackEnd::isLandmarkSmart(const LandmarkId& lmk_id,
                                        const LandmarkIds& mesh_lmk_ids,
                                        LmkIdIsSmart* lmk_id_is_smart) {
  // TODOOOOO completely change this function: it should be
  // if the lmk_id is not found in is_lmk_smart
  // then add it as smart
  // else if it was smart, but now it is in regularity, add it as regularity
  // if it was not smart, keep it as such...
  CHECK_NOTNULL(lmk_id_is_smart);

  // WARNING I think this loop should not be over lmks_kf, which are in the
  // current keyframe but over the time horizon instead!!!
  // Otherwise we can have some lmks that are not set as projection factors
  // but will be in the future involved in some kind of regularity...
  // MORE WARNING: this function is tightly coupled with the landmarks we
  // feed to the mesher that then sends mesh_lmk_ids.
  // i.e. if the mesher has lmks that are in the keyframe but not in the
  // optimization, it won't work...
  const auto& lmk_id_slot = lmk_id_is_smart->find(lmk_id);
  if (std::find(mesh_lmk_ids.begin(),
                mesh_lmk_ids.end(), lmk_id) ==
      mesh_lmk_ids.end()) {
    VLOG(20) << "Lmk_id = " << lmk_id
             << " needs to stay as it is since it is NOT involved in any regularity.";
    // This lmk is not involved in any regularity.
    if (lmk_id_slot == lmk_id_is_smart->end()) {
      // We did not find the lmk_id in the lmk_id_is_smart_ map.
      // Add it as a smart factor.
      lmk_id_is_smart->insert(std::make_pair(lmk_id, true));
    } else {
      // Let the lmk be as it was before (note is not allowed to go from
      // projection to smart.
    }
  } else {
    // This lmk is involved in a regularity, hence it should be a variable in
    // the factor graph (connected to projection factor).
    VLOG(20) << "Lmk_id = " << lmk_id
             << " needs to be a proj. factor, as it is involved in a regularity.";
    const auto& old_smart_factors_it = old_smart_factors_.find(lmk_id);
    if (old_smart_factors_it == old_smart_factors_.end()) {
      // This should only happen if the lmk was already in a regularity,
      // and subsequently updated asa a projection factor...
      VLOG(20) << "Landmark with id: " << lmk_id
               << " not found in old_smart_factors.";
      // This lmk must be tracked.
      CHECK(lmk_id_slot != lmk_id_is_smart->end());
      // This lmk must not be a smart factor.
      CHECK(lmk_id_is_smart->at(lmk_id) == false);
    } else {
      // We found the factor.

      // Get whether the smart factor is valid or not.
      bool lmk_is_valid = true;

      SmartStereoFactor::shared_ptr old_factor =
          old_smart_factors_it->second.first;
      CHECK(old_factor);
      CHECK(old_factor->point().is_initialized());

      if (!old_factor->point().valid()) {
        // The point is not valid.
        VLOG(20) << "Smart factor for lmk: " << lmk_id << "is NOT valid.";
        lmk_is_valid = false;
      } else {
        // If the smart factor has less than x number of observations,
        // then do not consider the landmark as valid.
        // This param is different from the one counting the track length
        // as it controls only the quality of the subsequent proj factors,
        // and has no effect on how the smart factors are created.
        // This param is very correlated to the one we use to get lmks,
        // in time horizon (aka min_age) since only the ones that have reached
        // the mesher and have been clustered will have the possibility of
        // being here, so this param must be higher than the min_age one to
        // have any impact.
        // TODO Do this when we convert the factor not when we decide if
        // it should be a proj or a smart factor! But then we must be
        // careful that the lmk_id_is_smart is re-changed to smart right?
        static constexpr size_t min_num_obs_for_proj_factor = 4;
        if (old_factor->measured().size() >= min_num_obs_for_proj_factor) {
          VLOG(20) << "Smart factor for lmk: " << lmk_id << " is valid.";
          lmk_is_valid = true;
        } else {
          // Should not be a warning this, but just in case.
          LOG(WARNING) << "Smart factor for lmk: " << lmk_id << " has not enough"
                       << " observations: " << old_factor->measured().size()
                       << ", but should be more or equal to "
                       << min_num_obs_for_proj_factor;
          lmk_is_valid = false;
        }
      }

      if (lmk_id_slot == lmk_id_is_smart->end()) {
        // We did not find the lmk_id in the lmk_id_is_smart_ map.
        // Add it as a projection factor.
        // TODO use an enum instead of bool for lmk_id_is_smart, otherwise
        // it is too difficult to read.
        lmk_id_is_smart->insert(std::make_pair(lmk_id, lmk_is_valid?false:true));
      } else {
        if (lmk_is_valid) {
          // Change it to a projection factor.
          lmk_id_is_smart->at(lmk_id) = false;
        } else {
          // Keep it to a smart factor.
          // We cannot allow conversion back from proj to smart factor.
          // So just check that this is still smart, otherwise we are stuck.
          CHECK(lmk_id_is_smart->at(lmk_id) == true);
          //lmk_id_is_smart->at(lmk_id) = true;
        }
      }
    }
  }

  CHECK(lmk_id_is_smart->find(lmk_id) != lmk_id_is_smart->end());
  return lmk_id_is_smart->at(lmk_id);
}

/* -------------------------------------------------------------------------- */
// TODO we have a: terminate called after throwing an instance of 'std::out_of_range'
//  what():  map::at when running this function... And it does not appear very often.
void RegularVioBackEnd::addRegularityFactors(
    const LandmarkIds& mesh_lmk_ids,
    gtsam::Symbol* plane_symbol,
    std::vector<std::pair<Slot, LandmarkId>>* idx_of_point_plane_factors_to_add) {
  CHECK_NOTNULL(plane_symbol);
  CHECK_NOTNULL(idx_of_point_plane_factors_to_add);
  idx_of_point_plane_factors_to_add->resize(0);

  VLOG(10) << "Starting addRegularityFactors...";

  // Plane key.
  static gtsam::Key plane_key (gtsam::Symbol('P', 0));
  *plane_symbol = plane_key;

  // Vars to check whether the new plane is going to be fully constrained or not.
  static bool is_plane_constrained = false;
  static std::vector<LandmarkId> list_of_constraints;

  if (!state_.exists(plane_key)) {
    VLOG(10) << "Plane key, " << gtsam::DefaultKeyFormatter(plane_key)
             << " does NOT exist.";
    // If the plane is new, and we are only using one plane as regularity
    // then there should be no lmk_id with a regularity now...
    lmk_id_to_regularity_type_map_.clear();

    // Verify that the plane is going to be fully constrained before adding it.
    // TODO it might be more robust to increase this, to reduce the
    // possibility of having degenerate configs such as points aligned...
    static const size_t min_number_of_constraints =
        vio_params_.minPlaneConstraints_;

    // Loop over all lmks which are involved in the regularity, to both
    // check that the new plane is going to be fully constrained and that
    // in case it is we add all the corresponding factors.
    for (const LandmarkId& lmk_id: mesh_lmk_ids) {
      if (state_.exists(gtsam::Symbol('l', lmk_id)) ||
          new_values_.exists(gtsam::Symbol('l', lmk_id))) {
        // Lmk id exists either in the state or it is going to be added in the
        // next optimization.
        VLOG(20) << "Lmk id: " << lmk_id
                 << " is in state_ or is going to be added in next optimization.";

        const auto& lmk_id_regularity_type =
            lmk_id_to_regularity_type_map_.find(lmk_id);
        if (lmk_id_regularity_type == lmk_id_to_regularity_type_map_.end()) {
          // Lmk has not been used in a regularity.
          VLOG(20) << "Lmk id: " << lmk_id
                   << " has not yet been used in a regularity.";

          // Check that the plane is constrained before adding factors.
          if (!is_plane_constrained) {
            // Still need to check whether the plane is fully constrained.
            // Add lmk to list of potential constraints.
            VLOG(10) << "Checking that plane has enough constraints...";
            list_of_constraints.push_back(lmk_id);
          } else {
            // Act as usual, the plane is already fully constrained, keep
            // adding new factors.
            VLOG(20) << "Adding PointPlaneFactor.";
            idx_of_point_plane_factors_to_add->push_back(
                  std::make_pair(new_imu_prior_and_other_factors_.size(),
                                 lmk_id));
            new_imu_prior_and_other_factors_.push_back(
                  boost::make_shared<gtsam::PointPlaneFactor>(
                    gtsam::Symbol('l', lmk_id),
                    plane_key,
                    point_plane_regularity_noise_));
            // Acknowledge that this lmk has been used in a regularity.
            lmk_id_to_regularity_type_map_[lmk_id] =
                RegularityType::POINT_PLANE;
          }

          if (list_of_constraints.size() > min_number_of_constraints) {
            // Acknowledge that the plane is constrained.
            is_plane_constrained = true;

            static size_t i = 0;
            *plane_symbol = gtsam::Symbol('P', 0); // gtsam::Symbol('P', i);
            plane_key = plane_symbol->key();
            i++;

            // TODO find a way to add initial guess, maybe when sending the lmk_ids
            // having a regularity we could regress a plane through it?
            // The mesher should send the plane!
            static const gtsam::OrientedPlane3 plane(0.0, 0.0, 1.0, -0.2);

            // The plane is constrained, add it.
            VLOG(10) << "Adding new plane with key: "
                     << gtsam::DefaultKeyFormatter(plane_key);
            new_values_.insert(plane_key, plane);

            // TODO Remove! DEBUG add a prior to the plane.
            //static const gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
            //    gtsam::noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.5));
            //new_imu_prior_and_other_factors_.push_back(
            //      boost::make_shared<gtsam::PriorFactor<gtsam::OrientedPlane3> >(
            //        plane_key,
            //        plane,
            //        prior_noise));

            // Add the factors for the lmks that we skipped while checking
            // that the plane is fully constrained.
            for (const LandmarkId& prev_lmk_id: list_of_constraints) {
              VLOG(20) << "Adding PointPlaneFactor.";
              idx_of_point_plane_factors_to_add->push_back(
                  std::make_pair(new_imu_prior_and_other_factors_.size(),
                                 prev_lmk_id));
              new_imu_prior_and_other_factors_.push_back(
                    boost::make_shared<gtsam::PointPlaneFactor>(
                      gtsam::Symbol('l', prev_lmk_id),
                      plane_key,
                      point_plane_regularity_noise_));
              // Acknowledge that this lmk has been used in a regularity.
              lmk_id_to_regularity_type_map_[prev_lmk_id] =
                  RegularityType::POINT_PLANE;
            }

            // Clear list of constraints, so that we do not enter in this
            // condition anymore.
            list_of_constraints.resize(0);
          }
        } else {
          LOG(FATAL) << "If this is a new plane, the lmks should not have any "
                        "regularities...";
          // And this if condition will prevent multiple regularities on the
          // same point...
        }
      } else {
        LOG(WARNING) << "Lmk id: " << lmk_id
                     << " is NOT in state_ or in new_values_,"
                     << " NOT adding PointPlaneFactor.";

        // Erase from map of lmk id to regularity type, since the lmk does not
        // exist anymore, or has never existed.
        // Avoid the world of undefined behaviour by checking that the lmk_id
        // we want to erase is actually there.
        if (lmk_id_to_regularity_type_map_.find(lmk_id) !=
            lmk_id_to_regularity_type_map_.end()) {
          lmk_id_to_regularity_type_map_.erase(lmk_id);
        }
      }
    }

    if (!is_plane_constrained) {
      VLOG(10) << "Plane with key " << gtsam::DefaultKeyFormatter(plane_key)
               << " is not sufficiently constrained, not adding it.";
    }

    // Reset the flag for next time we have to add a plane.
    is_plane_constrained = false;
    // Also reset vector of constraints to empty.
    list_of_constraints.resize(0);
  } else {
    VLOG(10) << "Plane key does exist already: "
             << gtsam::DefaultKeyFormatter(plane_key);

    // The plane exists, just add regularities that are ok.
    for (const LandmarkId& lmk_id: mesh_lmk_ids) {
      if (state_.exists(gtsam::Symbol('l', lmk_id)) ||
          new_values_.exists(gtsam::Symbol('l', lmk_id))) {
        // Lmk id exists either in the state or it is going to be added in the
        // next optimization.
        VLOG(20) << "Lmk id: " << lmk_id
                 << " is in state_ or is going to be added in next optimization.";

        const auto& lmk_id_regularity_type =
            lmk_id_to_regularity_type_map_.find(lmk_id);
        if (lmk_id_regularity_type == lmk_id_to_regularity_type_map_.end() ||
            lmk_id_regularity_type->second != RegularityType::POINT_PLANE) {
          // Lmk has not been used in a regularity or is not in a point plane
          // factor.
          VLOG(20) << "Lmk id: " << lmk_id
                   << " has not yet been used in a regularity.";

          // The plane is already in the graph so it must be fully constrained,
          // keep adding new factors.
          VLOG(20) << "Adding PointPlaneFactor.";
          idx_of_point_plane_factors_to_add->push_back(
                  std::make_pair(new_imu_prior_and_other_factors_.size(),
                                 lmk_id));
          new_imu_prior_and_other_factors_.push_back(
                boost::make_shared<gtsam::PointPlaneFactor>(
                  gtsam::Symbol('l', lmk_id),
                  plane_key,
                  point_plane_regularity_noise_));
          // Acknowledge that this lmk has been used in a regularity.
          lmk_id_to_regularity_type_map_[lmk_id] =
              RegularityType::POINT_PLANE;
        } else {
          // This lmk has already been used in a regularity and is a point plane
          // factor, avoid duplication of factors.
          VLOG(20) << "Avoiding duplicated regularity factor for lmk id: " << lmk_id;
        }
      } else {
        LOG(WARNING) << "Lmk id: " << lmk_id
                   << " is NOT in state_ or in new_values_,"
                   << " NOT adding PointPlaneFactor.";

        // Erase from map of lmk id to regularity type, since the lmk does not
        // exist anymore, or has never existed.
        // Avoid the world of undefined behaviour by checking that the lmk_id
        // we want to erase is actually there.
        if (lmk_id_to_regularity_type_map_.find(lmk_id) !=
            lmk_id_to_regularity_type_map_.end()) {
          lmk_id_to_regularity_type_map_.erase(lmk_id);
        }
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::removeOldRegularityFactors_Slow(
    const gtsam::Symbol& plane_symbol,
    const std::vector<std::pair<Slot, LandmarkId>>& idx_of_point_plane_factors_to_add,
    const LandmarkIds& mesh_lmk_ids,
    std::vector<size_t>* delete_slots) {
  CHECK_NOTNULL(delete_slots);
  delete_slots->resize(0);

  CHECK(!(state_.exists(plane_symbol.key()) &&
        new_values_.exists(plane_symbol.key())))
      << "Inconsistency: plane is in current state,"
         " but it is going to be added.";
  if (!state_.exists(plane_symbol.key())) {
    // The plane is not in the state, it is probably going to be added in
    // this iteration, so it must be already well constrained and attached
    // to the right lmks.
    // Or it could be that it is not going to be added at all, so we
    // do not need to do anything.
    VLOG(10) << "Plane with id " << gtsam::DefaultKeyFormatter(plane_symbol)
             << " is not in state, skipping removeOldRegularityFactors.";
    return; // Leave this function.
  }

  if (new_values_.exists(plane_symbol.key())) {
    // The plane is not in the state, and it is going to be added in
    // this iteration, so it must be already well constrained and attached
    // to the right lmks.
    VLOG(10) << "Plane with id " << gtsam::DefaultKeyFormatter(plane_symbol)
             << " is in new_values_, skipping removeOldRegularityFactors.";
    return;
  }

  VLOG(10) << "Starting removeOldRegularityFactors_Slow...";

  // If the plane exists in the state_ and not in new_values_,
  // then let us remove old regularity factors.
  const gtsam::NonlinearFactorGraph& graph = smoother_->getFactors();
  Slot slot = 0;
  std::vector<std::pair<Slot, LandmarkId>> point_plane_factor_slots_bad;
  std::vector<std::pair<Slot, LandmarkId>> point_plane_factor_slots_good;
  bool has_plane_a_prior = false;
  // Loop over current graph.
  for (const auto& g: graph) {
    const auto& ppf =
        boost::dynamic_pointer_cast<gtsam::PointPlaneFactor>(g);
    if (ppf) {
      // We found a PointPlaneFactor.
      if (plane_symbol.key() == ppf->getPlaneKey()) {
        // We found a PointPlaneFactor that involves our plane.
        // Get point symbol.
        gtsam::Symbol point_symbol (ppf->getPointKey());
        // Get lmk id of this point.
        LandmarkId lmk_id = point_symbol.index();
        // Find this lmk id in the set of regularities.
        if (std::find(mesh_lmk_ids.begin(),
                      mesh_lmk_ids.end(), lmk_id) == mesh_lmk_ids.end()) {
          // We did not find the point in mesh_lmk_ids, therefore it should
          // not be involved in a regularity anymore, delete this slot.
          // (but I want to remove the landmark! and all its factors,
          // to avoid having underconstrained lmks...)
          VLOG(20) << "Found bad point plane factor on lmk with id: "
                   << point_symbol.index();
          point_plane_factor_slots_bad.push_back(
                std::make_pair(slot, lmk_id));

          // Before deleting this slot, we must ensure that both the plane
          // and the landmark are well constrained!
        } else {
          // Store those factors that we will potentially keep.
          point_plane_factor_slots_good.push_back(
                std::make_pair(slot, lmk_id));
        }
      } else {
        LOG(ERROR) << "Point plane keys do not match..."
                      " Are we using multiple planes?";
      }
    }

    // Check for priors attached to the plane.
    const auto& plane_prior = boost::dynamic_pointer_cast<
                              gtsam::PriorFactor<gtsam::OrientedPlane3>>(g);
    if (plane_prior) {
      if (plane_prior->find(plane_symbol.key()) != plane_prior->end()) {
        LOG(WARNING) << "Found plane prior factor.";
        has_plane_a_prior = true;
      }
    }

    // Check for linear container factors having the plane.
    const auto& lcf = boost::dynamic_pointer_cast<
                      gtsam::LinearContainerFactor>(g);
    if (lcf) {
      if (lcf->find(plane_symbol.key()) != lcf->end()) {
        VLOG(10) << "Found linear container factor with our plane.";
        has_plane_a_prior = true;
      }
    }

    // Next slot.
    slot++;
  }

  // Decide whether we can just delete the bad point plane factors,
  // or whether we need to delete all the factors involving the plane
  // so that it is removed.
  // For this we need to check if the plane is fully constrained!

  /// If there are enough new constraints to be added then delete only delete_slots
  /// else, if there are enough constraints left, only delete delete_slots
  /// otherwise delete ALL constraints, both old and new, so that the plane
  /// disappears (take into account priors!).
  /// Priors affecting planes: linear container factor & prior on OrientedPlane3
  static constexpr size_t min_num_of_constraints = 10;
  size_t total_nr_of_plane_constraints =
      point_plane_factor_slots_good.size() +
      idx_of_point_plane_factors_to_add.size();
  VLOG(10) << "Plane total number of constraints is: "
           << total_nr_of_plane_constraints << "\n"
           << "\tConstraints in graph which are good: "
           << point_plane_factor_slots_good.size() << "\n"
           << "\tConstraints that are going to be added: "
           << idx_of_point_plane_factors_to_add.size() << "\n"
           << "Constraints in graph which are bad: "
           << point_plane_factor_slots_bad.size();
  if (total_nr_of_plane_constraints > min_num_of_constraints) {
    // The plane is fully constrained.
    // We can just delete bad factors, assuming lmks will be well constrained.
    // TODO ensure the lmks are themselves well constrained.
    VLOG(10) << "Plane is fully constrained, removing only bad factors.";
    fillDeleteSlots(point_plane_factor_slots_bad,
                    delete_slots);
  } else {
    // The plane is NOT fully constrained if we remove all bad factors,
    // unless the plane has a prior.
    // Check if the plane has a prior.
    VLOG(10) << "Plane is NOT fully constrained if we just remove"
                " the bad factors.";
    if (has_plane_a_prior) {
      // The plane has a prior.
      // Delete just the bad factors.
      VLOG(10) << "Plane has a prior, delete just the bad factors.";
      fillDeleteSlots(point_plane_factor_slots_bad,
                      delete_slots);
    } else {
      // The plane has NOT a prior.
      static constexpr bool use_unstable = false;
      if (use_unstable) {
        // Delete all factors involving the plane so that iSAM removes the plane
        // from the optimization.
        LOG(ERROR) << "Plane has no prior, trying to forcefully"
                      " remove the PLANE!";
        DEBUG_ = true;
        std::vector<std::pair<Slot, LandmarkId>> point_plane_factor_slots_all (
              point_plane_factor_slots_bad);
        point_plane_factor_slots_all.insert(point_plane_factor_slots_all.end(),
                                            point_plane_factor_slots_good.begin(),
                                            point_plane_factor_slots_good.end());
        fillDeleteSlots(point_plane_factor_slots_all,
                        delete_slots);

        // Remove as well the factors that are going to be added in this iteration.
        deleteNewSlots(idx_of_point_plane_factors_to_add,
                       &new_imu_prior_and_other_factors_);
      } else {
        // Do not use unstable implementation...
        // Just add a prior on the plane and remove only bad factors...
        // Add a prior to the plane.
        VLOG(10) << "Adding a prior to the plane, delete just the bad factors.";
        gtsam::OrientedPlane3 plane_estimate;
        CHECK(getEstimateOfKey(plane_symbol.key(), &plane_estimate));
        LOG(WARNING) << "Using plane prior on plane with id "
                     << gtsam::DefaultKeyFormatter(plane_symbol);
        CHECK(!has_plane_a_prior) << "Check that the plane has no prior.";
        static const gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
            gtsam::noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
        new_imu_prior_and_other_factors_.push_back(
              boost::make_shared<gtsam::PriorFactor<gtsam::OrientedPlane3>>(
                plane_symbol.key(),
                plane_estimate,
                prior_noise));

        // Delete just the bad factors.
        fillDeleteSlots(point_plane_factor_slots_bad,
                        delete_slots);
      }
    } // The plane has NOT a prior.
  } // The plane is NOT fully constraint.

  //  // TODO now the plane could be floating around with a prior attached, but
  //  // not really attached to the rest of the graph...

  //  //  gtsam::Point3 point;
  //  //  if (getEstimateOfKey(point_symbol.key(), &point)) {
  //  //    LOG(WARNING) << "Using lmk prior on lmk with id " <<
  //  //                    gtsam::DefaultKeyFormatter(point_symbol);
  //  //    // TODO make sure this prior is not repeated over and over on the same
  //  //    // lmk id.
  //  //    // TODO is there the possibility that this lmk just have a prior attached
  //  //    // to it? and nothing else, I guess that it is unlikely because to
  //  //    // be first added it must have had some proj factors besides PointPlane
  //  //    // factor...
  //  //    static const gtsam::noiseModel::Diagonal::shared_ptr prior_lmk_noise =
  //  //        gtsam::noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 1.0));
  //  //    new_imu_prior_and_other_factors_.push_back(
  //  //          boost::make_shared<gtsam::PriorFactor<gtsam::Point3>>(
  //  //            point_symbol.key(),
  //  //            point,
  //  //            prior_lmk_noise));
  //  //  } else {
  //  //    LOG(ERROR) << "Lmk with id " << gtsam::DefaultKeyFormatter(point_symbol)
  //  //               << " does not exist in graph.";
  //  //  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::fillDeleteSlots(
    const std::vector<std::pair<Slot, LandmarkId>>& point_plane_factor_slots_bad,
    std::vector<size_t>* delete_slots) {
  CHECK_NOTNULL(delete_slots);
  VLOG(10) << "Starting fillDeleteSlots...";
  delete_slots->resize(point_plane_factor_slots_bad.size());
  size_t i = 0;
  if (point_plane_factor_slots_bad.size() > 0) {
    for (const std::pair<Slot, LandmarkId>& ppf_bad:
         point_plane_factor_slots_bad) {
      CHECK_LT(i, delete_slots->size());
      CHECK(smoother_->getFactors().exists(ppf_bad.first));

      // Add factor slot to delete slots.
      delete_slots->at(i) = ppf_bad.first;
      i++;

      // Acknowledge that these lmks are not in a regularity anymore.
      // TODO this does not generalize to multiple planes...
      const LandmarkId& lmk_id = ppf_bad.second;
      CHECK(lmk_id_to_regularity_type_map_.find(lmk_id) !=
          lmk_id_to_regularity_type_map_.end())
          << "Avoid undefined behaviour by checking that the lmk was in the "
             "container.";
      lmk_id_to_regularity_type_map_.erase(lmk_id);
    }
  } else {
    VLOG(10) << "There are no bad factors to remove.";
  }

  VLOG(10) << "Finished fillDeleteSlots...";
}

/* -------------------------------------------------------------------------- */
// Remove as well the factors that are going to be added in this iteration.
void RegularVioBackEnd::deleteNewSlots(
    const std::vector<std::pair<Slot, LandmarkId>>& idx_of_point_plane_factors_to_add,
    gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors) {
  CHECK_NOTNULL(new_imu_prior_and_other_factors);

  bool clean_nullptrs = false;
  for (const std::pair<Slot, LandmarkId>& i:
       idx_of_point_plane_factors_to_add) {
    // Sanity checks.
    // The slot is valid.
    CHECK(new_imu_prior_and_other_factors->exists(i.first));

    const auto& ppf =
        boost::dynamic_pointer_cast<gtsam::PointPlaneFactor>(
          new_imu_prior_and_other_factors->at(i.first));
    // The factor is really a point plane one.
    CHECK(ppf);
    // The factor is the one related to the right lmk.
    CHECK(ppf->getPointKey() == gtsam::Symbol('l', i.second).key());

    // WARNING using erase moves all factors! aka the idx_of_point_plane_factors_to_add will
    // be wrong!!!!
    // Remove will just insert a nullptr, is this ok for iSAM??
    new_imu_prior_and_other_factors->remove(i.first);
    clean_nullptrs = true;

    // Acknowledge that these lmks are not in a regularity anymore.
    // TODO this does not generalize to multiple planes...
    const LandmarkId& lmk_id = i.second;
    CHECK(lmk_id_to_regularity_type_map_.find(lmk_id) !=
        lmk_id_to_regularity_type_map_.end())
        << "Avoid undefined behaviour by checking that the lmk was in the "
           "container.";
    lmk_id_to_regularity_type_map_.erase(lmk_id);
    // Sanity check that lmk_id_is_smart makes sense, aka it should be
    // false...
    CHECK(lmk_id_is_smart_.exists(lmk_id));
    CHECK(lmk_id_is_smart_.at(lmk_id) == false);
  }

  // TODO since we do not know if iSAM handles well nullptr as new factors,
  // clean the new_imu_prior_and_other_factors_
  // This is because we are using "remove" to remove factors.
  if (clean_nullptrs) {
    cleanNullPtrsFromGraph(new_imu_prior_and_other_factors);
  } else {
    // Avoid unnecessary loop if there are no nullptrs in the graph.
    VLOG(10) << "Avoid cleaning graph of null pointers, since we did "
                "not remove any new factor.";
  }
}

} // namespace VIO
