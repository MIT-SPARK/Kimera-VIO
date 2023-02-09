/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RegularVioBackend.h
 * @brief  Derived class from VioBackend which enforces regularity constraints
 * on the factor graph.
 *
 * A. Rosinol, T. Sattler, M. Pollefeys, and L. Carlone. Incremental
 * Visual-Inertial 3D Mesh Generation with Structural Regularities. IEEE Intl.
 * Conf. on Robotics and Automation (ICRA), 2019
 *
 * @author Antoni Rosinol
 */

#include "kimera-vio/backend/RegularVioBackend.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include "kimera-vio/factors/PointPlaneFactor.h"
#include "kimera-vio/utils/UtilsNumerical.h"

DEFINE_int32(min_num_of_observations,
             2,
             "Minimum number of observations for a feature track to be added "
             "in the optimization problem (corresponds to number of "
             "measurements in smart factors. Only insert feature tracks of "
             "length at least 2 (otherwise uninformative).");
DEFINE_double(max_parallax,
              150,
              "Maximum parallax to be considered correct. This is a patch to "
              "remove outliers when using mono and stereo projection factors.");
DEFINE_int32(min_num_obs_for_proj_factor,
             4,
             "If the smart factor has less than x number of observations, "
             "then do not consider the landmark as valid to transform to proj."
             "This param is different from the one counting the track length "
             "as it controls only the quality of the subsequent proj factors, "
             "and has no effect on how the smart factors are created. "
             "This param is very correlated to the one we use to get lmks "
             "in time horizon (aka min_age) since only the ones that have "
             "reached the mesher and have been clustered will have the "
             "possibility of being here, so this param must be higher than the "
             "min_age one to have any impact.");
DEFINE_int32(min_num_of_plane_constraints_to_add_factors,
             20,
             "Minimum number of plane constraints to ");

DEFINE_bool(convert_extra_smart_factors_to_proj_factors,
            true,
            "Whether to convert all smart factors in time horizon to "
            "projection factors, instead of just the ones in current frame.");
DEFINE_bool(remove_old_reg_factors,
            true,
            "Remove regularity factors for those landmarks that were "
            "originally associated to the plane, but which are not anymore.");
DEFINE_int32(min_num_of_plane_constraints_to_remove_factors,
             10,
             "Number of constraints for a plane to be considered "
             "underconstrained when trying to remove old regularity factors. "
             "If a plane is thought to be underconstrained, we'll try to "
             "remove it from the optimization or set a prior to it (depending "
             "on the use_unstable_plane_removal flag.");

DEFINE_bool(use_unstable_plane_removal,
            false,
            "Remove planes from optimization using unstable implementation, "
            "which tries to remove all factors attached to the plane so that "
            "ISAM2 deletes it. Unfortunately, ISAM2 has a bug and leads to seg "
            "faults if we do so. The stable implementation instead puts a "
            "prior on the plane and removes as many factors from the plane as "
            "possible to avoid seg fault.");
DEFINE_int32(min_num_of_plane_constraints_to_avoid_seg_fault,
             3,
             "Minimum number of constraints from landmark to plane to keep in "
             "order to avoid seg fault when removing factors for a specific "
             "plane. If all the factors are removed, then ISAM2 will seg fault,"
             " check issue:https://github.mit.edu/lcarlone/VIO/issues/32.");
DEFINE_double(prior_noise_sigma_normal,
              0.1,
              "Sigma for the noise model of the prior on the normal of the "
              "plane.");
DEFINE_double(prior_noise_sigma_distance,
              0.1,
              "Sigma for the noise model of the prior on the distance of the "
              "plane.");

namespace VIO {

/* -------------------------------------------------------------------------- */
RegularVioBackend::RegularVioBackend(
    const Pose3& B_Pose_leftCamRect,
    const StereoCalibPtr& stereo_calibration,
    const BackendParams& backend_params,
    const ImuParams& imu_params,
    const BackendOutputParams& backend_output_params,
    const bool& log_output,
    boost::optional<OdometryParams> odom_params)
    : VioBackend(B_Pose_leftCamRect,
                 stereo_calibration,
                 backend_params,
                 imu_params,
                 backend_output_params,
                 log_output,
                 odom_params),
      regular_vio_params_(RegularVioBackendParams::safeCast(backend_params)) {
  LOG(INFO) << "Using Regular VIO Backend.\n";

  // Set type of mono_noise_ for generic projection factors.
  gtsam::SharedNoiseModel gaussian_dim_2 = gtsam::noiseModel::Isotropic::Sigma(
      2, regular_vio_params_.monoNoiseSigma_);

  selectNormType(&mono_noise_,
                 gaussian_dim_2,
                 regular_vio_params_.monoNormType_,
                 regular_vio_params_.monoNormParam_);

  // Set type of stereo_noise_ for generic stereo projection factors.
  gtsam::SharedNoiseModel gaussian_dim_3 = gtsam::noiseModel::Isotropic::Sigma(
      3, regular_vio_params_.stereoNoiseSigma_);

  selectNormType(&stereo_noise_,
                 gaussian_dim_3,
                 regular_vio_params_.stereoNormType_,
                 regular_vio_params_.stereoNormParam_);

  // Set type of regularity noise for point plane factors.
  gtsam::SharedNoiseModel gaussian_dim_1 = gtsam::noiseModel::Isotropic::Sigma(
      1, regular_vio_params_.regularityNoiseSigma_);

  selectNormType(&point_plane_regularity_noise_,
                 gaussian_dim_1,
                 regular_vio_params_.regularityNormType_,
                 regular_vio_params_.regularityNormParam_);

  mono_cal_ = boost::make_shared<Cal3_S2>(stereo_cal_->calibration());
  CHECK(mono_cal_->equals(stereo_cal_->calibration()))
      << "Monocular calibration should match Stereo calibration";
}

/* -------------------------------------------------------------------------- */
bool RegularVioBackend::addVisualInertialStateAndOptimize(
    const Timestamp& timestamp_kf_nsec,
    const StatusStereoMeasurements& status_smart_stereo_measurements_kf,
    const gtsam::PreintegrationType& pim,
    boost::optional<gtsam::Pose3> odometry_body_pose,
    boost::optional<gtsam::Velocity3> odometry_vel) {
  debug_info_.resetAddedFactorsStatistics();

  // Features and IMU line up --> do iSAM update.
  last_kf_id_ = curr_kf_id_;
  ++curr_kf_id_;

  VLOG(7) << "Processing keyframe " << curr_kf_id_
          << " at timestamp: " << UtilsNumerical::NsecToSec(timestamp_kf_nsec)
          << " (nsec)\n";

  // Add initial guess.
  addStateValues(curr_kf_id_, status_smart_stereo_measurements_kf.first, pim);

  /////////////////// IMU FACTORS //////////////////////////////////////////////
  // Add imu factors between consecutive keyframe states.
  VLOG(10) << "Adding IMU factor between pose id: " << last_kf_id_
           << " and pose id: " << curr_kf_id_;
  addImuFactor(last_kf_id_, curr_kf_id_, pim);

  /////////////////// STEREO RANSAC FACTORS ////////////////////////////////////
  // Add between factor from RANSAC
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

  /////////////////// VISION MEASUREMENTS //////////////////////////////////////
  const StereoMeasurements& smart_stereo_measurements_kf =
      status_smart_stereo_measurements_kf.second;

  // Extract relevant information from stereo frame.
  // Get the landmarks visible in current keyframe. (These are not all the lmks
  // in time horizon used for the optimization!)
  LandmarkIds lmks_kf;
  addStereoMeasurementsToFeatureTracks(
      curr_kf_id_, smart_stereo_measurements_kf, &lmks_kf);

  if (VLOG_IS_ON(20)) {
    printFeatureTracks();
  }

  // Decide which factors to add.
  TrackingStatus kfTrackingStatus_mono =
      status_smart_stereo_measurements_kf.first.kfTrackingStatus_mono_;

  gtsam::FactorIndices delete_slots(delete_slots_of_converted_smart_factors_);
  switch (kfTrackingStatus_mono) {
    case TrackingStatus::LOW_DISPARITY: {
      // Vehicle is not moving.
      VLOG(0) << "Tracker has a LOW_DISPARITY status.";
      VLOG(10) << "Add zero velocity and no motion factors.";
      addZeroVelocityPrior(curr_kf_id_);
      addNoMotionFactor(last_kf_id_, curr_kf_id_);
      // TODO why are we not adding the regularities here as well...?
      break;
    }
    default: {
      kfTrackingStatus_mono == TrackingStatus::VALID
          ? VLOG(1) << "Tracker has a VALID status."
          : kfTrackingStatus_mono == TrackingStatus::FEW_MATCHES
                ? VLOG(1) << "Tracker has a FEW_MATCHES status."
                : kfTrackingStatus_mono == TrackingStatus::INVALID
                      ? VLOG(1) << "Tracker has a INVALID status."
                      : kfTrackingStatus_mono == TrackingStatus::DISABLED
                            ? VLOG(1) << "Tracker has a DISABLED status."
                            : VLOG(10) << "";

      if (kfTrackingStatus_mono == TrackingStatus::VALID) {
        // Extract lmk ids that are involved in a regularity.
        VLOG(10) << "Starting extracting lmk ids from set of planes...";
        LandmarkIds lmk_ids_with_regularity;
        switch (regular_vio_params_.backend_modality_) {
          case RegularBackendModality::STRUCTURELESS: {
            // Do nothing, lmk_ids_with_regularity should be empty.
            CHECK_EQ(lmk_ids_with_regularity.size(), 0);
            planes_.clear();
            break;
          }
          case RegularBackendModality::PROJECTION: {
            // Transform all smart factors to projection factors,
            // and clear all planes.
            lmk_ids_with_regularity = lmks_kf;
            planes_.clear();
            break;
          }
          case RegularBackendModality::PROJECTION_AND_REGULARITY: {
            // Keep the planes, but change all smart factors to projection
            // factors.
            lmk_ids_with_regularity = lmks_kf;
            break;
          }
          case RegularBackendModality::STRUCTURELESS_AND_PROJECTION: {
            // Transforms to projection factors only the ones that should
            // have regularities, but do not use the planes anymore.
            extractLmkIdsFromPlanes(planes_, &lmk_ids_with_regularity);
            planes_.clear();
            break;
          }
          case RegularBackendModality::
              STRUCTURELESS_PROJECTION_AND_REGULARITY: {
            // Act as usual, keep planes, and transform smart factors to projj
            // factors for those that will have regularities.
            extractLmkIdsFromPlanes(planes_, &lmk_ids_with_regularity);
            break;
          }
          default: {
            LOG(ERROR)
                << "Backend modality: "
                << static_cast<
                       std::underlying_type<RegularBackendModality>::type>(
                       regular_vio_params_.backend_modality_)
                << " is not supported.";
            break;
          }
        }
        VLOG(10) << "Finished extracting lmk ids from set of planes, total of "
                 << lmk_ids_with_regularity.size()
                 << " lmks with regularities.";

        // We add features in VIO.
        VLOG(10) << "Starting adding/updating landmarks to graph...";
        addLandmarksToGraph(lmks_kf, lmk_ids_with_regularity);
        VLOG(10) << "Finished adding/updating landmarks to graph.";

        // Convert all smart factors of lmks in time horizon that have
        // regularities to projection factors.
        // Most conversions from smart to proj are done before,
        // in addLandmarksToGraph, but here we also make sure we have converted
        // the ones with regularities in time horizon.
        if (FLAGS_convert_extra_smart_factors_to_proj_factors) {
          VLOG(10)
              << "Starting converting extra smart factors to proj factors...";
          convertExtraSmartFactorToProjFactor(lmk_ids_with_regularity);
          VLOG(10)
              << "Finished converting extra smart factors to proj factors...";
        }

        if (planes_.size() > 0) {
          /////////////////// REGULARITY FACTORS
          //////////////////////////////////////////
          // Add regularity factor on vertices of the mesh.
          // WARNING if a plane has been removed by the mesher, then we will not
          // remove the plane from the optimization, since it won't be in
          // planes.
          std::map<PlaneId, std::vector<std::pair<Slot, LandmarkId>>>
              idx_of_point_plane_factors_to_add;
          for (const Plane& plane : planes_) {
            const PlaneId& plane_key = plane.getPlaneSymbol().key();

            VLOG(10) << "Adding regularity factors.";
            addRegularityFactors(
                plane,
                // Creates a new entry if the plane key was not found.
                // So make sure you use [plane_key] instead of .at(plane_key).
                &(plane_id_to_lmk_id_reg_type_[plane_key]),
                &(idx_of_point_plane_factors_to_add[plane_key]));
            VLOG(10) << "Finished adding regularity factors.";
          }

          if (FLAGS_remove_old_reg_factors) {
            VLOG(10) << "Removing old regularity factors.";
            gtsam::FactorIndices delete_old_regularity_factors;
            removeOldRegularityFactors_Slow(planes_,
                                            idx_of_point_plane_factors_to_add,
                                            &plane_id_to_lmk_id_reg_type_,
                                            &delete_old_regularity_factors);
            if (delete_old_regularity_factors.size() > 0) {
              delete_slots.insert(delete_slots.end(),
                                  delete_old_regularity_factors.begin(),
                                  delete_old_regularity_factors.end());
            }
            VLOG(10) << "Finished removing old regularity factors.";
          }
        } else {
          // TODO shouldn't we "removeOldRegularityFactors_Slow" because there
          // are no planes anymore? shouldn't we delete them or something?
          // Not really because the mesher will only add planes, it won't delete
          // an existing plane from planes structure...
          // Log warning only if we are not using a structureless approach
          // (since it does not require planes).
          LOG_IF(WARNING,
                 regular_vio_params_.backend_modality_ !=
                     RegularBackendModality::STRUCTURELESS)
              << "We are not receiving planes for the Backend. If planes have "
                 "been added to the optimization, we are not removing them.";
        }
      }
      break;
    }
  }

  // Add odometry factors if they're available and have non-zero precision
  if (odometry_body_pose && odom_params_ &&
      (odom_params_->betweenRotationPrecision_ > 0.0 ||
       odom_params_->betweenTranslationPrecision_ > 0.0)) {
    VLOG(1) << "Added external factor between " << last_kf_id_ << " and "
            << curr_kf_id_;
    addBetweenFactor(last_kf_id_,
                     curr_kf_id_,
                     *odometry_body_pose,
                     odom_params_->betweenRotationPrecision_,
                     odom_params_->betweenTranslationPrecision_);
  }
  if (odometry_vel && odom_params_ && odom_params_->velocityPrecision_ > 0.0) {
    LOG_FIRST_N(WARNING, 1)
        << "Using velocity priors from external odometry: "
        << "This only works if you have velocity estimates in the world frame! "
        << "(not provided by typical odometry sensors)";
    addVelocityPrior(
        curr_kf_id_, *odometry_vel, odom_params_->velocityPrecision_);
  }

  /////////////////// OPTIMIZE /////////////////////////////////////////////////
  // This lags 1 step behind to mimic hw.
  imu_bias_prev_kf_ = imu_bias_lkf_;

  VLOG(10) << "Starting optimize...";
  bool is_smoother_ok = optimize(timestamp_kf_nsec,
                                 curr_kf_id_,
                                 backend_params_.numOptimize_,
                                 delete_slots);
  VLOG(10) << "Finished optimize.";

  if (is_smoother_ok) {
    // Sanity check: ensure no one is removing planes outside
    // updatePlaneEstimates.
    CHECK_LE(nr_of_planes_, planes_.size());

    // Update estimates of planes, and remove planes that are not in the state.
    VLOG(10) << "Starting updatePlaneEstimates...";
    updatePlaneEstimates(&planes_);
    VLOG(10) << "Finished updatePlaneEstimates.";
    nr_of_planes_ = planes_.size();

    // Reset list of factors to delete.
    // These are the smart factors that have been converted to projection
    // factors
    // and must be deleted from the factor graph.
    delete_slots_of_converted_smart_factors_.resize(0);
  }

  return is_smoother_ok;
}

/* -------------------------------------------------------------------------- */
// TODO Virtualize this appropriately,
void RegularVioBackend::addLandmarksToGraph(
    const LandmarkIds& lmks_kf,
    const LandmarkIds& lmk_ids_with_regularity) {
  // Add selected landmarks to graph:
  size_t n_new_landmarks = 0;
  size_t n_updated_landmarks = 0;
  debug_info_.numAddedSmartF_ += lmks_kf.size();

  // Iterate over all landmarks in current key frame.
  for (const LandmarkId& lmk_id : lmks_kf) {
    CHECK(feature_tracks_.find(lmk_id) != feature_tracks_.end());
    FeatureTrack& feature_track = feature_tracks_.at(lmk_id);

    // Only insert feature tracks of length at least 2
    // (otherwise uninformative)
    // TODO(TONI): parametrize this min_num_of_obs... should be in Frontend
    // rather than Backend though...
    if (feature_track.obs_.size() >=
        static_cast<size_t>(FLAGS_min_num_of_observations)) {
      // We have enough observations of the lmk.
      if (!feature_track.in_ba_graph_) {
        // The lmk has not yet been added to the graph.
        VLOG(20) << "Adding lmk " << lmk_id << " to graph.";
        addLandmarkToGraph(lmk_id, feature_track, &lmk_id_is_smart_);
        // Acknowledge that we have added the landmark in the graph.
        feature_track.in_ba_graph_ = true;
        ++n_new_landmarks;
      } else {
        // The lmk has already been added to the graph.
        CHECK_GE(feature_track.obs_.size(), 1);
        const std::pair<FrameId, StereoPoint2>& obs_kf =
            feature_track.obs_.back();

        // Sanity check.
        CHECK_EQ(obs_kf.first, curr_kf_id_)
            << "Last obs is not from the current"
               " keyframe!";

        // For each landmark we decide if it's going to be a smart factor or
        // not. Here there is a timeline mismatch, while landmarks_kf are only
        // currently visible lmks, lmk_ids_with_regularity contains lmks in
        // time_horizon. To further convert the lmks in time_horizon we use the
        // convertExtraSmartFactorToProjFactor function.
        const bool& is_lmk_smart = updateLmkIdIsSmart(
            lmk_id, lmk_ids_with_regularity, &lmk_id_is_smart_);

        VLOG(20) << "Updating lmk " << lmk_id << " to graph.";
        updateLandmarkInGraph(lmk_id, is_lmk_smart, obs_kf);
        ++n_updated_landmarks;
      }
    } else {
      VLOG(20) << "Feature track is shorter (" << feature_track.obs_.size()
               << ") than min_num_of_observations ("
               << FLAGS_min_num_of_observations
               << ") for lmk with id: " << lmk_id;
    }
  }

  // Convert to projection factors those landmarks that are not in the current
  // key frame, but that have a regularity.
  VLOG(10) << "Added " << n_new_landmarks << " new landmarks.\n"
           << "Updated " << n_updated_landmarks << " landmarks in graph.";
}

/* -------------------------------------------------------------------------- */
// TODO reuse base class Backend addLandmarkToGraph because this one is
// too similar!
void RegularVioBackend::addLandmarkToGraph(const LandmarkId& lmk_id,
                                           const FeatureTrack& ft,
                                           LmkIdIsSmart* lmk_id_is_smart) {
  CHECK_NOTNULL(lmk_id_is_smart);
  // All landmarks should be smart the first time we add them to the graph.
  // Add as a smart factor.
  // We use a unit pinhole projection camera for the smart factors to be
  // more efficient.
  SmartStereoFactor::shared_ptr new_factor =
      boost::make_shared<SmartStereoFactor>(
          smart_noise_, smart_factors_params_, B_Pose_leftCamRect_);

  VLOG(20) << "Adding landmark with id: " << lmk_id
           << " for the first time to graph. \n"
           << "Nr of observations of the lmk: " << ft.obs_.size()
           << " observations.\n";
  if (VLOG_IS_ON(30)) {
    new_factor->print();
  }

  // Add observations to smart factor.
  VLOG(20) << "Creating smart factor involving lmk with id: " << lmk_id;
  for (const std::pair<FrameId, StereoPoint2>& obs : ft.obs_) {
    VLOG(20) << "SmartFactor: adding observation of lmk with id: " << lmk_id
             << " from frame with id: " << obs.first;
    gtsam::Symbol pose_symbol('x', obs.first);
    new_factor->add(obs.second, pose_symbol, stereo_cal_);
  }

  /////////////////////// BOOK KEEPING /////////////////////////////////////////
  // Add new factor to suitable structures.
  // TODO why do we need to store the new_factor in both structures??
  VLOG(10) << "Add lmk with id: " << lmk_id << " to new_smart_factors_";
  CHECK_NE(lmk_id, -1)
      << "Lmk id should not be -1 when adding to factor graph.";
  new_smart_factors_.insert(std::make_pair(lmk_id, new_factor));
  VLOG(10) << "Add lmk with id: " << lmk_id << " to old_smart_factors_";
  old_smart_factors_.insert(
      std::make_pair(lmk_id, std::make_pair(new_factor, -1)));
  lmk_id_is_smart->insert(std::make_pair(lmk_id, true));
  //////////////////////////////////////////////////////////////////////////////
}

/* -------------------------------------------------------------------------- */
void RegularVioBackend::updateLandmarkInGraph(
    const LandmarkId& lmk_id,
    const bool& is_lmk_smart,
    const std::pair<FrameId, StereoPoint2>& new_obs) {
  if (is_lmk_smart) {
    // Lmk is meant to be smart.
    VLOG(20) << "Lmk with id: " << lmk_id << " is set to be smart.\n";

    updateExistingSmartFactor(
        lmk_id, new_obs, &new_smart_factors_, &old_smart_factors_);
  } else {
    VLOG(20) << "Lmk with id: " << lmk_id
             << " is set to be a projection factor.\n";

    // Update lmk_id as a projection factor.
    if (state_.find(gtsam::Symbol('l', lmk_id).key()) == state_.end()) {
      VLOG(20) << "Lmk with id: " << lmk_id << " is not found in state.\n";
      // We did not find the lmk in the state.
      // It was a smart factor before.
      CHECK(old_smart_factors_.exists(lmk_id));
      // Convert smart to projection.
      bool is_conversion_done = convertSmartToProjectionFactor(
          lmk_id,
          &new_smart_factors_,
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
        addProjectionFactor(lmk_id, new_obs, &new_imu_prior_and_other_factors_);
        // Sanity check, if the conversion was successful, then we should
        // not be able to see the smart factor anymore.
        CHECK(!old_smart_factors_.exists(lmk_id));
        CHECK(new_smart_factors_.find(lmk_id) == new_smart_factors_.end());
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
      addProjectionFactor(lmk_id, new_obs, &new_imu_prior_and_other_factors_);
    }
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackend::updateExistingSmartFactor(
    const LandmarkId& lmk_id,
    const std::pair<FrameId, StereoPoint2>& new_obs,
    LandmarkIdSmartFactorMap* new_smart_factors,
    SmartFactorMap* old_smart_factors) {
  CHECK_NOTNULL(new_smart_factors);
  CHECK_NOTNULL(old_smart_factors);
  CHECK_NE(lmk_id, -1) << "When calling update existing smart factor, the slot "
                          "should already be != -1! \n";

  // Update existing smart-factor.
  const SmartFactorMap::iterator& old_smart_factors_it =
      old_smart_factors->find(lmk_id);
  CHECK(old_smart_factors_it != old_smart_factors->end())
      << "Landmark with id: " << lmk_id << " not found in old_smart_factors_\n";

  // Get old factor.
  SmartStereoFactor::shared_ptr old_factor = old_smart_factors_it->second.first;
  CHECK(old_factor);

  // Clone old factor as a new factor.
  SmartStereoFactor::shared_ptr new_factor =
      boost::make_shared<SmartStereoFactor>(*old_factor);

  // Add observation to new factor.
  VLOG(20) << "Added observation for smart factor of lmk with id: " << lmk_id;
  new_factor->add(
      new_obs.second, gtsam::Symbol('x', new_obs.first), stereo_cal_);

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
bool RegularVioBackend::convertSmartToProjectionFactor(
    const LandmarkId& lmk_id,
    LandmarkIdSmartFactorMap* new_smart_factors,
    SmartFactorMap* old_smart_factors,
    gtsam::Values* new_values,
    gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors,
    gtsam::FactorIndices* delete_slots_of_converted_smart_factors) {
  CHECK_NOTNULL(new_smart_factors);
  CHECK_NOTNULL(old_smart_factors);
  CHECK_NOTNULL(new_values);
  CHECK_NOTNULL(new_imu_prior_and_other_factors);
  CHECK_NOTNULL(delete_slots_of_converted_smart_factors);

  const auto& old_smart_factors_it = old_smart_factors->find(lmk_id);
  CHECK(old_smart_factors_it != old_smart_factors->end())
      << "Landmark not found in old_smart_factors_.";

  SmartStereoFactor::shared_ptr old_factor = old_smart_factors_it->second.first;
  CHECK(old_factor);

  // Add landmark value to graph.
  VLOG(30) << "Print old_factor of lmk_id: " << lmk_id;
  if (VLOG_IS_ON(30)) {
    old_factor->print();
  }

  // Check triangulation result is initialized.
  if (old_factor->point().valid()) {
    CHECK(old_factor->point().is_initialized());
    VLOG(10) << "Performing conversion for lmk with id: " << lmk_id << " from "
             << " smart factor to projection factor.";

    // TODO check all the * whatever, for segmentation fault!
    gtsam::Key lmk_key = gtsam::Symbol('l', lmk_id).key();
    new_values->insert(lmk_key, *(old_factor->point()));

    // DEBUG add prior to lmks.
    // LOG_EVERY_N(ERROR, 100) << "Do not forget to remove lmk prior!";
    // static const gtsam::noiseModel::Diagonal::shared_ptr prior_lmk_noise =
    //    gtsam::noiseModel::Diagonal::Sigmas(Vector3(1, 1, 1));
    // new_imu_prior_and_other_factors_.push_back(
    //      boost::make_shared<gtsam::PriorFactor<gtsam::Point3> >(
    //        lmk_key,
    //        *(old_factor->point()),
    //        prior_lmk_noise));

    // Convert smart factor to multiple projection factors.
    // There is no need to have a threshold for how many observations we want
    // , since it is done already in the updateLmkIsSmart function.
    // Nevertheless, check that there are at least 2 observations.
    CHECK_GE(old_factor->measured().size(), 2);
    for (size_t i = 0; i < old_factor->keys().size(); i++) {
      const gtsam::Symbol& cam_sym = gtsam::Symbol(old_factor->keys().at(i));
      CHECK_LT(i, old_factor->measured().size());
      const StereoPoint2& sp2 = old_factor->measured().at(i);
      std::pair<FrameId, StereoPoint2> obs(
          std::make_pair(cam_sym.index(), sp2));
      VLOG(20) << "Lmk with id: " << lmk_id
               << " added as a new projection factor attached to pose with id: "
               << static_cast<int>(cam_sym.index()) << ".\n";
      addProjectionFactor(lmk_id, obs, &new_imu_prior_and_other_factors_);
    }

    ////////////////// BOOKKEEPING /////////////////////////////////////////
    // Make sure that the smart factor that we converted to projection
    // gets deleted from the graph. If the slot is -1, then it is not in the
    // graph so we do not need to delete it.
    VLOG(20) << "Starting bookkeeping for converted smart factor to "
                "projection factor for lmk with id: "
             << lmk_id;
    if (old_smart_factors_it->second.second != -1) {
      // Get current slot (if factor is already there it must be deleted).
      VLOG(20)
          << "Remove smart factor by adding its slot to the list of slots to "
             "delete from optimization graph";
      delete_slots_of_converted_smart_factors->push_back(
          old_smart_factors_it->second.second);
      // Check that we are not actually updating the smart factor.
      // Otherwise we would have an sporadic smart factor.
      CHECK(new_smart_factors->find(lmk_id) == new_smart_factors->end())
          << "Someone is updating the smart factor while it should be a "
             "projection factor...";
      // NOTE If the slot is -1, so the smart factor is not in the graph yet,
      // but it is maybe going to be added in this iteration as a
      // new_smart_factor...
    }

    // Erase from old_smart_factors_ list since this has been converted into
    // projection factors.
    // Check to avoid undefined behaviour.
    CHECK(old_smart_factors->find(lmk_id) != old_smart_factors->end());
    VLOG(20) << "Erasing lmk with id " << lmk_id << " from old_smart_factors";
    old_smart_factors->erase(lmk_id);
    CHECK(deleteLmkFromFeatureTracks(lmk_id));
    ////////////////////////////////////////////////////////////////////////

    return true;
  } else {
    LOG(WARNING) << "Cannot convert smart factor to proj. factor.\n"
                 << "Smart factor does not have a valid 3D position for lmk: "
                 << lmk_id << "\n"
                 << "Smart factor point status: \n"
                 << old_factor->point();
    return false;
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackend::convertExtraSmartFactorToProjFactor(
    const LandmarkIds& lmk_ids_with_regularity) {
  for (const LandmarkId& lmk_id : lmk_ids_with_regularity) {
    // Check that we are tracking this lmk to avoid not calling
    // updateLmkIdIsSmart beforehand.
    VLOG(20) << "Dealing with lmk id: " << lmk_id;
    if (lmk_id_is_smart_.exists(lmk_id)) {
      if (lmk_id_is_smart_.at(lmk_id)) {
        // We have found a smart factor that should be a projection factor.
        // Convert it to a projection factor, so that we can enforce
        // regularities on it, but only if it is valid.
        CHECK(old_smart_factors_.exists(lmk_id));
        VLOG(20)
            << "We found a smart factor that should be in a regularity for "
               "lmk with id: "
            << lmk_id;
        // We do FLAGS_min_num_obs_for_proj_factor + 1,
        // because we have to substract the fact that in this iteration we added
        // an extra measurement to the factor. This is to be consistent with the
        // check of isSmartFactor3dPointGood when deciding if the 3d point is
        // valid for changing from smart to projection factor. Well this is only
        // true if lmk_id is in the set of lmks observed now... But if it is
        // not, then there is no new observation... Check nwe_smart_factors_ to
        // see if there is indeed a new observation or not.
        if (isSmartFactor3dPointGood(
                old_smart_factors_.at(lmk_id).first,
                (new_smart_factors_.find(lmk_id) != new_smart_factors_.end())
                    ? FLAGS_min_num_obs_for_proj_factor + 1
                    : FLAGS_min_num_obs_for_proj_factor)) {
          VLOG(20) << "Converting extra smart factor to proj factor for lmk"
                      " with id: "
                   << lmk_id << ", since 3d point is good enough";
          if (convertSmartToProjectionFactor(
                  lmk_id,
                  &new_smart_factors_,
                  &old_smart_factors_,
                  &new_values_,
                  &new_imu_prior_and_other_factors_,
                  &delete_slots_of_converted_smart_factors_)) {
            VLOG(30) << "Converted smart factor to proj factor for lmk"
                        " with id: "
                     << lmk_id;
            // Acknowledge the lmk is now in a projection factor.
            lmk_id_is_smart_.at(lmk_id) = false;
          } else {
            VLOG(30) << "NOT converted smart factor to proj factor for lmk"
                        " with id: "
                     << lmk_id;
          }
        } else {
          VLOG(20) << "Not converting extra smart factor to proj factor for lmk"
                      " with id: "
                   << lmk_id << ", since 3d point is not good enough";
        }
      } else {
        VLOG(20) << "The current factor for lmk: " << lmk_id
                 << " is already a projection factor.";
      }
    } else {
      // We are not (yet) tracking this lmk id.
      // We should only get to this condition when the lmk id is
      // so new that it has not even passed the check on minimum number of
      // feature_track size in addLandmarksToGraph.
      CHECK(!old_smart_factors_.exists(lmk_id));
      CHECK(new_smart_factors_.find(lmk_id) == new_smart_factors_.end());
    }
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackend::deleteLmkFromExtraStructures(const LandmarkId& lmk_id) {
  if (lmk_id_is_smart_.find(lmk_id) != lmk_id_is_smart_.end()) {
    LOG(WARNING) << "Delete entrance in lmk_id_is_smart_"
                    " for lmk with id: "
                 << lmk_id;
    lmk_id_is_smart_.erase(lmk_id);
  }

  // Delete the entries related to this lmk id for all planes.
  // NOT tested when using multiple planes...
  for (PlaneIdToLmkIdRegType::value_type& plane_id_to_map :
       plane_id_to_lmk_id_reg_type_) {
    if (plane_id_to_map.second.find(lmk_id) != plane_id_to_map.second.end()) {
      LOG(WARNING) << "Delete entrance in lmk_id_to_regularity_type_map"
                      " for lmk with id: "
                   << lmk_id;
      plane_id_to_map.second.erase(lmk_id);
    }
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackend::addProjectionFactor(
    const LandmarkId& lmk_id,
    const std::pair<FrameId, StereoPoint2>& new_obs,
    gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors) {
  CHECK_NOTNULL(new_imu_prior_and_other_factors);
  if (!std::isnan(new_obs.second.uR())) {
    double parallax = new_obs.second.uL() - new_obs.second.uR();
    if (!std::isnan(parallax)) {
      if (parallax < FLAGS_max_parallax) {
        CHECK_GT(parallax, 0.0);
        new_imu_prior_and_other_factors->push_back(
            boost::make_shared<gtsam::GenericStereoFactor<Pose3, Point3>>(
                new_obs.second,
                stereo_noise_,
                gtsam::Symbol('x', new_obs.first),
                gtsam::Symbol('l', lmk_id),
                stereo_cal_,
                true,
                true,
                B_Pose_leftCamRect_));
      } else {
        LOG(ERROR) << "Parallax for lmk_id: " << lmk_id << " is = " << parallax;
      }
    } else {
      LOG(ERROR) << "Parallax for lmk_id: " << lmk_id << " is NAN.";
    }
  } else {
    // Right pixel has a NAN value for u, use GenericProjectionFactor instead
    // of stereo.
    new_imu_prior_and_other_factors->push_back(
        boost::make_shared<gtsam::GenericProjectionFactor<Pose3, Point3>>(
            gtsam::Point2(new_obs.second.uL(), new_obs.second.v()),
            mono_noise_,
            gtsam::Symbol('x', new_obs.first),
            gtsam::Symbol('l', lmk_id),
            mono_cal_,
            true,
            true,
            B_Pose_leftCamRect_));
  }
}

/* -------------------------------------------------------------------------- */
// TODO keep deleting the used lmk_ids_with_regularities to diminish
// runtime of the algorithm.
bool RegularVioBackend::updateLmkIdIsSmart(
    const LandmarkId& lmk_id,
    const LandmarkIds& lmk_ids_with_regularity,
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
  // feed to the mesher that then sends lmk_ids_with_regularity.
  // i.e. if the mesher has lmks that are in the keyframe but not in the
  // optimization, it won't work...
  const auto& lmk_id_slot = lmk_id_is_smart->find(lmk_id);
  if (std::find(lmk_ids_with_regularity.begin(),
                lmk_ids_with_regularity.end(),
                lmk_id) == lmk_ids_with_regularity.end()) {
    VLOG(20) << "Lmk_id = " << lmk_id
             << " needs to stay as it is since it is "
                "NOT involved in any regularity.";
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
             << " needs to be a proj. factor, as it "
                "is involved in a regularity.";
    const auto& old_smart_factors_it = old_smart_factors_.find(lmk_id);
    if (old_smart_factors_it == old_smart_factors_.end()) {
      // This should only happen if the lmk was already in a regularity,
      // and subsequently updated as a a projection factor...
      VLOG(20) << "Landmark with id: " << lmk_id
               << " not found in old_smart_factors.";
      // This lmk must be tracked.
      CHECK(lmk_id_slot != lmk_id_is_smart->end());
      // This lmk must not be a smart factor.
      CHECK(lmk_id_is_smart->at(lmk_id) == false);
    } else {
      // We found the factor.

      // Get whether the smart factor is valid or not.
      SmartStereoFactor::shared_ptr old_factor =
          old_smart_factors_it->second.first;
      // Mind that we are going to add an extra measurement in this iteration
      // so in reality we are checking that the factor number of measurements
      // (without the new measurement) accounts for at least min_num_obs...
      bool lmk_is_valid = isSmartFactor3dPointGood(
          old_factor, FLAGS_min_num_obs_for_proj_factor);

      if (lmk_id_slot == lmk_id_is_smart->end()) {
        // We did not find the lmk_id in the lmk_id_is_smart_ map.
        // Add it as a projection factor only if it is valid.
        // TODO use an enum instead of bool for lmk_id_is_smart, otherwise
        // it is too difficult to read.
        lmk_id_is_smart->insert(
            std::make_pair(lmk_id, lmk_is_valid ? false : true));
      } else {
        if (lmk_is_valid) {
          // Change it to a projection factor.
          lmk_id_is_smart->at(lmk_id) = false;
        } else {
          // Keep it to a smart factor.
          // We cannot allow conversion back from proj to smart factor.
          // So just check that this is still smart, otherwise we are stuck.
          CHECK(lmk_id_is_smart->at(lmk_id) == true);
          // lmk_id_is_smart->at(lmk_id) = true;
        }
      }
    }
  }

  CHECK(lmk_id_is_smart->find(lmk_id) != lmk_id_is_smart->end());
  return lmk_id_is_smart->at(lmk_id);
}

/* -------------------------------------------------------------------------- */
// Returns whether the lmk of the smart factor is valid:
// - It checks that the point is valid.
// - It checks that the point has been seen at least a certain number of times.
bool RegularVioBackend::isSmartFactor3dPointGood(
    SmartStereoFactor::shared_ptr factor,
    const size_t& min_num_of_observations) {
  CHECK(factor);
  if (!(factor->point().valid())) {
    // The point is not valid.
    VLOG(20) << "Smart factor is NOT valid.";
    return false;
  } else {
    CHECK(factor->point().is_initialized());
    CHECK(factor->point());
    CHECK(!factor->isDegenerate());
    CHECK(!factor->isFarPoint());
    CHECK(!factor->isOutlier());
    CHECK(!factor->isPointBehindCamera());
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
    // ALSO, this is not taking into account that we are going to add a new
    // observation next...
    if (factor->measured().size() >= min_num_of_observations) {
      VLOG(20) << "Smart factor is valid with: " << factor->measured().size()
               << " observations (wo the one we are going to add now).";
      return true;
    } else {
      // Should not be a warning this, but just in case.
      LOG(WARNING) << "Smart factor has not enough"
                   << " observations: " << factor->measured().size()
                   << ", but should be more or equal to "
                   << min_num_of_observations;
      return false;
    }
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackend::addRegularityFactors(
    const Plane& plane,
    LmkIdToRegularityTypeMap* lmk_id_to_regularity_type_map,
    std::vector<std::pair<Slot, LandmarkId>>*
        idx_of_point_plane_factors_to_add) {
  CHECK_NOTNULL(lmk_id_to_regularity_type_map);
  CHECK_NOTNULL(idx_of_point_plane_factors_to_add);
  idx_of_point_plane_factors_to_add->resize(0);

  const LandmarkIds& plane_lmk_ids = plane.lmk_ids_;

  VLOG(10) << "Starting addRegularityFactors...";
  const gtsam::Symbol& plane_symbol = plane.getPlaneSymbol();
  const gtsam::Key& plane_key = plane_symbol.key();
  if (!state_.exists(plane_key)) {
    VLOG(10) << "Plane key, " << gtsam::DefaultKeyFormatter(plane_key)
             << " does NOT exist.";
    // WARNING one plane assumption!
    // If the plane is new, and we are only using one plane as regularity
    // then there should be no lmk_id with a regularity now...
    CHECK_EQ(lmk_id_to_regularity_type_map->size(), 0);

    // Verify that the plane is going to be fully constrained before adding it.
    // Variables to check whether the new plane is going to be fully
    // constrained or not.
    bool is_plane_constrained = false;
    std::vector<LandmarkId> list_of_constraints;

    // Loop over all lmks which are involved in the regularity, to both
    // check that the new plane is going to be fully constrained and that
    // in case it is we add all the corresponding factors.
    for (const LandmarkId& lmk_id : plane_lmk_ids) {
      if (state_.exists(gtsam::Symbol('l', lmk_id)) ||
          new_values_.exists(gtsam::Symbol('l', lmk_id))) {
        // Lmk id exists either in the state or it is going to be added in the
        // next optimization.
        VLOG(20)
            << "Lmk id: " << lmk_id
            << " is in state_ or is going to be added in next optimization.";

        const auto& lmk_id_regularity_type =
            lmk_id_to_regularity_type_map->find(lmk_id);
        if (lmk_id_regularity_type == lmk_id_to_regularity_type_map->end()) {
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
            idx_of_point_plane_factors_to_add->push_back(std::make_pair(
                new_imu_prior_and_other_factors_.size(), lmk_id));
            new_imu_prior_and_other_factors_.push_back(
                boost::make_shared<gtsam::PointPlaneFactor>(
                    gtsam::Symbol('l', lmk_id),
                    plane_key,
                    point_plane_regularity_noise_));
            // Acknowledge that this lmk has been used in a regularity.
            (*lmk_id_to_regularity_type_map)[lmk_id] =
                RegularityType::POINT_PLANE;
          }

          const auto min_num_plane_constraints = static_cast<size_t>(
              FLAGS_min_num_of_plane_constraints_to_add_factors);
          if (list_of_constraints.size() > min_num_plane_constraints) {
            // Acknowledge that the plane is constrained.
            is_plane_constrained = true;

            //            static size_t i = 0;
            //            *plane_symbol = gtsam::Symbol('P', 0); //
            //            gtsam::Symbol('P', i); plane_key =
            //            plane_symbol->key(); i++;

            // TODO find a way to add initial guess, maybe when sending the
            // lmk_ids having a regularity we could regress a plane through it?
            // The mesher should send the plane!
            const gtsam::OrientedPlane3 plane_value(plane.normal_.x,
                                                    plane.normal_.y,
                                                    plane.normal_.z,
                                                    plane.distance_);

            // The plane is constrained, add it.
            VLOG(10) << "Adding new plane with key: "
                     << gtsam::DefaultKeyFormatter(plane_key)
                     << " as a new variable in Backend.\n"
                     << "\tWith normal: " << plane_value.normal()
                     << "\tWith distance: " << plane_value.distance();
            new_values_.insert(plane_key, plane_value);

            // TODO Remove! DEBUG add a prior to the plane.
            // static const gtsam::noiseModel::Diagonal::shared_ptr prior_noise
            // =
            //    gtsam::noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.5));
            // new_imu_prior_and_other_factors_.push_back(
            //      boost::make_shared<gtsam::PriorFactor<gtsam::OrientedPlane3>
            //      >(
            //        plane_key,
            //        plane_value,
            //        prior_noise));

            // Add the factors for the lmks that we skipped while checking
            // that the plane is fully constrained.
            for (const LandmarkId& prev_lmk_id : list_of_constraints) {
              VLOG(20) << "Adding PointPlaneFactor.";
              idx_of_point_plane_factors_to_add->push_back(std::make_pair(
                  new_imu_prior_and_other_factors_.size(), prev_lmk_id));
              new_imu_prior_and_other_factors_.push_back(
                  boost::make_shared<gtsam::PointPlaneFactor>(
                      gtsam::Symbol('l', prev_lmk_id),
                      plane_key,
                      point_plane_regularity_noise_));
              // Acknowledge that this lmk has been used in a regularity.
              (*lmk_id_to_regularity_type_map)[prev_lmk_id] =
                  RegularityType::POINT_PLANE;
            }

            // Clear list of constraints, so that we do not enter in this
            // condition anymore.
            list_of_constraints.resize(0);
          }
        } else {
          LOG(ERROR) << "If this is a new plane, the lmks should not have any "
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
        if (lmk_id_to_regularity_type_map->find(lmk_id) !=
            lmk_id_to_regularity_type_map->end()) {
          lmk_id_to_regularity_type_map->erase(lmk_id);
        }
      }
    }

    if (!is_plane_constrained) {
      VLOG(10) << "Plane with key: " << gtsam::DefaultKeyFormatter(plane_key)
               << " is not sufficiently constrained:\n"
               << "\tConstraints: " << list_of_constraints.size() << "\n"
               << "\tMin number of constraints: "
               << FLAGS_min_num_of_plane_constraints_to_add_factors;
    } else {
      VLOG(10) << "Plane with key: " << gtsam::DefaultKeyFormatter(plane_key)
               << " is sufficiently constrained with "
               << idx_of_point_plane_factors_to_add->size() << " constraints.";
    }

    // Reset the flag for next time we have to add a plane.
    is_plane_constrained = false;
    // Also reset vector of constraints to empty.
    list_of_constraints.resize(0);
  } else {
    VLOG(10) << "Plane key does exist already: "
             << gtsam::DefaultKeyFormatter(plane_key);

    // The plane exists, just add regularities that are ok.
    for (const LandmarkId& lmk_id : plane_lmk_ids) {
      if (state_.exists(gtsam::Symbol('l', lmk_id)) ||
          new_values_.exists(gtsam::Symbol('l', lmk_id))) {
        // Lmk id exists either in the state or it is going to be added in the
        // next optimization.
        VLOG(20)
            << "Lmk id: " << lmk_id
            << " is in state_ or is going to be added in next optimization.";

        const auto& lmk_id_regularity_type =
            lmk_id_to_regularity_type_map->find(lmk_id);
        if (lmk_id_regularity_type == lmk_id_to_regularity_type_map->end() ||
            lmk_id_regularity_type->second != RegularityType::POINT_PLANE) {
          // Lmk has not been used in a regularity or is not in a point plane
          // factor.
          VLOG(20) << "Lmk id: " << lmk_id
                   << " has not yet been used in a regularity.";

          // The plane is already in the graph so it must be fully constrained,
          // keep adding new factors.
          VLOG(20) << "Adding PointPlaneFactor.";
          idx_of_point_plane_factors_to_add->push_back(
              std::make_pair(new_imu_prior_and_other_factors_.size(), lmk_id));
          new_imu_prior_and_other_factors_.push_back(
              boost::make_shared<gtsam::PointPlaneFactor>(
                  gtsam::Symbol('l', lmk_id),
                  plane_key,
                  point_plane_regularity_noise_));
          // Acknowledge that this lmk has been used in a regularity.
          (*lmk_id_to_regularity_type_map)[lmk_id] =
              RegularityType::POINT_PLANE;
        } else {
          // This lmk has already been used in a regularity and is a point plane
          // factor, avoid duplication of factors.
          VLOG(20) << "Avoiding duplicated regularity factor for lmk id: "
                   << lmk_id;
        }
      } else {
        LOG(WARNING) << "Lmk id: " << lmk_id
                     << " is NOT in state_ or in new_values_,"
                     << " NOT adding PointPlaneFactor.";

        // Erase from map of lmk id to regularity type, since the lmk does not
        // exist anymore, or has never existed.
        // Avoid the world of undefined behaviour by checking that the lmk_id
        // we want to erase is actually there.
        if (lmk_id_to_regularity_type_map->find(lmk_id) !=
            lmk_id_to_regularity_type_map->end()) {
          lmk_id_to_regularity_type_map->erase(lmk_id);
        }
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackend::removeOldRegularityFactors_Slow(
    const std::vector<Plane>& planes,
    const std::map<PlaneId, std::vector<std::pair<Slot, LandmarkId>>>&
        map_idx_of_point_plane_factors_to_add,
    PlaneIdToLmkIdRegType* plane_id_to_lmk_id_to_reg_type_map,
    gtsam::FactorIndices* delete_slots) {
  CHECK_NOTNULL(plane_id_to_lmk_id_to_reg_type_map);
  CHECK_NOTNULL(delete_slots);

  std::vector<size_t> plane_idx_to_clean;
  std::map<size_t, std::vector<std::pair<Slot, LandmarkId>>>
      plane_id_to_factor_slots_bad;
  std::map<size_t, std::vector<std::pair<Slot, LandmarkId>>>
      plane_id_to_factor_slots_good;
  std::map<size_t, bool> has_plane_a_prior_map;
  std::map<size_t, bool> has_plane_a_linear_factor_map;
  std::map<size_t, Slot> plane_prior_slot_map;
  size_t i = 0;
  for (const Plane& plane : planes) {
    const gtsam::Symbol& plane_symbol = plane.getPlaneSymbol();
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
               << " is not in state.";
      i++;
      continue;
    }

    if (new_values_.exists(plane_symbol.key())) {
      // The plane is not in the state, and it is going to be added in
      // this iteration, so it must be already well constrained and attached
      // to the right lmks.
      VLOG(10) << "Plane with id " << gtsam::DefaultKeyFormatter(plane_symbol)
               << " is in new_values_.";
      i++;
      continue;
    }

    plane_idx_to_clean.push_back(i);
    // Init data structures empty, by using [] instead of .at().
    plane_id_to_factor_slots_bad[i];
    plane_id_to_factor_slots_good[i];
    has_plane_a_prior_map[i] = false;
    has_plane_a_linear_factor_map[i] = false;
    plane_prior_slot_map[i] = 0;  // Set as invalid slot.

    i++;
  }

  VLOG(10) << "Starting removeOldRegularityFactors_Slow...";

  // If the plane exists in the state_ and not in new_values_,
  // then let us remove old regularity factors.
  const gtsam::NonlinearFactorGraph& graph = smoother_->getFactors();
  Slot slot = 0;
  // Loop over current graph.
  for (const auto& g : graph) {
    if (g) {
      const auto& ppf = boost::dynamic_pointer_cast<gtsam::PointPlaneFactor>(g);
      const auto& plane_prior = boost::dynamic_pointer_cast<
          gtsam::PriorFactor<gtsam::OrientedPlane3>>(g);
      const auto& lcf =
          boost::dynamic_pointer_cast<gtsam::LinearContainerFactor>(g);
      if (ppf) {
        // We found a PointPlaneFactor.
        for (const size_t& plane_id : plane_idx_to_clean) {
          const gtsam::Symbol& plane_symbol =
              planes.at(plane_id).getPlaneSymbol().key();
          if (plane_symbol == ppf->getPlaneKey()) {
            // We found the plane involved in current PointPlaneFactor.
            const LandmarkId& lmk_id =
                gtsam::Symbol(ppf->getPointKey()).index();
            const LandmarkIds& plane_lmk_ids = planes.at(plane_id).lmk_ids_;
            // Try to find this lmk id in the set of lmks of the plane.
            if (std::find(plane_lmk_ids.begin(), plane_lmk_ids.end(), lmk_id) ==
                plane_lmk_ids.end()) {
              // We did not find the point in plane's lmks, therefore it should
              // not be involved in a regularity anymore, delete this slot.
              // (but I want to remove the landmark! and all its factors,
              // to avoid having underconstrained lmks...)
              VLOG(20) << "Found bad point plane factor on lmk with id: "
                       << lmk_id;
              plane_id_to_factor_slots_bad.at(plane_id).push_back(
                  std::make_pair(slot, lmk_id));

              // Before deleting this slot, we must ensure that both the plane
              // and the landmark are well constrained!
            } else {
              // Store those factors that we will potentially keep.
              plane_id_to_factor_slots_good.at(plane_id).push_back(
                  std::make_pair(slot, lmk_id));
            }
          }
        }
      } else if (plane_prior) {
        for (const size_t& plane_idx : plane_idx_to_clean) {
          const gtsam::Symbol& plane_symbol =
              planes.at(plane_idx).getPlaneSymbol().key();
          if (plane_prior->find(plane_symbol) != plane_prior->end()) {
            LOG(WARNING) << "Found plane prior for plane: "
                         << gtsam::DefaultKeyFormatter(plane_symbol.key());
            // Store slot of plane_prior, since we might have to delete it
            // if the plane has no constraints.
            plane_prior_slot_map.at(plane_idx) = slot;
            has_plane_a_prior_map.at(plane_idx) = true;
          }
        }
      } else if (lcf) {
        for (const size_t& plane_idx : plane_idx_to_clean) {
          const gtsam::Symbol& plane_symbol =
              planes.at(plane_idx).getPlaneSymbol().key();
          if (lcf->find(plane_symbol.key()) != lcf->end()) {
            VLOG(10) << "Found linear container factor for plane: "
                     << gtsam::DefaultKeyFormatter(plane_symbol.key());
            has_plane_a_linear_factor_map.at(plane_idx) = true;
          }
        }
      }
    }

    // Next slot.
    slot++;
  }

  // Decide whether we can just delete the bad point plane factors,
  // or whether we need to delete all the factors involving the plane
  // so that it is removed.
  // For this we need to check if the plane is fully constrained!
  for (const size_t& plane_idx : plane_idx_to_clean) {
    const gtsam::Symbol& plane_symbol =
        planes.at(plane_idx).getPlaneSymbol().key();
    const std::vector<std::pair<Slot, LandmarkId>>&
        point_plane_factor_slots_bad =
            plane_id_to_factor_slots_bad.at(plane_idx);
    const std::vector<std::pair<Slot, LandmarkId>>&
        point_plane_factor_slots_good =
            plane_id_to_factor_slots_good.at(plane_idx);
    DCHECK(map_idx_of_point_plane_factors_to_add.find(plane_symbol.key()) !=
           map_idx_of_point_plane_factors_to_add.end());
    const std::vector<std::pair<Slot, LandmarkId>>&
        idx_of_point_plane_factors_to_add =
            map_idx_of_point_plane_factors_to_add.at(plane_symbol.key());
    DCHECK(plane_id_to_lmk_id_to_reg_type_map->find(plane_symbol.key()) !=
           plane_id_to_lmk_id_to_reg_type_map->end());
    LmkIdToRegularityTypeMap& lmk_id_to_regularity_type_map =
        (*plane_id_to_lmk_id_to_reg_type_map).at(plane_symbol.key());
    const bool& has_plane_a_prior = has_plane_a_prior_map.at(plane_idx);
    const bool& has_plane_a_linear_factor =
        has_plane_a_linear_factor_map.at(plane_idx);
    const size_t& plane_prior_slot = plane_prior_slot_map.at(plane_idx);
    /// If there are enough new constraints to be added then delete only
    /// delete_slots else, if there are enough constraints left, only delete
    /// delete_slots otherwise delete ALL constraints, both old and new, so that
    /// the plane disappears (take into account priors!). Priors affecting
    /// planes: linear container factor & prior on OrientedPlane3
    const int32_t total_nr_of_plane_constraints =
        point_plane_factor_slots_good.size() +
        idx_of_point_plane_factors_to_add.size();
    VLOG(10) << "Total number of constraints of plane "
             << gtsam::DefaultKeyFormatter(plane_symbol.key())
             << " is: " << total_nr_of_plane_constraints << "\n"
             << "\tConstraints in graph which are good: "
             << point_plane_factor_slots_good.size() << "\n"
             << "\tConstraints that are going to be added: "
             << idx_of_point_plane_factors_to_add.size() << "\n"
             << "Constraints in graph which are bad: "
             << point_plane_factor_slots_bad.size() << "\n"
             << "Has the plane a prior? " << (has_plane_a_prior ? "Yes" : "No")
             << ".\n"
             << "Has the plane a linear factor? "
             << (has_plane_a_linear_factor ? "Yes" : "No") << ".";
    if (total_nr_of_plane_constraints >
        FLAGS_min_num_of_plane_constraints_to_remove_factors) {
      // The plane is fully constrained.
      // We can just delete bad factors, assuming lmks will be well constrained.
      // TODO ensure the lmks are themselves well constrained.
      VLOG(10) << "Plane is fully constrained, removing only bad factors.";
      fillDeleteSlots(point_plane_factor_slots_bad,
                      &lmk_id_to_regularity_type_map,
                      delete_slots);
    } else {
      // The plane is NOT fully constrained if we remove all bad factors,
      // unless the plane has a prior.
      // Check if the plane has a prior.
      VLOG(10) << "Plane is NOT fully constrained if we just remove"
                  " the bad factors.";
      if (has_plane_a_prior || has_plane_a_linear_factor) {
        // The plane has a prior.
        VLOG(10) << "Plane has a prior.";
        // TODO Remove: this is just a patch to avoid issue 32:
        // https://github.mit.edu/lcarlone/VIO/issues/32
        if (FLAGS_use_unstable_plane_removal) {
          // This should be the correct way to do it, but a bug in gtsam will
          // make the optimization break.
          if (total_nr_of_plane_constraints == 0 && has_plane_a_prior &&
              !has_plane_a_linear_factor) {
            // Not only the plane is not fully constrained, it has no
            // constraints at all, and we are going to delete the bad ones, so
            // plane floating with a plane prior, not attached to anything
            // else... Delete the prior as well, to get rid of this plane.
            LOG(ERROR)
                << "Plane has no constraints at all, deleting prior as well.";
            CHECK_NE(plane_prior_slot, 0);
            delete_slots->push_back(plane_prior_slot);
          }
        } else {
          // This is just a patch...
          // TODO maybe if we are deleting too much constraints, add a no
          // information factor btw the plane and a lmk!
          if (total_nr_of_plane_constraints >
              FLAGS_min_num_of_plane_constraints_to_avoid_seg_fault) {
            // Delete just the bad factors, since we still have some factors
            // that won't make the optimizer try to delete the plane variable,
            // which at the current time breaks gtsam.
            VLOG(10) << "Delete bad factors attached to plane.";
            fillDeleteSlots(point_plane_factor_slots_bad,
                            &lmk_id_to_regularity_type_map,
                            delete_slots);
          } else {
            // Do not delete all factors, otherwise gtsam will break.
            VLOG(10)
                << "Not deleting bad factors attached to plane, or gtsam will "
                   "break.";
          }
        }
      } else {
        // The plane has NOT a prior.
        if (FLAGS_use_unstable_plane_removal) {
          // Delete all factors involving the plane so that iSAM removes the
          // plane from the optimization.
          LOG(ERROR) << "Plane has no prior, trying to forcefully"
                        " remove the PLANE!";
          debug_smoother_ = true;
          fillDeleteSlots(point_plane_factor_slots_bad,
                          &lmk_id_to_regularity_type_map,
                          delete_slots);
          fillDeleteSlots(point_plane_factor_slots_good,
                          &lmk_id_to_regularity_type_map,
                          delete_slots);

          // Remove as well the factors that are going to be added in this
          // iteration.
          deleteNewSlots(plane_symbol.key(),
                         idx_of_point_plane_factors_to_add,
                         &lmk_id_to_regularity_type_map,
                         &new_imu_prior_and_other_factors_);
        } else {
          // Do not use unstable implementation...
          // Just add a prior on the plane and remove only bad factors...
          // Add a prior to the plane.
          VLOG(10)
              << "Adding a prior to the plane, delete just the bad factors.";
          gtsam::OrientedPlane3 plane_estimate;
          CHECK(getEstimateOfKey(state_, plane_symbol.key(), &plane_estimate));
          LOG(WARNING) << "Using plane prior on plane with id "
                       << gtsam::DefaultKeyFormatter(plane_symbol);
          CHECK(!has_plane_a_prior && !has_plane_a_linear_factor)
              << "Check that the plane has no prior.";
          static const gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
              gtsam::noiseModel::Diagonal::Sigmas(
                  Vector3(FLAGS_prior_noise_sigma_normal,
                          FLAGS_prior_noise_sigma_normal,
                          FLAGS_prior_noise_sigma_distance));
          new_imu_prior_and_other_factors_.push_back(
              boost::make_shared<gtsam::PriorFactor<gtsam::OrientedPlane3>>(
                  plane_symbol.key(), plane_estimate, prior_noise));

          // Delete just the bad factors.
          // TODO Remove: this is just a patch to avoid issue 32:
          // https://github.mit.edu/lcarlone/VIO/issues/32
          if (total_nr_of_plane_constraints >
              FLAGS_min_num_of_plane_constraints_to_avoid_seg_fault) {
            // Delete just the bad factors, since we still have some factors
            // that won't make the optimizer try to delete the plane variable,
            // which at the current time breaks gtsam.
            VLOG(10) << "Delete bad factors attached to plane.";
            fillDeleteSlots(point_plane_factor_slots_bad,
                            &lmk_id_to_regularity_type_map,
                            delete_slots);
          } else {
            // Do not delete all factors, otherwise gtsam will break.
            VLOG(10)
                << "Not deleting bad factors attached to plane, or gtsam will "
                   "break.";
          }
        }
      }  // The plane has NOT a prior.
    }    // The plane is NOT fully constraint.
  }
  //  // TODO now the plane could be floating around with a prior attached, but
  //  // not really attached to the rest of the graph...

  //  //  gtsam::Point3 point;
  //  //  if (getEstimateOfKey(state_, point_symbol.key(), &point)) {
  //  //    LOG(WARNING) << "Using lmk prior on lmk with id " <<
  //  //                    gtsam::DefaultKeyFormatter(point_symbol);
  //  //    // TODO make sure this prior is not repeated over and over on the
  //  same
  //  //    // lmk id.
  //  //    // TODO is there the possibility that this lmk just have a prior
  //  attached
  //  //    // to it? and nothing else, I guess that it is unlikely because to
  //  //    // be first added it must have had some proj factors besides
  //  PointPlane
  //  //    // factor...
  //  //    static const gtsam::noiseModel::Diagonal::shared_ptr prior_lmk_noise
  //  =
  //  //        gtsam::noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 1.0));
  //  //    new_imu_prior_and_other_factors_.push_back(
  //  //          boost::make_shared<gtsam::PriorFactor<gtsam::Point3>>(
  //  //            point_symbol.key(),
  //  //            point,
  //  //            prior_lmk_noise));
  //  //  } else {
  //  //    LOG(ERROR) << "Lmk with id " <<
  //  gtsam::DefaultKeyFormatter(point_symbol)
  //  //               << " does not exist in graph.";
  //  //  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackend::fillDeleteSlots(
    const std::vector<std::pair<Slot, LandmarkId>>&
        point_plane_factor_slots_bad,
    LmkIdToRegularityTypeMap* lmk_id_to_regularity_type_map,
    gtsam::FactorIndices* delete_slots) {
  CHECK_NOTNULL(lmk_id_to_regularity_type_map);
  CHECK_NOTNULL(delete_slots);
  VLOG(10) << "Starting fillDeleteSlots...";
  if (point_plane_factor_slots_bad.size() > 0) {
    size_t prev_delete_slots_size = delete_slots->size();
    delete_slots->resize(prev_delete_slots_size +
                         point_plane_factor_slots_bad.size());
    size_t i = prev_delete_slots_size;
    for (const std::pair<Slot, LandmarkId>& ppf_bad :
         point_plane_factor_slots_bad) {
      CHECK_LT(i, delete_slots->size());
      CHECK(smoother_->getFactors().exists(ppf_bad.first));

      // Add factor slot to delete slots.
      delete_slots->at(i) = ppf_bad.first;
      i++;

      // Acknowledge that these lmks are not in a regularity anymore.
      // TODO this does not generalize to multiple planes...
      const LandmarkId& lmk_id = ppf_bad.second;
      CHECK(lmk_id_to_regularity_type_map->find(lmk_id) !=
            lmk_id_to_regularity_type_map->end())
          << "Could not delete lmk_id " << lmk_id
          << " from lmk_id_to_regularity_type_map.";
      VLOG(20) << "Deleting lmk_id " << lmk_id
               << " from lmk_id_to_regularity_type_map_";
      lmk_id_to_regularity_type_map->erase(lmk_id);
    }
  } else {
    VLOG(10) << "There are no bad factors to remove.";
  }

  VLOG(10) << "Finished fillDeleteSlots...";
}

/* -------------------------------------------------------------------------- */
// Remove as well the factors that are going to be added in this iteration.
void RegularVioBackend::deleteNewSlots(
    const PlaneId& plane_key,
    const std::vector<std::pair<Slot, LandmarkId>>&
        idx_of_point_plane_factors_to_add,
    LmkIdToRegularityTypeMap* lmk_id_to_regularity_type_map,
    gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors) {
  CHECK_NOTNULL(lmk_id_to_regularity_type_map);
  CHECK_NOTNULL(new_imu_prior_and_other_factors);

  bool clean_nullptrs = false;
  for (const std::pair<Slot, LandmarkId>& i :
       idx_of_point_plane_factors_to_add) {
    // Sanity checks.
    // The slot is valid.
    CHECK(new_imu_prior_and_other_factors->exists(i.first));

    const auto& ppf = boost::dynamic_pointer_cast<gtsam::PointPlaneFactor>(
        new_imu_prior_and_other_factors->at(i.first));
    // The factor is really a point plane one.
    CHECK(ppf);
    // The factor is the one related to the right lmk.
    CHECK(ppf->getPointKey() == gtsam::Symbol('l', i.second).key());
    // The factor is the one related to the right plane.
    CHECK(ppf->getPlaneKey() == plane_key);

    // WARNING using erase moves all factors! aka the
    // idx_of_point_plane_factors_to_add will be wrong!!!! Remove will just
    // insert a nullptr, is this ok for iSAM??
    new_imu_prior_and_other_factors->remove(i.first);
    clean_nullptrs = true;

    // Acknowledge that these lmks are not in a regularity anymore.
    // TODO this does not generalize to multiple planes...
    const LandmarkId& lmk_id = i.second;
    CHECK(lmk_id_to_regularity_type_map->find(lmk_id) !=
          lmk_id_to_regularity_type_map->end())
        << "Avoid undefined behaviour by checking that the lmk was in the "
           "container.";
    lmk_id_to_regularity_type_map->erase(lmk_id);
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

/* -------------------------------------------------------------------------- */
// Output a noise model with a selected norm type:
// norm_type = 0: l-2.
// norm_type = 1: Huber.
// norm_type = 2: Tukey.
void RegularVioBackend::selectNormType(
    gtsam::SharedNoiseModel* noise_model_output,
    const gtsam::SharedNoiseModel& noise_model_input,
    const size_t& norm_type,
    const double& norm_type_parameter) {
  CHECK_NOTNULL(noise_model_output);
  switch (norm_type) {
    case 0: {
      VLOG(1) << "Using l-2 norm.";
      *noise_model_output = noise_model_input;
      break;
    }
    case 1: {
      VLOG(1) << "Using Huber norm, with parameter value: "
              << norm_type_parameter;
      *noise_model_output = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(
              norm_type_parameter,
              gtsam::noiseModel::mEstimator::Huber::Scalar),  // Default is
                                                              // Block
          noise_model_input);
      break;
    }
    case 2: {
      VLOG(1) << "Using Tukey norm, with parameter value: "
              << norm_type_parameter;
      *noise_model_output = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Tukey::Create(
              norm_type_parameter,
              gtsam::noiseModel::mEstimator::Tukey::Scalar),  // Default is
                                                              // Block
          noise_model_input);                                 // robust
      break;
    }
    default: {
      LOG(ERROR) << "Wrong norm_type passed...";
      break;
    }
  }
}

/* -------------------------------------------------------------------------- */
// Extract all lmk ids, wo repetition, from the set of planes.
void RegularVioBackend::extractLmkIdsFromPlanes(
    const std::vector<Plane>& planes,
    LandmarkIds* lmk_ids_with_regularity) const {
  CHECK_NOTNULL(lmk_ids_with_regularity);
  for (const Plane& plane : planes) {
    for (const LandmarkId& lmk_id : plane.lmk_ids_) {
      // Ensure we are not adding more than once the same lmk_id.
      const auto& it = std::find(lmk_ids_with_regularity->begin(),
                                 lmk_ids_with_regularity->end(),
                                 lmk_id);
      if (it == lmk_ids_with_regularity->end()) {
        // The lmk id is not present in the lmk_ids vector, add it.
        lmk_ids_with_regularity->push_back(lmk_id);
      } else {
        // The lmk id is already in the lmk_ids vector, do not add it.
        continue;
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
// Update plane normal and distance if the plane could be found in the state.
// Otherwise, erase the plane.
void RegularVioBackend::updatePlaneEstimates(std::vector<Plane>* planes) {
  CHECK_NOTNULL(planes);
  gtsam::OrientedPlane3 plane_estimate;
  for (std::vector<Plane>::iterator plane_it = planes->begin();
       plane_it != planes->end();) {
    const PlaneId& plane_key = plane_it->getPlaneSymbol().key();
    if (getEstimateOfKey(state_, plane_key, &plane_estimate)) {
      // We found the plane in the state.
      // Update the plane.
      VLOG(10) << "Update plane with id "
               << gtsam::DefaultKeyFormatter(plane_key)
               << " from the set of planes.";
      plane_it->normal_ = cv::Point3d(plane_estimate.normal().point3().x(),
                                      plane_estimate.normal().point3().y(),
                                      plane_estimate.normal().point3().z());
      plane_it->distance_ = plane_estimate.distance();
      VLOG(10) << "\t Updated plane normal = " << plane_it->normal_ << "\n"
               << "\t Updated plane distance = " << plane_it->distance_;

      plane_it++;
    } else {
      // We did not find the plane in the state.
      VLOG(10) << "Erase plane with id "
               << gtsam::DefaultKeyFormatter(plane_key)
               << " from the set of planes, since it is not in the state"
                  " anymore, or it has never been added.";

      // Clean data structures involving this plane.
      if (plane_id_to_lmk_id_reg_type_.find(plane_key) !=
          plane_id_to_lmk_id_reg_type_.end()) {
        plane_id_to_lmk_id_reg_type_.erase(plane_key);
      } else {
        LOG(WARNING) << "Plane " << gtsam::DefaultKeyFormatter(plane_key) << " "
                     << "not found in plane_id_to_lmk_id_reg_type_, this should"
                        " only happen if we are not having a VALID tracking "
                        "status, since then we are not updating planes.";
      }

      // Delete the plane.
      plane_it = planes->erase(plane_it);
    }
  }
}

}  // namespace VIO
