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

  // Set type of mono_noise_ for projection factors.
  gtsam::SharedNoiseModel gaussian =
      gtsam::noiseModel::Isotropic::Sigma(2, vio_params_.monoNoiseSigma_);

  switch (vio_params_.normType_) {
    case 0: {
      LOG(INFO) << "Using square norm.";
      mono_noise_ = gaussian;
      break;
    }
    case 1: {
      LOG(INFO) << "Using Huber norm, with parameter value: " << vio_params_.huberParam_;
      mono_noise_ =
          gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(
              vio_params_.huberParam_,
              gtsam::noiseModel::mEstimator::Huber::Scalar), // Default is Block
            gaussian);
      break;
    }
    case 2: {
      LOG(INFO) << "Using Tukey norm, with parameter value: " << vio_params_.tukeyParam_;
      mono_noise_ = gtsam::noiseModel::Robust::Create(
                      gtsam::noiseModel::mEstimator::Tukey::Create(
                        vio_params_.tukeyParam_,
                        gtsam::noiseModel::mEstimator::Tukey::Scalar), // Default is Block
                      gaussian); //robust
      break;
    }
    default: {
      LOG(INFO) << "Using square norm.";
      mono_noise_ = gtsam::noiseModel::Isotropic::Sigma(
                      2, vio_params_.monoNoiseSigma_);

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
  VLOG(20) << "Adding IMU factor between pose id: " << last_kf_id_
          << " and pose id: " << cur_kf_id_;
  addImuFactor(last_kf_id_, cur_kf_id_);

  /////////////////// STEREO RANSAC FACTORS ////////////////////////////////////
  // Add between factor from RANSAC.
  if (stereo_ransac_body_pose) {
    VLOG(20) << "Adding RANSAC factor between pose id: " << last_kf_id_
            << " and pose id: " << cur_kf_id_;
    if (VLOG_IS_ON(20)) {
      stereo_ransac_body_pose->print();
    }
    addBetweenFactor(last_kf_id_, cur_kf_id_, *stereo_ransac_body_pose);
  }

  /////////////////// VISION MEASUREMENTS //////////////////////////////////////
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
        // For each landmark we decide if it's going to be a smart factor or not.
        // TODO here there is a timeline mismatch, while landmarks_kf are only currently
        // visible lmks, mesh_lmks_ids contains lmks in time_horizon!
        isLandmarkSmart(lmks_kf,
                        mesh_lmk_ids_ground_cluster,
                        &lmk_id_is_smart_);

        // We add features in VIO.
        VLOG(10) << "Adding/Updating landmarks to graph.";
        addLandmarksToGraph(lmks_kf);

        /////////////////// REGULARITY FACTORS ///////////////////////////////////////
        // Add regularity factor on vertices of the mesh.
        // TODO argument should be generalized to diff
        // type of cluster and regularities.
        if (mesh_lmk_ids_ground_cluster.size() != 0) {
          // TODO what happens if mesh has same ids over and over, are we duplicating
          // factors?
          gtsam::Symbol plane_symbol;
          size_t nr_of_plane_constraints;
          addRegularityFactors(mesh_lmk_ids_ground_cluster,
                               &plane_symbol,
                               &nr_of_plane_constraints);
          //removeOldRegularityFactors_Slow(plane_symbol,
          //                                nr_of_plane_constraints,
          //                                mesh_lmk_ids_ground_cluster,
          //                                &delete_old_regularity_factors);
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
void RegularVioBackEnd::addLandmarksToGraph(const LandmarkIds& lmks_kf) {
  // Add selected landmarks to graph:
  int n_new_landmarks = 0;
  int n_updated_landmarks = 0;
  debug_info_.numAddedSmartF_ += lmks_kf.size();

  for (const LandmarkId& lmk_id: lmks_kf) {
    FeatureTrack& feature_track = feature_tracks_.at(lmk_id);

    // Only insert feature tracks of length at least 2
    // (otherwise uninformative)
    static constexpr size_t min_num_of_observations = 2;
    if (feature_track.obs_.size() < min_num_of_observations) {
      VLOG(10) << "Feature track is shorter (" << feature_track.obs_.size()
               << ") than min_num_of_observations (" << min_num_of_observations
               << ") for lmk with id: " << lmk_id;
      continue;
    }

    if (!feature_track.in_ba_graph_) {
      // Acknowledge that we have added the landmark in the graph.
      feature_track.in_ba_graph_ = true;
      VLOG(10) << "Adding lmk " << lmk_id << " to graph.";
      addLandmarkToGraph(lmk_id, feature_track);
      ++n_new_landmarks;
    } else {
      const std::pair<FrameId, StereoPoint2>& obs_kf = feature_track.obs_.back();

      // Sanity check.
      CHECK_EQ(obs_kf.first, cur_kf_id_) << "Last obs is not from the current"
                                            " keyframe!";

      VLOG(10) << "Updating lmk " << lmk_id << " to graph.";
      updateLandmarkInGraph(lmk_id, obs_kf);
      ++n_updated_landmarks;
    }
  }
  VLOG(7) << "Added " << n_new_landmarks << " new landmarks.\n"
          << "Updated " << n_updated_landmarks << " landmarks in graph.";
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::addLandmarkToGraph(const LandmarkId& lmk_id,
                                           const FeatureTrack& ft) {
  // All landmarks should be smart the first time we add them to the graph.
  // Add as a smart factor.
  // We use a unit pinhole projection camera for the smart factors to be
  // more efficient.
  SmartStereoFactor::shared_ptr new_factor(new SmartStereoFactor(
                                             smart_noise_,
                                             smart_factors_params_,
                                             B_Pose_leftCam_));

  VLOG(10) << "Adding landmark with id: " << lmk_id
           << " for the first time to graph. \n"
           << "Nr of observations of the lmk: " << ft.obs_.size()
           << " observations.\n";
  if (VLOG_IS_ON(20)) {
    new_factor->print();
  }

  // Add observations to smart factor.
  VLOG(10) << "Creating smart factor involving lmk with id: " << lmk_id;
  for (const std::pair<FrameId, StereoPoint2>& obs: ft.obs_) {
    VLOG(10) << "SmartFactor: adding observation of lmk with id: " << lmk_id
            << " from frame with id: " << obs.first;

    new_factor->add(obs.second,
                    gtsam::Symbol('x', obs.first),
                    stereo_cal_);
  }

  /////////////////////// BOOK KEEPING /////////////////////////////////////////
  // Set lmk id to smart.
  CHECK(lmk_id_is_smart_.find(lmk_id) != lmk_id_is_smart_.end());
  CHECK(lmk_id_is_smart_.at(lmk_id) == true);

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
    const std::pair<FrameId, StereoPoint2>& newObs) {

  // We are not tracking whether the lmk is smart or not, but we should.
  CHECK(lmk_id_is_smart_.find(lmk_id) != lmk_id_is_smart_.end())
      << "Lmk with id: " << lmk_id << " is not being tracked whether it is "
                                      "smart or not...";

  const bool& is_lmk_smart = lmk_id_is_smart_.at(lmk_id);
  if (is_lmk_smart) {
    // Lmk is meant to be smart.
    VLOG(10) << "Lmk with id: " << lmk_id << " is set to be smart.\n";

    // Update existing smart-factor.
    const SmartFactorMap::iterator& old_smart_factors_it =
        old_smart_factors_.find(lmk_id);
    CHECK(old_smart_factors_it != old_smart_factors_.end())
        << "Landmark not found in old_smart_factors_\n";

    // Get old factor.
    SmartStereoFactor::shared_ptr old_factor =
        old_smart_factors_it->second.first;

    // Clone old factor as a new factor.
    SmartStereoFactor::shared_ptr new_factor =
        boost::make_shared<SmartStereoFactor>(*old_factor);

    // Add observation to new factor.
    VLOG(10) << "Added observation for smart factor of lmk with id: "
             << lmk_id;
    new_factor->add(newObs.second,
                    gtsam::Symbol('x', newObs.first),
                    stereo_cal_);

    // If slot is still -1, it means that the factor has not been inserted yet
    // in the graph.
    CHECK(old_smart_factors_it->second.second != -1) << "When calling update "
                      "the slot should be already != -1";

    // Slot is different than -1
    // It means that the factor has already been inserted in the graph.
    VLOG(10) << "Insert new smart factor to new_smart_factors_ for lmk with"
             << " id: " << lmk_id;

    ///// Book Keeping, update factors /////////////////////////////////////////
    // Update the set of new smart factors.
    new_smart_factors_.insert(std::make_pair(lmk_id, new_factor));

    // TODO Why do we do this??
    // if we don't the 3d points seem to be off.
    // is there a way to viz 3d points?
    old_smart_factors_it->second.first = new_factor;

    ////////////////////////////////////////////////////////////////////////////

  } else {
    VLOG(10) << "Lmk with id: " << lmk_id
              << " is set to be a projection factor.\n";

    // Update lmk_id as a projection factor.
    gtsam::Key lmk_key = gtsam::Symbol('l', lmk_id);
    bool lmk_position_available = true;
    if (state_.find(lmk_key) == state_.end()) {
      VLOG(10) << "Lmk with id: " << lmk_id << " is not found in state.\n";
      // We did not find the lmk in the state.
      // It was a smart factor before.
      // Convert smart to projection.
      const auto& old_smart_factors_it = old_smart_factors_.find(lmk_id);
      CHECK(old_smart_factors_it != old_smart_factors_.end())
          << "Landmark not found in old_smart_factors_ !";

      SmartStereoFactor::shared_ptr old_factor =
          old_smart_factors_it->second.first;

      // Add landmark value to graph.
      VLOG(20) << "Print old_factor of lmk_id: " << lmk_id;
      if (VLOG_IS_ON(20)) {
        old_factor->print();
      }
      // TODO make sure that if point is not valid it works as well...
      if (old_factor->point().valid()) {
        VLOG(10) << "Performing conversion for lmk_id: " << lmk_id;

        new_values_.insert(lmk_key, *(old_factor->point()));

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
        for (size_t i = 0; i < old_factor->keys().size(); i++) {
          const gtsam::Key& cam_key = old_factor->keys().at(i);
          const StereoPoint2& sp2   = old_factor->measured().at(i);
          VLOG(10) << "Lmk with id: " << lmk_id
                   << " added as a new projection factor with pose with id: "
                   << static_cast<int>(cam_key) << ".\n";
          if (!std::isnan(sp2.uR())) {
            new_imu_prior_and_other_factors_.push_back(
                  gtsam::GenericStereoFactor<Pose3, Point3>
                  (sp2,
                   smart_noise_,
                   cam_key,
                   lmk_key,
                   stereo_cal_, B_Pose_leftCam_));
          } else {
            // Right pixel has a NAN value for u, use GenericProjectionFactor instead
            // of stereo.
            new_imu_prior_and_other_factors_.push_back(
                  gtsam::GenericProjectionFactor<Pose3, Point3>
                  (gtsam::Point2(sp2.uL(),
                                 sp2.v()),
                   mono_noise_,
                   cam_key,
                   lmk_key,
                   mono_cal_, B_Pose_leftCam_));
          }
        }

        // Make sure that the smart factor that we converted to projection
        // gets deleted from the graph.
        if (old_smart_factors_it->second.second != -1) {
          // Get current slot (if factor is already there it must be deleted).
          delete_slots_of_converted_smart_factors_.push_back(
                old_smart_factors_it->second.second);
        }

        // Erase from old_smart_factors_ list since this has been converted into
        // projection factors.
        old_smart_factors_.erase(lmk_id);
      } else {
        LOG(WARNING) << "Cannot convert smart factor to proj. factor.\n"
                   << "Smart factor does not have a valid 3D position for lmk: "
                   << lmk_id << "\n"
                   << "Smart factor point status: \n" << old_factor->point();
        lmk_position_available = false;
      }
    } else {
      VLOG(10) << "Lmk with id: " << lmk_id << " has been found in state: "
               << "it is being used in a projection factor.";
    }

    // If it is not smart, just add current measurement.
    // It was a projection factor before.
    // Also add it if it was smart but now is projection factor, unless we could
    // not convert the smart factor to a set of projection factors, then do not
    // add it because we do not have a right value to use.
    if (lmk_position_available) {
      VLOG(10) << "Lmk with id: " << lmk_id
               << " added as a new projection factor with pose with id: "
               << newObs.first << ".\n";
      if (!std::isnan(newObs.second.uR())) {
        new_imu_prior_and_other_factors_.push_back(
              gtsam::GenericStereoFactor<Pose3, Point3>
              (newObs.second, smart_noise_,
               gtsam::Symbol('x', newObs.first),
               lmk_key,
               stereo_cal_, B_Pose_leftCam_));
      } else {
        // Right pixel has a NAN value for u, use GenericProjectionFactor instead
        // of stereo.
        new_imu_prior_and_other_factors_.push_back(
              gtsam::GenericProjectionFactor<Pose3, Point3>
              (gtsam::Point2(newObs.second.uL(),
                             newObs.second.v()),
               mono_noise_,
               gtsam::Symbol('x', newObs.first),
               lmk_key,
               mono_cal_, B_Pose_leftCam_)
              );
      }
    } else {
      LOG(ERROR) << "Not using new observation for lmk: " << lmk_id
                 << " because we do not have a good initial value for it.";
    }
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::isLandmarkSmart(const LandmarkIds& lmks_kf,
                                        const LandmarkIds& mesh_lmk_ids,
                                        LmkIdIsSmart* lmk_id_is_smart) {
  CHECK_NOTNULL(lmk_id_is_smart);
  // WARNING I think this loop should not be over lmks_kf, which are in the
  // current keyframe but over the time horizon instead!!!
  // Otherwise we can have some lmks that are not set as projection factors
  // but will be in the future involved in some kind of regularity...
  for (const LandmarkId& lmk_id: lmks_kf) {
    const auto& lmk_id_slot = lmk_id_is_smart->find(lmk_id);
    if (std::find(mesh_lmk_ids.begin(),
                  mesh_lmk_ids.end(), lmk_id) ==
        mesh_lmk_ids.end()) {
      VLOG(10) << "Lmk_id = " << lmk_id
               << " needs to stay as it is since it is NOT involved in any regularity.";
      // This lmk is not involved in any regularity.
      if (lmk_id_slot == lmk_id_is_smart->end()) {
        // We did not find the lmk_id in the lmk_id_is_smart_ map.
        // Add it as a smart factor.
        lmk_id_is_smart->insert(std::make_pair(lmk_id, true));
      } else {
        // Let the lmk be as it was before (note is not allowed to go from
        // projection to smart.
        continue;
      }
    } else {
      // This lmk is involved in a regularity, hence it should be a variable in
      // the factor graph (connected to projection factor).
      VLOG(10) << "Lmk_id = " << lmk_id
               << " needs to be a proj. factor, as it is involved in a regularity.";
      const auto& old_smart_factors_it = old_smart_factors_.find(lmk_id);
      if (old_smart_factors_it == old_smart_factors_.end()) {
        // We did not find the factor (this is the case when feature track is
        // shorter than the minimum, typically 1. And the factor is not
        // added to the graph.
        VLOG(10)  << "Landmark not found in old_smart_factors_ !";
      } else {
        // We found the factor.

        // Get whether the smart factor is valid or not.
        bool lmk_is_valid = true;

        SmartStereoFactor::shared_ptr old_factor =
            old_smart_factors_it->second.first;

        if (!old_factor->point().valid()) {
          // The point is not valid.
          VLOG(20) << "Smart factor for lmk: " << lmk_id << "is NOT valid.";
          lmk_is_valid = false;
        } else {
          VLOG(20) << "Smart factor for lmk: " << lmk_id << "is valid.";
          lmk_is_valid = true;
        }

        if (lmk_id_slot == lmk_id_is_smart->end()) {
          // We did not find the lmk_id in the lmk_id_is_smart_ map.
          // Add it as a projection factor.
          // TODO use an enum instead of bool for lmk_id_is_smart, otherwise
          // it is too difficult to read.
          lmk_id_is_smart->insert(std::make_pair(lmk_id, lmk_is_valid?false:true));
        } else {
          // Change it to a projection factor.
          lmk_id_is_smart->at(lmk_id) = lmk_is_valid?false:true;
        }
      }
    }
  }

  // TODO all lmks should be smart the first time they are added!
}

/* -------------------------------------------------------------------------- */
// TODO we have a: terminate called after throwing an instance of 'std::out_of_range'
//  what():  map::at when running this function... And it does not appear very often.
void RegularVioBackEnd::addRegularityFactors(const LandmarkIds& mesh_lmk_ids,
                                             gtsam::Symbol* plane_symbol,
                                             size_t* plane_constraints_to_be_added) {
  CHECK_NOTNULL(plane_symbol);
  CHECK_NOTNULL(plane_constraints_to_be_added);

  // Noise model.
  static const gtsam::noiseModel::Isotropic::shared_ptr regularityNoise =
      gtsam::noiseModel::Isotropic::Sigma(1, vio_params_.regularityNoiseSigma_);

  // Plane key.
  static gtsam::Key plane_key (gtsam::Symbol('P', 0));
  *plane_symbol = plane_key;

  // Vars to check whether the new plane is going to be fully constrained or not.
  static bool is_plane_constrained = false;
  static std::vector<LandmarkId> list_of_constraints;

  size_t nr_of_point_plane_factors = 0;
  if (!state_.exists(plane_key)) {
    VLOG(10) << "Plane key, " << gtsam::DefaultKeyFormatter(plane_key)
             << " does NOT exist.";

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
        VLOG(10) << "Lmk id: " << lmk_id
                 << " is in state_ or is going to be added in next optimization.";

        const auto& lmk_id_regularity_type =
            lmk_id_to_regularity_type_map_.find(lmk_id);
        if (lmk_id_regularity_type == lmk_id_to_regularity_type_map_.end()) {
          // Lmk has not been used in a regularity.
          VLOG(10) << "Lmk id: " << lmk_id
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
            VLOG(10) << "Adding PointPlaneFactor.";
            nr_of_point_plane_factors++;
            new_imu_prior_and_other_factors_.push_back(
                  boost::make_shared<gtsam::PointPlaneFactor>(
                    gtsam::Symbol('l', lmk_id),
                    plane_key,
                    regularityNoise));
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
              VLOG(10) << "Adding PointPlaneFactor.";
              nr_of_point_plane_factors++;
              new_imu_prior_and_other_factors_.push_back(
                    boost::make_shared<gtsam::PointPlaneFactor>(
                      gtsam::Symbol('l', prev_lmk_id),
                      plane_key,
                      regularityNoise));
              // Acknowledge that this lmk has been used in a regularity.
              lmk_id_to_regularity_type_map_[prev_lmk_id] =
                  RegularityType::POINT_PLANE;
            }

            // Clear list of constraints, so that we do not enter in this
            // condition anymore.
            list_of_constraints.resize(0);
          }
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

        // "It probably is still a smart factor that was not converted"
        //              " to a projection factor because it was not a regularity when"
        //              " it was being processed... since backend processing is per"
        //              " frame (current feature tracks)...";
      }
    }

    if (!is_plane_constrained) {
      VLOG(10) << "Plane with key " << gtsam::DefaultKeyFormatter(plane_key)
               << " is not sufficiently constrained, not adding it.";
    }

    // Reset the flag for next time we have to add a plane.
    is_plane_constrained = false;
    // Also reset vector of constraints, just to be sure.
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
        VLOG(10) << "Lmk id: " << lmk_id
                 << " is in state_ or is going to be added in next optimization.";

        const auto& lmk_id_regularity_type =
            lmk_id_to_regularity_type_map_.find(lmk_id);
        if (lmk_id_regularity_type == lmk_id_to_regularity_type_map_.end() ||
            lmk_id_regularity_type->second != RegularityType::POINT_PLANE) {
          // Lmk has not been used in a regularity.
          VLOG(10) << "Lmk id: " << lmk_id
                   << " has not yet been used in a regularity.";

          // The plane is already in the graph so it must be fully constrained,
          // keep adding new factors.
          VLOG(10) << "Adding PointPlaneFactor.";
          nr_of_point_plane_factors++;
          new_imu_prior_and_other_factors_.push_back(
                boost::make_shared<gtsam::PointPlaneFactor>(
                  gtsam::Symbol('l', lmk_id),
                  plane_key,
                  regularityNoise));
          // Acknowledge that this lmk has been used in a regularity.
          lmk_id_to_regularity_type_map_[lmk_id] =
              RegularityType::POINT_PLANE;
        } else {
          // This lmk has already been used in a regularity, avoid duplication
          // of factors.
          VLOG(10) << "Avoiding duplicated regularity factor for lmk id: " << lmk_id;
        }
      } else {
        LOG(WARNING) << "Lmk id: " << lmk_id
                   << " is NOT in state_ or in new_values_,"
                   << " NOT adding PointPlaneFactor.";
        // "It probably is still a smart factor that was not converted"
        //              " to a projection factor because it was not a regularity when"
        //              " it was being processed... since backend processing is per"
        //              " frame (current feature tracks)...";
      }
    }
  }
  LOG(ERROR) << "A total of " << nr_of_point_plane_factors
             << " point plane factors to be added.";
  *plane_constraints_to_be_added = nr_of_point_plane_factors;
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::removeOldRegularityFactors_Slow(
    const gtsam::Symbol& plane_symbol,
    const size_t& nr_of_plane_constraints,
    const LandmarkIds& mesh_lmk_ids,
    std::vector<size_t>* delete_slots) {
  CHECK_NOTNULL(delete_slots);
  delete_slots->resize(0);

  if (!state_.exists(plane_symbol.key())){
    // The plane is not in the state, it is probably going to be added in
    // this iteration, so it must be already well constrained and attached
    // to the right lmks.
    // Or it could be that it is not going to be added at all, so we
    // do not need to do anything.
    LOG(INFO) << "Plane with id " << gtsam::DefaultKeyFormatter(plane_symbol)
              << " is not in state, skipping removeOldRegularityFactors.";
    return; // Leave this function.
  }

  // If the plane exists, then let us remove old regularity factors.
  const gtsam::NonlinearFactorGraph& graph = smoother_->getFactors();
  size_t slot = 0;
  std::vector<size_t> point_plane_factor_slots;
  bool has_plane_a_prior = false;
  for (const auto& g: graph) {
    const auto& ppf =
        boost::dynamic_pointer_cast<gtsam::PointPlaneFactor>(g);
    if (ppf) {
      // We found a PointPlaneFactor.
      // Get point symbol.
      gtsam::Symbol point_symbol (ppf->getPointKey());
      // Check plane key.
      CHECK(plane_symbol.key() == ppf->getPlaneKey());
      LandmarkId lmk_id = point_symbol.index();
      if (std::find(mesh_lmk_ids.begin(), mesh_lmk_ids.end(), lmk_id)
          == mesh_lmk_ids.end()) {
        // We did not find the point in mesh_lmk_ids, therefore it should
        // not be involved in a regularity anymore, delete this slot (but I
        // want to remove the landmark! and all its factors, to avoid having
        // underconstrained lmks...)
        LOG(WARNING) << "Deleting point plane factor on lmk with id: "
                     << point_symbol.index();
        delete_slots->push_back(slot);

        // Acknoledge that this lmk is not in a regularity anymore.
        // Avoid undefined behaviour by checking that the lmk was in the
        // container.
        CHECK(lmk_id_to_regularity_type_map_.find(lmk_id) !=
              lmk_id_to_regularity_type_map_.end());
        lmk_id_to_regularity_type_map_.erase(lmk_id);

        // TODO And find all projection factors to delete!
        //for (const auto& g: graph) {
        //  const auto& ppf =
        //      boost::dynamic_pointer_cast<T>(g);
        // OR just add a prior to it!
        // Get point estimate.
        gtsam::Point3 point;
        if (getEstimateOfKey(point_symbol.key(), &point)) {
          LOG(WARNING) << "Using lmk prior on lmk with id " <<
                          gtsam::DefaultKeyFormatter(point_symbol);
          static const gtsam::noiseModel::Diagonal::shared_ptr prior_lmk_noise =
              gtsam::noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 1.0));
          // TODO make sure this prior is not repeated over and over on the same
          // lmk id.
          // TODO is there the possibility that this lmk just have a prior attached
          // to it? and nothing else, I guess that it is unlikely because to
          // be first added it must have had some proj factors besides PointPlane
          // factor...
          new_imu_prior_and_other_factors_.push_back(
                boost::make_shared<gtsam::PriorFactor<gtsam::Point3>>(
                  point_symbol.key(),
                  point,
                  prior_lmk_noise));
        } else {
          LOG(ERROR) << "Lmk with id " << gtsam::DefaultKeyFormatter(point_symbol)
                     << " does not exist in graph.";
        }
      } else {
        // Store those factors that we will potentially keep.
        point_plane_factor_slots.push_back(slot);
      }
    }

    // Check for priors attached to the plane.
    const auto& plane_prior =
        boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::OrientedPlane3>>(g);
    if (plane_prior &&
        (plane_prior->key() == plane_symbol.key())) {
      has_plane_a_prior = true;
    }

    // Next slot.
    slot++;
  }

  static constexpr size_t min_num_of_constraints = 10;
  // TODO this is not correct, since we should also account the new factors
  // that we are going to attach to the plane to determine if it is going to be
  // undersconstrained or not...
  size_t total_nr_of_plane_constraints = point_plane_factor_slots.size() +
                                         nr_of_plane_constraints;
  if (point_plane_factor_slots.size() < min_num_of_constraints) {
    // Delete all factors to the plane! so that it is removed.
    // TODO we are not removing the factors that will be attached in this
    // round!
    // TODO How do we delete the priors that might be affecting it?
    delete_slots->insert(delete_slots->end(),
                         point_plane_factor_slots.begin(),
                         point_plane_factor_slots.end());
    // TODO now the plane could be floating around with a prior attached, but
    // not really attached to the rest of the graph...
    LOG(WARNING) << "Plane with id " << gtsam::DefaultKeyFormatter(plane_symbol)
                 << " is insufficiently constraint, " << point_plane_factor_slots.size()
                 << " constraints (" << min_num_of_constraints << " min required).\n"
                 << "Deleting all point plane factors attached to this plane ("
                 << "total of " << delete_slots->size() << " factors) "
                 << ", except the ones that are going to be added now.";

    // Should add priors to the lmks in the point_plane_factor_slots that
    // we are about to delete!
    //for (const size_t& slot: point_plane_factor_slots) {
    //  boost::shared_ptr<gtsam::PointPlaneFactor> ppf =
    //      boost::dynamic_pointer_cast<gtsam::PointPlaneFactor>(graph.at(slot));
    //  CHECK(ppf);
    //  CHECK_NOTNULL(ppf.get());
    //  gtsam::Symbol point_symbol (ppf->getPointKey());
    //  CHECK(plane_symbol.key() == ppf->getPlaneKey());

    //  // Acknoledge that this lmk is not in a regularity anymore.
    //  // Avoid undefined behaviour by checking that the lmk was in the
    //  // container.
    //  CHECK(lmk_id_to_regularity_type_map_.find(point_symbol.index()) !=
    //      lmk_id_to_regularity_type_map_.end());
    //  lmk_id_to_regularity_type_map_.erase(point_symbol.index());

    //  gtsam::Point3 point;
    //  if (getEstimateOfKey(point_symbol.key(), &point)) {
    //    LOG(WARNING) << "Using lmk prior on lmk with id " <<
    //                    gtsam::DefaultKeyFormatter(point_symbol);
    //    // TODO make sure this prior is not repeated over and over on the same
    //    // lmk id.
    //    // TODO is there the possibility that this lmk just have a prior attached
    //    // to it? and nothing else, I guess that it is unlikely because to
    //    // be first added it must have had some proj factors besides PointPlane
    //    // factor...
    //    static const gtsam::noiseModel::Diagonal::shared_ptr prior_lmk_noise =
    //        gtsam::noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 1.0));
    //    new_imu_prior_and_other_factors_.push_back(
    //          boost::make_shared<gtsam::PriorFactor<gtsam::Point3>>(
    //            point_symbol.key(),
    //            point,
    //            prior_lmk_noise));
    //  } else {
    //    LOG(ERROR) << "Lmk with id " << gtsam::DefaultKeyFormatter(point_symbol)
    //               << " does not exist in graph.";
    //  }
    //}

    if (!has_plane_a_prior) {
      gtsam::OrientedPlane3 plane_estimate;
      if (getEstimateOfKey(plane_symbol.key(), &plane_estimate)) {
        // TODO we should remove the plane, not add a prior on it...
        // and to do that we must delete all factors attached to the plane!
        LOG(WARNING) << "Using plane prior on plane with id "
                     << gtsam::DefaultKeyFormatter(plane_symbol);
        // TODO avoid adding more than once this prior!
        static const gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
            gtsam::noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.5));
        new_imu_prior_and_other_factors_.push_back(
              boost::make_shared<gtsam::PriorFactor<gtsam::OrientedPlane3>>(
                plane_symbol.key(),
                plane_estimate,
                prior_noise));
      } else {
        LOG(ERROR) << "Plane with key "
                   << gtsam::DefaultKeyFormatter(plane_symbol)
                   << " is not in state.";
      }
    } else {
      LOG(WARNING) << "Plane with id "
                   << gtsam::DefaultKeyFormatter(plane_symbol)
                   << " has already a prior attached to it,"
                   << " avoiding duplicated prior.";
    }
  }
}

} // namespace VIO
