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

  gtsam::SharedNoiseModel gaussian =
      gtsam::noiseModel::Isotropic::Sigma(2, vioParams_.monoNoiseSigma_);

  switch (vioParams_.normType_) {
    case 0: {
      LOG(INFO) << "Using square norm.";
      mono_noise_ = gaussian;
      break;
    }
    case 1: {
      LOG(INFO) << "Using Huber norm, with parameter value: " << vioParams_.huberParam_;
      mono_noise_ =
          gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(
              vioParams_.huberParam_,
              gtsam::noiseModel::mEstimator::Huber::Scalar), // Default is Block
            gaussian);
      break;
    }
    case 2: {
      LOG(INFO) << "Using Tukey norm, with parameter value: " << vioParams_.tukeyParam_;
      mono_noise_ = gtsam::noiseModel::Robust::Create(
                      gtsam::noiseModel::mEstimator::Tukey::Create(
                        vioParams_.tukeyParam_,
                        gtsam::noiseModel::mEstimator::Tukey::Scalar), // Default is Block
                      gaussian); //robust
      break;
    }
    default: {
      LOG(INFO) << "Using square norm.";
      mono_noise_ = gtsam::noiseModel::Isotropic::Sigma(
                      2, vioParams_.monoNoiseSigma_);

      break;
    }
  }

  mono_cal_ = boost::make_shared<Cal3_S2>(stereoCal_->calibration());
  CHECK(mono_cal_->equals(stereoCal_->calibration()))
      << "Monocular calibration should match Stereo calibration";
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::addVisualInertialStateAndOptimize(
    const Timestamp& timestamp_kf_nsec,
    const StatusSmartStereoMeasurements& status_smart_stereo_measurements_kf,
    const ImuStamps& imu_stamps, const ImuAccGyr& imu_accgyr,
    const LandmarkIds& mesh_lmk_ids_ground_cluster,
    boost::optional<gtsam::Pose3> stereo_ransac_body_pose) {

  debugInfo_.resetAddedFactorsStatistics();

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

  /////////////////// MANAGE IMU MEASUREMENTS //////////////////////////////////
  // Predict next step, add initial guess.
  integrateImuMeasurements(imu_stamps, imu_accgyr);
  addImuValues(cur_kf_id_);

  // Add imu factors between consecutive keyframe states.
  VLOG(20) << "Adding IMU factor between pose id: " << last_kf_id_
          << " and pose id: " << cur_kf_id_;
  addImuFactor(last_kf_id_, cur_kf_id_);

  // Add between factor from RANSAC.
  if (stereo_ransac_body_pose) {
    VLOG(20) << "Adding RANSAC factor between pose id: " << last_kf_id_
            << " and pose id: " << cur_kf_id_;
    if (VLOG_IS_ON(20)) {
      stereo_ransac_body_pose->print();
    }
    addBetweenFactor(last_kf_id_, cur_kf_id_, *stereo_ransac_body_pose);
  }

  /////////////////// MANAGE VISION MEASUREMENTS ///////////////////////////////
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

  // For each landmark we decide if it's going to be a smart factor or not.
  // TODO here there is a timeline mismatch, while landmarks_kf are only currently
  // visible lmks, mesh_lmks_ids contains lmks in time_horizon!
  isLandmarkSmart(lmks_kf,
                  mesh_lmk_ids_ground_cluster,
                  &lmk_id_is_smart_);

  if (VLOG_IS_ON(20)) {
    printFeatureTracks();
  }

  // Decide which factors to add.
  Tracker::TrackingStatus kfTrackingStatus_mono =
                status_smart_stereo_measurements_kf.first.kfTrackingStatus_mono_;

  // Clear vector.
  delete_slots_converted_factors_.resize(0);

  switch(kfTrackingStatus_mono) {
    case Tracker::TrackingStatus::LOW_DISPARITY : {
      // Vehicle is not moving.
      VLOG(10) << "Tracker has a LOW_DISPARITY status.";
      VLOG(10) << "Add zero velocity and no motion factors.\n";
      addZeroVelocityPrior(cur_kf_id_);
      addNoMotionFactor(last_kf_id_, cur_kf_id_);
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
          VLOG(10) << "Tracker has a DISABLED status.":
                     VLOG(10) << "";

      if (kfTrackingStatus_mono == Tracker::TrackingStatus::VALID) {
        // Tracker::TrackingStatus::VALID, FEW_MATCHES, INVALID, DISABLED :
        // We add features in VIO.
        VLOG(10) << "Adding/Updating landmarks to graph.";
        addLandmarksToGraph(lmks_kf);
      }
      break;
    }
  }

  // Add regularity factor on vertices of the mesh.
  // TODO argument should be generalized to diff
  // type of cluster and regularities.
  if (mesh_lmk_ids_ground_cluster.size() != 0) {
    // TODO what happens if mesh has same ids over and over, are we duplicating
    // factors?
    //addRegularityFactors(mesh_lmk_ids_ground_cluster);
  }

  // This lags 1 step behind to mimic hw.
  imu_bias_prev_kf_ = imu_bias_lkf_;

  // TODO add conversion from Smart factor to regular.
  optimize(cur_kf_id_, vioParams_.numOptimize_, delete_slots_converted_factors_);
}

/* -------------------------------------------------------------------------- */
// TODO Virtualize this appropriately,
void RegularVioBackEnd::addLandmarksToGraph(const LandmarkIds& lmks_kf) {
  // Add selected landmarks to graph:
  int n_new_landmarks = 0;
  int n_updated_landmarks = 0;
  debugInfo_.numAddedSmartF_ += lmks_kf.size();

  //CHECK(lmk_id_is_smart_.size() == landmarks_kf.size());
  for (const LandmarkId& lmk_id: lmks_kf) {
    FeatureTrack& feature_track = featureTracks_.at(lmk_id);

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
      VLOG(10) << "Adding lmk " << lmk_id << " to graph.\n";
      addLandmarkToGraph(lmk_id, feature_track);
      ++n_new_landmarks;
    } else {
      const std::pair<FrameId, StereoPoint2>& obs_kf = feature_track.obs_.back();

      // Sanity check.
      CHECK_EQ(obs_kf.first, cur_kf_id_) << "Last obs is not from the current"
                                            " keyframe!\n";

      VLOG(10) << "Updating lmk " << lmk_id << " to graph.\n";
      updateLandmarkInGraph(lmk_id, obs_kf);
      ++n_updated_landmarks;
    }
  }
  VLOG(7) << "Added " << n_new_landmarks << " new landmarks\n"
          << "Updated " << n_updated_landmarks << " landmarks in graph\n";
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
                                             smartFactorsParams_,
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
                    stereoCal_);
  }

  // Add new factor to suitable structures.
  new_smart_factors_.insert(std::make_pair(lmk_id,
                                           new_factor));
  old_smart_factors_.insert(std::make_pair(lmk_id,
                                           std::make_pair(new_factor, -1)));
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
  if (is_lmk_smart == true) {
    VLOG(10) << "Lmk with id: " << lmk_id << " is set to be smart.\n";

    // Update existing smart-factor.
    const auto& old_smart_factors_it = old_smart_factors_.find(lmk_id);
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
                    stereoCal_);

    // Update the set of new smart factors.
    if (old_smart_factors_it->second.second != -1) {
      // Slot is different than -1
      // It means that the factor has already been inserted in the graph.
      VLOG(10) << "Update new_smart_factors_";
      new_smart_factors_.insert(std::make_pair(lmk_id, new_factor));
    } else {
      // If slot is still -1, it means that the factor has not been inserted yet
      // in the graph.
      CHECK(false) << "When calling update "
                      "the slot should be already != -1";
    }

    // TODO Why do we do this??
    old_smart_factors_it->second.first = new_factor;

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
        LOG_EVERY_N(ERROR, 100) << "Do not forget to remove lmk prior!";
        static const gtsam::noiseModel::Diagonal::shared_ptr prior_lmk_noise =
            gtsam::noiseModel::Diagonal::Sigmas(Vector3(1, 1, 1));
        new_imu_prior_and_other_factors_.push_back(
              boost::make_shared<gtsam::PriorFactor<gtsam::Point3> >(
                lmk_key,
                *(old_factor->point()),
                prior_lmk_noise));

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
                   stereoCal_, B_Pose_leftCam_));
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
          delete_slots_converted_factors_.push_back(
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
               stereoCal_, B_Pose_leftCam_));
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
void RegularVioBackEnd::addRegularityFactors(const LandmarkIds& mesh_lmk_ids) {

  // Noise model.
  // TODO remove hardcoded value.
  static const gtsam::noiseModel::Diagonal::shared_ptr regularityNoise =
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(0.5));

  // Plane key.
  static const gtsam::Key plane_key (gtsam::Symbol('P', 0));

  if (!state_.exists(plane_key)) {
    VLOG(10) << "Plane key does NOT exist, adding new plane with key: "
             << plane_key;
    static const gtsam::OrientedPlane3 plane(0.0, 0.0, 1.0, -0.1);
    new_values_.insert(plane_key, plane);

    static const gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
        gtsam::noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.5));
    new_imu_prior_and_other_factors_.push_back(
          boost::make_shared<gtsam::PriorFactor<gtsam::OrientedPlane3> >(
            plane_key,
            plane,
            prior_noise));
  } else {
    VLOG(10) << "Plane key does exist already: " << plane_key;
  }

  // Temporal graph.
  gtsam::NonlinearFactorGraph tmp_graph;    //!< new factors to be added

  // For each lmk id, add a point plane factor.
  // TODO only add regularity for triangles! so check that.
  // TODO this will bring an error for sure in the sense that
  // not all lmks in mesh_lmk_ids will be used in projection factor
  // because mesh_lmk_ids is time-horizon vs backend operation is in per frame.
  for (const LandmarkId& lmk_id: mesh_lmk_ids) {
    if (state_.exists(gtsam::Symbol('l', lmk_id))) {
      VLOG(10) << "Lmk id: " << lmk_id
                << " is in state_, adding PointPlaneFactor.";
      // TODO we are repeating factor!
      new_imu_prior_and_other_factors_.push_back(
            boost::make_shared<gtsam::PointPlaneFactor>(
              gtsam::Symbol('l', lmk_id),
              plane_key,
              regularityNoise));
    } else {
      LOG(ERROR) << "Lmk id: "
              << lmk_id << " is NOT in state_, NOT adding PointPlaneFactor.";
    }
  }
}

} // namespace VIO
