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
  LOG(INFO) << "Using Regular VIO backend.\n";
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::addVisualInertialStateAndOptimize(
    const Timestamp& timestamp_kf_nsec,
    const StatusSmartStereoMeasurements& status_smart_stereo_measurements_kf,
    const ImuStamps& imu_stamps, const ImuAccGyr& imu_accgyr,
    const LandmarkIds& mesh_lmk_ids_ground_cluster,
    boost::optional<gtsam::Pose3> stereo_ransac_body_pose) {

  debugInfo_.resetAddedFactorsStatistics();

  if (VLOG_IS_ON(7)) {
    StereoVisionFrontEnd::PrintStatusStereoMeasurements(
                                          status_smart_stereo_measurements_kf);
  }

  // Features and IMU line up --> do iSAM update.
  last_id_ = cur_id_;
  ++cur_id_;

  timestamp_kf_ = UtilsOpenCV::NsecToSec(timestamp_kf_nsec);

  VLOG(7) << "Adding keyframe " << cur_id_
          << " at timestamp:" << timestamp_kf_ << " (sec)\n";

  /////////////////// MANAGE IMU MEASUREMENTS //////////////////////////////////
  // Predict next step, add initial guess.
  integrateImuMeasurements(imu_stamps, imu_accgyr);
  addImuValues(cur_id_);

  // Add imu factors between consecutive keyframe states.
  VLOG(7) << "Adding IMU factor between pose id: " << last_id_
          << " and pose id: " << cur_id_;
  addImuFactor(last_id_, cur_id_);

  // Add between factor from RANSAC.
  if (stereo_ransac_body_pose) {
    VLOG(7) << "Adding RANSAC factor between pose id: " << last_id_
            << " and pose id: " << cur_id_;
    if (VLOG_IS_ON(7)) {
      (*stereo_ransac_body_pose).print();
    }
    addBetweenFactor(last_id_, cur_id_, *stereo_ransac_body_pose);
  }

  /////////////////// MANAGE VISION MEASUREMENTS ///////////////////////////////
  const SmartStereoMeasurements& smart_stereo_measurements_kf =
                                    status_smart_stereo_measurements_kf.second;

  // Extract relevant information from stereo frame.
  LandmarkIds landmarks_kf;
  addStereoMeasurementsToFeatureTracks(
        cur_id_,
        smart_stereo_measurements_kf,
        &landmarks_kf);

  // For each landmark we decide if it's going to be a smart factor or not.
  isLandmarkSmart(landmarks_kf,
                  mesh_lmk_ids_ground_cluster,
                  &lmk_id_is_smart_);

  if (VLOG_IS_ON(8)) {
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
      VLOG(7) << "Landmark has a LOW_DISPARITY kfTrackingStatus_mono.";
      VLOG(7) << "Add zero velocity and no motion factors.\n";
      addZeroVelocityPrior(cur_id_);
      addNoMotionFactor(last_id_, cur_id_);
      break;
    }
    default: {
      kfTrackingStatus_mono == Tracker::TrackingStatus::VALID?
          VLOG(7) << "Landmark has a VALID kfTrackingStatus_mono.":
            kfTrackingStatus_mono == Tracker::TrackingStatus::FEW_MATCHES?
          VLOG(7) << "Landmark has a FEW_MATCHES kfTrackingStatus_mono.":
            kfTrackingStatus_mono == Tracker::TrackingStatus::INVALID?
          VLOG(7) << "Landmark has a INVALID kfTrackingStatus_mono.":
            kfTrackingStatus_mono == Tracker::TrackingStatus::DISABLED?
          VLOG(7) << "Landmark has a DISABLED kfTrackingStatus_mono.":
                     VLOG(7) << "";

      if (kfTrackingStatus_mono == Tracker::TrackingStatus::VALID) {
        // Tracker::TrackingStatus::VALID, FEW_MATCHES, INVALID, DISABLED :
        // We add features in VIO.
        addLandmarksToGraph(landmarks_kf);
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
    addRegularityFactors(mesh_lmk_ids_ground_cluster);
  }

  // This lags 1 step behind to mimic hw.
  imu_bias_prev_kf_ = imu_bias_lkf_;

  // TODO add conversion from Smart factor to regular.
  optimize(cur_id_, vioParams_.numOptimize_, delete_slots_converted_factors_);
}

/* -------------------------------------------------------------------------- */
// TODO Virtualize this appropriately,
void RegularVioBackEnd::addLandmarksToGraph(const LandmarkIds& landmarks_kf) {
  // Add selected landmarks to graph:
  int n_new_landmarks = 0;
  int n_updated_landmarks = 0;
  debugInfo_.numAddedSmartF_ += landmarks_kf.size();

  for (const LandmarkId& lmk_id: landmarks_kf) {
    FeatureTrack& ft = featureTracks_.at(lmk_id);

    // Only insert feature tracks of length at least 2
    // (otherwise uninformative)
    static constexpr size_t min_num_of_observations = 2;
    if (ft.obs_.size() < min_num_of_observations) {
      continue;
    }

    if (!ft.in_ba_graph_) {
      // Acknowledge that we have added the landmark in the graph.
      ft.in_ba_graph_ = true;
      addLandmarkToGraph(lmk_id, ft);
      ++n_new_landmarks;
    } else {
      const std::pair<FrameId, StereoPoint2>& obs_kf = ft.obs_.back();

      // Sanity check.
      CHECK_EQ(obs_kf.first, cur_id_) << "Last obs is not from the current"
                                         " keyframe!\n";

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

  if (VLOG_IS_ON(9)) {
    VLOG(9) << "Adding landmark with id: " << lmk_id
            << " for the first time to graph. \n"
            << "Nr of observations of the lmk: " << ft.obs_.size()
            << " observations.\n";
    new_factor->print();
  }

  // Add observations to smart factor.
  VLOG(9) << "Creating smart factor involving lmk with id: " << lmk_id;
  for (const std::pair<FrameId, StereoPoint2>& obs: ft.obs_) {
    VLOG(9) << "SmartFactor: adding observation of lmk with id: " << lmk_id
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

  if (lmk_id_is_smart_.find(lmk_id) == lmk_id_is_smart_.end()) {
    // We are not tracking whether the lmk is smart or not, assume it is smart.

  }
  bool is_lmk_smart = lmk_id_is_smart_.at(lmk_id);
  if (is_lmk_smart == true) {
    VLOG(10) << "Lmk with id: " << lmk_id << " is set to be smart.\n";

    // Update existing smart-factor.
    auto old_smart_factors_it = old_smart_factors_.find(lmk_id);
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
      VLOG(10) << "PRINT FACTOR of lmk_id: " << lmk_id;
      if (VLOG_IS_ON(9)) {
        old_factor->print();
      }
      // TODO make sure that if point is not valid it works as well...
      CHECK(old_factor->point().valid())
          << "Does not have a value for point in proj. factor.";
      new_values_.insert(lmk_key, *old_factor->point());
      VLOG(10) << "Performing conversion for lmk_id: " << lmk_id;

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
                (sp2, smart_noise_,
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
    }

    // If it is not smart, just add current measurement.
    // It was a projection factor before.
    // Also add it if it was smart but now is projection factor...
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
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::isLandmarkSmart(const LandmarkIds& lmk_kf,
                                        const LandmarkIds& mesh_lmk_ids,
                                        LmkIdIsSmart* lmk_id_is_smart) {
  CHECK_NOTNULL(lmk_id_is_smart);
  for (const LandmarkId& lmk_id: lmk_kf) {
    const auto& lmk_id_slot = lmk_id_is_smart->find(lmk_id);
    if (std::find(mesh_lmk_ids.begin(),
                  mesh_lmk_ids.end(), lmk_id) ==
        mesh_lmk_ids.end()) {
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
      if (std::find(mesh_lmk_ids.begin(),
                    mesh_lmk_ids.end(), lmk_id) ==
          mesh_lmk_ids.end()) {
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
        // This lmk is involved in regular factor, hence it should be a variable in
        // the factor graph (connected to projection factor).
        VLOG(10) << "Lmk_id = " << lmk_id
                 << " Needs to be proj. factor!";
        if (lmk_id_slot == lmk_id_is_smart->end()) {
          // We did not find the lmk_id in the lmk_id_is_smart_ map.
          // Add it as a projection factor.
          lmk_id_is_smart->insert(std::make_pair(lmk_id, false));
        } else {
          // Change it to a projection factor.
          lmk_id_is_smart->at(lmk_id) = false;
        }
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
void RegularVioBackEnd::addRegularityFactors(const LandmarkIds& mesh_lmk_ids) {

  // Noise model.
  static const gtsam::noiseModel::Diagonal::shared_ptr regularityNoise =
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(0.1));

  // Plane key.
  static const gtsam::Key plane_key (gtsam::Symbol('P', 0));

  if (!state_.exists(plane_key)) {
    VLOG(10) << "Plane key does NOT exist, adding new plane with key: "
             << plane_key;
    new_values_.insert(plane_key, gtsam::OrientedPlane3(0.0, 0.0, 1.0, 0.0));
  } else {
    VLOG(10) << "Plane key does exist already: " << plane_key;
  }

  // Temporal graph.
  gtsam::NonlinearFactorGraph tmp_graph;    //!< new factors to be added

  // For each lmk id, add a point plane factor.
  // TODO only add regularity for triangles! so check that.
  for (const LandmarkId& lmk_id: mesh_lmk_ids) {
    if (state_.exists(gtsam::Symbol('l', lmk_id))) {
      VLOG(10) << "Lmk id: " << lmk_id
                << " is in state_, adding PointPlaneFactor.";
      new_imu_prior_and_other_factors_.push_back(
            boost::make_shared<gtsam::PointPlaneFactor>(
              gtsam::Symbol('l', lmk_id),
              plane_key,
              regularityNoise));
    } else {
      VLOG(10) << "Lmk id: "
              << lmk_id << " is NOT in state_, NOT adding PointPlaneFactor.";
    }
  }
}

} // namespace VIO
