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

namespace VIO {

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

  std::cout << "RegularVIO: adding keyframe " << cur_id_
            << " at timestamp:" << timestamp_kf_ << " (sec)" << std::endl;

  /////////////////// MANAGE IMU MEASUREMENTS //////////////////////////////////
  // Predict next step, add initial guess.
  integrateImuMeasurements(imu_stamps, imu_accgyr);
  addImuValues(cur_id_);

  // Add imu factors between consecutive keyframe states.
  addImuFactor(last_id_, cur_id_);

  // Add between factor from RANSAC.
  if (stereo_ransac_body_pose) {
    std::cout << "RegularVIO: adding between " << std::endl;
    (*stereo_ransac_body_pose).print();
    addBetweenFactor(last_id_, cur_id_, *stereo_ransac_body_pose);
  }

  /////////////////// MANAGE VISION MEASUREMENTS ///////////////////////////////
  SmartStereoMeasurements smartStereoMeasurements_kf =
                                    status_smart_stereo_measurements_kf.second;

  // If stereo ransac failed, remove all right pixels:
  Tracker::TrackingStatus kfTrackingStatus_stereo =
            status_smart_stereo_measurements_kf.first.kfTrackingStatus_stereo_;
  // if(kfTrackingStatus_stereo == Tracker::TrackingStatus::INVALID){
  //   for(size_t i = 0; i < smartStereoMeasurements_kf.size(); i++)
  //     smartStereoMeasurements_kf[i].uR = std::numeric_limits<double>::quiet_NaN();;
  //}

  // Extract relevant information from stereo frame.
  LandmarkIds landmarks_kf = addStereoMeasurementsToFeatureTracks(
                                                     cur_id_,
                                                     smartStereoMeasurements_kf);

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
  switch(kfTrackingStatus_mono) {
  case Tracker::TrackingStatus::LOW_DISPARITY : {
    // Vehicle is not moving.
    VLOG(7) << "Add zero velocity and no motion factors\n";
    addZeroVelocityPrior(cur_id_);
    addNoMotionFactor(last_id_, cur_id_);
    break;
  }
    // This did not improve in any case
    //  case Tracker::TrackingStatus::INVALID :// ransac failed hence we cannot trust features
    //    if (verbosity_ >= 7) {printf("Add constant velocity factor (monoRansac is INVALID)\n");}
    //    addConstantVelocityFactor(last_id_, cur_id_);
    //    break;

  default: {
    // Clear vector.
    delete_slots_converted_factors_.resize(0);

    // Tracker::TrackingStatus::VALID, FEW_MATCHES, INVALID, DISABLED :
    // we add features in VIO
    addLandmarksToGraph(landmarks_kf);
    break;
  }
  }

  // Add regularity factor on vertices of the mesh.
  // TODO argument should be generalized to diff type of cluster and regularities.
  addRegularityFactors(mesh_lmk_ids_ground_cluster); // This currently assumes only planar triangles on the ground.

  // This lags 1 step behind to mimic hw.
  imu_bias_prev_kf_ = imu_bias_lkf_;

  // TODO add conversion from Smart factor to regular.
  optimize(cur_id_, vioParams_.numOptimize_, delete_slots_converted_factors_);
}

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
      // This lmk is involved in regular factor, hence it should be a variable in
      // the factor graph (connected to projection factor).
      std::cout << "Lmk_id = " << lmk_id
                << " Needs to be proj. factor!" << std::endl; // TODO delete this!
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


void RegularVioBackEnd::addRegularityFactors(
    const LandmarkIds& mesh_lmk_ids) {
 // Vector6 precisions;
 // precisions.head<3>().setConstant(vioParams_.betweenRotationPrecision_);
 // precisions.tail<3>().setConstant(vioParams_.betweenTranslationPrecision_);
 // gtsam::SharedNoiseModel betweenNoise_ = gtsam::noiseModel::Diagonal::Precisions(precisions);

 // new_imu_prior_and_other_factors_.push_back(
 //       boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
 //         gtsam::Symbol('x', from_id), gtsam::Symbol('x', to_id), from_id_POSE_to_id, betweenNoise_));

 // debugInfo_.numAddedBetweenStereoF_++;

 // if (verbosity_ >= 7) {std::cout << "addBetweenFactor" << std::endl;}
}

void RegularVioBackEnd::addLandmarkToGraph(const LandmarkId& lmk_id,
                                           FeatureTrack& ft) {
  if(ft.in_ba_graph_)
    throw std::runtime_error("addLandmarkToGraph:"
                             " feature already in the graph!");

  ft.in_ba_graph_ = true;

  // All landmarks should be smart the first time we add them to the graph.

  //if (lmk_id_is_smart_.at(lmk_id)) {
    // Add as a smart factor.
    // We use a unit pinhole projection camera for the smart factors to be
    // more efficient.
    SmartStereoFactor::shared_ptr new_factor(
          new SmartStereoFactor(smart_noise_,
                                smartFactorsParams_,
                                B_Pose_leftCam_));

    if (VLOG_IS_ON(9)) {
      VLOG(9) << "Adding landmark with: " << ft.obs_.size()
              << " observations to graph, with keys:\n";
      new_factor->print();
    }

    // Add observations to smart factor.
    for (const std::pair<FrameId,StereoPoint2>& obs: ft.obs_) {
      new_factor->add(obs.second, gtsam::Symbol('x', obs.first), stereoCal_);
      VLOG(9) << " " <<  obs.first;
    }

    // Add new factor to suitable structures:
    new_smart_factors_.insert(std::make_pair(lmk_id, new_factor));
    old_smart_factors_.insert(std::make_pair(lmk_id,
                                             std::make_pair(new_factor, -1)));
  //} else {
  //  // Add as a projection factor.
  //  for (const std::pair<FrameId,StereoPoint2>& obs: ft.obs_) {
  //    new_imu_prior_and_other_factors_.push_back(
  //          gtsam::GenericStereoFactor<Pose3, Point3>
  //          (obs.second, smart_noise_,
  //           gtsam::Symbol('x', obs.first),
  //           gtsam::Symbol('l', lmk_id),
  //           stereoCal_, B_Pose_leftCam_));
  //    if (verbosity_ >= 9) {
  //      std::cout << "Adding projection factors with observations: "
  //                << ft.obs_.size() << "for landmark key: " << lmk_id;
  //    }
  //  }
  //}
}

void RegularVioBackEnd::updateLandmarkInGraph(
    const LandmarkId& lmk_id,
    const std::pair<FrameId, StereoPoint2>& newObs) {

  bool is_lmk_smart = lmk_id_is_smart_.at(lmk_id);
  if (is_lmk_smart == true) {
    LOG(INFO) << "Lmk with id:" << lmk_id << " is set to be smart.\n";
    // Update existing smart-factor.
    auto old_smart_factors_it = old_smart_factors_.find(lmk_id);
    if (old_smart_factors_it == old_smart_factors_.end())
      throw std::runtime_error("updateLandmarkInGraph:"
                               " landmark not found in old_smart_factors_\n");

    SmartStereoFactor::shared_ptr old_factor =
                                            old_smart_factors_it->second.first;
    SmartStereoFactor::shared_ptr new_factor =
         boost::make_shared<SmartStereoFactor>(*old_factor); // clone old factor

    new_factor->add(newObs.second,
                    gtsam::Symbol('x', newObs.first), stereoCal_);

    // Update the factor.
    if (old_smart_factors_it->second.second != -1) {
      // if slot is still -1, it means that the factor has not been inserted yet
      // in the graph
      new_smart_factors_.insert(std::make_pair(lmk_id, new_factor));
    } else {
      throw std::runtime_error("updateLandmarkInGraph: when calling update "
                               "the slot should be already != -1! \n");
    }

    old_smart_factors_it->second.first = new_factor;
    VLOG(8) << "updateLandmarkInGraph: added observation to point: "
            << lmk_id;

  } else {
    // Update lmk_id as a projection factor.
    gtsam::Key lmk_key = gtsam::Symbol('l', lmk_id);
    LOG(INFO) << "Lmk with id:" << lmk_id << " is set to be a projection factor.\n";
    // TODO remove debug
    // state_.print("Smoother state\n");
    if (state_.find(lmk_key) == state_.end()) {
      LOG(INFO) << "Lmk with id:" << lmk_id << " is not found in state.\n";
      // We did not find the lmk in the state.
      // It was a smart factor before.
      // Convert smart to projection.
      auto old_smart_factors_it = old_smart_factors_.find(lmk_id);
      if (old_smart_factors_it == old_smart_factors_.end())
        throw std::runtime_error("updateLandmarkInGraph (Projection case):"
                                 " landmark not found in old_smart_factors_\n");

      SmartStereoFactor::shared_ptr old_factor =
          old_smart_factors_it->second.first;

      // Add landmark value to graph.
      CHECK(old_factor->point().valid())
          <<  "Does not have a value for point in proj. factor.\n";
      new_values_.insert(lmk_key, *old_factor->point());
      // TODO remove lose prior only used for debugging!
      gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(
        3, 0.5); // 0.5 meters std variation.
      new_imu_prior_and_other_factors_.push_back(gtsam::PriorFactor<Point3>(
                                                   lmk_key, *old_factor->point(),
                                                   model));
      LOG(INFO) << " Performing conversion for lmk_id: " << lmk_id << "\n";

      // Convert smart factor to multiple projection factors.
      for (size_t i = 0; i < old_factor->keys().size(); i++) {
        gtsam::Key cam_key = old_factor->keys().at(i);
        StereoPoint2 sp2 = old_factor->measured().at(i);
        new_imu_prior_and_other_factors_.push_back(
              gtsam::GenericStereoFactor<Pose3, Point3>
              (sp2, smart_noise_,
               cam_key,
               lmk_key,
               stereoCal_, B_Pose_leftCam_));
      }

      // Make sure that the smart factor that we converted to projection
      // gets deleted from the graph.
      if (old_smart_factors_it->second.second != -1) {// get current slot (if factor is already there it must be deleted)
        delete_slots_converted_factors_.push_back(old_smart_factors_it->second.second);
      }

      // Erase from old_smart_factors_ list since this has been converted into
      // projection factors.
      old_smart_factors_.erase(lmk_id);
    }

    // If it is not smart, just add current measurement.
    // It was a projection factor before.
    // Also add it if it was smart but now is projection factor...
    LOG(INFO) << "Lmk with id:" << lmk_id
              << " added as a new projection factor.\n";
    new_imu_prior_and_other_factors_.push_back(
          gtsam::GenericStereoFactor<Pose3, Point3>
          (newObs.second, smart_noise_,
           gtsam::Symbol('x', newObs.first),
           lmk_key,
           stereoCal_, B_Pose_leftCam_));
  }
}

// TODO Virtualize this appropriately,
void RegularVioBackEnd::addLandmarksToGraph(LandmarkIds landmarks_kf) {
  // Add selected landmarks to graph:
  int n_new_landmarks = 0;
  int n_updated_landmarks = 0;
  debugInfo_.numAddedSmartF_ += landmarks_kf.size();

  for (const LandmarkId& lmk_id : landmarks_kf) {
    FeatureTrack& ft = featureTracks_.at(lmk_id);
    if(ft.obs_.size()<2) // we only insert feature tracks of length at least 2 (otherwise uninformative)
      continue;

    if(!ft.in_ba_graph_) {
      addLandmarkToGraph(lmk_id, ft);
      ++n_new_landmarks;
    } else {
      const std::pair<FrameId, StereoPoint2> obs_kf = ft.obs_.back();

      // Sanity check.
      CHECK_EQ(obs_kf.first, cur_id_) << "Last obs is not from the current"
                                         " keyframe!\n";

      updateLandmarkInGraph(lmk_id, obs_kf);
      ++n_updated_landmarks;
    }
  }
  VLOG(7) << "Added " << n_new_landmarks << " new landmarks"
          << "Updated " << n_updated_landmarks << " landmarks in graph\n";
}

} // namespace VIO
