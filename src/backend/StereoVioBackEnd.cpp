/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVioBackEnd.cpp
 * @brief  Visual-Inertial Odometry pipeline, as described in these papers:
 *
 * C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza.
 * On-Manifold Preintegration Theory for Fast and Accurate Visual-Inertial
 * Navigation. IEEE Trans. Robotics, 33(1):1-21, 2016.
 *
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, and F. Dellaert.
 * Eliminating Conditionally Independent Sets in Factor Graphs: A Unifying
 * Perspective based on Smart Factors. In IEEE Intl. Conf. on Robotics and
 * Automation (ICRA), 2014.
 *
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include "kimera-vio/backend/StereoVioBackEnd.h"

namespace VIO {

/* -------------------------------------------------------------------------- */
StereoVioBackEnd::StereoVioBackEnd(
    const Pose3& B_Pose_leftCam,
    const StereoCalibPtr& stereo_calibration,
    const BackendParams& backend_params,
    const ImuParams& imu_params,
    const BackendOutputParams& backend_output_params,
    bool log_output)
    : VioBackEnd(backend_params,
                 imu_params,
                 backend_output_params,
                 log_output),
      B_Pose_leftCam_(B_Pose_leftCam),
      stereo_cal_(stereo_calibration) {}

/* -------------------------------------------------------------------------- */
BackendOutput::UniquePtr StereoVioBackEnd::spinOnce(const BackendInput& input) {
  if (VLOG_IS_ON(10)) input.print();

  bool backend_status = false;
  switch (backend_state_) {
    case BackendState::Bootstrap: {
      initializeBackend(input);
      backend_status = true;
      break;
    }
    case BackendState::Nominal: {
      // Process data with VIO.
      backend_status = addVisualInertialStateAndOptimize(input);
      break;
    }
    default: {
      LOG(FATAL) << "Unrecognized backend state.";
      break;
    }
  }

  // Fill ouput_payload (it will remain nullptr if the backend_status is not ok)
  BackendOutput::UniquePtr output_payload = nullptr;
  if (backend_status) {
    // If backend is doing ok, fill and return ouput_payload;
    if (VLOG_IS_ON(10)) {
      LOG(INFO) << "Latest backend IMU bias is: ";
      getLatestImuBias().print();
      LOG(INFO) << "Prev kf backend IMU bias is: ";
      getImuBiasPrevKf().print();
    }

    // Generate extra optional backend ouputs.
    static const bool kOutputLmkMap =
        backend_output_params_.output_map_lmk_ids_to_3d_points_in_time_horizon_;
    static const bool kMinLmkObs =
        backend_output_params_.min_num_obs_for_lmks_in_time_horizon_;
    static const bool kOutputLmkTypeMap =
        backend_output_params_.output_lmk_id_to_lmk_type_map_;
    LmkIdToLmkTypeMap lmk_id_to_lmk_type_map;
    PointsWithIdMap lmk_ids_to_3d_points_in_time_horizon;
    if (kOutputLmkMap) {
      // Generate this map only if requested, since costly.
      // Also, if lmk type requested, fill lmk id to lmk type object.
      // WARNING this also cleans the lmks inside the old_smart_factors map!
      lmk_ids_to_3d_points_in_time_horizon =
          getMapLmkIdsTo3dPointsInTimeHorizon(
              kOutputLmkTypeMap ? &lmk_id_to_lmk_type_map : nullptr,
              kMinLmkObs);
    }

    // Create Backend Output Payload.
    output_payload = VIO::make_unique<BackendOutput>(
        VioNavStateTimestamped(
            input.timestamp_, W_Pose_B_lkf_, W_Vel_B_lkf_, imu_bias_lkf_),
        // TODO(Toni): Make all below optional!!
        state_,
        getCurrentStateCovariance(),
        curr_kf_id_,
        landmark_count_,
        debug_info_,
        lmk_ids_to_3d_points_in_time_horizon,
        lmk_id_to_lmk_type_map);

    if (logger_) {
      logger_->logBackendOutput(*output_payload);
    }
  }

  return output_payload;
}

/* -------------------------------------------------------------------------- */
// Get valid 3D points and corresponding lmk id.
// Warning! it modifies old_smart_factors_!!
PointsWithIdMap StereoVioBackEnd::getMapLmkIdsTo3dPointsInTimeHorizon(
    LmkIdToLmkTypeMap* lmk_id_to_lmk_type_map,
    const size_t& min_age) {
  PointsWithIdMap points_with_id;

  if (lmk_id_to_lmk_type_map) {
    lmk_id_to_lmk_type_map->clear();
  }

  // Step 1:
  /////////////// Add landmarks encoded in the smart factors. //////////////////
  const gtsam::NonlinearFactorGraph& graph = smoother_->getFactors();

  // old_smart_factors_ has all smart factors included so far.
  // Retrieve lmk ids from smart factors in state.
  size_t nr_valid_smart_lmks = 0, nr_smart_lmks = 0, nr_proj_lmks = 0;
  for (SmartFactorMap::iterator old_smart_factor_it =
           old_smart_factors_.begin();
       old_smart_factor_it !=
       old_smart_factors_
           .end();) {  //!< landmarkId -> {SmartFactorPtr, SlotIndex}
    // Store number of smart lmks (one smart factor per landmark).
    nr_smart_lmks++;

    // Retrieve lmk_id of the smart factor.
    const LandmarkId& lmk_id = old_smart_factor_it->first;

    // Retrieve smart factor.
    const SmartStereoFactor::shared_ptr& smart_factor_ptr =
        old_smart_factor_it->second.first;
    // Check that pointer is well definied.
    CHECK(smart_factor_ptr) << "Smart factor is not well defined.";

    // Retrieve smart factor slot in the graph.
    const Slot& slot_id = old_smart_factor_it->second.second;

    // Check that slot is admissible.
    // Slot should be positive.
    DCHECK(slot_id >= 0) << "Slot of smart factor is not admissible.";
    // Ensure the graph size is small enough to cast to int.
    DCHECK_LT(graph.size(), std::numeric_limits<Slot>::max())
        << "Invalid cast, that would cause an overflow!";
    // Slot should be inferior to the size of the graph.
    DCHECK_LT(slot_id, static_cast<Slot>(graph.size()));

    // Check that this slot_id exists in the graph, aka check that it is
    // in bounds and that the pointer is live (aka at(slot_id) works).
    if (!graph.exists(slot_id)) {
      // This slot does not exist in the current graph...
      VLOG(5) << "The slot with id: " << slot_id
              << " does not exist in the graph.\n"
              << "Deleting old_smart_factor of lmk id: " << lmk_id;
      old_smart_factor_it = old_smart_factors_.erase(old_smart_factor_it);
      // Update as well the feature track....
      // TODO(TONI): please remove this and centralize how feature tracks
      // and new/old_smart_factors are added and removed!
      CHECK(deleteLmkFromFeatureTracks(lmk_id));
      continue;
    } else {
      VLOG(20) << "Slot id: " << slot_id
               << " for smart factor of lmk id: " << lmk_id;
    }

    // Check that the pointer smart_factor_ptr points to the right element
    // in the graph.
    if (smart_factor_ptr != graph.at(slot_id)) {
      // Pointer in the graph does not match
      // the one we stored in old_smart_factors_
      // ERROR: if the pointers don't match, then the code that follows does
      // not make any sense, since we are using lmk_id which comes from
      // smart_factor and result which comes from graph[slot_id], we should
      // use smart_factor_ptr instead then...
      LOG(ERROR) << "The factor with slot id: " << slot_id
                 << " in the graph does not match the old_smart_factor of "
                 << "lmk with id: " << lmk_id << "\n."
                 << "Deleting old_smart_factor of lmk id: " << lmk_id;
      old_smart_factor_it = old_smart_factors_.erase(old_smart_factor_it);
      CHECK(deleteLmkFromFeatureTracks(lmk_id));
      continue;
    }

    // Why do we do this? all info is in smart_factor_ptr
    // such as the triangulated point, whether it is valid or not
    // and the number of observations...
    // Is graph more up to date?
    boost::shared_ptr<SmartStereoFactor> gsf =
        boost::dynamic_pointer_cast<SmartStereoFactor>(graph.at(slot_id));
    CHECK(gsf) << "Cannot cast factor in graph to a smart stereo factor.";

    // Get triangulation result from smart factor.
    const gtsam::TriangulationResult& result = gsf->point();
    // Check that the boost::optional result is initialized.
    // Otherwise we will be dereferencing a nullptr and we will head
    // directly to undefined behaviour wonderland.
    if (result.is_initialized()) {
      if (result.valid()) {
        if (gsf->measured().size() >= min_age) {
          // Triangulation result from smart factor is valid and
          // we have observed the lmk at least min_age times.
          VLOG(20) << "Adding lmk with id: " << lmk_id
                   << " to list of lmks in time horizon";
          // Check that we have not added this lmk already...
          CHECK(points_with_id.find(lmk_id) == points_with_id.end());
          points_with_id[lmk_id] = *result;
          if (lmk_id_to_lmk_type_map) {
            (*lmk_id_to_lmk_type_map)[lmk_id] = LandmarkType::SMART;
          }
          nr_valid_smart_lmks++;
        } else {
          VLOG(20) << "Rejecting lmk with id: " << lmk_id
                   << " from list of lmks in time horizon: "
                   << "not enough measurements, " << gsf->measured().size()
                   << ", vs min_age of " << min_age << ".";
        }  // gsf->measured().size() >= min_age ?
      } else {
        VLOG(20) << "Rejecting lmk with id: " << lmk_id
                 << " from list of lmks in time horizon:\n"
                 << "triangulation result is not valid (result= {" << result
                 << "}).";
      }  // result.valid()?
    } else {
      VLOG(20) << "Triangulation result for smart factor of lmk with id "
               << lmk_id << " is not initialized...";
    }  // result.is_initialized()?

    // Next iteration.
    old_smart_factor_it++;
  }

  // Step 2:
  ////////////// Add landmarks that now are in projection factors. /////////////
  for (const gtsam::Values::Filtered<gtsam::Value>::ConstKeyValuePair&
           key_value : state_.filter(gtsam::Symbol::ChrTest('l'))) {
    DCHECK_EQ(gtsam::Symbol(key_value.key).chr(), 'l');
    const LandmarkId& lmk_id = gtsam::Symbol(key_value.key).index();
    DCHECK(points_with_id.find(lmk_id) == points_with_id.end());
    points_with_id[lmk_id] = key_value.value.cast<gtsam::Point3>();
    if (lmk_id_to_lmk_type_map) {
      (*lmk_id_to_lmk_type_map)[lmk_id] = LandmarkType::PROJECTION;
    }
    nr_proj_lmks++;
  }

  // TODO aren't these points post-optimization? Shouldn't we instead add
  // the points before optimization? Then the regularities we enforce will
  // have the most impact, otherwise the points in the optimization horizon
  // do not move that much after optimizing... they are almost frozen and
  // are not visually changing much...
  // They might actually not be changing that much because we are not
  // enforcing the regularities on the points that are out of current frame
  // in the backend currently...

  VLOG(10) << "Landmark typology to be used for the mesh:\n"
           << "Number of valid smart factors " << nr_valid_smart_lmks
           << " out of " << nr_smart_lmks << "\n"
           << "Number of landmarks (not involved in a smart factor) "
           << nr_proj_lmks << ".\n Total number of landmarks: "
           << (nr_valid_smart_lmks + nr_proj_lmks);
  return points_with_id;
}

/* -------------------------------------------------------------------------- */
// Workhorse that stores data and optimizes at each keyframe.
// [in] timestamp_kf_nsec, keyframe timestamp.
// [in] status_smart_stereo_measurements_kf, vision data.
// [in] stereo_ransac_body_pose, inertial data.
bool StereoVioBackEnd::addVisualInertialStateAndOptimize(
    const Timestamp& timestamp_kf_nsec,
    const StatusStereoMeasurements& status_smart_stereo_measurements_kf,
    const gtsam::PreintegrationType& pim,
    boost::optional<gtsam::Pose3> stereo_ransac_body_pose) {
  debug_info_.resetAddedFactorsStatistics();

  // Features and IMU line up --> do iSAM update
  last_kf_id_ = curr_kf_id_;
  ++curr_kf_id_;

  VLOG(1) << "VIO: adding keyframe " << curr_kf_id_
          << " at timestamp:" << UtilsNumerical::NsecToSec(timestamp_kf_nsec)
          << " (nsec).";

  /////////////////// MANAGE IMU MEASUREMENTS ///////////////////////////
  // Predict next step, add initial guess
  addImuValues(curr_kf_id_, pim);

  // Add imu factors between consecutive keyframe states
  addImuFactor(last_kf_id_, curr_kf_id_, pim);

  // Add between factor from RANSAC
  if (stereo_ransac_body_pose) {
    if (VLOG_IS_ON(10)) {
      LOG(INFO) << "VIO: adding between ";
      stereo_ransac_body_pose->print();
    }
    addBetweenFactor(last_kf_id_, curr_kf_id_, *stereo_ransac_body_pose);
  }

  /////////////////// MANAGE VISION MEASUREMENTS ///////////////////////////
  const StereoMeasurements& smart_stereo_measurements_kf =
      status_smart_stereo_measurements_kf.second;

  // if stereo ransac failed, remove all right pixels:
  // TrackingStatus kfTrackingStatus_stereo =
  //     status_smart_stereo_measurements_kf.first.kfTrackingStatus_stereo_;
  // if(kfTrackingStatus_stereo == TrackingStatus::INVALID){
  //   for(size_t i = 0; i < smartStereoMeasurements_kf.size(); i++)
  //     smartStereoMeasurements_kf[i].uR =
  //     std::numeric_limits<double>::quiet_NaN();;
  //}

  // extract relevant information from stereo frame
  LandmarkIds landmarks_kf;
  addStereoMeasurementsToFeatureTracks(
      curr_kf_id_, smart_stereo_measurements_kf, &landmarks_kf);

  if (VLOG_IS_ON(10)) {
    printFeatureTracks();
  }

  // decide which factors to add
  const TrackingStatus& kfTrackingStatus_mono =
      status_smart_stereo_measurements_kf.first.kfTrackingStatus_mono_;
  switch (kfTrackingStatus_mono) {
    // vehicle is not moving
    case TrackingStatus::LOW_DISPARITY: {
      LOG(WARNING)
          << "Low disparity: adding zero velocity and no motion factors.";
      addZeroVelocityPrior(curr_kf_id_);
      addNoMotionFactor(last_kf_id_, curr_kf_id_);
      break;
    }

    // This did not improve in any case
    //  case TrackingStatus::INVALID :// ransac failed hence we cannot
    //  trust features
    //    if (verbosity_ >= 7) {printf("Add constant velocity factor
    //    (monoRansac is INVALID)\n");}
    //    addConstantVelocityFactor(last_id_, cur_id_); break;

    // TrackingStatus::VALID, FEW_MATCHES, INVALID, DISABLED : //
    // we add features in VIO
    default: {
      addLandmarksToGraph(landmarks_kf);
      break;
    }
  }

  // Why do we do this??
  // This lags 1 step behind to mimic hw.
  // imu_bias_lkf_ gets updated in the optimize call.
  imu_bias_prev_kf_ = imu_bias_lkf_;

  return optimize(timestamp_kf_nsec, curr_kf_id_, backend_params_.numOptimize_);
}

bool StereoVioBackEnd::addVisualInertialStateAndOptimize(const BackendInput& input) {
  bool use_stereo_btw_factor =
      backend_params_.addBetweenStereoFactors_ &&
      input.stereo_tracking_status_ == TrackingStatus::VALID;
  VLOG(10) << "Add visual inertial state and optimize.";
  VLOG_IF(10, use_stereo_btw_factor) << "Using stereo between factor.";
  CHECK(input.status_stereo_measurements_kf_);
  CHECK(input.pim_);
  bool is_smoother_ok = addVisualInertialStateAndOptimize(
      input.timestamp_,  // Current time for fixed lag smoother.
      *input.status_stereo_measurements_kf_,  // Vision data.
      *input.pim_,                            // Imu preintegrated data.
      use_stereo_btw_factor
          ? input.stereo_ransac_body_pose_
          : boost::none);  // optional: pose estimate from stereo ransac
  // Bookkeeping
  timestamp_lkf_ = input.timestamp_;
  return is_smoother_ok;
}

// TODO(Toni): no need to pass landmarks_kf, can iterate directly over feature
// tracks...
// Uses landmark table to add factors in graph.
void StereoVioBackEnd::addLandmarksToGraph(const LandmarkIds& landmarks_kf) {
  // Add selected landmarks to graph:
  int n_new_landmarks = 0;
  int n_updated_landmarks = 0;
  debug_info_.numAddedSmartF_ += landmarks_kf.size();

  for (const LandmarkId& lmk_id : landmarks_kf) {
    FeatureTrack& ft = feature_tracks_.at(lmk_id);
    // TODO(TONI): parametrize this min_num_of_obs... should be in frontend
    // rather than backend though...
    if (ft.obs_.size() < 2) {  // we only insert feature tracks of length at
                               // least 2 (otherwise uninformative)
      continue;
    }

    if (!ft.in_ba_graph_) {
      ft.in_ba_graph_ = true;
      addLandmarkToGraph(lmk_id, ft);
      ++n_new_landmarks;
    } else {
      const std::pair<FrameId, StereoPoint2> obs_kf = ft.obs_.back();

      LOG_IF(FATAL, obs_kf.first != curr_kf_id_)
          << "addLandmarksToGraph: last obs is not from the current "
             "keyframe!\n";

      updateLandmarkInGraph(lmk_id, obs_kf);
      ++n_updated_landmarks;
    }
  }

  VLOG(10) << "Added " << n_new_landmarks << " new landmarks\n"
           << "Updated " << n_updated_landmarks << " landmarks in graph";
}

/* -------------------------------------------------------------------------- */
// TODO this function doesn't do just one thing... Should be refactored!
// It returns the landmark ids of the stereo measurements
// It also updates the feature tracks. Why is this in the backend???
// TODO(Toni): the FeatureTracks can be fully replaced by the StereoMeasurements
// class...
void StereoVioBackEnd::addStereoMeasurementsToFeatureTracks(
    const int& frame_num,
    const StereoMeasurements& stereo_meas_kf,
    LandmarkIds* landmarks_kf) {
  CHECK_NOTNULL(landmarks_kf);

  // TODO: feature tracks will grow unbounded.

  // Make sure the landmarks_kf vector is empty and has a suitable size.
  const size_t& n_stereo_measurements = stereo_meas_kf.size();
  landmarks_kf->resize(n_stereo_measurements);

  // Store landmark ids.
  // TODO(Toni): the concept of feature tracks should not be in the backend...
  for (size_t i = 0u; i < n_stereo_measurements; ++i) {
    const LandmarkId& lmk_id_in_kf_i = stereo_meas_kf[i].first;
    const StereoPoint2& stereo_px_i = stereo_meas_kf[i].second;

    // We filtered invalid lmks in the StereoTracker, so this should not happen.
    CHECK_NE(lmk_id_in_kf_i, -1) << "landmarkId_kf_i == -1?";

    // Thinner structure that only keeps landmarkIds.
    // These landmark ids are only the ones visible in current keyframe,
    // with a valid track...
    // CHECK that we do not have repeated lmk ids!
    DCHECK(std::find(landmarks_kf->begin(),
                     landmarks_kf->end(),
                     lmk_id_in_kf_i) == landmarks_kf->end());
    (*landmarks_kf)[i] = lmk_id_in_kf_i;

    // Add features to vio->featureTracks_ if they are new.
    const FeatureTracks::iterator& feature_track_it =
        feature_tracks_.find(lmk_id_in_kf_i);
    if (feature_track_it == feature_tracks_.end()) {
      // New feature.
      VLOG(20) << "Creating new feature track for lmk: " << lmk_id_in_kf_i
               << '.';
      feature_tracks_.insert(
          std::make_pair(lmk_id_in_kf_i, FeatureTrack(frame_num, stereo_px_i)));
      ++landmark_count_;
    } else {
      // @TODO: It seems that this else condition does not help --
      // conjecture that it creates long feature tracks with low information
      // (i.e. we're not moving)
      // This is problematic in conjunction with our landmark selection
      // mechanism which prioritizes long feature tracks

      // TODO: to avoid making the feature tracks grow unbounded we could
      // use a tmp feature tracks container to which we would add the old
      // feature track plus the new observation on it. (for new tracks, it
      // would be the same as above, using the tmp structure of course).

      // Add observation to existing landmark.
      VLOG(20) << "Updating feature track for lmk: " << lmk_id_in_kf_i << ".";
      feature_track_it->second.obs_.push_back(
          std::make_pair(frame_num, stereo_px_i));

      // TODO(Toni):
      // Mark feature tracks that have been re-observed, so that we can delete
      // the broken feature tracks efficiently.
    }
  }
}

/* -------------------------------------------------------------------------- */
// Updates a landmark already in the graph.
void StereoVioBackEnd::updateLandmarkInGraph(
    const LandmarkId& lmk_id,
    const std::pair<FrameId, StereoPoint2>& new_measurement) {
  // Update existing smart-factor
  auto old_smart_factors_it = old_smart_factors_.find(lmk_id);
  CHECK(old_smart_factors_it != old_smart_factors_.end())
      << "Landmark not found in old_smart_factors_ with id: " << lmk_id;

  const SmartStereoFactor::shared_ptr& old_factor =
      old_smart_factors_it->second.first;
  // Clone old factor to keep all previous measurements, now append one.
  SmartStereoFactor::shared_ptr new_factor =
      boost::make_shared<SmartStereoFactor>(*old_factor);
  gtsam::Symbol pose_symbol('x', new_measurement.first);
  if (smoother_->getFactors().exists(pose_symbol)) {
    const StereoPoint2& measurement = new_measurement.second;
    new_factor->add(measurement, pose_symbol, stereo_cal_);
  } else {
    VLOG(10) << "Factor with lmk id " << lmk_id
             << " is linking to a marginalized state!";
  }

  // Update the factor
  Slot slot = old_smart_factors_it->second.second;
  if (slot != -1) {
    new_smart_factors_.insert(std::make_pair(lmk_id, new_factor));
  } else {
    // If it's slot in the graph is still -1, it means that the factor has not
    // been inserted yet in the graph...
    LOG(FATAL) << "When updating the smart factor, its slot should not be -1!"
                  " Offensive lmk_id: "
               << lmk_id;
  }
  old_smart_factors_it->second.first = new_factor;
  VLOG(10) << "updateLandmarkInGraph: added observation to point: " << lmk_id;
}

/* -------------------------------------------------------------------------- */
// Adds a landmark to the graph for the first time.
void StereoVioBackEnd::addLandmarkToGraph(const LandmarkId& lmk_id,
                                    const FeatureTrack& ft) {
  // We use a unit pinhole projection camera for the smart factors to be
  // more efficient.
  SmartStereoFactor::shared_ptr new_factor =
      boost::make_shared<SmartStereoFactor>(
          smart_noise_, smart_factors_params_, B_Pose_leftCam_);

  VLOG(10) << "Adding landmark with: " << ft.obs_.size()
           << " landmarks to graph, with keys: ";

  // Add observations to smart factor
  if (VLOG_IS_ON(10)) new_factor->print();
  for (const std::pair<FrameId, StereoPoint2>& obs : ft.obs_) {
    const FrameId& frame_id = obs.first;
    const gtsam::Symbol& pose_symbol = gtsam::Symbol('x', frame_id);
    if (smoother_->getFactors().exists(pose_symbol)) {
      const StereoPoint2& measurement = obs.second;
      new_factor->add(measurement, pose_symbol, stereo_cal_);
    } else {
      VLOG(10) << "Factor with lmk id " << lmk_id
               << " is linking to a marginalized state!";
    }

    if (VLOG_IS_ON(10)) std::cout << " " << obs.first;
  }
  if (VLOG_IS_ON(10)) std::cout << std::endl;

  // add new factor to suitable structures:
  new_smart_factors_.insert(std::make_pair(lmk_id, new_factor));
  old_smart_factors_.insert(
      std::make_pair(lmk_id, std::make_pair(new_factor, -1)));
}

void StereoVioBackEnd::print() const {
  backend_params_.print();

  smoother_->params().print(std::string(10, '.') + "** ISAM2 Parameters **" +
                            std::string(10, '.'));

  LOG(INFO) << "Used stereo calibration in backend: ";
  if (FLAGS_minloglevel < 1) {
    stereo_cal_->print("\n stereoCal_\n");
  }

  LOG(INFO) << "** Backend Initial Members: \n"
            << "B_Pose_leftCam_: " << B_Pose_leftCam_ << '\n'
            << "W_Pose_B_lkf_: " << W_Pose_B_lkf_ << '\n'
            << "W_Vel_B_lkf_ (transpose): " << W_Vel_B_lkf_.transpose() << '\n'
            << "imu_bias_lkf_" << imu_bias_lkf_ << '\n'
            << "imu_bias_prev_kf_" << imu_bias_prev_kf_ << '\n'
            << "last_id_ " << last_kf_id_ << '\n'
            << "cur_id_ " << curr_kf_id_ << '\n'
            << "landmark_count_ " << landmark_count_;
}

}  // namespace VIO
