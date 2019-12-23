/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackEnd.cpp
 * @brief  Visual-Inertial Odometry pipeline, as described in these papers:
 *
 * A. Rosinol, M. Abate, Y. Chang, L. Carlone.
 * Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization
 * and Mapping. In IEEE Intl. Conf. on Robotics and Automation (ICRA), 2019.
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

#include "kimera-vio/backend/VioBackEnd.h"

#include <limits>  // for numeric_limits<>
#include <map>
#include <string>
#include <utility>  // for make_pair
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

// Only for gtNavState ...
#include "kimera-vio/dataprovider/DataProviderInterface-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd-definitions.h"  // for safeCast
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"

DEFINE_bool(debug_graph_before_opt,
            false,
            "Store factor graph before optimization for later printing if the "
            "optimization fails.");
DEFINE_bool(process_cheirality,
            false,
            "Handle cheirality exception by removing problematic landmarks and "
            "re-running optimization.");
DEFINE_int32(max_number_of_cheirality_exceptions,
             5,
             "Sets the maximum number of times we process a cheirality "
             "exception for a given optimization problem. This is to avoid too "
             "many recursive calls to update the smoother");
DEFINE_bool(compute_state_covariance,
            false,
            "Flag to compute state covariance from optimization backend");

namespace VIO {

/* -------------------------------------------------------------------------- */
VioBackEnd::VioBackEnd(const Pose3& B_Pose_leftCam,
                       const StereoCalibPtr& stereo_calibration,
                       const VioBackEndParams& backend_params,
                       const ImuParams& imu_params,
                       const BackendOutputParams& backend_output_params,
                       bool log_output)
    : backend_params_(backend_params),
      imu_params_(imu_params),
      backend_output_params_(backend_output_params),
      backend_state_(BackendState::Bootstrap),
      timestamp_lkf_(-1),
      imu_bias_lkf_(ImuBias()),
      W_Vel_B_lkf_(Vector3::Zero()),
      W_Pose_B_lkf_(Pose3()),
      imu_bias_prev_kf_(ImuBias()),
      B_Pose_leftCam_(B_Pose_leftCam),
      stereo_cal_(stereo_calibration),
      last_kf_id_(-1),
      curr_kf_id_(0),
      landmark_count_(0),
      log_output_(log_output),
      logger_(log_output ? VIO::make_unique<BackendLogger>() : nullptr) {
  // TODO the parsing of the params should be done inside here out from the
  // path to the params file, otherwise other derived VIO backends will be
  // stuck with the parameters used by vanilla VIO, as there is no polymorphic
  // container in C++...
  // This way VioBackEnd can parse the params it cares about, while others can
  // have the opportunity to parse their own parameters as well.
  // Unfortunately, doing that would not work because many other modules use
  // VioBackEndParams as weird as this may sound...
  // For now we have polymorphic params, with dynamic_cast to derived class,
  // aka suboptimal...

  //////////////////////////////////////////////////////////////////////////////
  // Initialize smoother.
#ifdef INCREMENTAL_SMOOTHER
  gtsam::ISAM2Params isam_param;
  setIsam2Params(backend_params, &isam_param);

  smoother_ = VIO::make_unique<Smoother>(backend_params.horizon_, isam_param);
#else  // BATCH SMOOTHER
  gtsam::LevenbergMarquardtParams lmParams;
  lmParams.setlambdaInitial(0.0);     // same as GN
  lmParams.setlambdaLowerBound(0.0);  // same as GN
  lmParams.setlambdaUpperBound(0.0);  // same as GN)
  smoother_ = VIO::make_unique<Smoother>(vioParams.horizon_, lmParams);
#endif

  // Set parameters for all factors.
  setFactorsParams(backend_params,
                   &smart_noise_,
                   &smart_factors_params_,
                   &no_motion_prior_noise_,
                   &zero_velocity_prior_noise_,
                   &constant_velocity_prior_noise_);

  // Reset debug info.
  resetDebugInfo(&debug_info_);

  // Print parameters if verbose
  if (VLOG_IS_ON(1)) print();
}

/* -------------------------------------------------------------------------- */
BackendOutput::UniquePtr VioBackEnd::spinOnce(const BackendInput& input) {
  if (VLOG_IS_ON(10)) input.print();

  switch (backend_state_) {
    case BackendState::Bootstrap: {
      // Initialize backend.
      // TODO(Toni) we should do initialization here and follow the general
      // workflow of the pipeline instead of having a different module...
      backend_state_ = BackendState::Nominal;
      addVisualInertialStateAndOptimize(input);
      break;
    }
    case BackendState::Nominal: {
      // Process data with VIO.
      addVisualInertialStateAndOptimize(input);
      break;
    }
    default: {
      LOG(FATAL) << "Unrecognized backend state.";
      break;
    }
  }

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
    lmk_ids_to_3d_points_in_time_horizon = getMapLmkIdsTo3dPointsInTimeHorizon(
        kOutputLmkTypeMap ? &lmk_id_to_lmk_type_map : nullptr, kMinLmkObs);
  }

  // Create Backend Output Payload.
  BackendOutput::UniquePtr output_payload = VIO::make_unique<BackendOutput>(
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
  return output_payload;
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::registerImuBiasUpdateCallback(
    const ImuBiasCallback& imu_bias_update_callback) {
  // Register callback.
  imu_bias_update_callback_ = imu_bias_update_callback;
  // Update imu bias just in case. This is useful specially because the
  // backend initializes the imu bias to some value. So whoever is asking
  // to register this callback should have the newest imu bias.
  imu_bias_update_callback(imu_bias_lkf_);
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::initStateAndSetPriors(
    const VioNavStateTimestamped& vio_nav_state_initial_seed) {
  // Update member variables.
  timestamp_lkf_ = vio_nav_state_initial_seed.timestamp_;
  W_Pose_B_lkf_ = vio_nav_state_initial_seed.pose_;
  W_Vel_B_lkf_ = vio_nav_state_initial_seed.velocity_;
  imu_bias_lkf_ = vio_nav_state_initial_seed.imu_bias_;
  imu_bias_prev_kf_ = vio_nav_state_initial_seed.imu_bias_;

  LOG(INFO) << "Initial state seed: \n"
            << " - Initial pose: " << W_Pose_B_lkf_ << '\n'
            << " - Initial vel: " << W_Vel_B_lkf_.transpose() << '\n'
            << " - Initial IMU bias: " << imu_bias_lkf_;

  // Can't add inertial prior factor until we have a state measurement.
  addInitialPriorFactors(curr_kf_id_);

  // TODO encapsulate this in a function, code duplicated in addImuValues.
  // Add initial state seed
  new_values_.insert(gtsam::Symbol('x', curr_kf_id_), W_Pose_B_lkf_);
  new_values_.insert(gtsam::Symbol('v', curr_kf_id_), W_Vel_B_lkf_);
  new_values_.insert(gtsam::Symbol('b', curr_kf_id_), imu_bias_lkf_);

  VLOG(2) << "Start optimize with initial state and priors!";
  optimize(vio_nav_state_initial_seed.timestamp_,
           curr_kf_id_,
           backend_params_.numOptimize_);
}

/* --------------------------------------------------------------------------
 */
// Workhorse that stores data and optimizes at each keyframe.
// [in] timestamp_kf_nsec, keyframe timestamp.
// [in] status_smart_stereo_measurements_kf, vision data.
// [in] stereo_ransac_body_pose, inertial data.
void VioBackEnd::addVisualInertialStateAndOptimize(
    const Timestamp& timestamp_kf_nsec,
    const StatusStereoMeasurements& status_smart_stereo_measurements_kf,
    const gtsam::PreintegrationType& pim,
    boost::optional<gtsam::Pose3> stereo_ransac_body_pose) {
  debug_info_.resetAddedFactorsStatistics();

  // Features and IMU line up --> do iSAM update
  last_kf_id_ = curr_kf_id_;
  ++curr_kf_id_;

  VLOG(1) << "VIO: adding keyframe " << curr_kf_id_
          << " at timestamp:" << UtilsOpenCV::NsecToSec(timestamp_kf_nsec)
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
  const SmartStereoMeasurements& smart_stereo_measurements_kf =
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
    case TrackingStatus::LOW_DISPARITY:  // vehicle is not moving
      if (VLOG_IS_ON(10)) {
        printf("Add zero velocity and no motion factors\n");
      }
      addZeroVelocityPrior(curr_kf_id_);
      addNoMotionFactor(last_kf_id_, curr_kf_id_);
      break;

      // This did not improve in any case
      //  case TrackingStatus::INVALID :// ransac failed hence we cannot
      //  trust features
      //    if (verbosity_ >= 7) {printf("Add constant velocity factor
      //    (monoRansac is INVALID)\n");}
      //    addConstantVelocityFactor(last_id_, cur_id_); break;

    default:  // TrackingStatus::VALID, FEW_MATCHES, INVALID, DISABLED : //
              // we add features in VIO
      addLandmarksToGraph(landmarks_kf);
      break;
  }

  // Why do we do this??
  // This lags 1 step behind to mimic hw.
  // imu_bias_lkf_ gets updated in the optimize call.
  imu_bias_prev_kf_ = imu_bias_lkf_;

  optimize(timestamp_kf_nsec, curr_kf_id_, backend_params_.numOptimize_);
}

void VioBackEnd::addVisualInertialStateAndOptimize(const BackendInput& input) {
  bool use_stereo_btw_factor =
      backend_params_.addBetweenStereoFactors_ &&
      input.stereo_tracking_status_ == TrackingStatus::VALID;
  VLOG(10) << "Add visual inertial state and optimize.";
  VLOG_IF(10, use_stereo_btw_factor) << "Using stereo between factor.";
  CHECK(input.status_stereo_measurements_kf_);
  CHECK(input.pim_);
  addVisualInertialStateAndOptimize(
      input.timestamp_,  // Current time for fixed lag smoother.
      *input.status_stereo_measurements_kf_,  // Vision data.
      *input.pim_,                            // Imu preintegrated data.
      use_stereo_btw_factor
          ? input.stereo_ransac_body_pose_
          : boost::none);  // optional: pose estimate from stereo ransac
  // Bookkeeping
  timestamp_lkf_ = input.timestamp_;
}

/* -------------------------------------------------------------------------- */
// Uses landmark table to add factors in graph.
void VioBackEnd::addLandmarksToGraph(const LandmarkIds& landmarks_kf) {
  // Add selected landmarks to graph:
  int n_new_landmarks = 0;
  int n_updated_landmarks = 0;
  debug_info_.numAddedSmartF_ += landmarks_kf.size();

  for (const LandmarkId& lm_id : landmarks_kf) {
    FeatureTrack& ft = feature_tracks_.at(lm_id);
    if (ft.obs_.size() < 2) {  // we only insert feature tracks of length at
                               // least 2 (otherwise uninformative)
      continue;
    }

    if (!ft.in_ba_graph_) {
      ft.in_ba_graph_ = true;
      addLandmarkToGraph(lm_id, ft);
      ++n_new_landmarks;
    } else {
      const std::pair<FrameId, StereoPoint2> obs_kf = ft.obs_.back();

      if (obs_kf.first != curr_kf_id_)  // sanity check
        LOG(FATAL) << "addLandmarksToGraph: last obs is not from the current "
                      "keyframe!\n";

      updateLandmarkInGraph(lm_id, obs_kf);
      ++n_updated_landmarks;
    }
  }

  if (VLOG_IS_ON(10)) {
    std::cout << "Added " << n_new_landmarks << " new landmarks" << std::endl;
    std::cout << "Updated " << n_updated_landmarks << " landmarks in graph"
              << std::endl;
  }
}

/* --------------------------------------------------------------------------
 */
// Adds a landmark to the graph for the first time.
void VioBackEnd::addLandmarkToGraph(const LandmarkId& lm_id,
                                    const FeatureTrack& ft) {
  // We use a unit pinhole projection camera for the smart factors to be
  // more efficient.
  SmartStereoFactor::shared_ptr new_factor =
      boost::make_shared<SmartStereoFactor>(
          smart_noise_, smart_factors_params_, B_Pose_leftCam_);

  if (VLOG_IS_ON(10)) {
    std::cout << "Adding landmark with: " << ft.obs_.size()
              << " landmarks to graph, with keys: ";
  }
  if (VLOG_IS_ON(10)) {
    new_factor->print();
  }

  // add observations to smart factor
  for (const std::pair<FrameId, StereoPoint2>& obs : ft.obs_) {
    new_factor->add(obs.second, gtsam::Symbol('x', obs.first), stereo_cal_);
    if (VLOG_IS_ON(10)) {
      std::cout << " " << obs.first;
    }
  }
  if (VLOG_IS_ON(10)) {
    std::cout << std::endl;
  }
  // add new factor to suitable structures:
  new_smart_factors_.insert(std::make_pair(lm_id, new_factor));
  old_smart_factors_.insert(
      std::make_pair(lm_id, std::make_pair(new_factor, -1)));
}

/* --------------------------------------------------------------------------
 */
void VioBackEnd::updateLandmarkInGraph(
    const LandmarkId& lmk_id,
    const std::pair<FrameId, StereoPoint2>& newObs) {
  // Update existing smart-factor
  auto old_smart_factors_it = old_smart_factors_.find(lmk_id);
  LOG_IF(FATAL, old_smart_factors_it == old_smart_factors_.end())
      << "Landmark not found in old_smart_factors_";

  const SmartStereoFactor::shared_ptr& old_factor =
      old_smart_factors_it->second.first;
  // TODO(Toni) this looks super sketchy!
  SmartStereoFactor::shared_ptr new_factor =
      boost::make_shared<SmartStereoFactor>(*old_factor);  // clone old factor
  new_factor->add(newObs.second, gtsam::Symbol('x', newObs.first), stereo_cal_);

  // update the factor
  if (old_smart_factors_it->second.second !=
      -1) {  // if slot is still -1, it means that the factor has not been
             // inserted yet in the graph
    new_smart_factors_.insert(std::make_pair(lmk_id, new_factor));
  } else {
    LOG(FATAL) << "updateLandmarkInGraph: when calling update the slot should "
                  "be already != -1! \n";
  }
  old_smart_factors_it->second.first = new_factor;
  if (VLOG_IS_ON(10)) {
    std::cout << "updateLandmarkInGraph: added observation to point: " << lmk_id
              << std::endl;
  }
}

/* -------------------------------------------------------------------------- */
// Get valid 3D points and corresponding lmk id.
// Warning! it modifies old_smart_factors_!!
PointsWithIdMap VioBackEnd::getMapLmkIdsTo3dPointsInTimeHorizon(
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
// NOT TESTED (--> There is a UnitTest function in UtilsOpenCV)
void VioBackEnd::computeStateCovariance() {
  gtsam::Marginals marginals(smoother_->getFactors(),
                             state_,
                             gtsam::Marginals::Factorization::CHOLESKY);

  // Current state includes pose, velocity and imu biases.
  gtsam::KeyVector keys;
  keys.push_back(gtsam::Symbol('x', curr_kf_id_));
  keys.push_back(gtsam::Symbol('v', curr_kf_id_));
  keys.push_back(gtsam::Symbol('b', curr_kf_id_));

  // Return the marginal covariance matrix.
  state_covariance_lkf_ = UtilsOpenCV::Covariance_bvx2xvb(
      marginals.jointMarginalCovariance(keys)
          .fullMatrix());  // 6 + 3 + 6 = 15x15matrix
}

/* -------------------------------------------------------------------------- */
// TODO this function doesn't do just one thing... Should be refactored!
// It returns the landmark ids of the stereo measurements
// It also updates the feature tracks. Why is this in the backend???
void VioBackEnd::addStereoMeasurementsToFeatureTracks(
    const int& frame_num,
    const SmartStereoMeasurements& stereoMeasurements_kf,
    LandmarkIds* landmarks_kf) {
  CHECK_NOTNULL(landmarks_kf);

  // TODO: feature tracks will grow unbounded.

  // Make sure the landmarks_kf vector is empty and has a suitable size.
  const size_t& n_stereo_measurements = stereoMeasurements_kf.size();
  landmarks_kf->resize(n_stereo_measurements);

  // Store landmark ids.
  // TODO(Toni): the concept of feature tracks should not be in the backend...
  for (size_t i = 0; i < n_stereo_measurements; ++i) {
    const LandmarkId& lmk_id_in_kf_i = stereoMeasurements_kf[i].first;
    const StereoPoint2& stereo_px_i = stereoMeasurements_kf[i].second;

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
    }
  }
}

/// Value adders.
/* -------------------------------------------------------------------------- */
void VioBackEnd::addImuValues(const FrameId& cur_id,
                              const gtsam::PreintegrationType& pim) {
  gtsam::NavState navstate_lkf(W_Pose_B_lkf_, W_Vel_B_lkf_);
  gtsam::NavState navstate_k = pim.predict(navstate_lkf, imu_bias_lkf_);

  debug_info_.navstate_k_ = navstate_k;

  // Update state with initial guess
  new_values_.insert(gtsam::Symbol('x', cur_id), navstate_k.pose());
  new_values_.insert(gtsam::Symbol('v', cur_id), navstate_k.velocity());
  new_values_.insert(gtsam::Symbol('b', cur_id), imu_bias_lkf_);
}

/// Factor adders.
/* -------------------------------------------------------------------------- */
void VioBackEnd::addImuFactor(const FrameId& from_id,
                              const FrameId& to_id,
                              const gtsam::PreintegrationType& pim) {
  switch (imu_params_.imu_preintegration_type_) {
    case ImuPreintegrationType::kPreintegratedCombinedMeasurements: {
      new_imu_prior_and_other_factors_.push_back(
          boost::make_shared<gtsam::CombinedImuFactor>(
              gtsam::Symbol('x', from_id),
              gtsam::Symbol('v', from_id),
              gtsam::Symbol('x', to_id),
              gtsam::Symbol('v', to_id),
              gtsam::Symbol('b', from_id),
              gtsam::Symbol('b', to_id),
              safeCastToPreintegratedCombinedImuMeasurements(pim)));
      break;
    }
    case ImuPreintegrationType::kPreintegratedImuMeasurements: {
      new_imu_prior_and_other_factors_.push_back(
          boost::make_shared<gtsam::ImuFactor>(
              gtsam::Symbol('x', from_id),
              gtsam::Symbol('v', from_id),
              gtsam::Symbol('x', to_id),
              gtsam::Symbol('v', to_id),
              gtsam::Symbol('b', from_id),
              safeCastToPreintegratedImuMeasurements(pim)));

      static const gtsam::imuBias::ConstantBias zero_bias(Vector3(0, 0, 0),
                                                          Vector3(0, 0, 0));

      // Factor to discretize and move normalize by the interval between
      // measurements:
      CHECK_NE(imu_params_.nominal_rate_, 0.0)
          << "Nominal IMU rate param cannot be 0.";
      // 1/sqrt(nominalImuRate_) to discretize, then
      // sqrt(pim_->deltaTij()/nominalImuRate_) to count the nr of measurements.
      const double d = std::sqrt(pim.deltaTij()) / imu_params_.nominal_rate_;
      Vector6 biasSigmas;
      biasSigmas.head<3>().setConstant(d * imu_params_.acc_walk_);
      biasSigmas.tail<3>().setConstant(d * imu_params_.gyro_walk_);
      const gtsam::SharedNoiseModel& bias_noise_model =
          gtsam::noiseModel::Diagonal::Sigmas(biasSigmas);

      new_imu_prior_and_other_factors_.push_back(
          boost::make_shared<
              gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
              gtsam::Symbol('b', from_id),
              gtsam::Symbol('b', to_id),
              zero_bias,
              bias_noise_model));
      break;
    }
    default: {
      LOG(FATAL) << "Unknown IMU Preintegration Type.";
      break;
    }
  }

  debug_info_.imuR_lkf_kf = pim.deltaRij();
  debug_info_.numAddedImuF_++;

  // TODO reset preintegration should be done by the preintegrator itself!
  // Right now we are working with a pim which is a copy of the one used
  // by the ImuFrontend so resetting this copy won't do much!

  // pim_.reset();
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::addBetweenFactor(const FrameId& from_id,
                                  const FrameId& to_id,
                                  const gtsam::Pose3& from_id_POSE_to_id) {
  Vector6 precisions;
  precisions.head<3>().setConstant(backend_params_.betweenRotationPrecision_);
  precisions.tail<3>().setConstant(
      backend_params_.betweenTranslationPrecision_);
  static const gtsam::SharedNoiseModel& betweenNoise_ =
      gtsam::noiseModel::Diagonal::Precisions(precisions);

  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          gtsam::Symbol('x', from_id),
          gtsam::Symbol('x', to_id),
          from_id_POSE_to_id,
          betweenNoise_));

  debug_info_.numAddedBetweenStereoF_++;
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::addNoMotionFactor(const FrameId& from_id,
                                   const FrameId& to_id) {
  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          gtsam::Symbol('x', from_id),
          gtsam::Symbol('x', to_id),
          Pose3(),
          no_motion_prior_noise_));

  debug_info_.numAddedNoMotionF_++;

  if (VLOG_IS_ON(10)) {
    std::cout << "No motion detected, adding no relative motion prior"
              << std::endl;
  }
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::addZeroVelocityPrior(const FrameId& frame_id) {
  VLOG(10) << "No motion detected, adding zero velocity prior.";
  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
          gtsam::Symbol('v', frame_id),
          gtsam::Vector3::Zero(),
          zero_velocity_prior_noise_));
}

/* -------------------------------------------------------------------------- */
// TODO remove global variables from optimize, pass them as local
// parameters...
// TODO make changes to global variables to the addVisualInertial blah blah.
// TODO remove timing logging and use Statistics.h instead.
void VioBackEnd::optimize(
    const Timestamp& timestamp_kf_nsec,
    const FrameId& cur_id,
    const size_t& max_extra_iterations,
    const gtsam::FactorIndices& extra_factor_slots_to_delete) {
  DCHECK(smoother_) << "Incremental smoother is a null pointer.";

  // Only for statistics and debugging.
  // Store start time to calculate absolute total time taken.
  const auto& total_start_time = utils::Timer::tic();
  // Store start time to calculate per module total time.
  auto start_time = total_start_time;
  // Reset all timing info.
  debug_info_.resetTimes();

  /////////////////////// BOOKKEEPING
  /////////////////////////////////////////////
  const size_t& number_of_new_smart_factors = new_smart_factors_.size();
  // We need to remove all previous smart factors in the factor graph
  // for which we have new observations.
  // The following is just to update the vector delete_slots with those
  // slots in the factor graph that correspond to smart factors for which
  // we've got new observations.
  // We initialize delete_slots with Extra factor slots to delete contains
  // potential factors that we want to delete, it is typically an empty
  // vector, and is only used to give flexibility to subclasses (regular
  // vio).
  gtsam::FactorIndices delete_slots = extra_factor_slots_to_delete;

  std::vector<LandmarkId> lmk_ids_of_new_smart_factors_tmp(
      number_of_new_smart_factors);
  gtsam::NonlinearFactorGraph new_factors_tmp;
  new_factors_tmp.reserve(number_of_new_smart_factors +
                          new_imu_prior_and_other_factors_.size());
  // TODO we know the actual end size... but I am not sure how to use factor
  // graph API for appending factors without copying or re-allocation...
  new_factors_tmp.resize(number_of_new_smart_factors);
  size_t i = 0;
  for (const auto& new_smart_factor : new_smart_factors_) {
    // Push back the smart factor to the list of new factors to add to the
    // graph. // Smart factor, so same address right?
    new_factors_tmp.at(i) = new_smart_factor.second;
    // Store lmk id of the smart factor to add to the graph.
    lmk_ids_of_new_smart_factors_tmp.at(i) = new_smart_factor.first;
    i++;

    // Find smart factor and slot in old_smart_factors_ corresponding to
    // the lmk with id of the new smart factor.
    const auto& it = old_smart_factors_.find(new_smart_factor.first);
    DCHECK(it != old_smart_factors_.end())
        << "Lmk with id: " << new_smart_factor.first
        << " could not be found in old_smart_factors_.";

    if (it->second.second != -1) {
      // Smart factor Slot is different than -1, therefore the factor is
      // already in the factor graph.
      // We must delete the smart factor from the graph.
      // We need to remove all smart factors that have new observations.
      // TODO what happens if delete_slots has repeated elements?
      DCHECK_GE(it->second.second, 0);
      delete_slots.push_back(it->second.second);
    }
  }

  // Add also other factors (imu, priors).
  // SMART FACTORS MUST BE FIRST, otherwise when recovering the slots
  // for the smart factors we will mess up.
  // push back many factors with an iterator over shared_ptr
  // (factors are not copied)
  new_factors_tmp.push_back(new_imu_prior_and_other_factors_.begin(),
                            new_imu_prior_and_other_factors_.end());

  //////////////////////////////////////////////////////////////////////////////

  if (VLOG_IS_ON(10) || log_output_) {
    debug_info_.factorsAndSlotsTime_ =
        utils::Timer::toc<std::chrono::seconds>(start_time).count();
    start_time = utils::Timer::tic();
  }

  if (VLOG_IS_ON(10)) {
    // Get state before optimization to compute error.
    debug_info_.stateBeforeOpt = gtsam::Values(state_);
    BOOST_FOREACH (const gtsam::Values::ConstKeyValuePair& key_value,
                   new_values_) {
      debug_info_.stateBeforeOpt.insert(key_value.key, key_value.value);
    }
  }

  if (VLOG_IS_ON(10)) {
    printSmootherInfo(new_factors_tmp,
                      delete_slots,
                      "Smoother status before update:",
                      VLOG_IS_ON(10));
  }

  // Recreate the graph before marginalization.
  if (VLOG_IS_ON(10) || FLAGS_debug_graph_before_opt) {
    debug_info_.graphBeforeOpt = smoother_->getFactors();
    debug_info_.graphToBeDeleted = gtsam::NonlinearFactorGraph();
    debug_info_.graphToBeDeleted.resize(delete_slots.size());
    for (size_t i = 0; i < delete_slots.size(); i++) {
      // If the factor is to be deleted, store it as graph to be deleted.
      CHECK(smoother_->getFactors().exists(delete_slots.at(i)));
      debug_info_.graphToBeDeleted.at(i) =
          smoother_->getFactors().at(delete_slots.at(i));
    }
  }

  // Use current timestamp for each new value. This timestamp will be used
  // to determine if the variable should be marginalized.
  // Needs to use DOUBLE because gtsam works with that, but we are working
  // with int64_t (nsecs).
  std::map<Key, double> timestamps;
  // Also needs to convert to seconds...
  double timestamp_kf = static_cast<double>(timestamp_kf_nsec) * 1e-9;
  BOOST_FOREACH (const gtsam::Values::ConstKeyValuePair& key_value,
                 new_values_) {
    timestamps[key_value.key] =
        timestamp_kf;  // for the latest pose, velocity, and bias
  }
  DCHECK_EQ(timestamps.size(), new_values_.size());

  // Store time before iSAM update.
  if (VLOG_IS_ON(10) || log_output_) {
    debug_info_.updateTime_ =
        utils::Timer::toc<std::chrono::seconds>(start_time).count();
    start_time = utils::Timer::tic();
  }

  // Compute iSAM update.
  VLOG(10) << "iSAM2 update with " << new_factors_tmp.size() << " new factors "
           << ", " << new_values_.size() << " new values "
           << ", and " << delete_slots.size() << " deleted factors.";
  Smoother::Result result;
  VLOG(10) << "Starting first update.";
  updateSmoother(
      &result, new_factors_tmp, new_values_, timestamps, delete_slots);
  VLOG(10) << "Finished first update.";

  // Store time after iSAM update.
  if (VLOG_IS_ON(10) || log_output_) {
    debug_info_.updateTime_ =
        utils::Timer::toc<std::chrono::seconds>(start_time).count();
    start_time = utils::Timer::tic();
  }

  /////////////////////////// BOOKKEEPING
  /////////////////////////////////////////

  // Reset everything for next round.
  // TODO what about the old_smart_factors_?
  VLOG(10) << "Clearing new_smart_factors_!";
  new_smart_factors_.clear();

  // Reset list of new imu, prior and other factors to be added.
  // TODO could this be used to check whether we are repeating factors?
  new_imu_prior_and_other_factors_.resize(0);

  // Clear values.
  new_values_.clear();

  // Update slots of smart factors:.
  VLOG(10) << "Starting to find smart factors slots.";
#ifdef INCREMENTAL_SMOOTHER
  updateNewSmartFactorsSlots(lmk_ids_of_new_smart_factors_tmp,
                             &old_smart_factors_);
#else
  findSmartFactorsSlotsSlow(new_smart_factors_lmkID_tmp);
#endif
  VLOG(10) << "Finished to find smart factors slots.";

  if (VLOG_IS_ON(5) || log_output_) {
    debug_info_.updateSlotTime_ =
        utils::Timer::toc<std::chrono::seconds>(start_time).count();
    start_time = utils::Timer::tic();
  }

  //////////////////////////////////////////////////////////////////////////////

  // Do some more optimization iterations.
  for (size_t n_iter = 1; n_iter < max_extra_iterations; ++n_iter) {
    VLOG(10) << "Doing extra iteration nr: " << n_iter;
    updateSmoother(&result);
  }

  if (VLOG_IS_ON(5) || log_output_) {
    debug_info_.extraIterationsTime_ =
        utils::Timer::toc<std::chrono::seconds>(start_time).count();
    start_time = utils::Timer::tic();
  }

  // Update states we need for next iteration.
  updateStates(cur_id);

  // TODO: Add Update latest covariance --> move flag
  if (FLAGS_compute_state_covariance) {
    computeStateCovariance();
  }

  // Debug.
  postDebug(total_start_time, start_time);
}

/// Private methods.
/* -------------------------------------------------------------------------- */
void VioBackEnd::addInitialPriorFactors(const FrameId& frame_id) {
  // Set initial covariance for inertial factors
  // W_Pose_Blkf_ set by motion capture to start with
  Matrix3 B_Rot_W = W_Pose_B_lkf_.rotation().matrix().transpose();

  // Set initial pose uncertainty: constrain mainly position and global yaw.
  // roll and pitch is observable, therefore low variance.
  Matrix6 pose_prior_covariance = Matrix6::Zero();
  pose_prior_covariance.diagonal()[0] = backend_params_.initialRollPitchSigma_ *
                                        backend_params_.initialRollPitchSigma_;
  pose_prior_covariance.diagonal()[1] = backend_params_.initialRollPitchSigma_ *
                                        backend_params_.initialRollPitchSigma_;
  pose_prior_covariance.diagonal()[2] =
      backend_params_.initialYawSigma_ * backend_params_.initialYawSigma_;
  pose_prior_covariance.diagonal()[3] = backend_params_.initialPositionSigma_ *
                                        backend_params_.initialPositionSigma_;
  pose_prior_covariance.diagonal()[4] = backend_params_.initialPositionSigma_ *
                                        backend_params_.initialPositionSigma_;
  pose_prior_covariance.diagonal()[5] = backend_params_.initialPositionSigma_ *
                                        backend_params_.initialPositionSigma_;

  // Rotate initial uncertainty into local frame, where the uncertainty is
  // specified.
  pose_prior_covariance.topLeftCorner(3, 3) =
      B_Rot_W * pose_prior_covariance.topLeftCorner(3, 3) * B_Rot_W.transpose();

  // Add pose prior.
  // TODO(Toni): Make this noise model a member constant.
  gtsam::SharedNoiseModel noise_init_pose =
      gtsam::noiseModel::Gaussian::Covariance(pose_prior_covariance);
  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          gtsam::Symbol('x', frame_id), W_Pose_B_lkf_, noise_init_pose));

  // Add initial velocity priors.
  // TODO(Toni): Make this noise model a member constant.
  gtsam::SharedNoiseModel noise_init_vel_prior =
      gtsam::noiseModel::Isotropic::Sigma(
          3, backend_params_.initialVelocitySigma_);
  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
          gtsam::Symbol('v', frame_id), W_Vel_B_lkf_, noise_init_vel_prior));

  // Add initial bias priors:
  Vector6 prior_biasSigmas;
  prior_biasSigmas.head<3>().setConstant(backend_params_.initialAccBiasSigma_);
  prior_biasSigmas.tail<3>().setConstant(backend_params_.initialGyroBiasSigma_);
  // TODO(Toni): Make this noise model a member constant.
  gtsam::SharedNoiseModel imu_bias_prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(prior_biasSigmas);
  if (VLOG_IS_ON(10)) {
    LOG(INFO) << "Imu bias for backend prior:";
    imu_bias_lkf_.print();
  }
  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
          gtsam::Symbol('b', frame_id), imu_bias_lkf_, imu_bias_prior_noise));

  VLOG(2) << "Added initial priors for frame " << frame_id;
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::addConstantVelocityFactor(const FrameId& from_id,
                                           const FrameId& to_id) {
  VLOG(10) << "Adding constant velocity factor.";
  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
          gtsam::Symbol('v', from_id),
          gtsam::Symbol('v', to_id),
          gtsam::Vector3::Zero(),
          constant_velocity_prior_noise_));

  // Log number of added constant velocity factors.
  debug_info_.numAddedConstantVelF_++;
}

/* -------------------------------------------------------------------------- */
// Update states.
void VioBackEnd::updateStates(const FrameId& cur_id) {
  VLOG(10) << "Starting to calculate estimate.";
  state_ = smoother_->calculateEstimate();
  VLOG(10) << "Finished to calculate estimate.";

  DCHECK(state_.find(gtsam::Symbol('x', cur_id)) != state_.end());
  DCHECK(state_.find(gtsam::Symbol('v', cur_id)) != state_.end());
  DCHECK(state_.find(gtsam::Symbol('b', cur_id)) != state_.end());

  W_Pose_B_lkf_ = state_.at<Pose3>(gtsam::Symbol('x', cur_id));
  W_Vel_B_lkf_ = state_.at<Vector3>(gtsam::Symbol('v', cur_id));
  imu_bias_lkf_ =
      state_.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', cur_id));

  VLOG(1) << "Backend: Update IMU Bias.";
  CHECK(imu_bias_update_callback_) << "Did you forget to register the IMU bias "
                                      "update callback for at least the "
                                      "frontend? Do so by using "
                                      "registerImuBiasUpdateCallback function";
  imu_bias_update_callback_(imu_bias_lkf_);
}

/* -------------------------------------------------------------------------- */
// Update smoother.
void VioBackEnd::updateSmoother(Smoother::Result* result,
                                const gtsam::NonlinearFactorGraph& new_factors,
                                const gtsam::Values& new_values,
                                const std::map<Key, double>& timestamps,
                                const gtsam::FactorIndices& delete_slots) {
  CHECK_NOTNULL(result);
  // Store smoother as backup.
  CHECK(smoother_);
  // This is not doing a full deep copy: it is keeping same shared_ptrs for
  // factors but copying the isam result.
  Smoother smoother_backup(*smoother_);

  bool got_cheirality_exception = false;
  gtsam::Symbol lmk_symbol_cheirality;
  try {
    // Update smoother.
    VLOG(10) << "Starting update of smoother_...";
    *result =
        smoother_->update(new_factors, new_values, timestamps, delete_slots);
    VLOG(10) << "Finished update of smoother_.";
    if (debug_smoother_) {
      printSmootherInfo(new_factors, delete_slots, "CATCHING EXCEPTION", false);
      debug_smoother_ = false;
    }
  } catch (const gtsam::IndeterminantLinearSystemException& e) {
    LOG(ERROR) << e.what();

    const gtsam::Key& var = e.nearbyVariable();
    gtsam::Symbol symb(var);

    LOG(ERROR) << "ERROR: Variable has type '" << symb.chr() << "' "
               << "and index " << symb.index() << std::endl;

    smoother_->getFactors().print("Smoother's factors:\n[\n\t");
    std::cout << " ]" << std::endl;
    state_.print("State values\n[\n\t");
    std::cout << " ]" << std::endl;

    printSmootherInfo(new_factors, delete_slots);
    throw;
  } catch (const gtsam::InvalidNoiseModel& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors, delete_slots);
  } catch (const gtsam::InvalidMatrixBlock& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors, delete_slots);
  } catch (const gtsam::InvalidDenseElimination& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors, delete_slots);
  } catch (const gtsam::InvalidArgumentThreadsafe& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors, delete_slots);
  } catch (const gtsam::ValuesKeyDoesNotExist& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors, delete_slots);
  } catch (const gtsam::CholeskyFailed& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors, delete_slots);
  } catch (const gtsam::CheiralityException& e) {
    LOG(ERROR) << e.what();
    const gtsam::Key& lmk_key = e.nearbyVariable();
    lmk_symbol_cheirality = gtsam::Symbol(lmk_key);
    LOG(ERROR) << "ERROR: Variable has type '" << lmk_symbol_cheirality.chr()
               << "' "
               << "and index " << lmk_symbol_cheirality.index();
    printSmootherInfo(new_factors, delete_slots);
    got_cheirality_exception = true;
  } catch (const gtsam::StereoCheiralityException& e) {
    LOG(ERROR) << e.what();
    const gtsam::Key& lmk_key = e.nearbyVariable();
    lmk_symbol_cheirality = gtsam::Symbol(lmk_key);
    LOG(ERROR) << "ERROR: Variable has type '" << lmk_symbol_cheirality.chr()
               << "' "
               << "and index " << lmk_symbol_cheirality.index();
    printSmootherInfo(new_factors, delete_slots);
    got_cheirality_exception = true;
  } catch (const gtsam::RuntimeErrorThreadsafe& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors, delete_slots);
  } catch (const gtsam::OutOfRangeThreadsafe& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors, delete_slots);
  } catch (const std::out_of_range& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors, delete_slots);
  } catch (const std::exception& e) {
    // Catch anything thrown within try block that derives from
    // std::exception.
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors, delete_slots);
  } catch (...) {
    // Catch the rest of exceptions.
    LOG(ERROR) << "Unrecognized exception.";
    printSmootherInfo(new_factors, delete_slots);
    // Do not intentionally throw to see what checks fail later.
  }

  if (FLAGS_process_cheirality) {
    static size_t counter_of_exceptions = 0;
    if (got_cheirality_exception) {
      LOG(WARNING) << "Starting processing cheirality exception # "
                   << counter_of_exceptions;
      counter_of_exceptions++;

      // Restore smoother as it was before failure.
      *smoother_ = smoother_backup;

      // Limit the number of cheirality exceptions per run.
      CHECK_LE(counter_of_exceptions,
               FLAGS_max_number_of_cheirality_exceptions);

      // Check that we have a landmark.
      CHECK_EQ(lmk_symbol_cheirality.chr(), 'l');

      // Now that we know the lmk id, delete all factors attached to it!
      gtsam::NonlinearFactorGraph new_factors_tmp_cheirality;
      gtsam::Values new_values_cheirality;
      std::map<Key, double> timestamps_cheirality;
      gtsam::FactorIndices delete_slots_cheirality;
      const gtsam::NonlinearFactorGraph& graph = smoother_->getFactors();
      VLOG(10) << "Starting cleanCheiralityLmk...";
      cleanCheiralityLmk(lmk_symbol_cheirality,
                         &new_factors_tmp_cheirality,
                         &new_values_cheirality,
                         &timestamps_cheirality,
                         &delete_slots_cheirality,
                         graph,
                         new_factors,
                         new_values,
                         timestamps,
                         delete_slots);
      VLOG(10) << "Finished cleanCheiralityLmk.";

      // Recreate the graph before marginalization.
      if (VLOG_IS_ON(5) || FLAGS_debug_graph_before_opt) {
        debug_info_.graphBeforeOpt = graph;
        debug_info_.graphToBeDeleted = gtsam::NonlinearFactorGraph();
        debug_info_.graphToBeDeleted.resize(delete_slots_cheirality.size());
        for (size_t i = 0; i < delete_slots_cheirality.size(); i++) {
          // If the factor is to be deleted, store it as graph to be
          // deleted.
          CHECK(graph.exists(delete_slots_cheirality.at(i)))
              << "Slot # " << delete_slots_cheirality.at(i)
              << "does not exist in smoother graph.";
          // TODO here we can get the right slot that we are going to
          // delete, extend graphToBeDeleted to have both the factor and the
          // slot.
          debug_info_.graphToBeDeleted.at(i) =
              graph.at(delete_slots_cheirality.at(i));
        }
      }

      // Try again to optimize. This is a recursive call.
      LOG(WARNING) << "Starting updateSmoother after handling "
                      "cheirality exception.";
      updateSmoother(result,
                     new_factors_tmp_cheirality,
                     new_values_cheirality,
                     timestamps_cheirality,
                     delete_slots_cheirality);
      LOG(WARNING) << "Finished updateSmoother after handling "
                      "cheirality exception";
    } else {
      counter_of_exceptions = 0;
    }
  }
}

/* --------------------------------------------------------------------------
 */
void VioBackEnd::cleanCheiralityLmk(
    const gtsam::Symbol& lmk_symbol,
    gtsam::NonlinearFactorGraph* new_factors_tmp_cheirality,
    gtsam::Values* new_values_cheirality,
    std::map<Key, double>* timestamps_cheirality,
    gtsam::FactorIndices* delete_slots_cheirality,
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::NonlinearFactorGraph& new_factors_tmp,
    const gtsam::Values& new_values,
    const std::map<Key, double>& timestamps,
    const gtsam::FactorIndices& delete_slots) {
  CHECK_NOTNULL(new_factors_tmp_cheirality);
  CHECK_NOTNULL(new_values_cheirality);
  CHECK_NOTNULL(timestamps_cheirality);
  CHECK_NOTNULL(delete_slots_cheirality);
  const gtsam::Key& lmk_key = lmk_symbol.key();

  // Delete from new factors.
  VLOG(10) << "Starting delete from new factors...";
  deleteAllFactorsWithKeyFromFactorGraph(
      lmk_key, new_factors_tmp, new_factors_tmp_cheirality);
  VLOG(10) << "Finished delete from new factors.";

  // Delete from new values.
  VLOG(10) << "Starting delete from new values...";
  bool is_deleted_from_values =
      deleteKeyFromValues(lmk_key, new_values, new_values_cheirality);
  VLOG(10) << "Finished delete from timestamps.";

  // Delete from new values.
  VLOG(10) << "Starting delete from timestamps...";
  bool is_deleted_from_timestamps =
      deleteKeyFromTimestamps(lmk_key, timestamps, timestamps_cheirality);
  VLOG(10) << "Finished delete from timestamps.";

  // Check that if we deleted from values, we should have deleted as well
  // from timestamps.
  CHECK_EQ(is_deleted_from_values, is_deleted_from_timestamps);

  // Delete slots in current graph.
  VLOG(10) << "Starting delete from current graph...";
  *delete_slots_cheirality = delete_slots;
  std::vector<size_t> slots_of_extra_factors_to_delete;
  // Achtung: This has the chance to make the plane underconstrained, if
  // we delete too many point_plane factors.
  findSlotsOfFactorsWithKey(lmk_key, graph, &slots_of_extra_factors_to_delete);
  delete_slots_cheirality->insert(delete_slots_cheirality->end(),
                                  slots_of_extra_factors_to_delete.begin(),
                                  slots_of_extra_factors_to_delete.end());
  VLOG(10) << "Finished delete from current graph.";

  //////////////////////////// BOOKKEEPING
  ////////////////////////////////////////
  const LandmarkId& lmk_id = lmk_symbol.index();

  // Delete from feature tracks.
  VLOG(10) << "Starting delete from feature tracks...";
  CHECK(deleteLmkFromFeatureTracks(lmk_id));
  VLOG(10) << "Finished delete from feature tracks.";

  // Delete from extra structures (for derived classes).
  VLOG(10) << "Starting delete from extra structures...";
  deleteLmkFromExtraStructures(lmk_id);
  VLOG(10) << "Finished delete from extra structures.";
  //////////////////////////////////////////////////////////////////////////////
}

void VioBackEnd::deleteLmkFromExtraStructures(const LandmarkId& lmk_id) {
  LOG(ERROR) << "There is nothing to delete for lmk with id: " << lmk_id;
  return;
}

/* --------------------------------------------------------------------------
 */
// BOOKKEEPING: updates the SlotIdx in the old_smart_factors such that
// this idx points to the updated slots in the graph after optimization.
// for next iteration to know which slots have to be deleted
// before adding the new smart factors.
void VioBackEnd::updateNewSmartFactorsSlots(
    const std::vector<LandmarkId>& lmk_ids_of_new_smart_factors,
    SmartFactorMap* old_smart_factors) {
  CHECK_NOTNULL(old_smart_factors);

  // Get result.
  const gtsam::ISAM2Result& result = smoother_->getISAM2Result();

  // Simple version of find smart factors.
  for (size_t i = 0; i < lmk_ids_of_new_smart_factors.size(); ++i) {
    DCHECK(i < result.newFactorsIndices.size())
        << "There are more new smart factors than new factors added to the "
           "graph.";
    // Get new slot in the graph for the newly added smart factor.
    const size_t& slot = result.newFactorsIndices.at(i);

    // TODO this will not work if there are non-smart factors!!!
    // Update slot using isam2 indices.
    // ORDER of inclusion of factors in the ISAM2::update() function
    // matters, as these indices have a 1-to-1 correspondence with the
    // factors.

    // BOOKKEEPING, for next iteration to know which slots have to be
    // deleted before adding the new smart factors. Find the entry in
    // old_smart_factors_.
    const auto& it =
        old_smart_factors->find(lmk_ids_of_new_smart_factors.at(i));

    DCHECK(it != old_smart_factors->end())
        << "Trying to access unavailable factor.";
    // CHECK that the factor in the graph at slot position is a smart
    // factor.
    DCHECK(boost::dynamic_pointer_cast<SmartStereoFactor>(
        smoother_->getFactors().at(slot)));
    // CHECK that shared ptrs point to the same smart factor.
    // make sure no one is cloning SmartSteroFactors.
    DCHECK_EQ(it->second.first,
              boost::dynamic_pointer_cast<SmartStereoFactor>(
                  smoother_->getFactors().at(slot)))
        << "Non-matching addresses for same factors for lmk with id: "
        << lmk_ids_of_new_smart_factors.at(i) << " in old_smart_factors_ "
        << "VS factor in graph at slot: " << slot
        << ". Slot previous to update was: " << it->second.second;

    // Update slot number in old_smart_factors_.
    it->second.second = slot;
  }
}

/* --------------------------------------------------------------------------
 */
// Set parameters for ISAM 2 incremental smoother.
void VioBackEnd::setIsam2Params(const VioBackEndParams& vio_params,
                                gtsam::ISAM2Params* isam_param) {
  CHECK_NOTNULL(isam_param);
  // iSAM2 SETTINGS
  gtsam::ISAM2GaussNewtonParams gauss_newton_params;
  // TODO remove this hardcoded value...
  gauss_newton_params.wildfireThreshold = -1.0;
  // gauss_newton_params.setWildfireThreshold(0.001);

  gtsam::ISAM2DoglegParams dogleg_params;
  // dogleg_params.setVerbose(false); // only for debugging.

  if (vio_params.useDogLeg_) {
    isam_param->optimizationParams = dogleg_params;
  } else {
    isam_param->optimizationParams = gauss_newton_params;
  }

  // TODO Luca: Here there was commented code about setRelinearizeThreshold.
  // was it important?
  // gtsam::FastMap<char,gtsam::Vector> thresholds;
  // gtsam::Vector xThresh(6); // = {0.05, 0.05, 0.05, 0.1, 0.1, 0.1};
  // gtsam::Vector vThresh(3); //= {1.0, 1.0, 1.0};
  // gtsam::Vector bThresh(6); // = {1.0, 1.0, 1.0};
  // xThresh << relinearizeThresholdRot_, relinearizeThresholdRot_,
  // relinearizeThresholdRot_, relinearizeThresholdPos_,
  // relinearizeThresholdPos_, relinearizeThresholdPos_; vThresh <<
  // relinearizeThresholdVel_, relinearizeThresholdVel_,
  // relinearizeThresholdVel_; bThresh << relinearizeThresholdIMU_,
  // relinearizeThresholdIMU_, relinearizeThresholdIMU_,
  // relinearizeThresholdIMU_, relinearizeThresholdIMU_,
  // relinearizeThresholdIMU_; thresholds['x'] = xThresh; thresholds['v'] =
  // vThresh; thresholds['b'] = bThresh;
  // isam_param.setRelinearizeThreshold(thresholds);

  // TODO (Toni): remove hardcoded
  // Cache Linearized Factors seems to improve performance.
  isam_param->setCacheLinearizedFactors(true);
  isam_param->relinearizeThreshold = vio_params.relinearizeThreshold_;
  isam_param->relinearizeSkip = vio_params.relinearizeSkip_;
  isam_param->findUnusedFactorSlots = true;
  // isam_param->enablePartialRelinearizationCheck = true;
  isam_param->setEvaluateNonlinearError(false);  // only for debugging
  isam_param->enableDetailedResults = false;     // only for debugging.
  isam_param->factorization = gtsam::ISAM2Params::CHOLESKY;  // QR
  if (VLOG_IS_ON(1)) isam_param->print("isam_param");
}

/* --------------------------------------------------------------------------
 */
// Set parameters for all the factors.
void VioBackEnd::setFactorsParams(
    const VioBackEndParams& vio_params,
    gtsam::SharedNoiseModel* smart_noise,
    gtsam::SmartStereoProjectionParams* smart_factors_params,
    gtsam::SharedNoiseModel* no_motion_prior_noise,
    gtsam::SharedNoiseModel* zero_velocity_prior_noise,
    gtsam::SharedNoiseModel* constant_velocity_prior_noise) {
  CHECK_NOTNULL(smart_noise);
  CHECK_NOTNULL(smart_factors_params);
  CHECK_NOTNULL(no_motion_prior_noise);
  CHECK_NOTNULL(zero_velocity_prior_noise);
  CHECK_NOTNULL(constant_velocity_prior_noise);

  //////////////////////// SMART PROJECTION FACTORS SETTINGS
  //////////////////////
  setSmartFactorsParams(smart_noise,
                        smart_factors_params,
                        vio_params.smartNoiseSigma_,
                        vio_params.rankTolerance_,
                        vio_params.landmarkDistanceThreshold_,
                        vio_params.retriangulationThreshold_,
                        vio_params.outlierRejection_);

  //////////////////////// NO MOTION FACTORS SETTINGS
  /////////////////////////////
  Vector6 sigmas;
  sigmas.head<3>().setConstant(vio_params.noMotionRotationSigma_);
  sigmas.tail<3>().setConstant(vio_params.noMotionPositionSigma_);
  *no_motion_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  //////////////////////// ZERO VELOCITY FACTORS SETTINGS
  /////////////////////////
  *zero_velocity_prior_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, vio_params.zeroVelocitySigma_);

  //////////////////////// CONSTANT VELOCITY FACTORS SETTINGS
  /////////////////////
  *constant_velocity_prior_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, vio_params.constantVelSigma_);
}

/* --------------------------------------------------------------------------
 */
// Set parameters for smart factors.
void VioBackEnd::setSmartFactorsParams(
    gtsam::SharedNoiseModel* smart_noise,
    gtsam::SmartStereoProjectionParams* smart_factors_params,
    const double& smart_noise_sigma,
    const double& rank_tolerance,
    const double& landmark_distance_threshold,
    const double& retriangulation_threshold,
    const double& outlier_rejection) {
  CHECK_NOTNULL(smart_noise);
  CHECK_NOTNULL(smart_factors_params);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(
      3,
      smart_noise_sigma);  // vio_smart_reprojection_err_thresh
                           // / cam_->fx());
  // smart_noise_ = gtsam::noiseModel::Robust::Create(
  //                  gtsam::noiseModel::mEstimator::Huber::Create(1.345),
  //                  model);
  *smart_noise = model;
  *smart_factors_params =
      SmartFactorParams(gtsam::HESSIAN,             // JACOBIAN_SVD
                        gtsam::ZERO_ON_DEGENERACY,  // IGNORE_DEGENERACY
                        false,                      // ThrowCherality = false
                        true);                      // verboseCherality = true
  smart_factors_params->setRankTolerance(rank_tolerance);
  smart_factors_params->setLandmarkDistanceThreshold(
      landmark_distance_threshold);
  smart_factors_params->setRetriangulationThreshold(retriangulation_threshold);
  smart_factors_params->setDynamicOutlierRejectionThreshold(outlier_rejection);
}

/// Printers.
/* --------------------------------------------------------------------------
 */
// THIS IS NOT THREAD-SAFE !!
void VioBackEnd::print() const {
  LOG(INFO) << "((((((((((((((((((((((((((((((((((((((((( VIO PRINT )))))))))"
            << ")))))))))))))))))))))))))))))))) ";
  if (FLAGS_minloglevel < 1) {
    stereo_cal_->print("\n stereoCal_\n");
  }
  backend_params_.print();

  LOG(INFO) << "\n B_Pose_leftCam_: " << B_Pose_leftCam_ << '\n'
            << "W_Pose_B_lkf_: " << W_Pose_B_lkf_ << '\n'
            << "W_Vel_B_lkf_ (transpose): " << W_Vel_B_lkf_.transpose() << '\n'
            << "imu_bias_lkf_" << imu_bias_lkf_ << '\n'
            << "imu_bias_prev_kf_" << imu_bias_prev_kf_ << '\n'
            << "last_id_ " << last_kf_id_ << '\n'
            << "cur_id_ " << curr_kf_id_ << '\n'
            << "landmark_count_ " << landmark_count_ << '\n'
            << "(((((((((((((((((((((((((((((((((((((((((((((((()))))))))))))"
            << "))))))))))))))))))))))))))))))))) ";
}

/* --------------------------------------------------------------------------
 */
void VioBackEnd::printFeatureTracks() const {
  std::cout << "---- Feature tracks: --------- " << std::endl;
  BOOST_FOREACH (auto keyTrack_j, feature_tracks_) {
    std::cout << "Landmark " << keyTrack_j.first << " having ";
    keyTrack_j.second.print();
  }
}

/* --------------------------------------------------------------------------
 */
void VioBackEnd::printSmootherInfo(
    const gtsam::NonlinearFactorGraph& new_factors_tmp,
    const gtsam::FactorIndices& delete_slots,
    const std::string& message,
    const bool& showDetails) const {
  LOG(INFO) << " =============== START:" << message
            << " =============== " << std::endl;

  const std::string* which_graph = nullptr;
  const gtsam::NonlinearFactorGraph* graph = nullptr;
  // Pick the graph that makes more sense:
  // This is code is mostly run post update, when it throws exception,
  // shouldn't we print the graph before optimization instead?
  // Yes if available, but if not, then just ask the smoother.
  static const std::string graph_before_opt = "(graph before optimization)";
  static const std::string smoother_get_factors = "(smoother getFactors)";
  if (debug_info_.graphBeforeOpt.size() != 0) {
    which_graph = &graph_before_opt;
    graph = &(debug_info_.graphBeforeOpt);
  } else {
    which_graph = &smoother_get_factors;
    graph = &(smoother_->getFactors());
  }
  CHECK_NOTNULL(which_graph);
  CHECK_NOTNULL(graph);

  static constexpr bool print_smart_factors = false;
  static constexpr bool print_point_plane_factors = true;
  static constexpr bool print_plane_priors = true;
  static constexpr bool print_point_priors = true;
  static constexpr bool print_linear_container_factors = true;
  ////////////////////// Print all factors.
  ///////////////////////////////////////
  LOG(INFO) << "Nr of factors in graph " + *which_graph << ": " << graph->size()
            << ", with factors:" << std::endl;
  LOG(INFO) << "[\n";
  printSelectedGraph(*graph,
                     print_smart_factors,
                     print_point_plane_factors,
                     print_plane_priors,
                     print_point_priors,
                     print_linear_container_factors);
  LOG(INFO) << " ]" << std::endl;

  ///////////// Print factors that were newly added to the
  /// optimization.////////
  LOG(INFO) << "Nr of new factors to add: " << new_factors_tmp.size()
            << " with factors:" << std::endl;
  LOG(INFO) << "[\n (slot # wrt to new_factors_tmp graph) \t";
  printSelectedGraph(new_factors_tmp,
                     print_smart_factors,
                     print_point_plane_factors,
                     print_plane_priors,
                     print_point_priors,
                     print_linear_container_factors);
  LOG(INFO) << " ]" << std::endl;

  ////////////////////////////// Print deleted
  /// slots.///////////////////////////
  LOG(INFO) << "Nr deleted slots: " << delete_slots.size()
            << ", with slots:" << std::endl;
  LOG(INFO) << "[\n\t";
  if (debug_info_.graphToBeDeleted.size() != 0) {
    // If we are storing the graph to be deleted, then print extended info
    // besides the slot to be deleted.
    CHECK_EQ(debug_info_.graphToBeDeleted.size(), delete_slots.size());
    for (size_t i = 0; i < delete_slots.size(); ++i) {
      CHECK(debug_info_.graphToBeDeleted.at(i));
      if (print_point_plane_factors) {
        printSelectedFactors(debug_info_.graphToBeDeleted.at(i),
                             delete_slots.at(i),
                             false,
                             print_point_plane_factors,
                             false,
                             false,
                             false);
      } else {
        std::cout << "\tSlot # " << delete_slots.at(i) << ":";
        std::cout << "\t";
        debug_info_.graphToBeDeleted.at(i)->printKeys();
      }
    }
  } else {
    for (size_t i = 0; i < delete_slots.size(); ++i) {
      std::cout << delete_slots.at(i) << " ";
    }
  }
  std::cout << std::endl;
  LOG(INFO) << " ]" << std::endl;

  //////////////////////// Print all values in state.
  /////////////////////////////
  LOG(INFO) << "Nr of values in state_ : " << state_.size() << ", with keys:";
  std::cout << "[\n\t";
  BOOST_FOREACH (const gtsam::Values::ConstKeyValuePair& key_value, state_) {
    std::cout << gtsam::DefaultKeyFormatter(key_value.key) << " ";
  }
  std::cout << std::endl;
  LOG(INFO) << " ]";

  // Print only new values.
  LOG(INFO) << "Nr values in new_values_ : " << new_values_.size()
            << ", with keys:";
  std::cout << "[\n\t";
  BOOST_FOREACH (const gtsam::Values::ConstKeyValuePair& key_value,
                 new_values_) {
    std::cout << " " << gtsam::DefaultKeyFormatter(key_value.key) << " ";
  }
  std::cout << std::endl;
  LOG(INFO) << " ]";

  if (showDetails) {
    graph->print("isam2 graph:\n");
    new_factors_tmp.print("new_factors_tmp:\n");
    new_values_.print("new values:\n");
    // LOG(INFO) << "new_smart_factors_: "  << std::endl;
    // for (auto& s : new_smart_factors_)
    //	s.second->print();
  }

  LOG(INFO) << " =============== END: " << message
            << " =============== " << std::endl;
}

/* --------------------------------------------------------------------------
 */
void VioBackEnd::printSmartFactor(
    boost::shared_ptr<SmartStereoFactor> gsf) const {
  CHECK(gsf);
  std::cout << "Smart Factor (valid: " << (gsf->isValid() ? "yes" : "NO!")
            << ", deg: " << (gsf->isDegenerate() ? "YES!" : "no")
            << " isCheir: " << (gsf->isPointBehindCamera() ? "YES!" : "no")
            << "): \t";
  gsf->printKeys();
}

/* --------------------------------------------------------------------------
 */
void VioBackEnd::printPointPlaneFactor(
    boost::shared_ptr<gtsam::PointPlaneFactor> ppf) const {
  CHECK(ppf);
  std::cout << "Point Plane Factor: plane key "
            << gtsam::DefaultKeyFormatter(ppf->getPlaneKey()) << ", point key "
            << gtsam::DefaultKeyFormatter(ppf->getPointKey()) << "\n";
}

/* --------------------------------------------------------------------------
 */
void VioBackEnd::printPlanePrior(
    boost::shared_ptr<gtsam::PriorFactor<gtsam::OrientedPlane3>> ppp) const {
  CHECK(ppp);
  std::cout << "Plane Prior: plane key \t";
  ppp->printKeys();
}

/* --------------------------------------------------------------------------
 */
void VioBackEnd::printPointPrior(
    boost::shared_ptr<gtsam::PriorFactor<gtsam::Point3>> ppp) const {
  CHECK(ppp);
  std::cout << "Point Prior: point key \t";
  ppp->printKeys();
}

/* --------------------------------------------------------------------------
 */
void VioBackEnd::printLinearContainerFactor(
    boost::shared_ptr<gtsam::LinearContainerFactor> lcf) const {
  CHECK(lcf);
  std::cout << "Linear Container Factor: \t";
  lcf->printKeys();
}

/* --------------------------------------------------------------------------
 */
void VioBackEnd::printSelectedFactors(
    const boost::shared_ptr<gtsam::NonlinearFactor>& g,
    const size_t& slot,
    const bool print_smart_factors,
    const bool print_point_plane_factors,
    const bool print_plane_priors,
    const bool print_point_priors,
    const bool print_linear_container_factors) const {
  if (print_smart_factors) {
    const auto& gsf = boost::dynamic_pointer_cast<SmartStereoFactor>(g);
    if (gsf) {
      std::cout << "\tSlot # " << slot << ": ";
      printSmartFactor(gsf);
    }
  }

  if (print_point_plane_factors) {
    const auto& ppf = boost::dynamic_pointer_cast<gtsam::PointPlaneFactor>(g);
    if (ppf) {
      std::cout << "\tSlot # " << slot << ": ";
      printPointPlaneFactor(ppf);
    }
  }

  if (print_plane_priors) {
    const auto& ppp =
        boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::OrientedPlane3>>(
            g);
    if (ppp) {
      std::cout << "\tSlot # " << slot << ": ";
      printPlanePrior(ppp);
    }
  }

  if (print_point_priors) {
    const auto& ppp =
        boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Point3>>(g);
    if (ppp) {
      std::cout << "\tSlot # " << slot << ": ";
      printPointPrior(ppp);
    }
  }

  if (print_linear_container_factors) {
    const auto& lcf =
        boost::dynamic_pointer_cast<gtsam::LinearContainerFactor>(g);
    if (lcf) {
      std::cout << "\tSlot # " << slot << ": ";
      printLinearContainerFactor(lcf);
    }
  }
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::printSelectedGraph(
    const gtsam::NonlinearFactorGraph& graph,
    const bool& print_smart_factors,
    const bool& print_point_plane_factors,
    const bool& print_plane_priors,
    const bool& print_point_priors,
    const bool& print_linear_container_factors) const {
  size_t slot = 0;
  for (const auto& g : graph) {
    printSelectedFactors(g,
                         slot,
                         print_smart_factors,
                         print_point_plane_factors,
                         print_plane_priors,
                         print_point_priors,
                         print_linear_container_factors);
    slot++;
  }
  std::cout << std::endl;
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::computeSmartFactorStatistics() {
  // Compute number of valid/degenerate
  debug_info_.resetSmartFactorsStatistics();
  gtsam::NonlinearFactorGraph graph = smoother_->getFactors();
  for (const auto& g : graph) {
    if (g) {
      const auto& gsf = boost::dynamic_pointer_cast<SmartStereoFactor>(g);
      if (gsf) {
        debug_info_.numSF_ += 1;

        // Check for consecutive Keys: this check is wrong: if there is
        // LOW_DISPARITY at some frame, we do not add the measurement to the
        // smart factor, hence keys are not necessarily consecutive
        // auto keys = g->keys();
        // Key last_key;
        // bool first_key = true;
        // for (Key key : keys)
        //{
        //  if (!first_key && key - last_key != 1){
        //    std::cout << " Last: " << gtsam::DefaultKeyFormatter(last_key)
        //    << " Current: " << gtsam::DefaultKeyFormatter(key) <<
        //    std::endl; for (Key k : keys){ std::cout << " " <<
        //    gtsam::DefaultKeyFormatter(k)
        //    << " "; } throw std::runtime_error("\n
        //    computeSmartFactorStatistics: found nonconsecutive keys in
        //    smart factors \n");
        //  }
        //  last_key = key;
        //  first_key = false;
        //}

        // Check SF status
        const gtsam::TriangulationResult& result = gsf->point();
        if (result.is_initialized()) {
          if (result.degenerate()) debug_info_.numDegenerate_ += 1;
          if (result.farPoint()) debug_info_.numFarPoints_ += 1;
          if (result.outlier()) debug_info_.numOutliers_ += 1;
          if (result.behindCamera()) debug_info_.numCheirality_ += 1;
          if (result.valid()) {
            debug_info_.numValid_ += 1;
            // Check track length
            size_t trackLength = gsf->keys().size();
            if (trackLength > debug_info_.maxTrackLength_) {
              debug_info_.maxTrackLength_ = trackLength;
            }
            debug_info_.meanTrackLength_ += trackLength;
          }
        } else {
          VLOG(1) << "Triangulation result is not initialized...";
          debug_info_.numNonInitialized_ += 1;
        }
      }
    }
  }
  if (debug_info_.numValid_ > 0) {
    debug_info_.meanTrackLength_ = debug_info_.meanTrackLength_ /
                                   static_cast<double>(debug_info_.numValid_);
  } else {
    debug_info_.meanTrackLength_ = 0;
  }
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::computeSparsityStatistics() {
  gtsam::NonlinearFactorGraph graph = smoother_->getFactors();
  gtsam::GaussianFactorGraph::shared_ptr gfg = graph.linearize(state_);
  gtsam::Matrix Hessian = gfg->hessian().first;
  debug_info_.nrElementsInMatrix_ = Hessian.rows() * Hessian.cols();
  debug_info_.nrZeroElementsInMatrix_ = 0;
  for (int i = 0; i < Hessian.rows(); ++i) {
    for (int j = 0; j < Hessian.cols(); ++j) {
      if (std::fabs(Hessian(i, j)) < 1e-15) {
        debug_info_.nrZeroElementsInMatrix_ += 1;
      }
    }
  }

  CHECK_EQ(Hessian.rows(), Hessian.cols())
      << "computeSparsityStatistics: hessian is not a square matrix?";

  VLOG(10) << "Hessian stats: ===========\n"
           << "rows: " << Hessian.rows() << '\n'
           << "nrElementsInMatrix_: " << debug_info_.nrElementsInMatrix_ << '\n'
           << "nrZeroElementsInMatrix_: "
           << debug_info_.nrZeroElementsInMatrix_;
}

/* -------------------------------------------------------------------------- */
// Debugging post optimization and estimate calculation.
void VioBackEnd::postDebug(
    const std::chrono::high_resolution_clock::time_point& total_start_time,
    const std::chrono::high_resolution_clock::time_point& start_time) {
  if (log_output_) {
    computeSparsityStatistics();
    computeSmartFactorStatistics();
  }

  if (VLOG_IS_ON(10)) {
    // Print old_smart_factors_
    LOG(INFO) << "Landmarks in old_smart_factors_:";
    for (const auto& it : old_smart_factors_) {
      LOG(INFO) << " - Landmark " << it.first << " with slot "
                << it.second.second;
    }

    // Print debug_info_
    debug_info_.print();

    // Print times.
    debug_info_.printTimes();

    // Sanity check timings
    const auto& end_time =
        utils::Timer::toc<std::chrono::seconds>(total_start_time).count();
    const auto& end_time_from_sum = debug_info_.sumAllTimes();
    CHECK_EQ(end_time, end_time_from_sum)
        << "Optimize: time measurement mismatch."
           "The sum of the parts is not equal to the total.";

    // Print error.
    gtsam::NonlinearFactorGraph graph = gtsam::NonlinearFactorGraph(
        smoother_->getFactors());  // clone, expensive but safer!
    VLOG(10) << "Optimization Errors:\n"
             << " - Error before :" << graph.error(debug_info_.stateBeforeOpt)
             << '\n'
             << " - Error after  :" << graph.error(state_);
  }
}

/* -------------------------------------------------------------------------- */
// Reset state of debug info.
void VioBackEnd::resetDebugInfo(DebugVioInfo* debug_info) {
  CHECK_NOTNULL(debug_info);
  debug_info->resetSmartFactorsStatistics();
  debug_info->resetTimes();
  debug_info->resetAddedFactorsStatistics();
  debug_info->nrElementsInMatrix_ = 0;
  debug_info->nrZeroElementsInMatrix_ = 0;
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::cleanNullPtrsFromGraph(
    gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors) {
  CHECK_NOTNULL(new_imu_prior_and_other_factors);
  gtsam::NonlinearFactorGraph tmp_graph = *new_imu_prior_and_other_factors;
  new_imu_prior_and_other_factors->resize(0);
  for (const auto& factor : tmp_graph) {
    if (factor != nullptr) {
      new_imu_prior_and_other_factors->push_back(factor);
    }
  }
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::deleteAllFactorsWithKeyFromFactorGraph(
    const gtsam::Key& key,
    const gtsam::NonlinearFactorGraph& factor_graph,
    gtsam::NonlinearFactorGraph* factor_graph_output) {
  CHECK_NOTNULL(factor_graph_output);
  size_t new_factors_slot = 0;
  *factor_graph_output = factor_graph;
  for (auto it = factor_graph_output->begin();
       it != factor_graph_output->end();) {
    if (*it) {
      if ((*it)->find(key) != (*it)->end()) {
        // We found our lmk in the list of keys of the factor.
        // Sanity check, this lmk has no priors right?
        CHECK(!boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Point3>>(
            *it));
        // We are not deleting a smart factor right?
        // Otherwise we need to update structure:
        // lmk_ids_of_new_smart_factors...
        CHECK(!boost::dynamic_pointer_cast<SmartStereoFactor>(*it));
        // Whatever factor this is, it has our lmk...
        // Delete it.
        LOG(WARNING) << "Delete factor in new_factors at slot # "
                     << new_factors_slot << " of new_factors graph.";
        it = factor_graph_output->erase(it);
      } else {
        it++;
      }
    } else {
      LOG(ERROR) << "*it, which is itself a pointer, is null.";
      it++;
    }
    new_factors_slot++;
  }
}

/* -------------------------------------------------------------------------- */
// Returns if the key in timestamps could be removed or not.
bool VioBackEnd::deleteKeyFromTimestamps(
    const gtsam::Key& key,
    const std::map<Key, double>& timestamps,
    std::map<Key, double>* timestamps_output) {
  CHECK_NOTNULL(timestamps_output);
  *timestamps_output = timestamps;
  if (timestamps_output->find(key) != timestamps_output->end()) {
    timestamps_output->erase(key);
    return true;
  }
  return false;
}

/* -------------------------------------------------------------------------- */
// Returns if the key in timestamps could be removed or not.
bool VioBackEnd::deleteKeyFromValues(const gtsam::Key& key,
                                     const gtsam::Values& values,
                                     gtsam::Values* values_output) {
  CHECK_NOTNULL(values_output);
  *values_output = values;
  if (values.find(key) != values.end()) {
    // We found the lmk in new values, delete it.
    LOG(WARNING) << "Delete value in new_values for key "
                 << gtsam::DefaultKeyFormatter(key);
    CHECK(values_output->find(key) != values_output->end());
    try {
      values_output->erase(key);
    } catch (const gtsam::ValuesKeyDoesNotExist& e) {
      LOG(FATAL) << e.what();
    } catch (...) {
      LOG(FATAL) << "Unhandled exception when erasing key"
                    " in new_values_cheirality";
    }
    return true;
  }
  return false;
}

/* -------------------------------------------------------------------------- */
// Returns if the key in timestamps could be removed or not.
void VioBackEnd::findSlotsOfFactorsWithKey(
    const gtsam::Key& key,
    const gtsam::NonlinearFactorGraph& graph,
    std::vector<size_t>* slots_of_factors_with_key) {
  CHECK_NOTNULL(slots_of_factors_with_key);
  slots_of_factors_with_key->resize(0);
  size_t slot = 0;
  for (const boost::shared_ptr<gtsam::NonlinearFactor>& g : graph) {
    if (g) {
      // Found a valid factor.
      if (g->find(key) != g->end()) {
        // Whatever factor this is, it has our lmk...
        // Sanity check, this lmk has no priors right?
        CHECK(!boost::dynamic_pointer_cast<gtsam::LinearContainerFactor>(g));
        CHECK(
            !boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Point3>>(g));
        // Sanity check that we are not deleting a smart factor.
        CHECK(!boost::dynamic_pointer_cast<SmartStereoFactor>(g));
        // Delete it.
        LOG(WARNING) << "Delete factor in graph at slot # " << slot
                     << " corresponding to lmk with id: "
                     << gtsam::Symbol(key).index();
        CHECK(graph.exists(slot));
        slots_of_factors_with_key->push_back(slot);
      }
    }
    slot++;
  }
}

/* -------------------------------------------------------------------------- */
// Returns if the key in feature tracks could be removed or not.
bool VioBackEnd::deleteLmkFromFeatureTracks(const LandmarkId& lmk_id) {
  if (feature_tracks_.find(lmk_id) != feature_tracks_.end()) {
    LOG(WARNING) << "Deleting feature track for lmk with id: " << lmk_id;
    feature_tracks_.erase(lmk_id);
    return true;
  }
  return false;
}

}  // namespace VIO.
