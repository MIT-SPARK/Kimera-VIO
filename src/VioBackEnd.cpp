/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackEnd.cpp
 * @brief  Visual-Inertial Odometry pipeline, as described in this papers:
 *
 * C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza.
 * On-Manifold Preintegration Theory for Fast and Accurate Visual-Inertial Navigation.
 * IEEE Trans. Robotics, 33(1):1-21, 2016.
 *
 * C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza.
 * On-Manifold Preintegration Theory for Fast and Accurate Visual-Inertial Navigation.
 * IEEE Trans. Robotics, 33(1):1-21, 2016.
 *
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, and F. Dellaert.
 * Eliminating Conditionally Independent Sets in Factor Graphs: A Unifying Perspective based on Smart Factors.
 * In IEEE Intl. Conf. on Robotics and Automation (ICRA), 2014.
 *
 * @author Luca Carlone, Antoni Rosinol
 */

#include "VioBackEnd.h"

#include <glog/logging.h>
#include <gflags/gflags.h>

#include "ETH_parser.h" // Only for gtNavState ...

DEFINE_bool(debug_graph_before_opt, false,
            "Store factor graph before optimization for later printing if the "
            "optimization fails.");
DEFINE_bool(process_cheirality, false,
            "Handle cheirality exception by removing problematic landmarks and "
            "re-running optimization.");
DEFINE_int32(max_number_of_cheirality_exceptions, 5,
             "Sets the maximum number of times we process a cheirality "
             "exception for a given optimization problem. This is to avoid too "
             "many recursive calls to update the smoother");

using namespace std;

namespace VIO {

/// VioBackEnd Methods.
/// Public methods.
/* -------------------------------------------------------------------------- */
// initial_state_gt is a non-null pointer to a shared_ptr which might be null
// if there is no ground-truth available, otherwise it contains the initial
// ground-truth state.
VioBackEnd::VioBackEnd(const Pose3& leftCamPose,
                       const Cal3_S2& leftCameraCalRectified,
                       const double& baseline,
                       std::shared_ptr<gtNavState>* initial_state_gt,
                       const Timestamp& timestamp_k,
                       const ImuAccGyr& imu_accgyr,
                       const VioBackEndParams& vioParams,
                       const bool log_timing):
  B_Pose_leftCam_(leftCamPose),
  stereo_cal_(boost::make_shared<gtsam::Cal3_S2Stereo>(
               leftCameraCalRectified.fx(),
               leftCameraCalRectified.fy(), leftCameraCalRectified.skew(),
               leftCameraCalRectified.px(), leftCameraCalRectified.py(),
               baseline)),
  vio_params_(vioParams),
  imu_bias_lkf_(ImuBias()),
  imu_bias_prev_kf_(ImuBias()),
  W_Vel_Blkf_(Vector3::Zero()),
  W_Pose_Blkf_(Pose3()),
  last_kf_id_(-1),
  cur_kf_id_(0),
  verbosity_(0),
  log_timing_(log_timing),
  landmark_count_(0) {
    CHECK_NOTNULL(initial_state_gt);

  // TODO the parsing of the params should be done inside here out from the
  // path to the params file, otherwise other derived VIO backends will be stuck
  // with the parameters used by vanilla VIO, as there is no polymorphic
  // container in C++...
  // This way VioBackEnd can parse the params it cares about, while others can
  // have the opportunity to parse their own parameters as well.
  // Unfortunately, doing that would not work because many other modules use
  // VioBackEndParams as weird as this may sound...
  // For now we have polymorphic params, with dynamic_cast to derived class, aka
  // suboptimal...

  //////////////////////////////////////////////////////////////////////////////
  // Initialize smoother.
#ifdef INCREMENTAL_SMOOTHER
  gtsam::ISAM2Params isam_param;
  setIsam2Params(vioParams, &isam_param);

  smoother_ = std::make_shared<Smoother>(vioParams.horizon_, isam_param);
#else // BATCH SMOOTHER
  gtsam::LevenbergMarquardtParams lmParams;
  lmParams.setlambdaInitial(0.0); // same as GN
  lmParams.setlambdaLowerBound(0.0); // same as GN
  lmParams.setlambdaUpperBound(0.0); // same as GN)
  smoother_ = std::make_shared<Smoother>(vioParams.horizon_,lmParams);
#endif

  // Set parameters for all factors.
  setFactorsParams(vioParams,
                   &smart_noise_,
                   &smart_factors_params_,
                   &imu_params_,
                   &no_motion_prior_noise_,
                   &zero_velocity_prior_noise_,
                   &constant_velocity_prior_noise_);


  // Reset debug info.
  resetDebugInfo(&debug_info_);

  // TODO change VIO initialization logic, because right now if it is not
  // auto-initialized it still asks for ImuAccGyr data.
  // Initialize VIO.
  if (vio_params_.autoInitialize_ || !*initial_state_gt) {
    // Use initial IMU measurements to guess first pose
    LOG_IF(WARNING, !vio_params_.autoInitialize_)
        << "Could not initialize from ground truth, since it is not "
           "available. Autoinitializing instead.";
    *initial_state_gt = std::make_shared<gtNavState>();
    (*initial_state_gt)->pose_ =
            guessPoseFromIMUmeasurements(imu_accgyr,
                                         vio_params_.n_gravity_,
                                         vio_params_.roundOnAutoInitialize_);
    initStateAndSetPriors(timestamp_k,
                          (*initial_state_gt)->pose_,
                          imu_accgyr);

    // Only for display later on.
    (*initial_state_gt)->velocity_ = W_Vel_Blkf_;
    (*initial_state_gt)->imu_bias_ = imu_bias_lkf_;
  } else {
    // Use ground-truth as first pose.
    initStateAndSetPriors(timestamp_k,
                          (*initial_state_gt)->pose_,
                          (*initial_state_gt)->velocity_,
                          (*initial_state_gt)->imu_bias_);
  }
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::initStateAndSetPriors(const Timestamp& timestamp_kf_nsec,
                                       const Pose3& initialPose,
                                       const ImuAccGyr& accGyroRaw) {
  Vector3 localGravity = initialPose.rotation().inverse().matrix() *
                         imu_params_->n_gravity;
  initStateAndSetPriors(timestamp_kf_nsec,
                        initialPose,
                        Vector3::Zero(),
                        initImuBias(accGyroRaw, localGravity));
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::initStateAndSetPriors(const Timestamp& timestamp_kf_nsec,
                                       const Pose3& initialPose,
                                       const Vector3& initialVel,
                                       const ImuBias& initialBias) {
  timestamp_kf_ = UtilsOpenCV::NsecToSec(timestamp_kf_nsec);

  W_Pose_Blkf_ = initialPose;
  W_Vel_Blkf_ = initialVel;
  imu_bias_lkf_ = initialBias;
  imu_bias_prev_kf_ = initialBias;

  LOG(INFO) << "Initialized state: ";
  W_Pose_Blkf_.print("Initial pose");
  LOG(INFO) << "\n Initial vel: " << W_Vel_Blkf_.transpose();
  imu_bias_lkf_.print("Initial bias: \n");

  // Can't add inertial prior factor until we have a state measurement.
  addInitialPriorFactors(cur_kf_id_, imu_bias_lkf_.vector());

  // TODO encapsulate this in a function, code duplicated in addImuValues.
  new_values_.insert(gtsam::Symbol('x', cur_kf_id_), W_Pose_Blkf_);
  new_values_.insert(gtsam::Symbol('v', cur_kf_id_), W_Vel_Blkf_);
  new_values_.insert(gtsam::Symbol('b', cur_kf_id_), imu_bias_lkf_);

  optimize(cur_kf_id_, vio_params_.numOptimize_);
}

/* -------------------------------------------------------------------------- */
// Workhorse that stores data and optimizes at each keyframe.
// [in] timestamp_kf_nsec, keyframe timestamp.
// [in] status_smart_stereo_measurements_kf, vision data.
// [in] imu_stamps, [in] imu_accgyr.
// [in] stereo_ransac_body_pose, inertial data.
void VioBackEnd::addVisualInertialStateAndOptimize(
    const Timestamp& timestamp_kf_nsec,
    const StatusSmartStereoMeasurements& status_smart_stereo_measurements_kf,
    const ImuStamps& imu_stamps, const ImuAccGyr& imu_accgyr,
    std::vector<Plane>* planes,
    boost::optional<gtsam::Pose3> stereo_ransac_body_pose) {
  debug_info_.resetAddedFactorsStatistics();

  if (verbosity_ >= 7) { StereoVisionFrontEnd::PrintStatusStereoMeasurements(status_smart_stereo_measurements_kf); }

  // Features and IMU line up --> do iSAM update
  last_kf_id_ = cur_kf_id_;
  ++cur_kf_id_;

  timestamp_kf_ = UtilsOpenCV::NsecToSec(timestamp_kf_nsec);

  std::cout << "VIO: adding keyframe " << cur_kf_id_ << " at timestamp:" << timestamp_kf_ << " (sec)" << std::endl;

  /////////////////// MANAGE IMU MEASUREMENTS ///////////////////////////
  // Predict next step, add initial guess
  integrateImuMeasurements(imu_stamps, imu_accgyr);
  addImuValues(cur_kf_id_);

  // add imu factors between consecutive keyframe states
  addImuFactor(last_kf_id_, cur_kf_id_);

  // add between factor from RANSAC
  if(stereo_ransac_body_pose){
    std::cout << "VIO: adding between " << std::endl;
    (*stereo_ransac_body_pose).print();
    addBetweenFactor(last_kf_id_, cur_kf_id_, *stereo_ransac_body_pose);
  }

  /////////////////// MANAGE VISION MEASUREMENTS ///////////////////////////
  SmartStereoMeasurements smartStereoMeasurements_kf = status_smart_stereo_measurements_kf.second;

  // if stereo ransac failed, remove all right pixels:
  Tracker::TrackingStatus kfTrackingStatus_stereo = status_smart_stereo_measurements_kf.first.kfTrackingStatus_stereo_;
  // if(kfTrackingStatus_stereo == Tracker::TrackingStatus::INVALID){
  //   for(size_t i = 0; i < smartStereoMeasurements_kf.size(); i++)
  //     smartStereoMeasurements_kf[i].uR = std::numeric_limits<double>::quiet_NaN();;
  //}

  // extract relevant information from stereo frame
  LandmarkIds landmarks_kf;
  addStereoMeasurementsToFeatureTracks(cur_kf_id_,
                                       smartStereoMeasurements_kf,
                                       &landmarks_kf);

  if (verbosity_ >= 8) { printFeatureTracks(); }

  // decide which factors to add
  Tracker::TrackingStatus kfTrackingStatus_mono = status_smart_stereo_measurements_kf.first.kfTrackingStatus_mono_;
  switch(kfTrackingStatus_mono){
    case Tracker::TrackingStatus::LOW_DISPARITY :  // vehicle is not moving
      if (verbosity_ >= 7) {printf("Add zero velocity and no motion factors\n");}
      addZeroVelocityPrior(cur_kf_id_);
      addNoMotionFactor(last_kf_id_, cur_kf_id_);
      break;

      // This did not improve in any case
      //  case Tracker::TrackingStatus::INVALID :// ransac failed hence we cannot trust features
      //    if (verbosity_ >= 7) {printf("Add constant velocity factor (monoRansac is INVALID)\n");}
      //    addConstantVelocityFactor(last_id_, cur_id_);
      //    break;

    default: // Tracker::TrackingStatus::VALID, FEW_MATCHES, INVALID, DISABLED : // we add features in VIO
      addLandmarksToGraph(landmarks_kf);
      break;
  }

  // Why do we do this??
  // This lags 1 step behind to mimic hw.
  imu_bias_prev_kf_ = imu_bias_lkf_;

  optimize(cur_kf_id_, vio_params_.numOptimize_);
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
    if (ft.obs_.size() < 2) {// we only insert feature tracks of length at least 2 (otherwise uninformative)
      continue;
    }

    if (!ft.in_ba_graph_) {
      ft.in_ba_graph_ = true;
      addLandmarkToGraph(lm_id, ft);
      ++n_new_landmarks;
    } else {
      const std::pair<FrameId, StereoPoint2> obs_kf = ft.obs_.back();

      if(obs_kf.first != cur_kf_id_) // sanity check
        throw std::runtime_error("addLandmarksToGraph: last obs is not from the current keyframe!\n");

      updateLandmarkInGraph(lm_id, obs_kf);
      ++n_updated_landmarks;
    }
  }

  if (verbosity_ >= 7) {
    std::cout << "Added " << n_new_landmarks << " new landmarks" << std::endl;
    std::cout << "Updated " << n_updated_landmarks << " landmarks in graph" << std::endl;
  }
}

/* -------------------------------------------------------------------------- */
// Adds a landmark to the graph for the first time.
void VioBackEnd::addLandmarkToGraph(const LandmarkId& lm_id,
                                    const FeatureTrack& ft) {

  // We use a unit pinhole projection camera for the smart factors to be more efficient.
  SmartStereoFactor::shared_ptr new_factor(
        new SmartStereoFactor(smart_noise_, smart_factors_params_, B_Pose_leftCam_));

  if (verbosity_ >= 9) {std::cout << "Adding landmark with: " << ft.obs_.size() << " landmarks to graph, with keys: ";}
  if (verbosity_ >= 9){new_factor->print();}

  // add observations to smart factor
  for (const std::pair<FrameId,StereoPoint2>& obs : ft.obs_)
  {
    new_factor->add(obs.second, gtsam::Symbol('x', obs.first), stereo_cal_);
    if (verbosity_ >= 9) {std::cout << " " <<  obs.first;}
  }
  if (verbosity_ >= 9) {std::cout << std::endl;}
  // add new factor to suitable structures:
  new_smart_factors_.insert(std::make_pair(lm_id, new_factor));
  old_smart_factors_.insert(std::make_pair(lm_id, std::make_pair(new_factor, -1)));
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::updateLandmarkInGraph(
    const LandmarkId& lm_id,
    const std::pair<FrameId, StereoPoint2>& newObs) {

  // Update existing smart-factor
  auto old_smart_factors_it = old_smart_factors_.find(lm_id);
  if (old_smart_factors_it == old_smart_factors_.end())
    throw std::runtime_error("updateLandmarkInGraph: landmark not found in old_smart_factors_\n");

  SmartStereoFactor::shared_ptr old_factor = old_smart_factors_it->second.first;
  SmartStereoFactor::shared_ptr new_factor = boost::make_shared<SmartStereoFactor>(*old_factor); // clone old factor
  new_factor->add(newObs.second, gtsam::Symbol('x', newObs.first), stereo_cal_);

  // update the factor
  if (old_smart_factors_it->second.second != -1){// if slot is still -1, it means that the factor has not been inserted yet in the graph
    new_smart_factors_.insert(std::make_pair(lm_id, new_factor));
  }else{
    throw std::runtime_error("updateLandmarkInGraph: when calling update the slot should be already != -1! \n");
  }
  old_smart_factors_it->second.first = new_factor;
  if (verbosity_ >= 8) {std::cout << "updateLandmarkInGraph: added observation to point: " << lm_id << std::endl;}
}

/* -------------------------------------------------------------------------- */
ImuBias VioBackEnd::initImuBias(const ImuAccGyr& accGyroRaw,
                                      const Vector3& n_gravity) {
  LOG(WARNING) << "imuBiasInitialization: currently assumes that the vehicle is"
                  " stationary and upright!";
  Vector3 sumAccMeasurements = Vector3::Zero();
  Vector3 sumGyroMeasurements = Vector3::Zero();
  const size_t& nrMeasured = accGyroRaw.cols();
  for (size_t i = 0; i < nrMeasured; i++){
    const Vector6& accGyroRaw_i = accGyroRaw.col(i);
    // std::cout << "accGyroRaw_i: " << accGyroRaw_i.transpose() << std::endl;
    sumAccMeasurements  += accGyroRaw_i.head(3);
    sumGyroMeasurements += accGyroRaw_i.tail(3);
  }

  // Avoid the dark world of Undefined Behaviour...
  CHECK_NE(nrMeasured, 0) << "About to divide by 0!";
  gtsam::imuBias::ConstantBias
      imuInit(sumAccMeasurements/double(nrMeasured) + n_gravity,
              sumGyroMeasurements/double(nrMeasured));

  return imuInit;
}

/* -------------------------------------------------------------------------- */
gtsam::Rot3 VioBackEnd::preintegrateGyroMeasurements(
    const ImuStamps& imu_stamps,
    const ImuAccGyr& imu_accgyr) const {
  gtsam::PreintegratedAhrsMeasurements pimRot(imu_bias_prev_kf_.gyroscope(),
                                              gtsam::Matrix3::Identity());
  for (int i = 0; i < imu_stamps.size() - 1; ++i) {
    Vector3 measured_omega = imu_accgyr.block<3, 1>(3, i);
    double delta_t = UtilsOpenCV::NsecToSec(imu_stamps(i + 1) - imu_stamps(i));
    pimRot.integrateMeasurement(measured_omega, delta_t);
  }
  gtsam::Rot3 body_Rot_cam_ = B_Pose_leftCam_.rotation(); // of the left camera!!
  return body_Rot_cam_.inverse() * pimRot.deltaRij() * body_Rot_cam_;
}

/* -------------------------------------------------------------------------- */
gtsam::Pose3 VioBackEnd::guessPoseFromIMUmeasurements(
    const ImuAccGyr& accGyroRaw,
    const Vector3& n_gravity,
    const bool& round) {
  // compute average of initial measurements
  std::cout << "GuessPoseFromIMUmeasurements: currently assumes that the vehicle is stationary and upright along some axis,"
               "and gravity vector is along a single axis!" << std::endl;
  Vector3 sumAccMeasurements = Vector3::Zero();
  size_t nrMeasured = accGyroRaw.cols();
  for(size_t i=0; i < nrMeasured; i++){
    Vector6 accGyroRaw_i = accGyroRaw.col(i);
    // std::cout << "accGyroRaw_i: " << accGyroRaw_i.transpose() << std::endl;
    sumAccMeasurements  += accGyroRaw_i.head(3);
  }
  sumAccMeasurements = sumAccMeasurements/double(nrMeasured);

  gtsam::Unit3 localGravityDir(-sumAccMeasurements); // a = localGravity (we measure the opposite of gravity)
  gtsam::Unit3 globalGravityDir(n_gravity); // b

  if(round){ // align vectors to dominant axis: e.g., [0.01 0.1 1] becomes [0 0 1]
    localGravityDir = UtilsOpenCV::RoundUnit3(localGravityDir);
    globalGravityDir = UtilsOpenCV::RoundUnit3(globalGravityDir);
  }

  gtsam::Unit3 v = localGravityDir.cross(globalGravityDir); // a x b
  double c = localGravityDir.dot(globalGravityDir);
  // compute rotation such that R * a = b
  // http://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/476311#476311
  gtsam::Rot3 R;

  if( fabs(1-c)<1e-3 ){ // already aligned
    R = gtsam::Rot3();
  }else if( fabs(1+c)<1e-3 ){ // degenerate condition a =-b
    gtsam::Unit3 perturbedGravity = gtsam::Unit3(localGravityDir.unitVector() + gtsam::Vector3(1,2,3)); // compute cross product with any nonparallel vector
    v = localGravityDir.cross(perturbedGravity);
    if(std::isnan(v.unitVector()(0))){ // if the resulting vector is still not a number (i.e., perturbedGravity // localGravityDir)
      perturbedGravity = gtsam::Unit3(localGravityDir.unitVector() + gtsam::Vector3(3,2,1)); // compute cross product with any nonparallel vector
      v = localGravityDir.cross(perturbedGravity);
    }
    R = gtsam::Rot3::Expmap(v.unitVector() * M_PI); // 180 rotation around an axis perpendicular to both vectors
  }else{
    R = gtsam::Rot3::AlignPair(v, globalGravityDir, localGravityDir);
  }
  return gtsam::Pose3(R, gtsam::Point3());// absolute position is not observable anyway
}

/* -------------------------------------------------------------------------- */
// Get valid 3D points - TODO: this copies the graph.
vector<gtsam::Point3> VioBackEnd::get3DPoints() const {
  vector<gtsam::Point3> points3D;
  gtsam::NonlinearFactorGraph graph = smoother_->getFactors(); // TODO: this copies the graph
  for (auto& g : graph){
    if(g){
      auto gsf = boost::dynamic_pointer_cast<SmartStereoFactor>(g);
      if (gsf){
        // Check SF status
        gtsam::TriangulationResult result = gsf->point();
        if (result.is_initialized()) {
          if(result.valid())
            points3D.push_back(*result);
        } else {
          LOG(ERROR) << "Triangulation result is not initialized...";
        }
      }
    }
  }
  return points3D;
}

/* -------------------------------------------------------------------------- */
// Get valid 3D points and corresponding lmk id.
// Warning! it modifies old_smart_factors_!!
void VioBackEnd::getMapLmkIdsTo3dPointsInTimeHorizon(
    PointsWithIdMap* points_with_id,
    LmkIdToLmkTypeMap* lmk_id_to_lmk_type_map,
    const size_t& min_age) {
  CHECK_NOTNULL(points_with_id);
  points_with_id->clear();

  if (lmk_id_to_lmk_type_map) {
    lmk_id_to_lmk_type_map->clear();
  }

  /////////////// Add landmarks encoded in the smart factors. //////////////////
  const gtsam::NonlinearFactorGraph& graph = smoother_->getFactors();

  // old_smart_factors_ has all smart factors included so far.
  // Retrieve lmk ids from smart factors in state.
  size_t nr_valid_smart_lmks = 0, nr_smart_lmks = 0, nr_proj_lmks = 0;
  for (SmartFactorMap::iterator old_smart_factor_it = old_smart_factors_.begin();
       old_smart_factor_it != old_smart_factors_.end(); ) { //!< landmarkId -> {SmartFactorPtr, SlotIndex}
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
    CHECK(slot_id >= 0) << "Slot of smart factor is not admissible.";
    // Ensure the graph size is small enough to cast to int.
    CHECK_LT(graph.size(), std::numeric_limits<Slot>::max())
        << "Invalid cast, that would cause an overflow!";
    // Slot should be inferior to the size of the graph.
    CHECK_LT(slot_id, static_cast<Slot>(graph.size()));

    // Check that this slot_id exists in the graph, aka check that it is
    // in bounds and that the pointer is live (aka at(slot_id) works).
    if (!graph.exists(slot_id)) {
      // This slot does not exist in the current graph...
      VLOG(20) << "The slot with id: " << slot_id
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
      // not make any sense, since we are using lmk_id which comes from smart_factor
      // and result which comes from graph[slot_id], we should use smart_factor_ptr
      // instead then...
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
    // Otherwise we will be dereferencing a nullptr and we will head directly
    // to undefined behaviour wonderland.
    if (result.is_initialized()) {
      if (result.valid()) {
        if (gsf->measured().size() >= min_age) {
          // Triangulation result from smart factor is valid and
          // we have observed the lmk at least min_age times.
          VLOG(20) << "Adding lmk with id: " << lmk_id
                   << " to list of lmks in time horizon";
          // Check that we have not added this lmk already...
          CHECK(points_with_id->find(lmk_id) == points_with_id->end());
          (*points_with_id)[lmk_id] = *result;
          if (lmk_id_to_lmk_type_map) {
            (*lmk_id_to_lmk_type_map)[lmk_id] = LandmarkType::SMART;
          }
          nr_valid_smart_lmks++;
        } else {
          VLOG(20) << "Rejecting lmk with id: " << lmk_id
                   << " from list of lmks in time horizon: "
                   << "not enough measurements, " << gsf->measured().size()
                   << ", vs min_age of " << min_age << ".";
        } // gsf->measured().size() >= min_age ?
      } else {
        VLOG(20) << "Rejecting lmk with id: " << lmk_id
                 << " from list of lmks in time horizon:\n"
                 << "triangulation result is not valid (result= {"
                 << result << "}).";
      } // result.valid()?
    } else {
      VLOG(20) << "Triangulation result for smart factor of lmk with id "
               << lmk_id << " is not initialized...";
    } // result.is_initialized()?

    // Next iteration.
    old_smart_factor_it++;
  }

  ////////////// Add landmarks that now are in projection factors. /////////////
  for(const gtsam::Values::Filtered<gtsam::Value>::ConstKeyValuePair& key_value:
      state_.filter(gtsam::Symbol::ChrTest('l'))) {
    DCHECK(gtsam::Symbol(key_value.key).chr() == 'l');
    const LandmarkId& lmk_id = gtsam::Symbol(key_value.key).index();
    DCHECK(points_with_id->find(lmk_id) == points_with_id->end());
    (*points_with_id)[lmk_id] = key_value.value.cast<gtsam::Point3>();
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
  // They might actually not be changing that much because we are not enforcing
  // the regularities on the points that are out of current frame in the backend
  // currently...

  VLOG(10) << "Landmark typology to be used for the mesh:\n"
           << "Number of valid smart factors " << nr_valid_smart_lmks
           << " out of " << nr_smart_lmks << "\n"
           << "Number of landmarks (not involved in a smart factor) "
           << nr_proj_lmks << ".\n Total number of landmarks: "
           << (nr_valid_smart_lmks + nr_proj_lmks);
}

/* -------------------------------------------------------------------------- */
// NOT TESTED
gtsam::Matrix VioBackEnd::getCurrentStateCovariance() const {
  gtsam::Marginals marginals(smoother_->getFactors(),
                             state_,
                             gtsam::Marginals::Factorization::CHOLESKY);

  // Current state includes pose, velocity and imu biases.
  std::vector<gtsam::Key> keys;
  keys.push_back(gtsam::Symbol('x', cur_kf_id_));
  keys.push_back(gtsam::Symbol('v', cur_kf_id_));
  keys.push_back(gtsam::Symbol('b', cur_kf_id_));

  // Return the marginal covariance matrix.
  return UtilsOpenCV::Covariance_bvx2xvb(
        marginals.jointMarginalCovariance(keys).fullMatrix()); // 6 + 3 + 6 = 15x15matrix
}

/* -------------------------------------------------------------------------- */
// NOT TESTED
gtsam::Matrix VioBackEnd::getCurrentStateInformation() const {
  gtsam::Marginals marginals(smoother_->getFactors(),
                             state_,
                             gtsam::Marginals::Factorization::CHOLESKY);

  // Current state includes pose, velocity and imu biases.
  std::vector<gtsam::Key> keys;
  keys.push_back(gtsam::Symbol('x', cur_kf_id_));
  keys.push_back(gtsam::Symbol('v', cur_kf_id_));
  keys.push_back(gtsam::Symbol('b', cur_kf_id_));

  // Return the marginal covariance.
  return UtilsOpenCV::Covariance_bvx2xvb(
        marginals.jointMarginalInformation(keys).fullMatrix()); // 6 + 3 + 6 = 15x15matrix
}

/// Protected methods.
/* -------------------------------------------------------------------------- */
// TODO this function doesn't do just one thing... Should be refactored!
// It returns the landmark ids of the stereo measurements
// It also updates the feature tracks. Why is this in the backend???
void VioBackEnd::addStereoMeasurementsToFeatureTracks(
    const int& frame_num,
    const SmartStereoMeasurements& stereoMeasurements_kf,
    LandmarkIds* landmarks_kf) {
  CHECK_NOTNULL(landmarks_kf);

  //TODO: feature tracks will grow unbounded.

  // Make sure the landmarks_kf vector is empty and has a suitable size.
  landmarks_kf->clear();
  landmarks_kf->reserve(stereoMeasurements_kf.size());

  // Store landmark ids.
  for (size_t i = 0; i < stereoMeasurements_kf.size(); ++i) {
    const LandmarkId& lmk_id_in_kf_i = stereoMeasurements_kf.at(i).first;
    const StereoPoint2& stereo_px_i   = stereoMeasurements_kf.at(i).second;

    // We filtered invalid lmks in the StereoTracker, so this should not happen.
    CHECK_NE(lmk_id_in_kf_i, -1)
        << "landmarkId_kf_i == -1?";

    // Thinner structure that only keeps landmarkIds.
    // These landmark ids are only the ones visible in current keyframe,
    // with a valid track...
    // CHECK that we do not have repeated lmk ids!
    DCHECK(std::find(landmarks_kf->begin(),
                    landmarks_kf->end(), lmk_id_in_kf_i) == landmarks_kf->end());
    landmarks_kf->push_back(lmk_id_in_kf_i);

    // Add features to vio->featureTracks_ if they are new.
    const FeatureTracks::iterator& feature_track_it =
        feature_tracks_.find(lmk_id_in_kf_i);
    if (feature_track_it == feature_tracks_.end()) {
      // New feature.
      VLOG(20) << "Creating new feature track for lmk: "
               << lmk_id_in_kf_i << ".";
      feature_tracks_.insert(std::make_pair(lmk_id_in_kf_i,
                                           FeatureTrack(frame_num,
                                                        stereo_px_i)));
      ++landmark_count_;
    } else {
      // @TODO: It seems that this else condition does not help -- conjecture
      // that it creates long feature tracks with low information
      // (i.e. we're not moving)
      // This is problematic in conjunction with our landmark selection
      // mechanism which prioritizes long feature tracks

      // TODO: to avoid making the feature tracks grow unbounded we could
      // use a tmp feature tracks container to which we would add the old feature track
      // plus the new observation on it. (for new tracks, it would be the same as
      // above, using the tmp structure of course).

      // Add observation to existing landmark.
      VLOG(20) << "Updating feature track for lmk: "
               << lmk_id_in_kf_i << ".";
      feature_track_it->second.obs_.push_back(
            std::make_pair(frame_num, stereo_px_i));
    }
  }
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::integrateImuMeasurements(const ImuStamps& imu_stamps,
                                          const ImuAccGyr& imu_accgyr) {
  CHECK(imu_stamps.size() >= 2) << "No Imu data found.";

  if (!pim_) {
    pim_ = std::make_shared<PreintegratedImuMeasurements>(imu_params_,
                                                          imu_bias_prev_kf_);
  } else {
    pim_->resetIntegrationAndSetBias(imu_bias_prev_kf_);
  }

  for (int i = 0; i < imu_stamps.size() - 1; ++i) {
    const Vector3& measured_acc = imu_accgyr.block<3,1>(0, i);
    const Vector3& measured_omega = imu_accgyr.block<3,1>(3, i);
    const double delta_t = UtilsOpenCV::NsecToSec(imu_stamps(i + 1) -
                                                  imu_stamps(i));
    pim_->integrateMeasurement(measured_acc, measured_omega, delta_t);
  }
}

/// Value adders.
/* -------------------------------------------------------------------------- */
void VioBackEnd::addImuValues(const FrameId& cur_id) {
  gtsam::NavState navstate_lkf(W_Pose_Blkf_, W_Vel_Blkf_);
  gtsam::NavState navstate_k = pim_->predict(navstate_lkf, imu_bias_lkf_);

  debug_info_.navstate_k_ = navstate_k;

  // Update state with initial guess
  new_values_.insert(gtsam::Symbol('x', cur_id), navstate_k.pose());
  new_values_.insert(gtsam::Symbol('v', cur_id), navstate_k.velocity());
  new_values_.insert(gtsam::Symbol('b', cur_id), imu_bias_lkf_);
}

/// Factor adders.
/* -------------------------------------------------------------------------- */
void VioBackEnd::addImuFactor(const FrameId& from_id,
                              const FrameId& to_id) {
#ifdef USE_COMBINED_IMU_FACTOR
  new_imu_and_prior_factors_.push_back(
        boost::make_shared<gtsam::CombinedImuFactor>(
          gtsam::Symbol('x', from_id), gtsam::Symbol('v', from_id),
          gtsam::Symbol('x', to_id), gtsam::Symbol('v', to_id),
          gtsam::Symbol('b', from_id),
          gtsam::Symbol('b', to_id),
          *pim_));
#else
  new_imu_prior_and_other_factors_.push_back(
        boost::make_shared<gtsam::ImuFactor>(
          gtsam::Symbol('x', from_id), gtsam::Symbol('v', from_id),
          gtsam::Symbol('x', to_id), gtsam::Symbol('v', to_id),
          gtsam::Symbol('b', from_id), *pim_));

  static const gtsam::imuBias::ConstantBias zero_bias(Vector3(0, 0, 0),
                                                      Vector3(0, 0, 0));

  // Factor to discretize and move normalize by the interval between measurements:
  CHECK_NE(vio_params_.nominalImuRate_, 0)
      << "Nominal IMU rate param cannot be 0.";
  // 1/sqrt(nominalImuRate_) to discretize, then
  // sqrt(pim_->deltaTij()/nominalImuRate_) to count the nr of measurements.
  const double d = sqrt(pim_->deltaTij()) / vio_params_.nominalImuRate_;
  Vector6 biasSigmas;
  biasSigmas.head<3>().setConstant(d * vio_params_.accBiasSigma_);
  biasSigmas.tail<3>().setConstant(d * vio_params_.gyroBiasSigma_);
  const gtsam::SharedNoiseModel& bias_noise_model =
      gtsam::noiseModel::Diagonal::Sigmas(biasSigmas);

  new_imu_prior_and_other_factors_.push_back(
        boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> >(
          gtsam::Symbol('b', from_id),
          gtsam::Symbol('b', to_id),
          zero_bias, bias_noise_model));
#endif

  debug_info_.imuR_lkf_kf = pim_->deltaRij();
  debug_info_.numAddedImuF_++;

  // reset preintegration
  pim_.reset();
}


/* -------------------------------------------------------------------------- */
void VioBackEnd::addBetweenFactor(const FrameId& from_id,
                                  const FrameId& to_id,
                                  const gtsam::Pose3& from_id_POSE_to_id) {
  Vector6 precisions;
  precisions.head<3>().setConstant(vio_params_.betweenRotationPrecision_);
  precisions.tail<3>().setConstant(vio_params_.betweenTranslationPrecision_);
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

  if (verbosity_ >= 7) {
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
// TODO remove global variables from optimize, pass them as local parameters...
// TODO make changes to global variables to the addVisualInertial blah blah.
void VioBackEnd::optimize(
    const FrameId& cur_id,
    const size_t& max_extra_iterations,
    const std::vector<size_t>& extra_factor_slots_to_delete) {
  CHECK(smoother_.get()) << "Incremental smoother is a null pointer.";

  // Only for statistics and debugging.
  // Store start time.
  double startTime;
  // Reset all timing info.
  debug_info_.resetTimes();
  if (verbosity_ >= 5 || log_timing_) {
    startTime = UtilsOpenCV::GetTimeInSeconds();
  }

  /////////////////////// BOOKKEEPING //////////////////////////////////////////
  // We need to remove all smart factors that have new observations.
  // Extra factor slots to delete contains potential factors that we want to delete, it is
  // typically an empty vector. And is only used to give flexibility to subclasses.
  std::vector<size_t> delete_slots = extra_factor_slots_to_delete;
  std::vector<LandmarkId> lmk_ids_of_new_smart_factors_tmp;
  gtsam::NonlinearFactorGraph new_factors_tmp;
  for (const auto& new_smart_factor: new_smart_factors_) {
    // Push back the smart factor to the list of new factors to add to the graph.
    new_factors_tmp.push_back(new_smart_factor.second); // Smart factor, so same address right?

    // Store lmk id of the smart factor to add to the graph.
    lmk_ids_of_new_smart_factors_tmp.push_back(new_smart_factor.first);

    // Find smart factor and slot in old_smart_factors_ corresponding to
    // the lmk with id of the new smart factor.
    const auto& it = old_smart_factors_.find(new_smart_factor.first);
    CHECK(it != old_smart_factors_.end())
        << "Lmk with id: " << new_smart_factor.first
        << " could not be found in old_smart_factors_.";

    if (it->second.second != -1) {
      // Smart factor Slot is different than -1, therefore the factor is
      // already in the factor graph.
      // We must delete the smart factor from the graph.
      // We need to remove all smart factors that have new observations.
      // TODO what happens if delete_slots has repeated elements?
      CHECK_GE(it->second.second, 0);
      delete_slots.push_back(it->second.second);
    }
  }

  // Add also other factors (imu, priors).
  // SMART FACTORS MUST BE FIRST, otherwise when recovering the slots
  // for the smart factors we will mess up.
  new_factors_tmp.push_back(new_imu_prior_and_other_factors_.begin(),
                            new_imu_prior_and_other_factors_.end());

  //////////////////////////////////////////////////////////////////////////////

  if (verbosity_ >= 5 || log_timing_) {
    debug_info_.factorsAndSlotsTime_ = UtilsOpenCV::GetTimeInSeconds() -
                                      startTime;
  }

  if (verbosity_ >= 5) {
    // Get state before optimization to compute error.
    debug_info_.stateBeforeOpt = gtsam::Values(state_);
    BOOST_FOREACH(const gtsam::Values::ConstKeyValuePair& key_value, new_values_) {
      debug_info_.stateBeforeOpt.insert(key_value.key, key_value.value);
    }
  }

  if (verbosity_ >= 8) {
    printSmootherInfo(new_factors_tmp, delete_slots,
                      "Smoother status before update:",
                      verbosity_ >= 9);
  }

  // Recreate the graph before marginalization.
  if (verbosity_ >= 5 || FLAGS_debug_graph_before_opt) {
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
  std::map<Key, double> timestamps;
  for(const auto& keyValue : new_values_) {
    timestamps[keyValue.key] = timestamp_kf_; // for the latest pose, velocity, and bias
  }
  CHECK_EQ(timestamps.size(), new_values_.size());

  // Store time before iSAM update.
  if (verbosity_ >= 5 || log_timing_) {
    debug_info_.preUpdateTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
  }

  // Compute iSAM update.
  VLOG(10) << "iSAM2 update with " << new_factors_tmp.size() << " new factors "
           << ", " << new_values_.size() << " new values "
           << ", and " << delete_slots.size() << " deleted factors.";
  Smoother::Result result;
  VLOG(10) << "Starting first update.";
  updateSmoother(&result,
                 new_factors_tmp,
                 new_values_,
                 timestamps,
                 delete_slots);
  VLOG(10) << "Finished first update.";

  // Store time after iSAM update.
  if (verbosity_ >= 5 || log_timing_) {
    debug_info_.updateTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
  }

  /////////////////////////// BOOKKEEPING //////////////////////////////////////

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

  if (verbosity_ >= 5 || log_timing_) {
    debug_info_.updateSlotTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
  }

  //////////////////////////////////////////////////////////////////////////////

  // Do some more optimization iterations.
  for (size_t n_iter = 1; n_iter < max_extra_iterations; ++n_iter) {
    VLOG(10) << "Doing extra iteration nr: " << n_iter;
    updateSmoother(&result);
  }

  if (verbosity_ >= 5 || log_timing_) {
    debug_info_.extraIterationsTime_ = UtilsOpenCV::GetTimeInSeconds() -
                                      startTime;
  }

  // Update states we need for next iteration.
  updateStates(cur_id);

  // DEBUG:
  postDebug(startTime);
}

/// Private methods.
/* -------------------------------------------------------------------------- */
void VioBackEnd::addInitialPriorFactors(const FrameId& frame_id,
                                        const ImuAccGyr& imu_accgyr) {
  // Set initial covariance for inertial factors
  // W_Pose_Blkf_ set by motion capture to start with
  Matrix3 B_Rot_W = W_Pose_Blkf_.rotation().matrix().transpose();

  // Set initial pose uncertainty: constrain mainly position and global yaw.
  // roll and pitch is observable, therefore low variance.
  Matrix6 pose_prior_covariance = Matrix6::Zero();
  pose_prior_covariance.diagonal()[0] = vio_params_.initialRollPitchSigma_ * vio_params_.initialRollPitchSigma_;
  pose_prior_covariance.diagonal()[1] = vio_params_.initialRollPitchSigma_ * vio_params_.initialRollPitchSigma_;
  pose_prior_covariance.diagonal()[2] = vio_params_.initialYawSigma_ * vio_params_.initialYawSigma_;
  pose_prior_covariance.diagonal()[3] = vio_params_.initialPositionSigma_ * vio_params_.initialPositionSigma_;
  pose_prior_covariance.diagonal()[4] = vio_params_.initialPositionSigma_ * vio_params_.initialPositionSigma_;
  pose_prior_covariance.diagonal()[5] = vio_params_.initialPositionSigma_ * vio_params_.initialPositionSigma_;

  // Rotate initial uncertainty into local frame, where the uncertainty is specified.
  pose_prior_covariance.topLeftCorner(3,3) =
      B_Rot_W * pose_prior_covariance.topLeftCorner(3,3) * B_Rot_W.transpose();

  // Add pose prior.
  gtsam::SharedNoiseModel noise_init_pose =
      gtsam::noiseModel::Gaussian::Covariance(pose_prior_covariance);
  new_imu_prior_and_other_factors_.push_back(
        boost::make_shared<gtsam::PriorFactor<gtsam::Pose3> >(
          gtsam::Symbol('x', frame_id), W_Pose_Blkf_, noise_init_pose));

  // Add initial velocity priors.
  gtsam::SharedNoiseModel initialVelocityPriorNoise =
      gtsam::noiseModel::Isotropic::Sigma(3, vio_params_.initialVelocitySigma_);
  new_imu_prior_and_other_factors_.push_back(
        boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
          gtsam::Symbol('v', frame_id), W_Vel_Blkf_, initialVelocityPriorNoise));

  // Add initial bias priors:
  Vector6 prior_biasSigmas;
  prior_biasSigmas.head<3>().setConstant(vio_params_.initialAccBiasSigma_);
  prior_biasSigmas.tail<3>().setConstant(vio_params_.initialGyroBiasSigma_);
  gtsam::SharedNoiseModel imu_bias_prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(prior_biasSigmas);
  new_imu_prior_and_other_factors_.push_back(
        boost::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
          gtsam::Symbol('b', frame_id), imu_bias_lkf_, imu_bias_prior_noise));

  if (verbosity_ >= 7) {std::cout << "Added initial priors for frame " << frame_id << std::endl;}
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

  debug_info_.numAddedConstantVelF_++;
}

/* -------------------------------------------------------------------------- */
// Update states.
void VioBackEnd::updateStates(const FrameId& cur_id) {
  VLOG(10) << "Starting to calculate estimate.";
  state_ = smoother_->calculateEstimate();
  VLOG(10) << "Finished to calculate estimate.";

  CHECK(state_.find(gtsam::Symbol('x', cur_id)) != state_.end());
  CHECK(state_.find(gtsam::Symbol('v', cur_id)) != state_.end());
  CHECK(state_.find(gtsam::Symbol('b', cur_id)) != state_.end());

  W_Pose_Blkf_  = state_.at<Pose3>(gtsam::Symbol('x', cur_id));
  W_Vel_Blkf_   = state_.at<Vector3>(gtsam::Symbol('v', cur_id));
  imu_bias_lkf_ = state_.at<gtsam::imuBias::ConstantBias>(
                    gtsam::Symbol('b', cur_id));
}

/* -------------------------------------------------------------------------- */
// Update smoother.
void VioBackEnd::updateSmoother(
    Smoother::Result* result,
    const gtsam::NonlinearFactorGraph& new_factors_tmp,
    const gtsam::Values& new_values,
    const std::map<Key, double>& timestamps,
    const std::vector<size_t>& delete_slots) {
  CHECK_NOTNULL(result);
  gtsam::NonlinearFactorGraph empty_graph;

  // Store smoother as backup.
  CHECK(smoother_);
  // This is not doing a full deep copy: it is keeping same shared_ptrs for factors
  // but copying the isam result.
  Smoother smoother_backup (*smoother_);

  bool got_cheirality_exception = false;
  gtsam::Symbol lmk_symbol_cheirality;
  try {
    // Update smoother.
    VLOG(10) << "Starting update of smoother_...";
    *result = smoother_->update(new_factors_tmp,
                                new_values,
                                timestamps,
                                delete_slots);
    VLOG(10) << "Finished update of smoother_.";
    if (DEBUG_) {
      printSmootherInfo(new_factors_tmp,
                        delete_slots,
                        "CATCHING EXCEPTION",
                        false);
      DEBUG_ = false;
    }
  } catch (const gtsam::IndeterminantLinearSystemException& e) {
    LOG(ERROR) << e.what();

    const gtsam::Key& var = e.nearbyVariable();
    gtsam::Symbol symb (var);

    LOG(ERROR) << "ERROR: Variable has type '" << symb.chr() << "' "
               << "and index " << symb.index() << std::endl;

    smoother_->getFactors().print("Smoother's factors:\n[\n\t");
    std::cout << " ]" << std::endl;
    state_.print("State values\n[\n\t");
    std::cout << " ]" << std::endl;

    printSmootherInfo(new_factors_tmp,
                      delete_slots);
    throw;
  } catch (const gtsam::InvalidNoiseModel& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors_tmp, delete_slots);
  } catch (const gtsam::InvalidMatrixBlock& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors_tmp, delete_slots);
  } catch (const gtsam::InvalidDenseElimination& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors_tmp, delete_slots);
  } catch (const gtsam::InvalidArgumentThreadsafe& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors_tmp, delete_slots);
  } catch (const gtsam::ValuesKeyDoesNotExist& e){
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors_tmp, delete_slots);
  } catch (const gtsam::CholeskyFailed& e){
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors_tmp, delete_slots);
  } catch (const gtsam::CheiralityException& e) {
    LOG(ERROR) << e.what();
    const gtsam::Key& lmk_key = e.nearbyVariable();
    lmk_symbol_cheirality = gtsam::Symbol(lmk_key);
    LOG(ERROR) << "ERROR: Variable has type '" << lmk_symbol_cheirality.chr() << "' "
               << "and index " << lmk_symbol_cheirality.index();
    printSmootherInfo(new_factors_tmp, delete_slots);
    got_cheirality_exception = true;
  } catch (const gtsam::StereoCheiralityException& e) {
    LOG(ERROR) << e.what();
    const gtsam::Key& lmk_key = e.nearbyVariable();
    lmk_symbol_cheirality = gtsam::Symbol(lmk_key);
    LOG(ERROR) << "ERROR: Variable has type '" << lmk_symbol_cheirality.chr() << "' "
               << "and index " << lmk_symbol_cheirality.index();
    printSmootherInfo(new_factors_tmp, delete_slots);
    got_cheirality_exception = true;
  } catch (const gtsam::RuntimeErrorThreadsafe& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors_tmp, delete_slots);
  } catch (const gtsam::OutOfRangeThreadsafe& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors_tmp, delete_slots);
  } catch (const std::out_of_range& e) {
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors_tmp, delete_slots);
  } catch (const std::exception& e) {
    // Catch anything thrown within try block that derives from std::exception.
    LOG(ERROR) << e.what();
    printSmootherInfo(new_factors_tmp, delete_slots);
  } catch (...) {
    // Catch the rest of exceptions.
    LOG(ERROR) << "Unrecognized exception.";
    printSmootherInfo(new_factors_tmp, delete_slots);
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
      CHECK_LE(counter_of_exceptions, FLAGS_max_number_of_cheirality_exceptions);

      // Check that we have a landmark.
      CHECK(lmk_symbol_cheirality.chr() == 'l');

      // Now that we know the lmk id, delete all factors attached to it!
      gtsam::NonlinearFactorGraph new_factors_tmp_cheirality;
      gtsam::Values new_values_cheirality;
      std::map<Key, double> timestamps_cheirality;
      std::vector<size_t> delete_slots_cheirality;
      const gtsam::NonlinearFactorGraph& graph = smoother_->getFactors();
      VLOG(10) << "Starting cleanCheiralityLmk...";
      cleanCheiralityLmk(lmk_symbol_cheirality,
                         &new_factors_tmp_cheirality,
                         &new_values_cheirality,
                         &timestamps_cheirality,
                         &delete_slots_cheirality,
                         graph,
                         new_factors_tmp,
                         new_values,
                         timestamps,
                         delete_slots);
      VLOG(10) << "Finished cleanCheiralityLmk.";

      // Recreate the graph before marginalization.
      if (verbosity_ >= 5 || FLAGS_debug_graph_before_opt) {
        debug_info_.graphBeforeOpt = graph;
        debug_info_.graphToBeDeleted = gtsam::NonlinearFactorGraph();
        debug_info_.graphToBeDeleted.resize(delete_slots_cheirality.size());
        for (size_t i = 0; i < delete_slots_cheirality.size(); i++) {
          // If the factor is to be deleted, store it as graph to be deleted.
          CHECK(graph.exists(delete_slots_cheirality.at(i)))
              << "Slot # " << delete_slots_cheirality.at(i)
              << "does not exist in smoother graph.";
          // TODO here we can get the right slot that we are going to delete,
          // extend graphToBeDeleted to have both the factor and the slot.
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

/* -------------------------------------------------------------------------- */
void VioBackEnd::cleanCheiralityLmk(
    const gtsam::Symbol& lmk_symbol,
    gtsam::NonlinearFactorGraph* new_factors_tmp_cheirality,
    gtsam::Values* new_values_cheirality,
    std::map<Key, double>* timestamps_cheirality,
    std::vector<size_t>* delete_slots_cheirality,
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::NonlinearFactorGraph& new_factors_tmp,
    const gtsam::Values& new_values,
    const std::map<Key, double>& timestamps,
    const std::vector<size_t>& delete_slots) {
  CHECK_NOTNULL(new_factors_tmp_cheirality);
  CHECK_NOTNULL(new_values_cheirality);
  CHECK_NOTNULL(timestamps_cheirality);
  CHECK_NOTNULL(delete_slots_cheirality);
  const gtsam::Key& lmk_key = lmk_symbol.key();

  // Delete from new factors.
  VLOG(10) << "Starting delete from new factors...";
  deleteAllFactorsWithKeyFromFactorGraph(lmk_key,
                                         new_factors_tmp,
                                         new_factors_tmp_cheirality);
  VLOG(10) << "Finished delete from new factors.";

  // Delete from new values.
  VLOG(10) << "Starting delete from new values...";
  bool is_deleted_from_values = deleteKeyFromValues(
                                  lmk_key,
                                  new_values,
                                  new_values_cheirality);
  VLOG(10) << "Finished delete from timestamps.";

  // Delete from new values.
  VLOG(10) << "Starting delete from timestamps...";
  bool is_deleted_from_timestamps = deleteKeyFromTimestamps(
                                      lmk_key,
                                      timestamps,
                                      timestamps_cheirality);
  VLOG(10) << "Finished delete from timestamps.";

  // Check that if we deleted from values, we should have deleted as well from
  // timestamps.
  CHECK_EQ(is_deleted_from_values, is_deleted_from_timestamps);

  // Delete slots in current graph.
  VLOG(10) << "Starting delete from current graph...";
  *delete_slots_cheirality = delete_slots;
  std::vector<size_t> slots_of_extra_factors_to_delete;
  // Achtung: This has the chance to make the plane underconstrained, if
  // we delete too many point_plane factors.
  findSlotsOfFactorsWithKey(lmk_key,
                            graph,
                            &slots_of_extra_factors_to_delete);
  delete_slots_cheirality->insert(delete_slots_cheirality->end(),
                                  slots_of_extra_factors_to_delete.begin(),
                                  slots_of_extra_factors_to_delete.end());
  VLOG(10) << "Finished delete from current graph.";

  //////////////////////////// BOOKKEEPING /////////////////////////////////////
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

/* -------------------------------------------------------------------------- */
// Modifies old_smart_factors_, it adds the slot number.
void VioBackEnd::findSmartFactorsSlotsSlow(
    const std::vector<gtsam::Key>& new_smart_factors_lmkID_tmp) {

  // OLD INEFFICIENT VERSION:
  const gtsam::NonlinearFactorGraph& graphFactors = smoother_->getFactors();

  vector<LandmarkId> landmarksToRemove;
  for (auto& it : old_smart_factors_) {
    bool found = false;
    for (size_t slot = 0; slot < graphFactors.size(); ++slot) {
      const gtsam::NonlinearFactor::shared_ptr& f = graphFactors[slot];
      if (f) {
        // TODO that will not work anymore!!! not all factors are smart!!!
        SmartStereoFactor::shared_ptr smartfactor =
            boost::dynamic_pointer_cast<SmartStereoFactor>(graphFactors.at(slot));
        // TODO smartfactor == won't work if cheirality exception is handled,
        // because we clone the graph.
        if (smartfactor &&
            it.second.first == smartfactor) {
          it.second.second = slot;
          found = true;
          break;
        }
      }
    }

    if (!found) {
      // If it's not in the graph we can delete if from the map.
      if (verbosity_ >= 6) {
        std::cout << "Smart factor with id: " << it.first
                  << " not found" << std::endl; }
      landmarksToRemove.push_back(it.first);
    }
  }

  // Erase factors that are no longer there.
  for (const auto& i: landmarksToRemove) {
    old_smart_factors_.erase(i);
  }
}

/* -------------------------------------------------------------------------- */
// BOOKKEEPING, for next iteration to know which slots have to be deleted
// before adding the new smart factors.
void VioBackEnd::updateNewSmartFactorsSlots(
    const std::vector<LandmarkId>& lmk_ids_of_new_smart_factors,
    SmartFactorMap* old_smart_factors) {
  CHECK_NOTNULL(old_smart_factors);

  // Get result.
  const gtsam::ISAM2Result& result = smoother_->getISAM2Result();

  // Simple version of find smart factors.
  for (size_t i = 0; i < lmk_ids_of_new_smart_factors.size(); ++i) {
    CHECK(i < result.newFactorsIndices.size())
        << "There are more new smart factors than new factors added to the graph.";
    // Get new slot in the graph for the newly added smart factor.
    const size_t& slot = result.newFactorsIndices.at(i);

    // TODO this will not work if there are non-smart factors!!!
    // Update slot using isam2 indices.
    // ORDER of inclusion of factors in the ISAM2::update() function matters,
    // as these indices have a 1-to-1 correspondence with the factors.

    // BOOKKEEPING, for next iteration to know which slots have to be deleted
    // before adding the new smart factors.
    // Find the entry in old_smart_factors_.
    const auto& it = old_smart_factors->find(
                       lmk_ids_of_new_smart_factors.at(i));

    CHECK(it != old_smart_factors->end())
        << "Trying to access unavailable factor.";

    // CHECK that the factor in the graph at slot position is a smart factor.
    CHECK(boost::dynamic_pointer_cast<SmartStereoFactor>(
            smoother_->getFactors().at(slot)));

    // CHECK that shared ptrs point to the same smart factor.
    // make sure no one is cloning SmartSteroFactors.
    CHECK_EQ(it->second.first, boost::dynamic_pointer_cast<SmartStereoFactor>(
               smoother_->getFactors().at(slot)))
        << "Non-matching addresses for same factors for lmk with id: "
        << lmk_ids_of_new_smart_factors.at(i) << " in old_smart_factors_ "
        << "VS factor in graph at slot: " << slot
        << ". Slot previous to update was: " << it->second.second;

    // Update slot number in old_smart_factors_.
    it->second.second = slot;
  }
}

/* -------------------------------------------------------------------------- */
// Set parameters for ISAM 2 incremental smoother.
void VioBackEnd::setIsam2Params(
    const VioBackEndParams& vio_params,
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
  //gtsam::FastMap<char,gtsam::Vector> thresholds;
    //gtsam::Vector xThresh(6); // = {0.05, 0.05, 0.05, 0.1, 0.1, 0.1};
    //gtsam::Vector vThresh(3); //= {1.0, 1.0, 1.0};
    //gtsam::Vector bThresh(6); // = {1.0, 1.0, 1.0};
    //xThresh << relinearizeThresholdRot_, relinearizeThresholdRot_, relinearizeThresholdRot_, relinearizeThresholdPos_, relinearizeThresholdPos_, relinearizeThresholdPos_;
    //vThresh << relinearizeThresholdVel_, relinearizeThresholdVel_, relinearizeThresholdVel_;
    //bThresh << relinearizeThresholdIMU_, relinearizeThresholdIMU_, relinearizeThresholdIMU_, relinearizeThresholdIMU_, relinearizeThresholdIMU_, relinearizeThresholdIMU_;
    //thresholds['x'] = xThresh;
    //thresholds['v'] = vThresh;
    //thresholds['b'] = bThresh;
    // isam_param.setRelinearizeThreshold(thresholds);

  // Cache Linearized Factors seems to improve performance.
  isam_param->setCacheLinearizedFactors(true);
  isam_param->setEvaluateNonlinearError(false);
  isam_param->relinearizeThreshold = vio_params.relinearizeThreshold_;
  isam_param->relinearizeSkip = vio_params.relinearizeSkip_;
  // isam_param->enablePartialRelinearizationCheck = true;
  isam_param->findUnusedFactorSlots = true;
  isam_param->enableDetailedResults = false;   // only for debugging.
  isam_param->factorization = gtsam::ISAM2Params::CHOLESKY; // QR
  isam_param->print("isam_param");
  //isam_param.evaluateNonlinearError = true;  // only for debugging.
}

/* -------------------------------------------------------------------------- */
// Set parameters for all the factors.
void VioBackEnd::setFactorsParams(
    const VioBackEndParams& vio_params,
    gtsam::SharedNoiseModel* smart_noise,
    gtsam::SmartStereoProjectionParams* smart_factors_params,
    boost::shared_ptr<PreintegratedImuMeasurements::Params>* imu_params,
    gtsam::SharedNoiseModel* no_motion_prior_noise,
    gtsam::SharedNoiseModel* zero_velocity_prior_noise,
    gtsam::SharedNoiseModel* constant_velocity_prior_noise) {
  CHECK_NOTNULL(smart_noise);
  CHECK_NOTNULL(smart_factors_params);
  CHECK_NOTNULL(imu_params);
  CHECK_NOTNULL(no_motion_prior_noise);
  CHECK_NOTNULL(zero_velocity_prior_noise);
  CHECK_NOTNULL(constant_velocity_prior_noise);

  //////////////////////// SMART PROJECTION FACTORS SETTINGS ///////////////////
  setSmartFactorsParams(smart_noise,
                        smart_factors_params,
                        vio_params.smartNoiseSigma_,
                        vio_params.rankTolerance_,
                        vio_params.landmarkDistanceThreshold_,
                        vio_params.retriangulationThreshold_,
                        vio_params.outlierRejection_);

  //////////////////////// IMU FACTORS SETTINGS ////////////////////////////////
  setImuFactorsParams(imu_params,
                      vio_params.n_gravity_,
                      vio_params.gyroNoiseDensity_,
                      vio_params.accNoiseDensity_,
                      vio_params.imuIntegrationSigma_);
#ifdef USE_COMBINED_IMU_FACTOR
  imuParams_->biasAccCovariance =
      std::pow(vioParams.accBiasSigma_, 2.0) * Eigen::Matrix3d::Identity();
  imuParams_->biasOmegaCovariance =
      std::pow(vioParams.gyroBiasSigma_, 2.0) * Eigen::Matrix3d::Identity();
#endif

  //////////////////////// NO MOTION FACTORS SETTINGS //////////////////////////
  Vector6 sigmas;
  sigmas.head<3>().setConstant(vio_params.noMotionRotationSigma_);
  sigmas.tail<3>().setConstant(vio_params.noMotionPositionSigma_);
  *no_motion_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  //////////////////////// ZERO VELOCITY FACTORS SETTINGS //////////////////////
  *zero_velocity_prior_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, vio_params.zeroVelocitySigma_);

  //////////////////////// CONSTANT VELOCITY FACTORS SETTINGS //////////////////
  *constant_velocity_prior_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, vio_params.constantVelSigma_);
}

/* -------------------------------------------------------------------------- */
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
                                    3, smart_noise_sigma); // vio_smart_reprojection_err_thresh / cam_->fx());
  // smart_noise_ = gtsam::noiseModel::Robust::Create(
  //                  gtsam::noiseModel::mEstimator::Huber::Create(1.345),
  //                  model);
  *smart_noise = model;
  *smart_factors_params = SmartFactorParams(
                            gtsam::HESSIAN,// JACOBIAN_SVD
                            gtsam::ZERO_ON_DEGENERACY, // IGNORE_DEGENERACY
                            false, // ThrowCherality = false
                            true); // verboseCherality = true
  smart_factors_params->setRankTolerance(
        rank_tolerance);
  smart_factors_params->setLandmarkDistanceThreshold(
        landmark_distance_threshold);
  smart_factors_params->setRetriangulationThreshold(
        retriangulation_threshold);
  smart_factors_params->setDynamicOutlierRejectionThreshold(
        outlier_rejection);
}

/* -------------------------------------------------------------------------- */
// Set parameters for imu factors.
void VioBackEnd::setImuFactorsParams(
    boost::shared_ptr<PreintegratedImuMeasurements::Params>* imu_params,
    const gtsam::Vector3& n_gravity,
    const double& gyro_noise_density,
    const double& acc_noise_density,
    const double& imu_integration_sigma) {
  CHECK_NOTNULL(imu_params);
  *imu_params = boost::make_shared<PreintegratedImuMeasurements::Params>(
                  n_gravity);
  (*imu_params)->gyroscopeCovariance =
      std::pow(gyro_noise_density, 2.0) * Eigen::Matrix3d::Identity();
  (*imu_params)->accelerometerCovariance =
      std::pow(acc_noise_density, 2.0) * Eigen::Matrix3d::Identity();
  (*imu_params)->integrationCovariance =
      std::pow(imu_integration_sigma, 2.0) * Eigen::Matrix3d::Identity();
  (*imu_params)->use2ndOrderCoriolis = false; // TODO: expose this parameter
}

/// Printers.
/* -------------------------------------------------------------------------- */
void VioBackEnd::print() const {
  std::cout << "((((((((((((((((((((((((((((((((((((((((( VIO PRINT )))))))))"
            << ")))))))))))))))))))))))))))))))) " <<std::endl;
  B_Pose_leftCam_.print("\n B_Pose_leftCam_\n");
  stereo_cal_->print("\n stereoCal_\n");
  vio_params_.print();
  W_Pose_Blkf_.print("\n W_Pose_Blkf_ \n");
  std::cout << "\n W_Vel_Blkf_ " << W_Vel_Blkf_.transpose() <<std::endl;
  imu_bias_lkf_.print("\n imu_bias_lkf_ \n");
  imu_bias_prev_kf_.print("\n imu_bias_prev_kf_ \n");
  std::cout << "last_id_ " << last_kf_id_ <<std::endl;
  std::cout << "cur_id_ " << cur_kf_id_ <<std::endl;
  std::cout << "verbosity_ " << verbosity_ <<std::endl;
  std::cout << "landmark_count_ " << landmark_count_ <<std::endl;
  std::cout << "(((((((((((((((((((((((((((((((((((((((((((((((()))))))))))))"
            << "))))))))))))))))))))))))))))))))) " <<std::endl;
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::printFeatureTracks() const {
  std::cout << "---- Feature tracks: --------- " << std::endl;
  BOOST_FOREACH(auto keyTrack_j, feature_tracks_) {
    std::cout << "Landmark " << keyTrack_j.first << " having ";
    keyTrack_j.second.print();
  }
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::printSmootherInfo(
    const gtsam::NonlinearFactorGraph& new_factors_tmp,
    const std::vector<size_t>& delete_slots,
    const string& message,
    const bool& showDetails) const {
  LOG(INFO) << " =============== START:" <<  message << " =============== "
            << std::endl;


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
  ////////////////////// Print all factors. ////////////////////////////////////
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

  ///////////// Print factors that were newly added to the optimization.////////
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

  ////////////////////////////// Print deleted slots.///////////////////////////
  LOG(INFO) << "Nr deleted slots: " << delete_slots.size()
            << ", with slots:" << std::endl;
  LOG(INFO) << "[\n\t";
  if (debug_info_.graphToBeDeleted.size() != 0) {
    // If we are storing the graph to be deleted, then print extended info
    // besides the slot to be deleted.
    CHECK_EQ(debug_info_.graphToBeDeleted.size(), delete_slots.size());
    for (size_t i = 0; i < delete_slots.size(); ++i) {
      CHECK_NOTNULL(debug_info_.graphToBeDeleted.at(i).get());
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

  //////////////////////// Print all values in state. //////////////////////////
  LOG(INFO) << "Nr of values in state_ : " << state_.size()
            << ", with keys:";
  std::cout << "[\n\t";
  BOOST_FOREACH(const gtsam::Values::ConstKeyValuePair& key_value, state_) {
    std::cout << gtsam::DefaultKeyFormatter(key_value.key) << " ";
  }
  std::cout << std::endl;
  LOG(INFO) << " ]";

  // Print only new values.
  LOG(INFO) << "Nr values in new_values_ : " << new_values_.size()
            << ", with keys:";
  std::cout << "[\n\t";
  BOOST_FOREACH(const gtsam::Values::ConstKeyValuePair& key_value, new_values_) {
    std::cout << " " << gtsam::DefaultKeyFormatter(key_value.key) << " ";
  }
  std::cout << std::endl;
  LOG(INFO) << " ]";

  if (showDetails) {
    graph->print("isam2 graph:\n");
    new_factors_tmp.print("new_factors_tmp:\n");
    new_values_.print("new values:\n");
    //LOG(INFO) << "new_smart_factors_: "  << std::endl;
    //for (auto& s : new_smart_factors_)
    //	s.second->print();
  }

  LOG(INFO) << " =============== END: " <<  message << " =============== "
            << std::endl;
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::printSmartFactor(
    boost::shared_ptr<SmartStereoFactor> gsf) const {
  CHECK(gsf);
  std::cout << "Smart Factor (valid: "
            << (gsf->isValid()? "yes" : "NO!")
            << ", deg: "
            << (gsf->isDegenerate()?"YES!" : "no")
            << " isCheir: "
            << (gsf->isPointBehindCamera()?"YES!" : "no")
            << "): \t";
  gsf->printKeys();
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::printPointPlaneFactor(
    boost::shared_ptr<gtsam::PointPlaneFactor> ppf) const {
  CHECK(ppf);
  std::cout << "Point Plane Factor: plane key "
            << gtsam::DefaultKeyFormatter(ppf->getPlaneKey())
            << ", point key " << gtsam::DefaultKeyFormatter(ppf->getPointKey())
            << "\n";
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::printPlanePrior(
    boost::shared_ptr<gtsam::PriorFactor<gtsam::OrientedPlane3> > ppp) const {
  CHECK(ppp);
  std::cout << "Plane Prior: plane key \t";
  ppp->printKeys();
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::printPointPrior(
    boost::shared_ptr<gtsam::PriorFactor<gtsam::Point3>> ppp) const {
  CHECK(ppp);
  std::cout << "Point Prior: point key \t";
  ppp->printKeys();
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::printLinearContainerFactor(
    boost::shared_ptr<gtsam::LinearContainerFactor> lcf) const {
  CHECK(lcf);
  std::cout << "Linear Container Factor: \t";
  lcf->printKeys();
}

/* -------------------------------------------------------------------------- */
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
    const auto& ppp = boost::dynamic_pointer_cast<gtsam::PriorFactor<
                      gtsam::OrientedPlane3>>(g);
    if (ppp) {
      std::cout << "\tSlot # " << slot << ": ";
      printPlanePrior(ppp);
    }
  }

  if (print_point_priors) {
    const auto& ppp = boost::dynamic_pointer_cast<gtsam::PriorFactor<
                      gtsam::Point3>>(g);
    if (ppp) {
      std::cout << "\tSlot # " << slot << ": ";
      printPointPrior(ppp);
    }
  }

  if (print_linear_container_factors) {
    const auto& lcf = boost::dynamic_pointer_cast<
                      gtsam::LinearContainerFactor>(g);
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
    printSelectedFactors(g, slot,
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
  if(verbosity_>8)
  {
    std::cout << "Landmarks in old_smart_factors_: " << std::endl;
    for (auto it : old_smart_factors_)
    {
      std::cout << "Landmark " << it.first << " with slot " << it.second.second << std::endl;
    }
  }
  // Compute number of valid/degenerate
  debug_info_.resetSmartFactorsStatistics();
  gtsam::NonlinearFactorGraph graph = smoother_->getFactors();
  for (auto& g : graph)
  {
    if (g)
    {
      auto gsf = boost::dynamic_pointer_cast<SmartStereoFactor>(g);
      if (gsf)
      {
        debug_info_.numSF_ += 1;

        // Check for consecutive Keys: this check is wrong: if there is LOW_DISPARITY
        // at some frame, we do not add the measurement to the smart factor, hence keys are not necessarily consecutive
        //auto keys = g->keys();
        //Key last_key;
        //bool first_key = true;
        //for (Key key : keys)
        //{
        //  if (!first_key && key - last_key != 1){
        //    std::cout << " Last: " << gtsam::DefaultKeyFormatter(last_key) << " Current: " << gtsam::DefaultKeyFormatter(key) << std::endl;
        //    for (Key k : keys){ std::cout << " " << gtsam::DefaultKeyFormatter(k) << " "; }
        //    throw std::runtime_error("\n computeSmartFactorStatistics: found nonconsecutive keys in smart factors \n");
        //  }
        //  last_key = key;
        //  first_key = false;
        //}
        // Check SF status
        gtsam::TriangulationResult result = gsf->point();
        if (result.is_initialized()) {
          if (result.degenerate())
            debug_info_.numDegenerate_ += 1;

          if (result.farPoint())
            debug_info_.numFarPoints_ += 1;

          if (result.outlier())
            debug_info_.numOutliers_ += 1;

          if (result.valid())
          {
            debug_info_.numValid_ += 1;
            // Check track length
            size_t trackLength = gsf->keys().size();
            if (trackLength > debug_info_.maxTrackLength_)
              debug_info_.maxTrackLength_ = trackLength;

            debug_info_.meanTrackLength_ += trackLength;
          }

          if (result.behindCamera())
            debug_info_.numCheirality_ += 1;
        } else {
          LOG(WARNING) << "Triangulation result is not initialized...";
        }
      }
    }
  }
  if (debug_info_.numValid_ > 0)
    debug_info_.meanTrackLength_ = debug_info_.meanTrackLength_/( (double) debug_info_.numValid_);
  else
    debug_info_.meanTrackLength_ = 0;
  if (verbosity_ >= 4) {debug_info_.print();}
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::computeSparsityStatistics() {
  gtsam::NonlinearFactorGraph graph = smoother_->getFactors();
  gtsam::GaussianFactorGraph::shared_ptr gfg = graph.linearize(state_);
  gtsam::Matrix Hessian = gfg->hessian().first;
  debug_info_.nrElementsInMatrix_ = Hessian.rows() * Hessian.cols();
  debug_info_.nrZeroElementsInMatrix_ = 0;
  for(int i = 0; i < Hessian.rows(); ++i){
    for(int j = 0; j < Hessian.cols(); ++j){
      if(fabs(Hessian(i,j))<1e-15)
        debug_info_.nrZeroElementsInMatrix_ += 1;
    }
  }
  // sanity check
  if(Hessian.rows() != Hessian.cols()) // matrix is not square
    throw std::runtime_error("computeSparsityStatistics: hessian is not a square matrix?");

  std::cout << "Hessian stats: =========== " << std::endl;
  std::cout << "rows: " << Hessian.rows() << std::endl;
  std::cout << "nrElementsInMatrix_: " << debug_info_.nrElementsInMatrix_ << std::endl;
  std::cout << "nrZeroElementsInMatrix_: " << debug_info_.nrZeroElementsInMatrix_ << std::endl;
}

/* -------------------------------------------------------------------------- */
// Debugging post optimization and estimate calculation.
void VioBackEnd::postDebug(const double& start_time) {
  if (verbosity_ >= 9) {
    computeSparsityStatistics();
  }

  if (verbosity_ >= 5) {
    std::cout << "starting computeSmartFactorStatistics" << std::endl;
  }
  if (verbosity_ >= 4) {
    computeSmartFactorStatistics();
  }
  if (verbosity_ >= 5) {
    std::cout << "finished computeSmartFactorStatistics" << std::endl;
  }
  if (verbosity_ >= 6) {
    gtsam::NonlinearFactorGraph graph =
        gtsam::NonlinearFactorGraph(smoother_->getFactors()); // clone, expensive but safer!
    std::cout << "Error before: " << graph.error(debug_info_.stateBeforeOpt)
              << "Error after: " << graph.error(state_) << std::endl;
  }
  if (verbosity_ >= 5 || log_timing_) {
    debug_info_.printTime_ = UtilsOpenCV::GetTimeInSeconds() - start_time;
  }

  if (verbosity_ >= 5 || log_timing_) {
    // order of the following is important:
    debug_info_.printTime_ -= debug_info_.extraIterationsTime_;
    debug_info_.extraIterationsTime_ -= debug_info_.updateSlotTime_;
    debug_info_.updateSlotTime_ -= debug_info_.updateTime_;
    debug_info_.updateTime_ -= debug_info_.preUpdateTime_;
    debug_info_.preUpdateTime_ -= debug_info_.factorsAndSlotsTime_;
    debug_info_.printTimes();
  }

  if (verbosity_ >= 5 || log_timing_) {
    double endTime = UtilsOpenCV::GetTimeInSeconds() - start_time;
    // sanity check:
    double endTimeFromSum = debug_info_.factorsAndSlotsTime_ +
                            debug_info_.preUpdateTime_ +
                            debug_info_.updateTime_ +
                            debug_info_.updateSlotTime_ +
                            debug_info_.extraIterationsTime_ +
                            debug_info_.printTime_;
    if (fabs(endTimeFromSum - endTime) > 1e-1) {
      std::cout << "endTime: " << endTime
                << " endTimeFromSum: " << endTimeFromSum;
      throw std::runtime_error("optimize: time measurement mismatch"
                               " (this check on timing might be too strict)");
    }
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
  gtsam::NonlinearFactorGraph tmp_graph =
      *new_imu_prior_and_other_factors;
  new_imu_prior_and_other_factors->resize(0);
  for (const auto& factor: tmp_graph) {
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
        CHECK(!boost::dynamic_pointer_cast<
              gtsam::PriorFactor<gtsam::Point3>>(*it));
        // We are not deleting a smart factor right?
        // Otherwise we need to update structure: lmk_ids_of_new_smart_factors...
        CHECK(!boost::dynamic_pointer_cast<
              SmartStereoFactor>(*it));
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
bool VioBackEnd::deleteKeyFromValues(
    const gtsam::Key& key,
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
  for (const boost::shared_ptr<gtsam::NonlinearFactor>& g: graph) {
    if (g) {
      // Found a valid factor.
      if (g->find(key) != g->end()) {
        // Whatever factor this is, it has our lmk...
        // Sanity check, this lmk has no priors right?
        CHECK(!boost::dynamic_pointer_cast<gtsam::LinearContainerFactor>(g));
        CHECK(!boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Point3>>(g));
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

} // namespace VIO.
