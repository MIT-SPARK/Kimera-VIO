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
 * @author Luca Carlone
 */

#include "VioBackEnd.h"

using namespace std;
using namespace VIO;

/* --------------------------------------------------------------------------------------- */
gtsam::Pose3 VioBackEnd::GuessPoseFromIMUmeasurements(const ImuAccGyr accGyroRaw, const Vector3 n_gravity, const bool round)
{
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
/* --------------------------------------------------------------------------------------- */
ImuBias VioBackEnd::InitializeImuBias(const ImuAccGyr accGyroRaw, const Vector3 n_gravity)
{
  std::cout << "imuBiasInitialization: currently assumes that the vehicle is stationary and upright!" << std::endl;
  Vector3 sumAccMeasurements = Vector3::Zero();
  Vector3 sumGyroMeasurements = Vector3::Zero();
  size_t nrMeasured = accGyroRaw.cols();
  for(size_t i=0; i < nrMeasured; i++){
    Vector6 accGyroRaw_i = accGyroRaw.col(i);
    // std::cout << "accGyroRaw_i: " << accGyroRaw_i.transpose() << std::endl;
    sumAccMeasurements  += accGyroRaw_i.head(3);
    sumGyroMeasurements += accGyroRaw_i.tail(3);
  }
  gtsam::imuBias::ConstantBias
  imuInit(sumAccMeasurements/double(nrMeasured) + n_gravity,
      sumGyroMeasurements/double(nrMeasured));

  return imuInit;
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::initializeStateAndSetPriors(const Timestamp timestamp_kf_nsec, const Pose3 initialPose, const ImuAccGyr accGyroRaw)
{
  Vector3 localGravity = initialPose.rotation().inverse().matrix() * imuParams_->n_gravity;
  initializeStateAndSetPriors(timestamp_kf_nsec, initialPose, Vector3::Zero(), InitializeImuBias(accGyroRaw, localGravity));
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::initializeStateAndSetPriors(const Timestamp timestamp_kf_nsec, const Pose3 initialPose, const Vector3 initialVel, const ImuBias initialBias)
{
  timestamp_kf_ = UtilsOpenCV::NsecToSec(timestamp_kf_nsec);

  W_Pose_Blkf_ = initialPose;
  W_Vel_Blkf_ = initialVel;
  imu_bias_lkf_ = initialBias;
  imu_bias_prev_kf_ = initialBias;

  std::cout << "Initialized state: " << std::endl;
  W_Pose_Blkf_.print("Initial pose");
  std::cout << "\n Initial vel: " << W_Vel_Blkf_.transpose() << std::endl;
  imu_bias_lkf_.print("Initial bias: \n");

  // Cant add inertial prior factor until we have a state measurement
  addInitialPriorFactors(cur_id_, imu_bias_lkf_.vector());

  new_values_.insert(gtsam::Symbol('x', cur_id_), W_Pose_Blkf_);
  new_values_.insert(gtsam::Symbol('v', cur_id_), W_Vel_Blkf_);
  new_values_.insert(gtsam::Symbol('b', cur_id_), imu_bias_lkf_);

  optimize(cur_id_, vioParams_.numOptimize_);
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addInitialPriorFactors(const FrameId& frame_id, const ImuAccGyr& imu_accgyr)
{
  // Set initial covariance for inertial factors
  // W_Pose_Blkf_ set by motion capture to start with
  Matrix3 B_Rot_W = W_Pose_Blkf_.rotation().matrix().transpose();

  // Set initial pose uncertainty: constrain mainly position and global yaw.
  // roll and pitch is observable, therefore low variance.
  Matrix6 pose_prior_covariance = Matrix6::Zero();
  pose_prior_covariance.diagonal()[0] = vioParams_.initialRollPitchSigma_ * vioParams_.initialRollPitchSigma_;
  pose_prior_covariance.diagonal()[1] = vioParams_.initialRollPitchSigma_ * vioParams_.initialRollPitchSigma_;
  pose_prior_covariance.diagonal()[2] = vioParams_.initialYawSigma_ * vioParams_.initialYawSigma_;
  pose_prior_covariance.diagonal()[3] = vioParams_.initialPositionSigma_ * vioParams_.initialPositionSigma_;
  pose_prior_covariance.diagonal()[4] = vioParams_.initialPositionSigma_ * vioParams_.initialPositionSigma_;
  pose_prior_covariance.diagonal()[5] = vioParams_.initialPositionSigma_ * vioParams_.initialPositionSigma_;

  // Rotate initial uncertainty into local frame, where the uncertainty is specified.
  pose_prior_covariance.topLeftCorner(3,3) =
      B_Rot_W * pose_prior_covariance.topLeftCorner(3,3) * B_Rot_W.transpose();

  // Add pose prior.
  gtsam::SharedNoiseModel noise_init_pose =
      gtsam::noiseModel::Gaussian::Covariance(pose_prior_covariance);
  new_imu_and_prior_factors_.push_back(
      boost::make_shared<gtsam::PriorFactor<gtsam::Pose3> >(
          gtsam::Symbol('x', frame_id), W_Pose_Blkf_, noise_init_pose));

  // Add initial velocity priors.
  gtsam::SharedNoiseModel initialVelocityPriorNoise =
      gtsam::noiseModel::Isotropic::Sigma(3, vioParams_.initialVelocitySigma_);
  new_imu_and_prior_factors_.push_back(
      boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
          gtsam::Symbol('v', frame_id), W_Vel_Blkf_, initialVelocityPriorNoise));

  // Add initial bias priors:
  Vector6 prior_biasSigmas;
  prior_biasSigmas.head<3>().setConstant(vioParams_.initialAccBiasSigma_);
  prior_biasSigmas.tail<3>().setConstant(vioParams_.initialGyroBiasSigma_);
  gtsam::SharedNoiseModel imu_bias_prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(prior_biasSigmas);
  new_imu_and_prior_factors_.push_back(
      boost::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
          gtsam::Symbol('b', frame_id), imu_bias_lkf_, imu_bias_prior_noise));

  if (verbosity_ >= 7) {std::cout << "Added initial priors for frame " << frame_id << std::endl;}
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addVisualInertialStateAndOptimize(
    const Timestamp timestamp_kf_nsec,
    const StatusSmartStereoMeasurements statusSmartStereoMeasurements_kf, // vision data
    ImuStamps imu_stamps, ImuAccGyr imu_accgyr,
    boost::optional<gtsam::Pose3> stereoRansacBodyPose) // inertial data
{
  debugInfo_.resetAddedFactorsStatistics();

  if (verbosity_ >= 7) { StereoVisionFrontEnd::PrintStatusStereoMeasurements(statusSmartStereoMeasurements_kf); }

  // Features and IMU line up --> do iSAM update
  last_id_ = cur_id_;
  ++cur_id_;

  timestamp_kf_ = UtilsOpenCV::NsecToSec(timestamp_kf_nsec);

  std::cout << "VIO: adding keyframe " << cur_id_ << " at timestamp:" << timestamp_kf_ << " (sec)" << std::endl;

  /////////////////// MANAGE IMU MEASUREMENTS ///////////////////////////
  // Predict next step, add initial guess
  integrateImuMeasurements(imu_stamps, imu_accgyr);
  addImuValues(cur_id_);

  // add imu factors between consecutive keyframe states
  addImuFactor(last_id_, cur_id_);

  // add between factor from RANSAC
  if(stereoRansacBodyPose){
    std::cout << "VIO: adding between " << std::endl;
    (*stereoRansacBodyPose).print();
    addBetweenFactor(last_id_, cur_id_, *stereoRansacBodyPose);
  }

  /////////////////// MANAGE VISION MEASUREMENTS ///////////////////////////
  SmartStereoMeasurements smartStereoMeasurements_kf = statusSmartStereoMeasurements_kf.second;

  // if stereo ransac failed, remove all right pixels:
  Tracker::TrackingStatus kfTrackingStatus_stereo = statusSmartStereoMeasurements_kf.first.kfTrackingStatus_stereo_;
  // if(kfTrackingStatus_stereo == Tracker::TrackingStatus::INVALID){
  //   for(size_t i = 0; i < smartStereoMeasurements_kf.size(); i++)
  //     smartStereoMeasurements_kf[i].uR = std::numeric_limits<double>::quiet_NaN();;
  //}

  // extract relevant information from stereo frame
  LandmarkIds landmarks_kf = addStereoMeasurementsToFeatureTracks(cur_id_, smartStereoMeasurements_kf);
  if (verbosity_ >= 8) { printFeatureTracks(); }

  // decide which factors to add
  Tracker::TrackingStatus kfTrackingStatus_mono = statusSmartStereoMeasurements_kf.first.kfTrackingStatus_mono_;
  switch(kfTrackingStatus_mono){
  case Tracker::TrackingStatus::LOW_DISPARITY :  // vehicle is not moving
    if (verbosity_ >= 7) {printf("Add zero velocity and no motion factors\n");}
    addZeroVelocityPrior(cur_id_);
    addNoMotionFactor(last_id_, cur_id_);
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

  imu_bias_prev_kf_ = imu_bias_lkf_; // this lags 1 step behind to mimic hw
  optimize(cur_id_, vioParams_.numOptimize_);
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::integrateImuMeasurements(const ImuStamps& imu_stamps, const ImuAccGyr& imu_accgyr)
{
  if (imu_stamps.size() >= 2){
    if (!pim_)
      pim_ = std::make_shared<PreintegratedImuMeasurements>(imuParams_, imu_bias_prev_kf_);
    else
      pim_->resetIntegrationAndSetBias(imu_bias_prev_kf_);

    for (int i = 0; i < imu_stamps.size()-1; ++i)
    {
      Vector3 measured_acc = imu_accgyr.block<3,1>(0,i);
      Vector3 measured_omega = imu_accgyr.block<3,1>(3,i);
      double delta_t = UtilsOpenCV::NsecToSec(imu_stamps(i+1) - imu_stamps(i));
      pim_->integrateMeasurement(measured_acc, measured_omega, delta_t);
    }
  }else{
    throw std::runtime_error("integrateImuMeasurements: no imu data found");
  }
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addImuValues(const FrameId& cur_id)
{
  gtsam::NavState navstate_lkf(W_Pose_Blkf_, W_Vel_Blkf_);
  gtsam::NavState navstate_k = pim_->predict(navstate_lkf, imu_bias_lkf_);

  debugInfo_.navstate_k_ = navstate_k;

  // Update state with initial guess
  new_values_.insert(gtsam::Symbol('x', cur_id), navstate_k.pose());
  new_values_.insert(gtsam::Symbol('v', cur_id), navstate_k.velocity());
  new_values_.insert(gtsam::Symbol('b', cur_id), imu_bias_lkf_);
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addImuFactor(const FrameId& from_id, const FrameId& to_id)
{
#ifdef USE_COMBINED_IMU_FACTOR
  new_imu_and_prior_factors_.push_back(
      boost::make_shared<gtsam::CombinedImuFactor>(
          gtsam::Symbol('x', from_id), gtsam::Symbol('v', from_id),
          gtsam::Symbol('x', to_id), gtsam::Symbol('v', to_id),
          gtsam::Symbol('b', from_id),
          gtsam::Symbol('b', to_id),
          *pim_));
#else
  new_imu_and_prior_factors_.push_back(
      boost::make_shared<gtsam::ImuFactor>(
          gtsam::Symbol('x', from_id), gtsam::Symbol('v', from_id),
          gtsam::Symbol('x', to_id), gtsam::Symbol('v', to_id),
          gtsam::Symbol('b', from_id), *pim_));

  gtsam::imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
  // factor to discretize and move normalize by the interval between measurements:
  double d = sqrt(pim_->deltaTij()) / vioParams_.nominalImuRate_; // 1/sqrt(nominalImuRate_) to discretize, then sqrt(pim_->deltaTij()/nominalImuRate_) to count the nr of measurements
  Vector6 biasSigmas;
  biasSigmas.head<3>().setConstant(d * vioParams_.accBiasSigma_);
  biasSigmas.tail<3>().setConstant(d * vioParams_.gyroBiasSigma_);
  gtsam::SharedNoiseModel bias_noise_model = gtsam::noiseModel::Diagonal::Sigmas(biasSigmas);

  new_imu_and_prior_factors_.push_back(
      boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> >(
          gtsam::Symbol('b', from_id), gtsam::Symbol('b', to_id), zero_bias, bias_noise_model));
#endif

  debugInfo_.imuR_lkf_kf = pim_->deltaRij();
  debugInfo_.numAddedImuF_++;

  // reset preintegration
  pim_.reset();
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
LandmarkIds VioBackEnd::addStereoMeasurementsToFeatureTracks(int frameNum, const SmartStereoMeasurements& stereoMeasurements_kf)
{
  //TODO: feature tracks will grow unbounded.
  LandmarkIds landmarks_kf;
  landmarks_kf.reserve(stereoMeasurements_kf.size());
  // Pack information in landmark structure
  for (size_t i = 0; i < stereoMeasurements_kf.size(); ++i)
  {
    LandmarkId landmarkId_kf_i = stereoMeasurements_kf[i].first;
    StereoPoint2 stereo_px_i = stereoMeasurements_kf[i].second;
    if(landmarkId_kf_i==-1) // we filtered them out in the StereoTracker, so this should not happen
      throw std::runtime_error("addStereoMeasurementsToFeatureTracks: landmarkId_kf_i==-1?");

    landmarks_kf.push_back(landmarkId_kf_i); // thinner structure that only keeps landmarkIds
    // Add features to vio->featureTracks_ if they are new
    auto lm_it = featureTracks_.find(landmarkId_kf_i);
    if (lm_it == featureTracks_.end()) // new feature
    {
      if(verbosity_ >= 7) { std::cout << "Adding landmark: " << landmarkId_kf_i << std::endl; }
      featureTracks_.insert(std::make_pair(landmarkId_kf_i, FeatureTrack(frameNum, stereo_px_i)));
      ++landmark_count_;
    }
    // @TODO: It seems that this else condition does not help -- conjecture that it creates long feature tracks with low information (i.e. we're not moving)
    // This is problematic in conjunction with our landmark selection mechanism which prioritizes long feature tracks
    else // add observation to existing landmark
    {
      (*lm_it).second.obs_.push_back(std::make_pair(frameNum, stereo_px_i));
    }
  }
  return landmarks_kf;
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addZeroVelocityPrior(const FrameId& frame_id)
{
  new_imu_and_prior_factors_.push_back(
      boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
          gtsam::Symbol('v', frame_id), gtsam::Vector3::Zero(), zeroVelocityPriorNoise_));
  if (verbosity_ >= 7) {std::cout << "No motion detected, adding zero velocity prior" << std::endl;}
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addBetweenFactor(const FrameId& from_id, const FrameId& to_id, const gtsam::Pose3 from_id_POSE_to_id)
{
  //  Vector6 sigmas;
  //  sigmas.head<3>().setConstant(0.05);
  //  sigmas.tail<3>().setConstant(0.1);
  //  gtsam::SharedNoiseModel betweenNoise_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  Vector6 precisions;
  precisions.head<3>().setConstant(vioParams_.betweenRotationPrecision_);
  precisions.tail<3>().setConstant(vioParams_.betweenTranslationPrecision_);
  gtsam::SharedNoiseModel betweenNoise_ = gtsam::noiseModel::Diagonal::Precisions(precisions);

  new_imu_and_prior_factors_.push_back(
      boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          gtsam::Symbol('x', from_id), gtsam::Symbol('x', to_id), from_id_POSE_to_id, betweenNoise_));

  debugInfo_.numAddedBetweenStereoF_++;

  if (verbosity_ >= 7) {std::cout << "addBetweenFactor" << std::endl;}
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addNoMotionFactor(const FrameId& from_id, const FrameId& to_id)
{
  new_imu_and_prior_factors_.push_back(
      boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          gtsam::Symbol('x', from_id), gtsam::Symbol('x', to_id), Pose3(), noMotionPriorNoise_));

  debugInfo_.numAddedNoMotionF_++;

  if (verbosity_ >= 7) {std::cout << "No motion detected, adding no relative motion prior" << std::endl;}
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addConstantVelocityFactor(const FrameId& from_id, const FrameId& to_id)
{
  new_imu_and_prior_factors_.push_back(
      boost::make_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
          gtsam::Symbol('v', from_id), gtsam::Symbol('v', to_id), gtsam::Vector3::Zero(), constantVelocityPriorNoise_));

  debugInfo_.numAddedConstantVelF_++;

  if (verbosity_ >= 7) {std::cout << "adding constant velocity factor" << std::endl;}
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addLandmarksToGraph(LandmarkIds landmarks_kf)
{
  // Add selected landmarks to graph:
  int n_new_landmarks = 0;
  int n_updated_landmarks = 0;
  debugInfo_.numAddedSmartF_ += landmarks_kf.size();

  for (const LandmarkId lm_id : landmarks_kf)
  {
    FeatureTrack& ft = featureTracks_.at(lm_id);
    if(ft.obs_.size()<2) // we only insert feature tracks of length at least 2 (otherwise uninformative)
      continue;

    if(!ft.in_ba_graph_)
    {
      addLandmarkToGraph(lm_id, ft);
      ++n_new_landmarks;
    }
    else
    {
      const std::pair<FrameId, StereoPoint2> obs_kf = ft.obs_.back();

      if(obs_kf.first != cur_id_) // sanity check
        throw std::runtime_error("addLandmarksToGraph: last obs is not from the current keyframe!\n");

      updateLandmarkInGraph(lm_id, obs_kf);
      ++n_updated_landmarks;
    }
  }
  if (verbosity_ >= 7)
  {
    std::cout << "Added " << n_new_landmarks << " new landmarks" << std::endl;
    std::cout << "Updated " << n_updated_landmarks << " landmarks in graph" << std::endl;
  }
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addLandmarkToGraph(LandmarkId lm_id, FeatureTrack& ft)
{
  if(ft.in_ba_graph_)
    throw std::runtime_error("addLandmarkToGraph: feature already in the graph!");

  ft.in_ba_graph_ = true;

  // We use a unit pinhole projection camera for the smart factors to be more efficient.
  SmartStereoFactor::shared_ptr new_factor(
      new SmartStereoFactor(smart_noise_, smartFactorsParams_, B_Pose_leftCam_));

  if (verbosity_ >= 9) {std::cout << "Adding landmark with: " << ft.obs_.size() << " landmarks to graph, with keys: ";}
  if (verbosity_ >= 9){new_factor->print();}

  // add observations to smart factor
  for (const std::pair<FrameId,StereoPoint2>& obs : ft.obs_)
  {
    new_factor->add(obs.second, gtsam::Symbol('x', obs.first), stereoCal_);
    if (verbosity_ >= 9) {std::cout << " " <<  obs.first;}
  }
  if (verbosity_ >= 9) {std::cout << std::endl;}
  // add new factor to suitable structures:
  new_smart_factors_.insert(std::make_pair(lm_id, new_factor));
  old_smart_factors_.insert(std::make_pair(lm_id, std::make_pair(new_factor, -1)));
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::updateLandmarkInGraph(const LandmarkId lm_id, const std::pair<FrameId, StereoPoint2>& newObs)
{
  // Update existing smart-factor
  auto old_smart_factors_it = old_smart_factors_.find(lm_id);
  if (old_smart_factors_it == old_smart_factors_.end())
    throw std::runtime_error("updateLandmarkInGraph: landmark not found in old_smart_factors_\n");

  SmartStereoFactor::shared_ptr old_factor = old_smart_factors_it->second.first;
  SmartStereoFactor::shared_ptr new_factor = boost::make_shared<SmartStereoFactor>(*old_factor); // clone old factor
  new_factor->add(newObs.second, gtsam::Symbol('x', newObs.first), stereoCal_);

  // update the factor
  if (old_smart_factors_it->second.second != -1){// if slot is still -1, it means that the factor has not been inserted yet in the graph
    new_smart_factors_.insert(std::make_pair(lm_id, new_factor));
  }else{
    throw std::runtime_error("updateLandmarkInGraph: when calling update the slot should be already != -1! \n");
  }
  old_smart_factors_it->second.first = new_factor;
  if (verbosity_ >= 8) {std::cout << "updateLandmarkInGraph: added observation to point: " << lm_id << std::endl;}
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::optimize(const FrameId& cur_id, const int max_extra_iterations)
{
  if (!smoother_.get())
    throw std::runtime_error("optimize: Incremental smoother is a null pointer\n");

  // only for statistics and debugging
  double startTime, endTime;
  debugInfo_.resetTimes();
  if (verbosity_ >= 5) startTime = UtilsOpenCV::GetTimeInSeconds();

  // We need to remove all smart factors that have new observations.
  std::vector<size_t> delete_slots;
  std::vector<Key> new_smart_factors_lmkID_tmp;
  gtsam::NonlinearFactorGraph new_factors_tmp;
  for (auto& s : new_smart_factors_)
  {
    new_factors_tmp.push_back(s.second); // push back the factor
    new_smart_factors_lmkID_tmp.push_back(s.first); // these are the lmk id of the factors to add to graph
    const auto& it = old_smart_factors_.find(s.first);
    if(it->second.second != -1) // get current slot (if factor is already there it must be deleted)
      delete_slots.push_back(it->second.second);
  }
  // add also other factors (imu, priors)
  new_factors_tmp.push_back(new_imu_and_prior_factors_.begin(), new_imu_and_prior_factors_.end());
  new_imu_and_prior_factors_.resize(0); // clean up stuff which is not in new_factors_tmp

  if (verbosity_ >= 5) debugInfo_.factorsAndSlotsTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;

  if (verbosity_ >= 5){
    std::cout << "iSAM2 update with " << new_factors_tmp.size() << " new factors"
        << " , " << new_values_.size() << " new values "
        << " , and " << delete_slots.size() << " delete indices" << std::endl;
  }
  if (verbosity_ >= 5){ // get state before optimization to compute error
    debugInfo_.stateBeforeOpt = gtsam::Values(state_);
    BOOST_FOREACH(const gtsam::Values::ConstKeyValuePair& key_value, new_values_) {
      debugInfo_.stateBeforeOpt.insert(key_value.key, key_value.value);
    }
  }
  if (verbosity_ >= 8){ showSmootherInfo(new_factors_tmp,delete_slots,"Smoother status before update:",verbosity_ >= 9); }

  // recreate the graph before marginalization
  if (verbosity_ >= 5){
    debugInfo_.graphBeforeOpt = gtsam::NonlinearFactorGraph();
    for(size_t i = 0; i<smoother_->getFactors().size(); i++){
      if(std::find(delete_slots.begin(), delete_slots.end(), i) == delete_slots.end()) // if not to be deleted
        debugInfo_.graphBeforeOpt.push_back(smoother_->getFactors().at(i));
    }
    for (auto& f : new_factors_tmp)
      debugInfo_.graphBeforeOpt.push_back(f);
  }

  // Compute iSAM update.
  Smoother::Result result;
  try
  {
    std::map<Key, double> timestamps;
    for(auto keyValue : new_values_)
      timestamps[keyValue.key] = timestamp_kf_; // for the latest pose, velocity, and bias

    if (verbosity_ >= 5) {std::cout << "starting first update:" << std::endl;}
    if (verbosity_ >= 5) debugInfo_.preUpdateTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
    result = smoother_->update(new_factors_tmp, new_values_,timestamps, delete_slots);
    if (verbosity_ >= 5) debugInfo_.updateTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
    if (verbosity_ >= 5) {std::cout << "finished first update!" << std::endl;}

    // reset everything for next round
    new_smart_factors_.clear();
    new_values_.clear();
  }
  catch (const gtsam::IndeterminantLinearSystemException& e)
  {
    std::cerr << e.what() << std::endl;
    gtsam::Key var = e.nearbyVariable();
    gtsam::Symbol symb(var);
    std::cout << "ERROR: Variable has type '" << symb.chr() << "' "
        << "and index " << symb.index() << std::endl;
    smoother_->getFactors().print("smoother's factors:\n");
    throw;
  }
  // update slots of smart factors:
  if (verbosity_ >= 5) {std::cout << "starting findSmartFactorsSlots" << std::endl;}
#ifdef INCREMENTAL_SMOOTHER
  findSmartFactorsSlots(new_smart_factors_lmkID_tmp);
#else
  findSmartFactorsSlotsSlow(new_smart_factors_lmkID_tmp);
#endif
  if (verbosity_ >= 5) {std::cout << "finished findSmartFactorsSlots" << std::endl;}

  if (verbosity_ >= 5) debugInfo_.updateSlotTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;

  // Do some more iterations.
  for  (size_t n_iter = 1; n_iter < max_extra_iterations; ++n_iter)
  {
    std::cout << "Doing extra iteration nr: " << n_iter << std::endl;
    try
    {
      result = smoother_->update();
    }
    catch(const gtsam::IndeterminantLinearSystemException& e)
    {
      std::cout << "ERROR: " << e.what() << std::endl;
      gtsam::Key var = e.nearbyVariable();
      gtsam::Symbol symb(var);
      std::cout << "ERROR: Variable has type '" << symb.chr() << "' "
          << "and index " << symb.index() << std::endl;
      smoother_->getFactors().print("smoother's factors:\n");
      throw;
    }
  }
  if (verbosity_ >= 5) debugInfo_.extraIterationsTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;

  if (verbosity_ >= 5) {std::cout << "starting calculateEstimate" << std::endl;}
  // Get states we need for next iteration
  state_ = smoother_->calculateEstimate();
  W_Pose_Blkf_ = state_.at<Pose3>(gtsam::Symbol('x', cur_id));
  W_Vel_Blkf_ = state_.at<Vector3>(gtsam::Symbol('v', cur_id));
  imu_bias_lkf_ = state_.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', cur_id));
  if (verbosity_ >= 5) {std::cout << "finished calculateEstimate!" << std::endl;}
  if (verbosity_ >= 9) { computeSparsityStatistics(); }

  // DEBUG:
  if (verbosity_ >= 5) {std::cout << "starting computeSmartFactorStatistics" << std::endl;}
  if (verbosity_ >= 4) {computeSmartFactorStatistics(); }
  if (verbosity_ >= 5) {std::cout << "finished computeSmartFactorStatistics" << std::endl;}
  if (verbosity_ >= 6){
    gtsam::NonlinearFactorGraph graph = gtsam::NonlinearFactorGraph(smoother_->getFactors()); // clone, expensive but safer!
    std::cout << "Error before: " << graph.error(debugInfo_.stateBeforeOpt) << std::endl;
    std::cout << "Error after: " << graph.error(state_) << std::endl;
  }
  if (verbosity_ >= 5) debugInfo_.printTime_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
  if (verbosity_ >= 5)
  {
    // order of the following is important:
    debugInfo_.printTime_ -= debugInfo_.extraIterationsTime_;
    debugInfo_.extraIterationsTime_ -= debugInfo_.updateSlotTime_;
    debugInfo_.updateSlotTime_ -= debugInfo_.updateTime_;
    debugInfo_.updateTime_ -= debugInfo_.preUpdateTime_;
    debugInfo_.preUpdateTime_ -= debugInfo_.factorsAndSlotsTime_;
    debugInfo_.printTimes();
  }
  if (verbosity_ >= 5){
    endTime = UtilsOpenCV::GetTimeInSeconds() - startTime;
    // sanity check:
    double endTimeFromSum = debugInfo_.factorsAndSlotsTime_ + debugInfo_.preUpdateTime_ +
        debugInfo_.updateTime_ + debugInfo_.updateSlotTime_ + debugInfo_.extraIterationsTime_ + debugInfo_.printTime_;
    if(fabs(endTimeFromSum - endTime)>1e-1){
      std::cout << "endTime: " << endTime << " endTimeFromSum: " << endTimeFromSum;
      throw std::runtime_error("optimize: time measurement mismatch (this check on timing might be too strict)");
    }
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::findSmartFactorsSlotsSlow(const std::vector<Key> new_smart_factors_lmkID_tmp) {

  // OLD INEFFICIENT VERSION:
  const gtsam::NonlinearFactorGraph& graphFactors = smoother_->getFactors();

  vector<LandmarkId> landmarksToRemove;
  for (auto& it : old_smart_factors_)
  {
    bool found = false;
    for (size_t slot = 0; slot < graphFactors.size(); ++slot)
    {
      const gtsam::NonlinearFactor::shared_ptr& f = graphFactors[slot];
      if(f) {
        SmartStereoFactor::shared_ptr smartfactor = boost::dynamic_pointer_cast<SmartStereoFactor>(graphFactors.at(slot));
        if (smartfactor && it.second.first == smartfactor) {
          it.second.second = slot;
          found = true;
          break;
        }
      }
    }
    if (!found) { // if it's not in the graph we can delete if from the map
      if (verbosity_ >= 6) { std::cout << "Smart factor with id: " << it.first << " not found" << std::endl; }
      landmarksToRemove.push_back(it.first);
    }
  }
  // erase factors that are no longer there
  for (auto& i : landmarksToRemove)
    old_smart_factors_.erase(i);
}
#ifdef INCREMENTAL_SMOOTHER
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::findSmartFactorsSlots(const std::vector<Key> new_smart_factors_lmkID_tmp) {

  gtsam::ISAM2Result result = smoother_->getISAM2Result();
  // Simple version of find smart factors
  for (size_t i = 0; i < new_smart_factors_lmkID_tmp.size(); ++i) // for each landmark id currently observed (just re-added to graph)
  {
    const auto& it = old_smart_factors_.find(new_smart_factors_lmkID_tmp.at(i)); // find the entry in old_smart_factors_
    it->second.second = result.newFactorsIndices.at(i); // update slot using isam2 indices
  }
}
#endif
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
//std::vector<std::pair<int, std::vector<double>>> Vio::getTriangulationPoints(bool isUseGroundtruth)
//{
//  // Find the location of triangularized features
//  std::vector<std::pair<int, std::vector<double>>> triPoints;
//  int numSF = 0;
//  for (auto id : frame_k_->landmarkIDs_ ) // for each landmark
//  {
//    auto s = old_smart_factors_.find(id);
//    if (s != old_smart_factors_.end()) // if we found the landmark
//    {
//      auto sf = s->second.first;
//      // s.second->print();
//      numSF += 1;
//      gtsam::TriangulationResult result;
//      if (isUseGroundtruth)
//        result = sf->point(gtValues_);
//      else
//        result = sf->point(state_);
//
//      if (result)
//      {
//        gtsam::Vector3 pt;
//        if (isUseGroundtruth)
//          pt = *sf->point(gtValues_);
//        else
//          pt = *sf->point(state_);
//
//        std::vector<double> ptd = {pt(0), pt(1), pt(2)};
//        triPoints.push_back(std::make_pair(id, ptd));
//      }
//    }
//  }
//  if (verbosity_ >= 5) {std::cout << "Num SF: " << numSF << " Num Tri: " << triPoints.size() << " Groundtruth: " << isUseGroundtruth << std::endl;}
//  return triPoints;
//}
//
////---------------------------------------------------------------------------
////int64_t Vio::featureTrackSort()
//{
//// The following loop is finding the number of observations for each landmark seen in the current keyframe,
//	// to pick the top N visible landmarks with most observations.
//	std::vector<std::pair<size_t, int>> lm_obs_count;
//	for (size_t i = 0; i < landmarks_kf.size(); ++i)
//	{
//		LandmarkId lm_id = landmarks_kf.at(i);
//		if (lm_id == -1)
//			continue;
//
//		auto lm_it = featureTracks_.find(lm_id);
//		if (lm_it == featureTracks_.end()){
//			if (verbosity_ >= 2) {std::cout << "FeatureTrack " << lm_id << " not in table" << std::endl;}
//			throw std::runtime_error("addLandmarksToGraph: feature track not found in table");
//		}
//
//		// If landmark is already in graph, check if it is marked as degenerate:
//		// TODO: make this more efficient, maybe add a flag in the landmark or
//		// completely get rid of landmarks and accumule observations directly in smart factors.
//		const FeatureTrack& ft = lm_it->second; // this contains the observations to that landmark
//		// @TODO: Figure out masking degeneracy
//		// if (ft.in_ba_graph_)
//		// {
//		// 	auto sf_it = old_smart_factors_.find(lm_id);  // TODO this was frame.id_???
//		// 	if (sf_it != old_smart_factors_.end())
//		// 	{
//		// 	    std::cout << "Num keys: " << sf_it->second.first->keys().size() << std::endl;
//		// 	    if (sf_it->second.first->isDegenerate())
//		// 		{
//		// 		    std::cout << "FeatureTrack was degenerate" << std::endl;
//		// 			continue;
//		// 		}
//		// 	}
//		// }
//		lm_obs_count.push_back(std::make_pair(i, ft.obs_.size()));
//	}
//
//	// // select at most FLAGS_vio_max_landmarks_per_frame landmarks to be used in the current frame
//	// size_t max_landmarks = numFeat_;
//	// if (lm_obs_count.size() > max_landmarks)
//	// {
//	//   std::sort(lm_obs_count.begin(), lm_obs_count.end(),
//	//       [](const std::pair<size_t, int>& lhs, const std::pair<size_t, int>& rhs)
//	//       { return lhs.second > rhs.second; });
//	//   lm_obs_count.erase(lm_obs_count.begin() + max_landmarks, lm_obs_count.end());
//	// }
//}
