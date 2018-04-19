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

/* -------------------------------------------------------------------------- */
VioBackEnd::VioBackEnd(const Pose3& leftCamPose,
                       const Cal3_S2& leftCameraCalRectified,
                       const double& baseline,
                       const VioBackEndParams& vioParams):
  B_Pose_leftCam_(leftCamPose),
  stereoCal_(boost::make_shared<gtsam::Cal3_S2Stereo>(
               leftCameraCalRectified.fx(),
               leftCameraCalRectified.fy(), leftCameraCalRectified.skew(),
               leftCameraCalRectified.px(), leftCameraCalRectified.py(),
               baseline)),
  vioParams_(vioParams),
  imu_bias_lkf_(ImuBias()), imu_bias_prev_kf_(ImuBias()),
  W_Vel_Blkf_(Vector3::Zero()), W_Pose_Blkf_(Pose3()),
  last_id_(-1), cur_id_(0),
  verbosity_(5), landmark_count_(0) {

  // SMART PROJECTION FACTORS SETTINGS
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(
                                    3, vioParams_.smartNoiseSigma_); //  vio_smart_reprojection_err_thresh / cam_->fx());
  // smart_noise_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), model);
  smart_noise_ = model;
  smartFactorsParams_ = SmartFactorParams( // JACOBIAN_SVD, IGNORE_DEGENERACY
                                           gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY, false, true); // ThrowCherality = false, verboseCherality = true
  smartFactorsParams_.setRankTolerance(vioParams_.rankTolerance_);
  smartFactorsParams_.setLandmarkDistanceThreshold(vioParams_.landmarkDistanceThreshold_);
  smartFactorsParams_.setRetriangulationThreshold(vioParams_.retriangulationThreshold_);
  smartFactorsParams_.setDynamicOutlierRejectionThreshold(vioParams_.outlierRejection_);

  // IMU FACTORS SETTINGS
  imuParams_ = boost::make_shared<PreintegratedImuMeasurements::Params>(vioParams.n_gravity_);
  imuParams_->gyroscopeCovariance =
      std::pow(vioParams.gyroNoiseDensity_, 2.0) * Eigen::Matrix3d::Identity();
  imuParams_->accelerometerCovariance =
      std::pow(vioParams.accNoiseDensity_, 2.0) * Eigen::Matrix3d::Identity();
  imuParams_->integrationCovariance =
      std::pow(vioParams.imuIntegrationSigma_, 2.0) * Eigen::Matrix3d::Identity();
#ifdef USE_COMBINED_IMU_FACTOR
  imuParams_->biasAccCovariance =
      std::pow(vioParams.accBiasSigma_, 2.0) * Eigen::Matrix3d::Identity();
  imuParams_->biasOmegaCovariance =
      std::pow(vioParams.gyroBiasSigma_, 2.0) * Eigen::Matrix3d::Identity();
#endif
  imuParams_->use2ndOrderCoriolis = false; // TODO: expose this parameter

  // NO MOTION FACTORS SETTINGS:
  zeroVelocityPriorNoise_ = gtsam::noiseModel::Isotropic::Sigma(3, vioParams.zeroVelocitySigma_);

  Vector6 sigmas;
  sigmas.head<3>().setConstant(vioParams.noMotionRotationSigma_);
  sigmas.tail<3>().setConstant(vioParams.noMotionPositionSigma_);
  noMotionPriorNoise_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  // CONSTANT VELOCITY FACTORS SETTINGS:
  constantVelocityPriorNoise_ = gtsam::noiseModel::Isotropic::Sigma(3, vioParams.constantVelSigma_);

#ifdef INCREMENTAL_SMOOTHER
  // iSAM2 SETTINGS
  gtsam::ISAM2GaussNewtonParams gauss_newton_params;
  gauss_newton_params.wildfireThreshold = -1.0;

  // gauss_newton_params.setWildfireThreshold(0.001);
  gtsam::ISAM2DoglegParams dogleg_params;
  // dogleg_params.setVerbose(false); // only for debugging.
  gtsam::ISAM2Params isam_param;

  if (vioParams_.useDogLeg_)
    isam_param.optimizationParams = dogleg_params;
  else
    isam_param.optimizationParams = gauss_newton_params;

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
  isam_param.setCacheLinearizedFactors(false);
  isam_param.setEvaluateNonlinearError(true);

  isam_param.relinearizeThreshold = vioParams.relinearizeThreshold_;
  isam_param.relinearizeSkip = vioParams.relinearizeSkip_;
  // isam_param.enablePartialRelinearizationCheck = true;
  isam_param.findUnusedFactorSlots = true;
  // isam_param.cacheLinearizedFactors = true;
  // isam_param.enableDetailedResults = true;   // only for debugging.
  isam_param.factorization = gtsam::ISAM2Params::CHOLESKY; // QR
  isam_param.print("isam_param");

  //isam_param.evaluateNonlinearError = true;  // only for debugging.

  smoother_ = std::make_shared<Smoother>(vioParams.horizon_, isam_param);
#else // BATCH SMOOTHER
  gtsam::LevenbergMarquardtParams lmParams;
  lmParams.setlambdaInitial(0.0); // same as GN
  lmParams.setlambdaLowerBound(0.0); // same as GN
  lmParams.setlambdaUpperBound(0.0); // same as GN)
  smoother_ = std::make_shared<Smoother>(vioParams.horizon_,lmParams);
#endif

  // reset debug info
  debugInfo_.resetSmartFactorsStatistics();
  debugInfo_.resetTimes();
  debugInfo_.resetAddedFactorsStatistics();
  debugInfo_.nrElementsInMatrix_ = 0;
  debugInfo_.nrZeroElementsInMatrix_ = 0;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::initializeStateAndSetPriors(const Timestamp& timestamp_kf_nsec,
                                             const Pose3& initialPose,
                                             const ImuAccGyr& accGyroRaw) {
  Vector3 localGravity = initialPose.rotation().inverse().matrix() * imuParams_->n_gravity;
  initializeStateAndSetPriors(timestamp_kf_nsec, initialPose, Vector3::Zero(), InitializeImuBias(accGyroRaw, localGravity));
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::initializeStateAndSetPriors(const Timestamp& timestamp_kf_nsec,
                                             const Pose3& initialPose,
                                             const Vector3& initialVel,
                                             const ImuBias& initialBias) {
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

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addInitialPriorFactors(const FrameId& frame_id,
                                        const ImuAccGyr& imu_accgyr) {
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
  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::PriorFactor<gtsam::Pose3> >(
          gtsam::Symbol('x', frame_id), W_Pose_Blkf_, noise_init_pose));

  // Add initial velocity priors.
  gtsam::SharedNoiseModel initialVelocityPriorNoise =
      gtsam::noiseModel::Isotropic::Sigma(3, vioParams_.initialVelocitySigma_);
  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
          gtsam::Symbol('v', frame_id), W_Vel_Blkf_, initialVelocityPriorNoise));

  // Add initial bias priors:
  Vector6 prior_biasSigmas;
  prior_biasSigmas.head<3>().setConstant(vioParams_.initialAccBiasSigma_);
  prior_biasSigmas.tail<3>().setConstant(vioParams_.initialGyroBiasSigma_);
  gtsam::SharedNoiseModel imu_bias_prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(prior_biasSigmas);
  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
          gtsam::Symbol('b', frame_id), imu_bias_lkf_, imu_bias_prior_noise));

  if (verbosity_ >= 7) {std::cout << "Added initial priors for frame " << frame_id << std::endl;}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addVisualInertialStateAndOptimize(
    const Timestamp& timestamp_kf_nsec,
    const StatusSmartStereoMeasurements& status_smart_stereo_measurements_kf,
    const ImuStamps& imu_stamps, const ImuAccGyr& imu_accgyr,
    const LandmarkIds& mesh_lmk_ids_ground_cluster,
    boost::optional<gtsam::Pose3> stereo_ransac_body_pose) {
  debugInfo_.resetAddedFactorsStatistics();

  if (verbosity_ >= 7) { StereoVisionFrontEnd::PrintStatusStereoMeasurements(status_smart_stereo_measurements_kf); }

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
  if(stereo_ransac_body_pose){
    std::cout << "VIO: adding between " << std::endl;
    (*stereo_ransac_body_pose).print();
    addBetweenFactor(last_id_, cur_id_, *stereo_ransac_body_pose);
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
  addStereoMeasurementsToFeatureTracks(cur_id_,
                                       smartStereoMeasurements_kf,
                                       &landmarks_kf);

  if (verbosity_ >= 8) { printFeatureTracks(); }

  // decide which factors to add
  Tracker::TrackingStatus kfTrackingStatus_mono = status_smart_stereo_measurements_kf.first.kfTrackingStatus_mono_;
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

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::integrateImuMeasurements(const ImuStamps& imu_stamps,
                                          const ImuAccGyr& imu_accgyr) {
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

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addImuValues(const FrameId& cur_id) {
  gtsam::NavState navstate_lkf(W_Pose_Blkf_, W_Vel_Blkf_);
  gtsam::NavState navstate_k = pim_->predict(navstate_lkf, imu_bias_lkf_);

  debugInfo_.navstate_k_ = navstate_k;

  // Update state with initial guess
  new_values_.insert(gtsam::Symbol('x', cur_id), navstate_k.pose());
  new_values_.insert(gtsam::Symbol('v', cur_id), navstate_k.velocity());
  new_values_.insert(gtsam::Symbol('b', cur_id), imu_bias_lkf_);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
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

  gtsam::imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
  // factor to discretize and move normalize by the interval between measurements:
  double d = sqrt(pim_->deltaTij()) / vioParams_.nominalImuRate_; // 1/sqrt(nominalImuRate_) to discretize, then sqrt(pim_->deltaTij()/nominalImuRate_) to count the nr of measurements
  Vector6 biasSigmas;
  biasSigmas.head<3>().setConstant(d * vioParams_.accBiasSigma_);
  biasSigmas.tail<3>().setConstant(d * vioParams_.gyroBiasSigma_);
  gtsam::SharedNoiseModel bias_noise_model = gtsam::noiseModel::Diagonal::Sigmas(biasSigmas);

  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> >(
          gtsam::Symbol('b', from_id), gtsam::Symbol('b', to_id), zero_bias, bias_noise_model));
#endif

  debugInfo_.imuR_lkf_kf = pim_->deltaRij();
  debugInfo_.numAddedImuF_++;

  // reset preintegration
  pim_.reset();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
// TODO this function doesn't do just one thing... Should be refactored!
// It returns the landmark ids of the stereo measurements
// It also updates the feature tracks. Why is this in the backend???
void VioBackEnd::addStereoMeasurementsToFeatureTracks(
    const int& frameNum,
    const SmartStereoMeasurements& stereoMeasurements_kf,
    LandmarkIds* landmarks_kf) {
  CHECK_NOTNULL(landmarks_kf);

  //TODO: feature tracks will grow unbounded.

  // Make sure the landmarks_kf vector is empty and has a suitable size.
  landmarks_kf->clear();
  landmarks_kf->reserve(stereoMeasurements_kf.size());

  // Store landmark ids.
  for (size_t i = 0; i < stereoMeasurements_kf.size(); ++i) {
    const LandmarkId& landmarkId_kf_i = stereoMeasurements_kf.at(i).first;
    const StereoPoint2& stereo_px_i   = stereoMeasurements_kf.at(i).second;

    // We filtered invalid lmks in the StereoTracker, so this should not happen.
    CHECK_NE(landmarkId_kf_i, -1)
        << "landmarkId_kf_i == -1?";

    // Thinner structure that only keeps landmarkIds.
    landmarks_kf->push_back(landmarkId_kf_i);

    // Add features to vio->featureTracks_ if they are new.
    auto lm_it = featureTracks_.find(landmarkId_kf_i);
    if (lm_it == featureTracks_.end()) {
      // New feature.
      VLOG(7) << "Adding landmark: " << landmarkId_kf_i
              << " to feature track.";

      featureTracks_.insert(std::make_pair(landmarkId_kf_i,
                                           FeatureTrack(frameNum,
                                                        stereo_px_i)));
      ++landmark_count_;
    } else {
      // Add observation to existing landmark.
      // @TODO: It seems that this else condition does not help -- conjecture
      // that it creates long feature tracks with low information
      // (i.e. we're not moving)
      // This is problematic in conjunction with our landmark selection
      // mechanism which prioritizes long feature tracks
      (*lm_it).second.obs_.push_back(std::make_pair(frameNum, stereo_px_i));
    }
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addZeroVelocityPrior(const FrameId& frame_id) {
  new_imu_prior_and_other_factors_.push_back(
        boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
          gtsam::Symbol('v', frame_id),
          gtsam::Vector3::Zero(),
          zeroVelocityPriorNoise_));
  if (verbosity_ >= 7) {
    std::cout << "No motion detected, adding zero velocity prior" << std::endl;
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addBetweenFactor(const FrameId& from_id,
                                  const FrameId& to_id,
                                  const gtsam::Pose3& from_id_POSE_to_id)
{
  //  Vector6 sigmas;
  //  sigmas.head<3>().setConstant(0.05);
  //  sigmas.tail<3>().setConstant(0.1);
  //  gtsam::SharedNoiseModel betweenNoise_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  Vector6 precisions;
  precisions.head<3>().setConstant(vioParams_.betweenRotationPrecision_);
  precisions.tail<3>().setConstant(vioParams_.betweenTranslationPrecision_);
  gtsam::SharedNoiseModel betweenNoise_ = gtsam::noiseModel::Diagonal::Precisions(precisions);

  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          gtsam::Symbol('x', from_id), gtsam::Symbol('x', to_id), from_id_POSE_to_id, betweenNoise_));

  debugInfo_.numAddedBetweenStereoF_++;

  if (verbosity_ >= 7) {std::cout << "addBetweenFactor" << std::endl;}
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addNoMotionFactor(const FrameId& from_id,
                                   const FrameId& to_id) {
  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          gtsam::Symbol('x', from_id),
          gtsam::Symbol('x', to_id),
          Pose3(),
          noMotionPriorNoise_));

  debugInfo_.numAddedNoMotionF_++;

  if (verbosity_ >= 7) {
    std::cout << "No motion detected, adding no relative motion prior"
              << std::endl;
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addConstantVelocityFactor(const FrameId& from_id,
                                           const FrameId& to_id) {
  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
          gtsam::Symbol('v', from_id), gtsam::Symbol('v', to_id), gtsam::Vector3::Zero(), constantVelocityPriorNoise_));

  debugInfo_.numAddedConstantVelF_++;

  if (verbosity_ >= 7) {std::cout << "adding constant velocity factor" << std::endl;}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addLandmarksToGraph(const LandmarkIds& landmarks_kf) {
  // Add selected landmarks to graph:
  int n_new_landmarks = 0;
  int n_updated_landmarks = 0;
  debugInfo_.numAddedSmartF_ += landmarks_kf.size();

  for (const LandmarkId& lm_id : landmarks_kf) {
    FeatureTrack& ft = featureTracks_.at(lm_id);
    if (ft.obs_.size() < 2) {// we only insert feature tracks of length at least 2 (otherwise uninformative)
      continue;
    }

    if (!ft.in_ba_graph_) {
      ft.in_ba_graph_ = true;
      addLandmarkToGraph(lm_id, ft);
      ++n_new_landmarks;
    } else {
      const std::pair<FrameId, StereoPoint2> obs_kf = ft.obs_.back();

      if(obs_kf.first != cur_id_) // sanity check
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

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::addLandmarkToGraph(const LandmarkId& lm_id,
                                    const FeatureTrack& ft) {

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

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::updateLandmarkInGraph(
    const LandmarkId& lm_id,
    const std::pair<FrameId, StereoPoint2>& newObs) {

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

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::optimize(
    const FrameId& cur_id,
    const int& max_extra_iterations,
    const std::vector<size_t>& extra_factor_slots_to_delete) {
  if (!smoother_.get())
    throw std::runtime_error("optimize: Incremental smoother is a null pointer\n");

  // only for statistics and debugging
  double startTime, endTime;
  debugInfo_.resetTimes();
  if (verbosity_ >= 5) startTime = UtilsOpenCV::GetTimeInSeconds();

  // We need to remove all smart factors that have new observations.
  // Extra factor slots to delete contains potential factors that we want to delete, it is ty
  // typically an empty vector. And is only used to give flexibility to subclasses.
  std::vector<size_t> delete_slots = extra_factor_slots_to_delete;
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
  new_factors_tmp.push_back(new_imu_prior_and_other_factors_.begin(), new_imu_prior_and_other_factors_.end());
  new_imu_prior_and_other_factors_.resize(0); // clean up stuff which is not in new_factors_tmp

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
  if (verbosity_ >= 8){
    printSmootherInfo(new_factors_tmp,delete_slots,
                     "Smoother status before update:",
                     verbosity_ >= 9);
  }

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
    state_.print("State values\n");

    printSmootherInfo(new_factors_tmp,delete_slots,
                     "CATCHING EXCEPTION",
                     false);
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

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VioBackEnd::findSmartFactorsSlotsSlow(
    const std::vector<Key> new_smart_factors_lmkID_tmp) {

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

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
#ifdef INCREMENTAL_SMOOTHER
void VioBackEnd::findSmartFactorsSlots(
    const std::vector<Key> new_smart_factors_lmkID_tmp) {

  gtsam::ISAM2Result result = smoother_->getISAM2Result();
  // Simple version of find smart factors
  for (size_t i = 0; i < new_smart_factors_lmkID_tmp.size(); ++i) // for each landmark id currently observed (just re-added to graph)
  {
    const auto& it = old_smart_factors_.find(new_smart_factors_lmkID_tmp.at(i)); // find the entry in old_smart_factors_
    it->second.second = result.newFactorsIndices.at(i); // update slot using isam2 indices
  }
}
#endif


/* -------------------------------------------------------------------------- */
ImuBias VioBackEnd::InitializeImuBias(const ImuAccGyr& accGyroRaw,
                                      const Vector3& n_gravity) {
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


/* -------------------------------------------------------------------------- */
gtsam::Pose3 VioBackEnd::GuessPoseFromIMUmeasurements(
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
void VioBackEnd::printSmootherInfo(const gtsam::NonlinearFactorGraph& new_factors_tmp,
    const std::vector<size_t>& delete_slots,
    const string& message,
    const bool& showDetails) const {
  std::cout << " =============== START:" <<  message << " =============== " << std::endl;
  gtsam::NonlinearFactorGraph graph = smoother_->getFactors();
  std::cout << "nr factors in isam2: " << graph.size() << ", with factors:" << std::endl;
  for (auto& g : graph){
    auto gsf = boost::dynamic_pointer_cast<SmartStereoFactor>(g);
    if (gsf){
      std::cout << " SF(valid: " << gsf->isValid() <<
                   ", deg: " << gsf->isDegenerate()
                << " isCheir: " << gsf->isPointBehindCamera() << "): " << std::endl;
    }
    if (g) {
      g->printKeys();
    }
  }
  std::cout << "nr of new factors to add: " << new_factors_tmp.size() << " with factors:" << std::endl;
  for (auto& g : new_factors_tmp) {
    auto gsf = boost::dynamic_pointer_cast<SmartStereoFactor>(g);
    if (gsf){
      std::cout << " SF(valid: " << gsf->isValid() <<
                   ", deg: " << gsf->isDegenerate() << " isCheir: " << gsf->isPointBehindCamera() << "): ";
    }
    if (g) { g->printKeys(); }
  }
  std::cout << "nr deleted slots: " << delete_slots.size() << ", with slots: " << std::endl;
  for (int i = 0; i < delete_slots.size(); ++i)
    std::cout << delete_slots[i] << " ";
  std::cout <<  std::endl;

  std::cout << "nr of values in state_ : " << state_.size() << ", with keys: " << std::endl;
  BOOST_FOREACH(const gtsam::Values::ConstKeyValuePair& key_value, state_) {
    std::cout << gtsam::DefaultKeyFormatter(key_value.key) << " ";
  }
  std::cout <<  std::endl;

  std::cout << "nr values in new_values_ : " << new_values_.size() << ", with keys: " << std::endl;
  BOOST_FOREACH(const gtsam::Values::ConstKeyValuePair& key_value, new_values_) {
    std::cout << gtsam::DefaultKeyFormatter(key_value.key) << " ";
  }
  std::cout <<  std::endl;
  if(showDetails){
    graph.print("isam2 graph:\n");
    new_factors_tmp.print("new_factors_tmp:\n");
    new_values_.print("new values:\n");
    //std::cout << "new_smart_factors_: "  << std::endl;
    //for (auto& s : new_smart_factors_)
    //	s.second->print();
  }
  std::cout << " =============== END: " <<  message << " =============== " << std::endl;
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
        if(result.valid())
          points3D.push_back(*result);
      }
    }
  }
  return points3D;
}

/* -------------------------------------------------------------------------- */
// Get valid 3D points and corresponding lmk id.
// TODO output a map instead of a vector for points_with_id.
void VioBackEnd::get3DPointsAndLmkIds(PointsWithIdMap* points_with_id,
                                      const int& min_age) const {
  CHECK_NOTNULL(points_with_id);

  // Add landmarks encoded in the smart factors.
  const gtsam::NonlinearFactorGraph& graph = smoother_->getFactors();

  // old_smart_factors_ has all smart factors included so far.
  int nr_valid_pts = 0, nr_pts = 0;
  for (const auto& smart_factor : old_smart_factors_) {//!< landmarkId -> {SmartFactorPtr, SlotIndex}
    const LandmarkId& lmk_id = smart_factor.first;
    const SmartStereoFactor::shared_ptr& smart_factor_ptr =
        smart_factor.second.first;
    const int& slot_id = smart_factor.second.second;

    if (smart_factor_ptr && // If pointer is well definied.
        (slot_id >= 0) && (graph.size() > slot_id) && // Slot is admissible.
        (smart_factor_ptr == graph[slot_id]) // Pointer in the graph matches the one we stored in old_smart_factors_.
        ) {
      boost::shared_ptr<SmartStereoFactor> gsf =
          boost::dynamic_pointer_cast<SmartStereoFactor>(graph[slot_id]);
      if (gsf) {
        nr_pts++;
        gtsam::TriangulationResult result = gsf->point();
        if (result.valid() &&
            gsf->measured().size() >= min_age) {
          nr_valid_pts++;
          (*points_with_id)[lmk_id] = *result;
        }
      }
    }
  }

  // Add landmarks that now are in projection factors.
  BOOST_FOREACH(const gtsam::Values::ConstKeyValuePair& key_value, state_) {
    // If we found a lmk.
    // TODO this loop is huge, as we check all variables in the graph...
    if (gtsam::Symbol(key_value.key).chr() == 'l') {
      std::cout << "We added the landmark" << std::endl;
      (*points_with_id)[key_value.key] = key_value.value.cast<gtsam::Point3>();
      //state_.at<gtsam::Point3> (key_value.key)
    }
  }

  VLOG(100) << "nrValidPts= "<< nr_valid_pts << " out of "
            << nr_pts << std::endl;
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
  debugInfo_.resetSmartFactorsStatistics();
  gtsam::NonlinearFactorGraph graph = smoother_->getFactors();
  for (auto& g : graph)
  {
    if (g)
    {
      auto gsf = boost::dynamic_pointer_cast<SmartStereoFactor>(g);
      if (gsf)
      {
        debugInfo_.numSF_ += 1;

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
        if (result.degenerate())
          debugInfo_.numDegenerate_ += 1;

        if (result.farPoint())
          debugInfo_.numFarPoints_ += 1;

        if (result.outlier())
          debugInfo_.numOutliers_ += 1;

        if (result.valid())
        {
          debugInfo_.numValid_ += 1;
          // Check track length
          size_t trackLength = gsf->keys().size();
          if (trackLength > debugInfo_.maxTrackLength_)
            debugInfo_.maxTrackLength_ = trackLength;

          debugInfo_.meanTrackLength_ += trackLength;
        }

        if (result.behindCamera())
          debugInfo_.numCheirality_ += 1;
      }
    }
  }
  if (debugInfo_.numValid_ > 0)
    debugInfo_.meanTrackLength_ = debugInfo_.meanTrackLength_/( (double) debugInfo_.numValid_);
  else
    debugInfo_.meanTrackLength_ = 0;
  if (verbosity_ >= 4) {debugInfo_.print();}
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::computeSparsityStatistics() {
  gtsam::NonlinearFactorGraph graph = smoother_->getFactors();
  gtsam::GaussianFactorGraph::shared_ptr gfg = graph.linearize(state_);
  gtsam::Matrix Hessian = gfg->hessian().first;
  debugInfo_.nrElementsInMatrix_ = Hessian.rows() * Hessian.cols();
  debugInfo_.nrZeroElementsInMatrix_ = 0;
  for(size_t i=0; i<Hessian.rows();++i){
    for(size_t j=0; j<Hessian.cols();++j){
      if(fabs(Hessian(i,j))<1e-15)
        debugInfo_.nrZeroElementsInMatrix_ += 1;
    }
  }
  // sanity check
  if(Hessian.rows() != Hessian.cols()) // matrix is not square
    throw std::runtime_error("computeSparsityStatistics: hessian is not a square matrix?");

  std::cout << "Hessian stats: =========== " << std::endl;
  std::cout << "rows: " << Hessian.rows() << std::endl;
  std::cout << "nrElementsInMatrix_: " << debugInfo_.nrElementsInMatrix_ << std::endl;
  std::cout << "nrZeroElementsInMatrix_: " << debugInfo_.nrZeroElementsInMatrix_ << std::endl;
}

/* -------------------------------------------------------------------------- */
void VioBackEnd::printFeatureTracks() const {
  std::cout << "---- Feature tracks: --------- " << std::endl;
  BOOST_FOREACH(auto keyTrack_j, featureTracks_) {
    std::cout << "Landmark " << keyTrack_j.first << " having ";
    keyTrack_j.second.print();
  }
}
