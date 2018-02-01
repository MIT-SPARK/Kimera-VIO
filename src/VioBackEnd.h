/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackEnd.h
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

#ifndef VioBackEnd_H_
#define VioBackEnd_H_

#include <memory>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#include "ImuFrontEnd.h"
#include "StereoVisionFrontEnd.h"
#include "VioBackEndParams.h"

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/AHRSFactor.h>

namespace VIO {

// Gtsam types.
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Point3;
using gtsam::Point2;
using gtsam::Key;
using gtsam::Cal3_S2;
using gtsam::StereoPoint2;
using ImuBias = gtsam::imuBias::ConstantBias;

#define INCREMENTAL_SMOOTHER
//#define USE_COMBINED_IMU_FACTOR

#ifdef INCREMENTAL_SMOOTHER
typedef gtsam::IncrementalFixedLagSmoother Smoother;
#else
typedef gtsam::BatchFixedLagSmoother Smoother;
#endif

///////////////////////////////////////////////////////////////////////////////////////
// FeatureTrack
class FeatureTrack
{
public:
  std::vector<std::pair<FrameId, StereoPoint2>> obs_; //! Observation: { FrameId, Px-Measurement}
  bool in_ba_graph_ = false;

  FeatureTrack(FrameId frame_id, const StereoPoint2& px)
  {
    obs_.push_back(std::make_pair(frame_id, px));
  }
  void print() const{
    std::cout << " feature track with cameras: ";
    for(size_t i=0; i<obs_.size();i++)
      std::cout << " " <<  obs_[i].first << " ";
    std::cout << std::endl;
  }
};
using FeatureTracks = std::unordered_map<Key, FeatureTrack>; // landmark if to measurements

///////////////////////////////////////////////////////////////////////////////////////
class DebugVioInfo
{
public:
  int numSF_, numValid_, numDegenerate_, numFarPoints_, numOutliers_, numCheirality_;

  gtsam::Rot3 imuR_lkf_kf = gtsam::Rot3();

  gtsam::Values stateBeforeOpt;
  gtsam::NonlinearFactorGraph graphBeforeOpt;

  double factorsAndSlotsTime_, preUpdateTime_, updateTime_, updateSlotTime_, extraIterationsTime_, printTime_;

  double meanPixelError_, maxPixelError_, meanTrackLength_;
  int maxTrackLength_;

  int numAddedSmartF_, numAddedImuF_, numAddedNoMotionF_, numAddedConstantVelF_, numAddedBetweenStereoF_;

  int nrElementsInMatrix_, nrZeroElementsInMatrix_;

  gtsam::NavState navstate_k_;

  double linearizeTime_, linearSolveTime_, retractTime_, linearizeMarginalizeTime_, marginalizeTime_, imuPreintegrateTime_;

  void resetSmartFactorsStatistics(){
    numSF_ = 0; numValid_ = 0; numDegenerate_ = 0; numFarPoints_ = 0; numOutliers_ = 0;  numCheirality_ = 0;
    meanPixelError_ = 0; maxPixelError_ = 0; meanTrackLength_ = 0; maxTrackLength_ = 0;
  }
  void resetTimes(){
    factorsAndSlotsTime_= 0; preUpdateTime_= 0; updateTime_= 0; updateSlotTime_= 0; extraIterationsTime_= 0; printTime_= 0;
    linearizeTime_= 0; linearSolveTime_= 0; retractTime_= 0; linearizeMarginalizeTime_= 0; marginalizeTime_= 0;
  }
  void resetAddedFactorsStatistics(){
    numAddedSmartF_ = 0; numAddedImuF_ = 0; numAddedNoMotionF_ = 0; numAddedConstantVelF_ = 0; numAddedBetweenStereoF_ = 0;
  }
  void printTimes() const
  {
    std::cout << "Find delete time: " << factorsAndSlotsTime_ << " s" << std::endl;
    std::cout << "preUpdate time: " << preUpdateTime_ << " s" << std::endl;
    std::cout << "Update Time time: " << updateTime_ << " s" << std::endl;
    std::cout << "Update slot time: " << updateSlotTime_ << " s" << std::endl;
    std::cout << "Extra iterations time: " << extraIterationsTime_ << " s" << std::endl;
    std::cout << "Print time: " << printTime_ << " s" << std::endl;
  }
  void print() const{
    std::cout << "----- DebugVioInfo: --------" << std::endl;
    std::cout << " numSF: " << numSF_ << " numValid: " << numValid_ << " numDegenerate: " << numDegenerate_
        << " numOutliers: " << numOutliers_ << " numFarPoints: " << numFarPoints_ << " numCheirality: " << numCheirality_
        << std::endl
        << " meanPixelError: " << meanPixelError_ << " maxPixelError: " << maxPixelError_
        << " meanTrackLength: " << meanTrackLength_ << " maxTrackLength: " << maxTrackLength_ << std::endl;
  }
};

///////////////////////////////////////////////////////////////////////////////////////
class VioBackEnd
{
public:
  using SmartStereoFactor = gtsam::SmartStereoProjectionPoseFactor;
  using SmartFactorParams = gtsam::SmartStereoProjectionParams;
#ifdef USE_COMBINED_IMU_FACTOR
  using PreintegratedImuMeasurements = gtsam::PreintegratedCombinedMeasurements;
  using PreintegratedImuMeasurementPtr = std::shared_ptr<PreintegratedImuMeasurements>;
#else
  using PreintegratedImuMeasurements = gtsam::PreintegratedImuMeasurements;
  using PreintegratedImuMeasurementPtr = std::shared_ptr<PreintegratedImuMeasurements>;
#endif
  using LandmarkIdSmartFactorMap = std::unordered_map<LandmarkId, SmartStereoFactor::shared_ptr>;
  using SmartFactorMap = gtsam::FastMap<LandmarkId, std::pair<SmartStereoFactor::shared_ptr, int>>;

  using PointWithId = std::pair<LandmarkId, gtsam::Point3>;
  using PointsWithId = std::vector<PointWithId>;

  // verbosity_ explanation
  /*
   * 4: display smart factors statistics
   * 5: display also timing
   * 6: display also error before and after optimization
   * 7: display also *if* factors are added
   * 8: display also factor keys and value keys, feature tracks, and landmarks added to graph, added observations to points
   * 9: display also factors and values
   */

  // constructor -------------------------------------------------------------------------
  VioBackEnd(const Pose3 leftCamPose, const Cal3_S2 leftCameraCalRectified, const double baseline,
      const VioBackEndParams vioParams = VioBackEndParams()) :
        B_Pose_leftCam_(leftCamPose),
        stereoCal_(boost::make_shared<gtsam::Cal3_S2Stereo>(leftCameraCalRectified.fx(),
            leftCameraCalRectified.fy(), leftCameraCalRectified.skew(),
            leftCameraCalRectified.px(), leftCameraCalRectified.py(), baseline)),
            vioParams_(vioParams),
            imu_bias_lkf_(ImuBias()), imu_bias_prev_kf_(ImuBias()),
            W_Vel_Blkf_(Vector3::Zero()), W_Pose_Blkf_(Pose3()),
            last_id_(-1), cur_id_(0), verbosity_(5), landmark_count_(0)
  {
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

  // Virtual destructor needed for potential derived class (i.e. RegularVioBackEnd)
  virtual ~VioBackEnd() = default;

  // STATE ESTIMATES
  ImuBias imu_bias_lkf_;       //!< Most recent bias estimate..
  ImuBias imu_bias_prev_kf_;   //!< bias estimate at previous keyframe
  Vector3 W_Vel_Blkf_;  		//!< Velocity of body at k-1 in world coordinates
  Pose3   W_Pose_Blkf_;      //!< Body pose at at k-1 in world coordinates.

  // counters
  int last_id_;
  int cur_id_;

  // RAW, user-specified params
  const VioBackEndParams vioParams_;

  // current time
  double timestamp_kf_; // timestamp in seconds attached to the last keyframe

  // IMU params
  boost::shared_ptr<PreintegratedImuMeasurements::Params> imuParams_;
  PreintegratedImuMeasurementPtr pim_;

  // VISION params
  gtsam::SmartStereoProjectionParams smartFactorsParams_;
  gtsam::SharedNoiseModel smart_noise_;
  const Pose3 B_Pose_leftCam_; // pose of the left camera wrt body
  const gtsam::Cal3_S2Stereo::shared_ptr stereoCal_; // stores calibration, baseline

  // NO MOTION FACTORS settings
  gtsam::SharedNoiseModel zeroVelocityPriorNoise_, noMotionPriorNoise_, constantVelocityPriorNoise_;

  // GTSAM:
  std::shared_ptr<Smoother> smoother_;

  gtsam::Values state_;                        //!< current state of the system.
  gtsam::NonlinearFactorGraph new_imu_and_prior_factors_;    //!< new factors to be added
  gtsam::Values new_values_;                   //!< new states to be added
  LandmarkIdSmartFactorMap new_smart_factors_; //!< landmarkId -> {SmartFactorPtr}
  SmartFactorMap old_smart_factors_;           //!< landmarkId -> {SmartFactorPtr, SlotIndex}

  // Data:
  FeatureTracks featureTracks_;
  int landmark_count_;

  // Flags
  const int verbosity_;

  // debug info
  DebugVioInfo debugInfo_;

  /* ++++++++++++++++++++++++++++++++++ NONCONST FUNCTIONS ++++++++++++++++++++++++++++++++++ */
  // sets initial state at given pose, zero velociy and with imu bias obtained by assuming steady upright platform
  void initializeStateAndSetPriors(const Timestamp timestamp_kf_nsec, const Pose3 initialPose, const ImuAccGyr accGyroRaw);
  // set initial state at given pose, velocity and bias
  void initializeStateAndSetPriors(const Timestamp timestamp_kf_nsec, const Pose3 initialPose, const Vector3 initialVel, const ImuBias initialBias);
  // add initial prior factors
  void addInitialPriorFactors(const FrameId& frame_id, const ImuAccGyr& imu_accgyr);
  // workhorse that stores data and optimizes at each keyframe
  void addVisualInertialStateAndOptimize(const Timestamp timestamp_kf_nsec, // keyframe timestamp
      const StatusSmartStereoMeasurements statusSmartStereoMeasurements_kf, // vision data
      ImuStamps imu_stamps, ImuAccGyr imu_accgyr, boost::optional<gtsam::Pose3> stereoRansacBodyPose = boost::none); // inertial data
  // integrate imu measurements into pim_
  void integrateImuMeasurements(const ImuStamps& imu_stamps, const ImuAccGyr& imu_accgyr);
  // set initial guess at current state
  void addValues(const FrameId& cur_id);
  // add imu factors:
  void addImuFactor(const FrameId& from_id, const FrameId& to_id);
  // store stereo frame info into landmarks table: returns landmarks observed in current frame
  LandmarkIds addStereoMeasurementsToFeatureTracks(int frameNum, const SmartStereoMeasurements& stereoMeasurements_kf);
  // add no motion factors in case of low disparity
  void addZeroVelocityPrior(const FrameId& frame_id);
  void addNoMotionFactor(const FrameId& from_id, const FrameId& to_id);
  void addBetweenFactor(const FrameId& from_id, const FrameId& to_id, const gtsam::Pose3 from_id_POSE_to_id);
  void addConstantVelocityFactor(const FrameId& from_id, const FrameId& to_id);
  // uses landmark table to add factors in graph
  void addLandmarksToGraph(LandmarkIds landmarks_kf);
  void addLandmarkToGraph(LandmarkId lm_id, FeatureTrack& lm);
  void updateLandmarkInGraph(const LandmarkId lm_id, const std::pair<FrameId, StereoPoint2>& newObs);
  void optimize(const FrameId& cur_id, const int max_iterations);
  void findSmartFactorsSlots(const std::vector<Key> new_smart_factors_keys_tmp);
  void findSmartFactorsSlotsSlow(const std::vector<Key> new_smart_factors_keys_tmp);
  /* --------------------------------- CONST FUNCTIONS ------------------------------------ */
  void print() const {
    std::cout << "((((((((((((((((((((((((((((((((((((((((( VIO PRINT ))))))))))))))))))))))))))))))))))))))))) " <<std::endl;
    B_Pose_leftCam_.print("\n B_Pose_leftCam_\n");
    stereoCal_->print("\n stereoCal_\n");
    vioParams_.print();
    W_Pose_Blkf_.print("\n W_Pose_Blkf_ \n");
    std::cout << "\n W_Vel_Blkf_ " << W_Vel_Blkf_.transpose() <<std::endl;
    imu_bias_lkf_.print("\n imu_bias_lkf_ \n");
    imu_bias_prev_kf_.print("\n imu_bias_prev_kf_ \n");
    std::cout << "last_id_ " << last_id_ <<std::endl;
    std::cout << "cur_id_ " << cur_id_ <<std::endl;
    std::cout << "verbosity_ " << verbosity_ <<std::endl;
    std::cout << "landmark_count_ " << landmark_count_ <<std::endl;
    std::cout << "(((((((((((((((((((((((((((((((((((((((((((((((()))))))))))))))))))))))))))))))))))))))))))))) " <<std::endl;
  }
  /* NOT TESTED ----------------------------------------------------------------------------------- */
  gtsam::Matrix getCurrentStateCovariance() const{
    gtsam::Marginals marginals(smoother_->getFactors(), state_, gtsam::Marginals::Factorization::CHOLESKY);
    // current state includes pose, velocity and imu biases
    std::vector<gtsam::Key> keys;
    keys.push_back(gtsam::Symbol('x', cur_id_));
    keys.push_back(gtsam::Symbol('v', cur_id_));
    keys.push_back(gtsam::Symbol('b', cur_id_));
    // return the marginal covariance matrix
    return UtilsOpenCV::Covariance_bvx2xvb(marginals.jointMarginalCovariance(keys).fullMatrix()); // 6 + 3 + 6 = 15x15matrix
  }
  /* NOT TESTED ----------------------------------------------------------------------------------- */
  gtsam::Matrix getCurrentStateInformation() const{
    gtsam::Marginals marginals(smoother_->getFactors(), state_, gtsam::Marginals::Factorization::CHOLESKY);
    // current state includes pose, velocity and imu biases
    std::vector<gtsam::Key> keys;
    keys.push_back(gtsam::Symbol('x', cur_id_));
    keys.push_back(gtsam::Symbol('v', cur_id_));
    keys.push_back(gtsam::Symbol('b', cur_id_));
    // return the marginal covariance
    return UtilsOpenCV::Covariance_bvx2xvb(marginals.jointMarginalInformation(keys).fullMatrix()); // 6 + 3 + 6 = 15x15matrix
  }
  /* ----------------------------------------------------------------------------------- */
  gtsam::Rot3 preintegrateGyroMeasurements(const ImuStamps imu_stamps, const ImuAccGyr imu_accgyr) const {
    gtsam::PreintegratedAhrsMeasurements pimRot(imu_bias_prev_kf_.gyroscope(), gtsam::Matrix3::Identity());
    for (int i = 0; i < imu_stamps.size()-1; ++i)
    {
      Vector3 measured_omega = imu_accgyr.block<3,1>(3,i);
      double delta_t = UtilsOpenCV::NsecToSec(imu_stamps(i+1) - imu_stamps(i));
      pimRot.integrateMeasurement(measured_omega, delta_t);
    }
    gtsam::Rot3 body_Rot_cam_ = B_Pose_leftCam_.rotation(); // of the left camera!!
    return body_Rot_cam_.inverse() * pimRot.deltaRij() * body_Rot_cam_;
  }
  /* ----------------------------------------------------------------------------------- */
  static ImuBias InitializeImuBias(const ImuAccGyr accGyroRaw, const Vector3 n_gravity);
  static gtsam::Pose3 GuessPoseFromIMUmeasurements(const ImuAccGyr accGyroRaw, const Vector3 n_gravity, const bool round = true);
  /* ----------------------------------------------------------------------------------- */
  void showSmootherInfo(const gtsam::NonlinearFactorGraph new_factors_tmp,
      const std::vector<size_t> delete_slots, const std::string message, const bool showDetails) const
  {
    std::cout << " =============== START:" <<  message << " =============== " << std::endl;
    gtsam::NonlinearFactorGraph graph = smoother_->getFactors();
    std::cout << "nr factors in isam2: " << graph.size() << ", with factors:" << std::endl;
    for (auto& g : graph){
      auto gsf = boost::dynamic_pointer_cast<SmartStereoFactor>(g);
      if (gsf){
        std::cout << " SF(valid: " << gsf->isValid() <<
            ", deg: " << gsf->isDegenerate() << " isCheir: " << gsf->isPointBehindCamera() << "): " << std::endl;
      }
      if (g) { g->printKeys(); }
    }
    std::cout << "nr of new factors to add: " << new_factors_tmp.size() << " with factors:" << std::endl;
    for (auto& g : new_factors_tmp){
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
  /* ----------------------------------------------------------------------------------- */
  // get valid 3D points - TODO: this copies the graph
  vector<gtsam::Point3> get3DPoints() const{
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
  /* ----------------------------------------------------------------------------------- */
  // get valid 3D points and corresponding lmk id
  PointsWithId get3DPointsAndLmkIds(const int minAge = 0) const{

    // output
    PointsWithId pointsWithId;

    gtsam::NonlinearFactorGraph graph = smoother_->getFactors(); // TODO: this copies the graph
    // old_smart_factors_ has all smart factors included so far
    int nrValidPts = 0, nrPts = 0;
    for (auto& sf : old_smart_factors_) //!< landmarkId -> {SmartFactorPtr, SlotIndex}
    {
      const LandmarkId lmkId = sf.first;
      const SmartStereoFactor::shared_ptr& sf_ptr = sf.second.first;
      const int slotId = sf.second.second;

      if(sf_ptr && // if pointer is well definied
    		  slotId >= 0 && graph.size() > slotId && // and slot is admissible
			  sf_ptr == graph[slotId]){ // and the pointer in the graph matches the one we stored in old_smart_factors_
        auto gsf = boost::dynamic_pointer_cast<SmartStereoFactor>(graph[slotId]);
        if (gsf){
          nrPts++;
          gtsam::TriangulationResult result = gsf->point();
          if(result.valid() && gsf->measured().size() >= minAge){
            nrValidPts++;
            pointsWithId.push_back(std::make_pair(lmkId,*result));
          }
        }
      }
    }
    std::cout << "nrValidPts= "<< nrValidPts << " out of " << nrPts << std::endl;
    return pointsWithId;
  }

  /* ----------------------------------------------------------------------------------- */
  void computeSmartFactorStatistics()
  {
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
  /* ----------------------------------------------------------------------------------- */
  void computeSparsityStatistics() {
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
  /* ----------------------------------------------------------------------------------- */
  void printFeatureTracks() const{
    std::cout << "---- Feature tracks: --------- " << std::endl;
    BOOST_FOREACH(auto keyTrack_j, featureTracks_) {
      std::cout << "Landmark " << keyTrack_j.first << " having ";
      keyTrack_j.second.print();
    }
  }
};

} // namespace VIO
#endif /* VioBackEnd_H_ */

