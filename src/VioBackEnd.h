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
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/AHRSFactor.h>

#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include "factors/PointPlaneFactor.h"

#include "utils/ThreadsafeQueue.h"
#include "UtilsOpenCV.h"

namespace VIO {

// Forward-declarations
class gtNavState;

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

////////////////////////////////////////////////////////////////////////////////
// FeatureTrack
class FeatureTrack {
public:
  //! Observation: {FrameId, Px-Measurement}
  std::vector<std::pair<FrameId, StereoPoint2>> obs_;

  // Is the lmk in the graph?
  bool in_ba_graph_ = false;

  FeatureTrack(FrameId frame_id, const StereoPoint2& px) {
    obs_.push_back(std::make_pair(frame_id, px));
  }

  void print() const {
    std::cout << "feature track with cameras: ";
    for (size_t i = 0; i < obs_.size() ; i++) {
      std::cout << " " <<  obs_[i].first << " ";
    }
    std::cout << std::endl;
  }
};

// Landmark id to measurements.
// Key is the lmk_id and feature track the collection of pairs of
// frame id and pixel location.
using FeatureTracks = std::unordered_map<Key, FeatureTrack>;

////////////////////////////////////////////////////////////////////////////////
class DebugVioInfo {
public:
  int numSF_;
  int numValid_;
  int numDegenerate_;
  int numFarPoints_;
  int numOutliers_;
  int numCheirality_;

  gtsam::Rot3 imuR_lkf_kf = gtsam::Rot3();

  gtsam::Values stateBeforeOpt;
  gtsam::NonlinearFactorGraph graphBeforeOpt;
  gtsam::NonlinearFactorGraph graphToBeDeleted;

  double factorsAndSlotsTime_;
  double preUpdateTime_;
  double updateTime_;
  double updateSlotTime_;
  double extraIterationsTime_;
  double printTime_;

  double meanPixelError_;
  double maxPixelError_;
  double meanTrackLength_;
  size_t maxTrackLength_;

  int numAddedSmartF_;
  int numAddedImuF_;
  int numAddedNoMotionF_;
  int numAddedConstantVelF_;
  int numAddedBetweenStereoF_;

  int nrElementsInMatrix_;
  int nrZeroElementsInMatrix_;

  gtsam::NavState navstate_k_;

  double linearizeTime_;
  double linearSolveTime_;
  double retractTime_;
  double linearizeMarginalizeTime_;
  double marginalizeTime_;
  double imuPreintegrateTime_;

  /* ------------------------------------------------------------------------ */
  void resetSmartFactorsStatistics() {
    numSF_ = 0;
    numValid_ = 0;
    numDegenerate_ = 0;
    numFarPoints_ = 0;
    numOutliers_ = 0;
    numCheirality_ = 0;

    meanPixelError_ = 0;
    maxPixelError_ = 0;
    meanTrackLength_ = 0;
    maxTrackLength_ = 0;
  }

  /* ------------------------------------------------------------------------ */
  void resetTimes() {
    factorsAndSlotsTime_= 0;
    preUpdateTime_= 0;
    updateTime_= 0;
    updateSlotTime_= 0;
    extraIterationsTime_= 0;
    printTime_= 0;
    linearizeTime_= 0;
    linearSolveTime_= 0;
    retractTime_= 0;
    linearizeMarginalizeTime_= 0;
    marginalizeTime_= 0;
  }

  /* ------------------------------------------------------------------------ */
  void resetAddedFactorsStatistics() {
    numAddedSmartF_ = 0;
    numAddedImuF_ = 0;
    numAddedNoMotionF_ = 0;
    numAddedConstantVelF_ = 0;
    numAddedBetweenStereoF_ = 0;
  }

  /* ------------------------------------------------------------------------ */
  void printTimes() const {
    std::cout << "Find delete time: " << factorsAndSlotsTime_ << std::endl
              << "preUpdate time: " << preUpdateTime_ << std::endl
              << "Update Time time: " << updateTime_ << std::endl
              << "Update slot time: " << updateSlotTime_ << std::endl
              << "Extra iterations time: " << extraIterationsTime_ << std::endl
              << "Print time: " << printTime_ << std::endl;
  }

  /* ------------------------------------------------------------------------ */
  void print() const {
    std::cout << "----- DebugVioInfo: --------" << std::endl;
    std::cout << " numSF: " << numSF_
              << " numValid: " << numValid_
              << " numDegenerate: " << numDegenerate_
              << " numOutliers: " << numOutliers_
              << " numFarPoints: " << numFarPoints_
              << " numCheirality: " << numCheirality_ << std::endl
              << " meanPixelError: " << meanPixelError_
              << " maxPixelError: " << maxPixelError_
              << " meanTrackLength: " << meanTrackLength_
              << " maxTrackLength: " << maxTrackLength_ << std::endl;
  }
};

////////////////////////////////////////////////////////////////////////////////
struct VioBackEndInputPayload {
  VioBackEndInputPayload (
      const Timestamp& timestamp_kf_nsec,
      const StatusSmartStereoMeasurements& status_smart_stereo_measurements_kf,
      const Tracker::TrackingStatus& stereo_tracking_status, // stereo_vision_frontend_->trackerStatusSummary_.kfTrackingStatus_stereo_;
      const ImuStamps& imu_stamps,
      const ImuAccGyr& imu_accgyr,
      std::vector<Plane>* planes = nullptr,
      boost::optional<gtsam::Pose3> stereo_ransac_body_pose = boost::none)
    : timestamp_kf_nsec_(timestamp_kf_nsec),
      status_smart_stereo_measurements_kf_(status_smart_stereo_measurements_kf),
      stereo_tracking_status_(stereo_tracking_status),
      imu_stamps_(imu_stamps),
      imu_accgyr_(imu_accgyr),
      planes_(planes),
      stereo_ransac_body_pose_(stereo_ransac_body_pose) {}
  const Timestamp timestamp_kf_nsec_;
  const StatusSmartStereoMeasurements status_smart_stereo_measurements_kf_;
  const Tracker::TrackingStatus stereo_tracking_status_; // stereo_vision_frontend_->trackerStatusSummary_.kfTrackingStatus_stereo_;
  const ImuStamps imu_stamps_;
  const ImuAccGyr imu_accgyr_;
  std::vector<Plane>* planes_;
  boost::optional<gtsam::Pose3> stereo_ransac_body_pose_;
};

struct VioBackEndOutputPayload {
  VioBackEndOutputPayload(const gtsam::Values state,
                          const gtsam::Pose3& W_Pose_Blkf,
                          const Vector3& W_Vel_Blkf,
                          const gtsam::Pose3& B_Pose_leftCam,
                          const ImuBias& imu_bias_lkf,
                          const int& cur_kf_id,
                          const int& landmark_count,
                          const DebugVioInfo& debug_info)
    : state_(state),
      W_Pose_Blkf_(W_Pose_Blkf),
      W_Vel_Blkf_(W_Vel_Blkf),
      B_Pose_leftCam_(B_Pose_leftCam),
      imu_bias_lkf_(imu_bias_lkf),
      cur_kf_id_(cur_kf_id),
      landmark_count_(landmark_count),
      debug_info_(debug_info) {}

  const gtsam::Values state_;
  const gtsam::Pose3 W_Pose_Blkf_;
  const Vector3 W_Vel_Blkf_;
  const gtsam::Pose3 B_Pose_leftCam_;
  const ImuBias imu_bias_lkf_;
  const int cur_kf_id_;
  const int landmark_count_;
  const DebugVioInfo debug_info_;
};

////////////////////////////////////////////////////////////////////////////////
class VioBackEnd {
public:
  using SmartStereoFactor = gtsam::SmartStereoProjectionPoseFactor;
  using SmartFactorParams = gtsam::SmartStereoProjectionParams;
#ifdef USE_COMBINED_IMU_FACTOR
  using PreintegratedImuMeasurements = gtsam::PreintegratedCombinedMeasurements;
  using PreintegratedImuMeasurementPtr = std::shared_ptr<PreintegratedImuMeasurements>;
#else
  using PreintegratedImuMeasurements   = gtsam::PreintegratedImuMeasurements;
  using PreintegratedImuMeasurementPtr = std::shared_ptr<
                                                  PreintegratedImuMeasurements>;
#endif
  using LandmarkIdSmartFactorMap = std::unordered_map<
                                                LandmarkId,
                                                SmartStereoFactor::shared_ptr>;
  using Slot = long int;
  using SmartFactorMap =
  gtsam::FastMap<LandmarkId, std::pair<SmartStereoFactor::shared_ptr, Slot>>;

  using PointWithId     = std::pair<LandmarkId, gtsam::Point3>;
  using PointsWithId    = std::vector<PointWithId>;
  using PointsWithIdMap = std::unordered_map<LandmarkId, gtsam::Point3>;
  using LmkIdToLmkTypeMap = std::unordered_map<LandmarkId, LandmarkType>;

  // verbosity_ explanation
  /*
   * 4: display smart factors statistics
   * 5: display also timing
   * 6: display also error before and after optimization
   * 7: display also *if* factors are added
   * 8: display also factor keys and value keys, feature tracks, and landmarks added to graph, added observations to points
   * 9: display also factors and values
   */

  /* ------------------------------------------------------------------------ */
  // Create and initialize VioBackEnd.
  /// @param: [in,out] initial_state_gt: information about the initial pose.
  /// non-null pointer to a shared_ptr which might be null if there is no
  /// ground-truth available, otw it contains the initial ground-truth state.
  /// @param: timestamp: timestamp of the initial state.
  /// @param: imu_accgyr: used for auto-initialization of VIO pipeline if no
  /// ground-truth is provided. Ignored if not auto-initialized.
  VioBackEnd(const Pose3& leftCamPose,
             const Cal3_S2& leftCameraCalRectified,
             const double& baseline,
             std::shared_ptr<gtNavState>* initial_state_gt,
             const Timestamp& timestamp,
             const ImuAccGyr& imu_accgyr,
             const VioBackEndParams& vioParams,
             const bool log_timing = false);

  // Virtual destructor needed for derived class (i.e. RegularVioBackEnd).
  virtual ~VioBackEnd() = default;

public:
  /* ------------------------------------------------------------------------ */
  bool spin(ThreadsafeQueue<VioBackEndInputPayload>& input_queue,
            ThreadsafeQueue<VioBackEndOutputPayload>& output_queue) {
    while (!shutdown_) {
      spinOnce(input_queue, output_queue);
    }
    return true;
  }

  /* ------------------------------------------------------------------------ */
  bool spinOnce(ThreadsafeQueue<VioBackEndInputPayload>& input_queue,
                ThreadsafeQueue<VioBackEndOutputPayload>& output_queue) {
    // Get input data from queue.
    std::shared_ptr<VioBackEndInputPayload> input = input_queue.popBlocking();
    // Process data with VIO.
    if (input) {
      addVisualInertialStateAndOptimize(input);
      output_queue.push(VioBackEndOutputPayload(state_,
                                                W_Pose_Blkf_,
                                                W_Vel_Blkf_,
                                                B_Pose_leftCam_,
                                                imu_bias_lkf_,
                                                cur_kf_id_,
                                                landmark_count_,
                                                debug_info_ ));
    } else {
      LOG(WARNING) << "No VioBackEnd Input Payload received";
      return false;
    }
    return true;
  }

  inline void shutdown() {
    shutdown_ = true;
  }

  /* ------------------------------------------------------------------------ */
  // Workhorse that stores data and optimizes at each keyframe.
  // [in] timestamp_kf_nsec, keyframe timestamp.
  // [in] status_smart_stereo_measurements_kf, vision data.
  // [in] imu_stamps, [in] imu_accgyr.
  // [in] stereo_ransac_body_pose, inertial data.
  virtual void addVisualInertialStateAndOptimize(
      const Timestamp& timestamp_kf_nsec,
      const StatusSmartStereoMeasurements& status_smart_stereo_measurements_kf,
      const ImuStamps& imu_stamps, const ImuAccGyr& imu_accgyr,
      std::vector<Plane>* planes = nullptr,
      boost::optional<gtsam::Pose3> stereo_ransac_body_pose = boost::none);

  void addVisualInertialStateAndOptimize(
          const std::shared_ptr<VioBackEndInputPayload>& input) {
    CHECK(input);
    if (vio_params_.addBetweenStereoFactors_ == true &&
        input->stereo_tracking_status_ == Tracker::TrackingStatus::VALID ) {
      VLOG(10) << "Add visual inertial state and optimize,"
                  " using stereo between factor.";
      addVisualInertialStateAndOptimize(
            input->timestamp_kf_nsec_, // Current time for fixed lag smoother.
            input->status_smart_stereo_measurements_kf_, // Vision data.
            input->imu_stamps_, input->imu_accgyr_, // Inertial data.
            input->planes_,
            input->stereo_ransac_body_pose_); // optional: pose estimate from stereo ransac
      VLOG(10) << "Finished addVisualInertialStateAndOptimize.";
    } else {
      VLOG(10) << "Add visual inertial state and optimize,"
                  " without using stereo between factor.";
      addVisualInertialStateAndOptimize(
            input->timestamp_kf_nsec_,
            input->status_smart_stereo_measurements_kf_,
            input->imu_stamps_, input->imu_accgyr_,
            input->planes_); // Same but no pose.
      VLOG(10) << "Finished addVisualInertialStateAndOptimize.";
    }
  }

  /* ------------------------------------------------------------------------ */
  // Uses landmark table to add factors in graph.
  void addLandmarksToGraph(const LandmarkIds& landmarks_kf);

  /* ------------------------------------------------------------------------ */
  // Adds a landmark to the graph for the first time.
  void addLandmarkToGraph(const LandmarkId& lm_id,
                          const FeatureTrack& lm);

  /* ------------------------------------------------------------------------ */
  void updateLandmarkInGraph(
      const LandmarkId& lm_id,
      const std::pair<FrameId, StereoPoint2>& newObs);

  /* ------------------------------------------------------------------------ */
  static ImuBias initImuBias(const ImuAccGyr& accGyroRaw,
                             const Vector3& n_gravity);


  /* ------------------------------------------------------------------------ */
  gtsam::Rot3 preintegrateGyroMeasurements(const ImuStamps& imu_stamps,
                                           const ImuAccGyr& imu_accgyr) const;

  /* ------------------------------------------------------------------------ */
  static gtsam::Pose3 guessPoseFromIMUmeasurements(const ImuAccGyr& accGyroRaw,
                                                   const Vector3& n_gravity,
                                                   const bool& round = true);

  /// Getters
  /* ------------------------------------------------------------------------ */
  // Get the most recent estimate of the given gtsam key.
  // Mind that to have the most up to date key, you should call this after
  // optimize (or addVisualInertialStateandOptimize)
  template<class T>
  bool getEstimateOfKey(const gtsam::Key& key, T* estimate) const {
    CHECK_NOTNULL(estimate);
    if (state_.exists(key)) {
      *estimate = state_.at<T>(key);
      return true;
    } else {
      return false;
    }
  }

  /* ------------------------------------------------------------------------ */
  // Get valid 3D points - TODO: this copies the graph.
  vector<gtsam::Point3> get3DPoints() const;

  /* ------------------------------------------------------------------------ */
  // Get valid 3D points and corresponding lmk id.
  // Warning! it modifies old_smart_factors_!!
  void getMapLmkIdsTo3dPointsInTimeHorizon(
      PointsWithIdMap* points_with_id,
      LmkIdToLmkTypeMap* lmk_id_to_lmk_type_map = nullptr,
      const size_t& min_age = 2);

  /* ------------------------------------------------------------------------ */
  // NOT TESTED
  gtsam::Matrix getCurrentStateCovariance() const;

  /* ------------------------------------------------------------------------ */
  // NOT TESTED
  gtsam::Matrix getCurrentStateInformation() const;

  /// Printers
  /* ------------------------------------------------------------------------ */
  void print() const;

protected:
  /* ------------------------------------------------------------------------ */
  // Store stereo frame info into landmarks table:
  // returns landmarks observed in current frame.
  void addStereoMeasurementsToFeatureTracks(
      const int& frameNum,
      const SmartStereoMeasurements& stereoMeasurements_kf,
      LandmarkIds* landmarks_kf);

  /* ------------------------------------------------------------------------ */
  // Integrate imu measurements into pim_.
  void integrateImuMeasurements(const ImuStamps& imu_stamps,
                                const ImuAccGyr& imu_accgyr);

  /* ------------------------------------------------------------------------ */
  // Set initial guess at current state.
  void addImuValues(const FrameId& cur_id);

  /* ------------------------------------------------------------------------ */
  // Add imu factors:
  void addImuFactor(const FrameId& from_id,
                    const FrameId& to_id);

  /* ------------------------------------------------------------------------ */
  // Add no motion factors in case of low disparity.
  void addZeroVelocityPrior(const FrameId& frame_id);

  /* ------------------------------------------------------------------------ */
  void addNoMotionFactor(const FrameId& from_id,
                         const FrameId& to_id);

  /* ------------------------------------------------------------------------ */
  void addBetweenFactor(const FrameId& from_id,
                        const FrameId& to_id,
                        const gtsam::Pose3& from_id_POSE_to_id);

  /* ------------------------------------------------------------------------ */
  void optimize(const FrameId& cur_id,
                const size_t& max_iterations,
                const std::vector<size_t>& extra_factor_slots_to_delete =
                                                        std::vector<size_t>());
  /// Printers.
  /* ------------------------------------------------------------------------ */
  void printFeatureTracks() const;

  /* ------------------------------------------------------------------------ */
  void cleanNullPtrsFromGraph(
      gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors);

private:
  /// Private Methods.
  ///
  /* ------------------------------------------------------------------------ */
  // Sets initial state at given pose, zero velociy and with imu bias obtained
  // by assuming steady upright platform.
  void initStateAndSetPriors(const Timestamp& timestamp_kf_nsec,
                             const Pose3& initialPose,
                             const ImuAccGyr& accGyroRaw);

  /* ------------------------------------------------------------------------ */
  // Set initial state at given pose, velocity and bias.
  void initStateAndSetPriors(const Timestamp& timestamp_kf_nsec,
                             const Pose3& initialPose,
                             const Vector3& initialVel,
                             const ImuBias& initialBias);

  /* ------------------------------------------------------------------------ */
  // Add initial prior factors.
  void addInitialPriorFactors(const FrameId& frame_id,
                              const ImuAccGyr& imu_accgyr);

  /* ------------------------------------------------------------------------ */
  void addConstantVelocityFactor(const FrameId& from_id,
                                 const FrameId& to_id);

  /* ------------------------------------------------------------------------ */
  // Update states.
  void updateStates(const FrameId& cur_id);

  /* ------------------------------------------------------------------------ */
  // Update smoother.
  void updateSmoother(
      Smoother::Result* result,
      const gtsam::NonlinearFactorGraph& new_factors_tmp =
      gtsam::NonlinearFactorGraph(),
      const gtsam::Values& new_values = gtsam::Values(),
      const std::map<Key, double>& timestamps =
      gtsam::FixedLagSmoother::KeyTimestampMap(),
      const std::vector<size_t>& delete_slots = gtsam::FastVector<size_t>());

  /* ------------------------------------------------------------------------ */
  void cleanCheiralityLmk(
      const gtsam::Symbol& lmk_symbol,
      gtsam::NonlinearFactorGraph* new_factors_tmp_cheirality,
      gtsam::Values* new_values_cheirality,
      std::map<Key, double>* timestamps_cheirality,
      std::vector<size_t>* delete_slots_cheirality,
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::NonlinearFactorGraph& new_factors_tmp,
      const gtsam::Values& new_values,
      const std::map<Key, double>& timestamps,
      const std::vector<size_t>& delete_slots);

  /* ------------------------------------------------------------------------ */
  void deleteAllFactorsWithKeyFromFactorGraph(
      const gtsam::Key& key,
      const gtsam::NonlinearFactorGraph& new_factors_tmp,
      gtsam::NonlinearFactorGraph* factor_graph_output);

  /* ------------------------------------------------------------------------ */
  // Returns if the key in timestamps could be removed or not.
  bool deleteKeyFromTimestamps(
      const gtsam::Key& key,
      const std::map<Key, double>& timestamps,
      std::map<Key, double>* timestamps_output);

  /* ------------------------------------------------------------------------ */
  // Returns if the key in timestamps could be removed or not.
  bool deleteKeyFromValues(
      const gtsam::Key& key,
      const gtsam::Values& values,
      gtsam::Values* values_output);

  /* ------------------------------------------------------------------------ */
  // Find all slots of factors that have the given key in the list of keys.
  void findSlotsOfFactorsWithKey(
      const gtsam::Key& key,
      const gtsam::NonlinearFactorGraph& graph,
      std::vector<size_t>* slots_of_factors_with_key);

  /* ------------------------------------------------------------------------ */
  bool deleteLmkFromFeatureTracks(const LandmarkId& lmk_id);

  /* ------------------------------------------------------------------------ */
  virtual void deleteLmkFromExtraStructures(const LandmarkId& lmk_id);

  /* ------------------------------------------------------------------------ */
  void findSmartFactorsSlotsSlow(
      const std::vector<Key>& new_smart_factors_keys_tmp);

  /* ------------------------------------------------------------------------ */
  void updateNewSmartFactorsSlots(
      const std::vector<LandmarkId>& lmk_ids_of_new_smart_factors_tmp,
      SmartFactorMap* old_smart_factors);

  /// Private setters.
  /* ------------------------------------------------------------------------ */
  // Set parameters for ISAM 2 incremental smoother.
  void setIsam2Params(const VioBackEndParams& vio_params,
                      gtsam::ISAM2Params* isam_param);

  /* ------------------------------------------------------------------------ */
  // Set parameters for all types of factors.
  void setFactorsParams(
      const VioBackEndParams& vio_params,
      gtsam::SharedNoiseModel* smart_noise,
      gtsam::SmartStereoProjectionParams* smart_factors_params,
      boost::shared_ptr<PreintegratedImuMeasurements::Params>* imu_params,
      gtsam::SharedNoiseModel* no_motion_prior_noise,
      gtsam::SharedNoiseModel* zero_velocity_prior_noise,
      gtsam::SharedNoiseModel* constant_velocity_prior_noise);

  /* ------------------------------------------------------------------------ */
  // Set parameters for smart factors.
  void setSmartFactorsParams(
      gtsam::SharedNoiseModel* smart_noise,
      gtsam::SmartStereoProjectionParams* smart_factors_params,
      const double& smart_noise_sigma,
      const double& rank_tolerance,
      const double& landmark_distance_threshold,
      const double& retriangulation_threshold,
      const double& outlier_rejection);

  /* ------------------------------------------------------------------------ */
  // Set parameters for imu factors.
  void setImuFactorsParams(
      boost::shared_ptr<PreintegratedImuMeasurements::Params>* imu_params,
      const gtsam::Vector3& n_gravity,
      const double& gyro_noise_density,
      const double& acc_noise_density,
      const double& imu_integration_sigma);

  /// Private printers.
  /* ------------------------------------------------------------------------ */
  void printSmootherInfo(const gtsam::NonlinearFactorGraph& new_factors_tmp,
                         const std::vector<size_t>& delete_slots,
                         const std::string& message = "CATCHING EXCEPTION",
                         const bool& showDetails = false) const;

  /* ------------------------------------------------------------------------ */
  void printSmartFactor(boost::shared_ptr<SmartStereoFactor> gsf) const;

  /* ------------------------------------------------------------------------ */
  void printPointPlaneFactor(
      boost::shared_ptr<gtsam::PointPlaneFactor> ppf) const;

  /* ------------------------------------------------------------------------ */
  void printPlanePrior(
      boost::shared_ptr<gtsam::PriorFactor<gtsam::OrientedPlane3>> ppp) const;

  /* ------------------------------------------------------------------------ */
  void printPointPrior(
      boost::shared_ptr<gtsam::PriorFactor<gtsam::Point3>> ppp) const;

  /* ------------------------------------------------------------------------ */
  void printLinearContainerFactor(
      boost::shared_ptr<gtsam::LinearContainerFactor> lcf) const;

  /* ------------------------------------------------------------------------ */
  // Provide a nonlinear factor, which will be casted to any of the selected
  // factors, and then printed.
  // Slot argument, is just to print the slot of the factor if you know it.
  // If slot is -1, there is no slot number printed.
  void printSelectedFactors(
      const boost::shared_ptr<gtsam::NonlinearFactor>& g,
      const size_t& slot = 0,
      const bool print_smart_factors = true,
      const bool print_point_plane_factors = true,
      const bool print_plane_priors = true,
      const bool print_point_priors = true,
      const bool print_linear_container_factors = true) const;

  /* ------------------------------------------------------------------------ */
  void printSelectedGraph(
      const gtsam::NonlinearFactorGraph& graph,
      const bool& print_smart_factors = true,
      const bool& print_point_plane_factors = true,
      const bool& print_plane_priors = true,
      const bool& print_point_priors = true,
      const bool& print_linear_container_factors = true) const ;

  /// Debuggers.
  /* ------------------------------------------------------------------------ */
  void computeSmartFactorStatistics();

  /* ------------------------------------------------------------------------ */
  void computeSparsityStatistics();

  /* ------------------------------------------------------------------------ */
  // Debugging post optimization and estimate calculation.
  void postDebug(const double& start_time);

  /* ------------------------------------------------------------------------ */
  // Reset state of debug info.
  void resetDebugInfo(DebugVioInfo* debug_info);

public:
  // RAW, user-specified params.
  const VioBackEndParams vio_params_;

  // STATE ESTIMATES.
  ImuBias imu_bias_lkf_;       //!< Most recent bias estimate..
  ImuBias imu_bias_prev_kf_;   //!< bias estimate at previous keyframe
  Vector3 W_Vel_Blkf_;  		   //!< Velocity of body at k-1 in world coordinates
  Pose3   W_Pose_Blkf_;        //!< Body pose at at k-1 in world coordinates.

  /// Counters.
  int last_kf_id_;
  // Id of current keyframe, increases from 0 to inf.
  int cur_kf_id_;

  // Current time.
  double timestamp_kf_; // timestamp in seconds attached to the last keyframe

  // IMU params.
  boost::shared_ptr<PreintegratedImuMeasurements::Params> imu_params_;
  PreintegratedImuMeasurementPtr pim_;

  // VISION params.
  gtsam::SmartStereoProjectionParams smart_factors_params_;
  gtsam::SharedNoiseModel smart_noise_;
  const Pose3 B_Pose_leftCam_; // pose of the left camera wrt body
  const gtsam::Cal3_S2Stereo::shared_ptr stereo_cal_; // stores calibration, baseline

  // NO MOTION FACTORS settings.
  gtsam::SharedNoiseModel zero_velocity_prior_noise_;
  gtsam::SharedNoiseModel no_motion_prior_noise_;
  gtsam::SharedNoiseModel constant_velocity_prior_noise_;

  // GTSAM:
  std::shared_ptr<Smoother> smoother_;

  // State
  gtsam::Values state_;                        //!< current state of the system.

  // Values
  gtsam::Values new_values_;                   //!< new states to be added

  // Factors.
  gtsam::NonlinearFactorGraph new_imu_prior_and_other_factors_;    //!< new factors to be added
  LandmarkIdSmartFactorMap new_smart_factors_; //!< landmarkId -> {SmartFactorPtr}
  SmartFactorMap old_smart_factors_;           //!< landmarkId -> {SmartFactorPtr, SlotIndex}

  // Data:
  // TODO grows unbounded currently, but it should be limited to time horizon.
  FeatureTracks feature_tracks_;
  int landmark_count_;

  // Flags.
  const int verbosity_;
  const bool log_timing_;

  // Debug info.
  DebugVioInfo debug_info_;

  // TODO remove this.
  bool DEBUG_ = false;

  // Thread related members.
  std::atomic_bool shutdown_ = {false};
};

} // namespace VIO
#endif /* VioBackEnd_H_ */

