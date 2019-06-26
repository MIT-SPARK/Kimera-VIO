/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackEnd.h
 * @brief  Visual-Inertial Odometry pipeline, as described in these papers:
 *
 * C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza.
 * On-Manifold Preintegration Theory for Fast and Accurate Visual-Inertial Navigation.
 * IEEE Trans. Robotics, 33(1):1-21, 2016.
 *
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, and F. Dellaert.
 * Eliminating Conditionally Independent Sets in Factor Graphs: A Unifying Perspective based on Smart Factors.
 * In IEEE Intl. Conf. on Robotics and Automation (ICRA), 2014.
 *
 * @author Antoni Rosinol, Luca Carlone
 */

#ifndef VioBackEnd_H_
#define VioBackEnd_H_

#include <memory>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include "factors/PointPlaneFactor.h"

#include "utils/ThreadsafeQueue.h"
#include "UtilsOpenCV.h"
#include "ImuFrontEnd.h"
#include "StereoVisionFrontEnd-definitions.h"
#include "VioBackEndParams.h"
#include "VioBackEnd-definitions.h"

namespace VIO {

// Forward-declarations
class gtNavState;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class VioBackEnd {
public:
  using SmartStereoFactor = gtsam::SmartStereoProjectionPoseFactor;
  using SmartFactorParams = gtsam::SmartStereoProjectionParams;
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
   * 8: display also factor keys and value keys, feature tracks, and landmarks
   *  added to graph, added observations to points
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
             const ImuAccGyrS& imu_accgyr,
             const VioBackEndParams& vioParams,
             const bool log_output = false);

  // Virtual destructor needed for derived class (i.e. RegularVioBackEnd).
  virtual ~VioBackEnd() {LOG(INFO) << "Backend destructor called.";};

public:
  /* ------------------------------------------------------------------------ */
  bool spin(ThreadsafeQueue<VioBackEndInputPayload>& input_queue,
            ThreadsafeQueue<VioBackEndOutputPayload>& output_queue,
            bool parallel_run = true);

  /* ------------------------------------------------------------------------ */
  VioBackEndOutputPayload spinOnce(
      const std::shared_ptr<VioBackEndInputPayload>& input);

  /* ------------------------------------------------------------------------ */
  inline void shutdown() {
    LOG_IF(WARNING, shutdown_) << "Shutdown requested, but Backend was already "
                                  "shutdown.";
    LOG(INFO) << "Shutting down Backend.";
    shutdown_ = true;
  }

  /* ------------------------------------------------------------------------ */
  // Checks if the thread is waiting for the input_queue or working.
  inline bool isWorking() const {return is_thread_working_;}

  /* ------------------------------------------------------------------------ */
  // Register (and trigger!) callback that will be called as soon as the backend
  // comes up with a new IMU bias update.
  // The callback is also triggered in this function to update the imu bias for
  // the callee of this function.
  void registerImuBiasUpdateCallback(
      std::function<void(const ImuBias& imu_bias)> imu_bias_update_callback);

  /* ------------------------------------------------------------------------ */
  // TODO only public because it is used for testing...
  static gtsam::Pose3 guessPoseFromIMUmeasurements(const ImuAccGyrS& accGyroRaw,
                                                   const Vector3& n_gravity,
                                                   const bool& round = true);

  /* ------------------------------------------------------------------------ */
  // Get valid 3D points - TODO: this copies the graph.
  std::vector<gtsam::Point3> get3DPoints() const;

  /* ------------------------------------------------------------------------ */
  // Get valid 3D points and corresponding lmk id.
  // Warning! it modifies old_smart_factors_!!
  PointsWithIdMap getMapLmkIdsTo3dPointsInTimeHorizon(
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
  // Workhorse that stores data and optimizes at each keyframe.
  // [in] timestamp_kf_nsec, keyframe timestamp.
  // [in] status_smart_stereo_measurements_kf, vision data.
  // [in] stereo_ransac_body_pose, inertial data.
  virtual void addVisualInertialStateAndOptimize(
      const Timestamp& timestamp_kf_nsec,
      const StatusSmartStereoMeasurements& status_smart_stereo_measurements_kf,
      const gtsam::PreintegratedImuMeasurements& pim,
      std::vector<Plane>* planes = nullptr,
      boost::optional<gtsam::Pose3> stereo_ransac_body_pose = boost::none);

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
  // Set initial guess at current state.
  void addImuValues(const FrameId& cur_id,
                    const gtsam::PreintegratedImuMeasurements& pim);

  /* ------------------------------------------------------------------------ */
  // Add imu factors:
  void addImuFactor(const FrameId& from_id,
                    const FrameId& to_id,
                    const gtsam::PreintegratedImuMeasurements& pim);

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
  void optimize(const Timestamp &timestamp_kf_nsec,
                const FrameId& cur_id,
                const size_t& max_iterations,
                const std::vector<size_t>& extra_factor_slots_to_delete =
                                                        std::vector<size_t>());
  /// Printers.
  /* ------------------------------------------------------------------------ */
  void printFeatureTracks() const;

  /* ------------------------------------------------------------------------ */
  void cleanNullPtrsFromGraph(
      gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors);

  /// Getters
  /* ------------------------------------------------------------------------ */
  // Get the most recent estimate of the given gtsam key.
  // Mind that to have the most up to date key, you should call this after
  // optimize (or addVisualInertialStateandOptimize)
  template<class T>
  bool getEstimateOfKey(const gtsam::Key& key, T* estimate) const;

private:
  /* ------------------------------------------------------------------------ */
  void addVisualInertialStateAndOptimize(
          const std::shared_ptr<VioBackEndInputPayload>& input);

  /* ------------------------------------------------------------------------ */
  // Sets initial state at given pose, zero velociy and with imu bias obtained
  // by assuming steady upright platform.
  void initStateAndSetPriors(const Timestamp& timestamp_kf_nsec,
                             const Pose3& initialPose,
                             const ImuAccGyrS& accGyroRaw);

  /* ------------------------------------------------------------------------ */
  // Set initial state at given pose, velocity and bias.
  void initStateAndSetPriors(const Timestamp& timestamp_kf_nsec,
                             const Pose3& initialPose,
                             const Vector3& initialVel,
                             const ImuBias& initialBias);

  /* ------------------------------------------------------------------------ */
  // Add initial prior factors.
  void addInitialPriorFactors(const FrameId& frame_id);

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
      const std::vector<size_t>& delete_slots = std::vector<size_t>());

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
  void postDebug(
      const std::chrono::high_resolution_clock::time_point& total_start_time,
      std::chrono::high_resolution_clock::time_point start_time);

  /* ------------------------------------------------------------------------ */
  // Reset state of debug info.
  void resetDebugInfo(DebugVioInfo* debug_info);

public:
  /// Getters
  // Thread-safe methods, but also the returns are const, so no problems.
  inline const VioBackEndParams& getBackEndParams() const {return vio_params_;}
  inline const Pose3& getBPoseLeftCam() const {return B_Pose_leftCam_;}

  // TODO NOT THREAD-SAFE! Should add critical sections.
  inline ImuBias getLatestImuBias() const {return imu_bias_lkf_;}
  inline ImuBias getImuBiasPrevKf() const {return imu_bias_prev_kf_;}
  inline Vector3 getWVelBLkf() const {return W_Vel_B_lkf_;}
  inline Pose3 getWPoseBLkf() const {return W_Pose_B_lkf_;}
  inline int getCurrKfId() const {return curr_kf_id_;}
  inline gtsam::Values getState() const {return state_;}
  inline int getLandmarkCount() const {return landmark_count_;}
  inline DebugVioInfo getCurrentDebugVioInfo() const {return debug_info_;}

  // TODO This call is unsafe, as we are sending a reference to something that
  // will be modified by the backend thread.
  inline const gtsam::NonlinearFactorGraph& getFactorsUnsafe() const {
    return smoother_->getFactors();
  }

  /* ------------------------------------------------------------------------ */
  static ImuBias initImuBias(const ImuAccGyrS& accGyroRaw,
                             const Vector3& n_gravity);

protected:
  // Raw, user-specified params.
  const VioBackEndParams vio_params_;

  // State estimates.
  ImuBias imu_bias_lkf_;       //!< Most recent bias estimate..
  Vector3 W_Vel_B_lkf_;  		   //!< Velocity of body at k-1 in world coordinates
  Pose3   W_Pose_B_lkf_;        //!< Body pose at at k-1 in world coordinates.

  ImuBias imu_bias_prev_kf_;   //!< bias estimate at previous keyframe

  // Vision params.
  gtsam::SmartStereoProjectionParams smart_factors_params_;
  gtsam::SharedNoiseModel smart_noise_;
  const Pose3 B_Pose_leftCam_; // pose of the left camera wrt body
  const gtsam::Cal3_S2Stereo::shared_ptr stereo_cal_; // stores calibration, baseline

  // State.
  gtsam::Values state_;                        //!< current state of the system.

  // GTSAM:
  std::shared_ptr<Smoother> smoother_;

  // Values
  gtsam::Values new_values_;                   //!< new states to be added

  // Factors.
  gtsam::NonlinearFactorGraph new_imu_prior_and_other_factors_;    //!< new factors to be added
  LandmarkIdSmartFactorMap new_smart_factors_; //!< landmarkId -> {SmartFactorPtr}
  SmartFactorMap old_smart_factors_;           //!< landmarkId -> {SmartFactorPtr, SlotIndex}

  // Imu Bias update callback. To be called as soon as we have a new IMU bias
  // update so that the frontend performs preintegration with the newest bias.
  std::function<void(const ImuBias& imu_bias)> imu_bias_update_callback_;

  // Debug info.
  DebugVioInfo debug_info_;

  // To print smoother info, useful when looking for optimization bugs.
  bool debug_smoother_ = false;

  // Data:
  // TODO grows unbounded currently, but it should be limited to time horizon.
  FeatureTracks feature_tracks_;

  /// Counters.
  int last_kf_id_;
  // Id of current keyframe, increases from 0 to inf.
  int curr_kf_id_;

private:
  // No motion factors settings.
  gtsam::SharedNoiseModel zero_velocity_prior_noise_;
  gtsam::SharedNoiseModel no_motion_prior_noise_;
  gtsam::SharedNoiseModel constant_velocity_prior_noise_;

  // Landmark count.
  int landmark_count_;

  // Flags.
  const int verbosity_;
  const bool log_output_ = {false};

  // Thread related members.
  std::atomic_bool shutdown_ = {false};
  std::atomic_bool is_thread_working_ = {false};
};

// Template implementations.
// TODO, should add in VioBackEnd_impl.h
/* -------------------------------------------------------------------------- */
template<class T>
bool VioBackEnd::getEstimateOfKey(const gtsam::Key& key, T* estimate) const {
  if (state_.exists(key)) {
    *CHECK_NOTNULL(estimate) = state_.at<T>(key);
    return true;
  }
  return false;
}

} // namespace VIO
#endif /* VioBackEnd_H_ */

