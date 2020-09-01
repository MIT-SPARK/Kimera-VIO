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

#pragma once

// TODO: move some of these to the cpp files!
#include <fstream>
#include <iostream>
#include <memory>
#include <unordered_map>

#include <boost/foreach.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/backend/VioBackEndParams.h"
#include "kimera-vio/factors/PointPlaneFactor.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd.h"
#include "kimera-vio/initial/InitializationFromImu.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/UtilsGTSAM.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace VIO {

// Forward-declarations
class VioNavState;

class VioBackEnd {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(VioBackEnd);
  KIMERA_POINTER_TYPEDEFS(VioBackEnd);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::function<void(const ImuBias& imu_bias)> ImuBiasCallback;

  /**
   * @brief VioBackEnd Constructor. Initialization must be done separately.
   * @param B_Pose_leftCam Pose from IMU Body (backend's reference pose) to
   * left camera.
   * @param stereo_calibration Stereo Calibration data for stereo factors.
   * @param backend_params Parameters for backend.
   * @param log_output Whether to log to CSV files the backend output.
   */
  VioBackEnd(const BackendParams& backend_params,
             const ImuParams& imu_params,
             const BackendOutputParams& backend_output_params,
             bool log_output);

  virtual ~VioBackEnd() = default;

 public:
  /* ------------------------------------------------------------------------ */
  virtual BackendOutput::UniquePtr spinOnce(const BackendInput& input) = 0;

  /**
   * @brief isInitialized Returns whether the frontend is initializing.
   * Needs to be Thread-Safe! Therefore, frontend_state_ is atomic.
   */
  inline bool isInitialized() const {
    return backend_state_ != BackendState::Bootstrap;
  }

  /* ------------------------------------------------------------------------ */
  // Register (and trigger!) callback that will be called as soon as the backend
  // comes up with a new IMU bias update.
  // The callback is also triggered in this function to update the imu bias for
  // the callee of this function.
  void registerImuBiasUpdateCallback(
      const ImuBiasCallback& imu_bias_update_callback);

  /* ------------------------------------------------------------------------ */
  // Get valid 3D points - TODO: this copies the graph.
  // virtual void get3DPoints(std::vector<gtsam::Point3>* points_3d) const;

  /* ------------------------------------------------------------------------ */
  inline gtsam::Matrix getCurrentStateCovariance() const {
    return state_covariance_lkf_;
  }

  /* ------------------------------------------------------------------------ */
  // Update covariance matrix using getCurrentStateCovariance()
  // NOT TESTED
  void computeStateCovariance();

  /* ------------------------------------------------------------------------ */
  // Set initial state at given pose, velocity and bias.
  virtual bool initStateAndSetPriors(
      const VioNavStateTimestamped& vio_nav_state_initial_seed);

  virtual void initializeBackend(const BackendInput& input) {
    CHECK(backend_state_ == BackendState::Bootstrap);
    switch (backend_params_.autoInitialize_) {
      case 0: {
        initializeFromGt(input);
        break;
      }
      case 1: {
        initializeFromIMU(input);
        break;
      }
      default: {
        LOG(FATAL) << "Wrong initialization mode.";
      }
    }
    // Signal that the backend has been initialized.
    backend_state_ = BackendState::Nominal;
  }

  virtual bool initializeFromGt(const BackendInput& input) {
    // If the gtNavState is identity, the params provider probably did a
    // mistake, although it can happen that the ground truth initial pose is
    // identity, but this is super unlikely
    CHECK(!backend_params_.initial_ground_truth_state_.equals(VioNavState()))
        << "Requested initialization from Ground-Truth pose but got an "
           "identity pose: did you parse your ground-truth correctly?";
    return initStateAndSetPriors(VioNavStateTimestamped(
        input.timestamp_, backend_params_.initial_ground_truth_state_));
  }

  /**
   * @brief initializeFromIMU
   * Assumes Zero Velocity & upright vehicle. Uses the InitializationFromIMU
   * class.
   * @param input BackendInput that MUST be castable to a
   * BackendInputImuInitialization
   * @return
   */
  virtual bool initializeFromIMU(const BackendInput& input) {
    LOG(INFO) << "------------------- Initialize Pipeline: timestamp = "
              << input.timestamp_ << "--------------------";
    // Guess pose from IMU, assumes vehicle to be static.
    const VioNavState& initial_state_estimate =
        InitializationFromImu::getInitialStateEstimate(
            input.imu_acc_gyrs_,
            imu_params_.n_gravity_,
            backend_params_.roundOnAutoInitialize_);

    // Initialize Backend using IMU data.
    return initStateAndSetPriors(
        VioNavStateTimestamped(input.timestamp_, initial_state_estimate));
  }

 protected:
  enum class BackendState {
    Bootstrap = 0u,  //! Initialize backend
    Nominal = 1u     //! Run backend
  };
  std::atomic<BackendState> backend_state_;

  /* ------------------------------------------------------------------------ */
  // Uses landmark table to add factors in graph.
  virtual void addLandmarksToGraph(const LandmarkIds& landmarks_kf) = 0;

  /* ------------------------------------------------------------------------ */
  // Adds a landmark to the graph for the first time.
  virtual void addLandmarkToGraph(const LandmarkId& lm_id, 
                                  const FeatureTrack& lm) = 0;

  /* ------------------------------------------------------------------------ */
  // Set initial guess at current state.
  virtual void addImuValues(const FrameId& cur_id,
                            const gtsam::PreintegrationType& pim);

  /* ------------------------------------------------------------------------ */
  // Add imu factors:
  virtual void addImuFactor(const FrameId& from_id,
                            const FrameId& to_id,
                            const gtsam::PreintegrationType& pim);

  /* ------------------------------------------------------------------------ */
  // Add no motion factors in case of low disparity.
  virtual void addZeroVelocityPrior(const FrameId& frame_id);

  /* ------------------------------------------------------------------------ */
  virtual void addNoMotionFactor(const FrameId& from_id,
                                 const FrameId& to_id);

  /* ------------------------------------------------------------------------ */
  virtual void addBetweenFactor(const FrameId& from_id,
                                const FrameId& to_id,
                                const gtsam::Pose3& from_id_POSE_to_id);

  /**
   * @brief optimize
   * @param timestamp_kf_nsec
   * @param cur_id
   * @param max_iterations
   * @param extra_factor_slots_to_delete
   * @return False if optimization failed, true otherwise
   */
  virtual bool optimize(
      const Timestamp& timestamp_kf_nsec,
      const FrameId& cur_id,
      const size_t& max_iterations,
      const gtsam::FactorIndices& extra_factor_slots_to_delete =
          gtsam::FactorIndices());
  /// Printers.
  /* ------------------------------------------------------------------------ */
  void printFeatureTracks() const;

  /* ------------------------------------------------------------------------ */
  virtual void cleanNullPtrsFromGraph(
      gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors);

  /* ------------------------------------------------------------------------ */
  virtual bool deleteLmkFromFeatureTracks(const LandmarkId& lmk_id);

 protected:
  /* ------------------------------------------------------------------------ */
  // Add initial prior factors.
  virtual void addInitialPriorFactors(const FrameId& frame_id);

  /* ------------------------------------------------------------------------ */
  virtual void addConstantVelocityFactor(const FrameId& from_id,
                                         const FrameId& to_id);

  /* ------------------------------------------------------------------------ */
  // Update states.
  virtual void updateStates(const FrameId& cur_id);

  /* ------------------------------------------------------------------------ */
  /**
   * @brief updateSmoother
   * @param result
   * @param new_factors_tmp
   * @param new_values
   * @param timestamps
   * @param delete_slots
   * @return False if the update failed, true otw.
   */
  bool updateSmoother(
      Smoother::Result* result,
      const gtsam::NonlinearFactorGraph& new_factors_tmp =
          gtsam::NonlinearFactorGraph(),
      const gtsam::Values& new_values = gtsam::Values(),
      const std::map<Key, double>& timestamps =
          gtsam::FixedLagSmoother::KeyTimestampMap(),
      const gtsam::FactorIndices& delete_slots = gtsam::FactorIndices());

  /* ------------------------------------------------------------------------ */
  void cleanCheiralityLmk(
      const gtsam::Symbol& lmk_symbol,
      gtsam::NonlinearFactorGraph* new_factors_tmp_cheirality,
      gtsam::Values* new_values_cheirality,
      std::map<Key, double>* timestamps_cheirality,
      gtsam::FactorIndices* delete_slots_cheirality,
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::NonlinearFactorGraph& new_factors_tmp,
      const gtsam::Values& new_values,
      const std::map<Key, double>& timestamps,
      const gtsam::FactorIndices& delete_slots);

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
  bool deleteKeyFromValues(const gtsam::Key& key,
                           const gtsam::Values& values,
                           gtsam::Values* values_output);

  /* ------------------------------------------------------------------------ */
  // Find all slots of factors that have the given key in the list of keys.
  void findSlotsOfFactorsWithKey(
      const gtsam::Key& key,
      const gtsam::NonlinearFactorGraph& graph,
      std::vector<size_t>* slots_of_factors_with_key);

  /* ------------------------------------------------------------------------ */
  virtual void deleteLmkFromExtraStructures(const LandmarkId& lmk_id);

  /* ------------------------------------------------------------------------ */
  void updateNewSmartFactorsSlots(
      const std::vector<LandmarkId>& lmk_ids_of_new_smart_factors_tmp,
      SmartFactorMap* old_smart_factors);

  /// Private setters.
  /* ------------------------------------------------------------------------ */
  // Set parameters for ISAM 2 incremental smoother.
  void setIsam2Params(const BackendParams& vio_params,
                      gtsam::ISAM2Params* isam_param);

  /* ------------------------------------------------------------------------ */
  // Set parameters for all types of factors.
  void setFactorsParams(
      const BackendParams& vio_params,
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
  virtual void print() const;

  /* ------------------------------------------------------------------------ */
  void printSmootherInfo(const gtsam::NonlinearFactorGraph& new_factors_tmp,
                         const gtsam::FactorIndices& delete_slots,
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
      const bool& print_linear_container_factors = true) const;

  /// Debuggers.
  /* ------------------------------------------------------------------------ */
  virtual void computeSmartFactorStatistics();

  /* ------------------------------------------------------------------------ */
  void computeSparsityStatistics();

  /* ------------------------------------------------------------------------ */
  // Debugging post optimization and estimate calculation.
  void postDebug(
      const std::chrono::high_resolution_clock::time_point& total_start_time,
      const std::chrono::high_resolution_clock::time_point& start_time);

  /* ------------------------------------------------------------------------ */
  // Reset state of debug info.
  void resetDebugInfo(DebugVioInfo* debug_info);

 public:
  /// Getters
  // Thread-safe methods, but also the returns are const, so no problems.
  inline const BackendParams& getBackEndParams() const {
    return backend_params_;
  }

  // TODO NOT THREAD-SAFE! Should add critical sections.
  inline Timestamp getTimestampLkf() const { return timestamp_lkf_; }
  inline ImuBias getLatestImuBias() const { return imu_bias_lkf_; }
  inline ImuBias getImuBiasPrevKf() const { return imu_bias_prev_kf_; }
  inline Vector3 getWVelBLkf() const { return W_Vel_B_lkf_; }
  inline Pose3 getWPoseBLkf() const { return W_Pose_B_lkf_; }
  inline gtsam::Matrix getStateCovarianceLkf() const {
    return state_covariance_lkf_;
  }
  inline int getCurrKfId() const { return curr_kf_id_; }
  inline gtsam::Values getState() const { return state_; }
  inline int getLandmarkCount() const { return landmark_count_; }
  inline DebugVioInfo getCurrentDebugVioInfo() const { return debug_info_; }

  // TODO This call is unsafe, as we are sending a reference to something that
  // will be modified by the backend thread.
  // TODO send this via the output payload...
  inline const gtsam::NonlinearFactorGraph& getFactorsUnsafe() const {
    return smoother_->getFactors();
  }

 protected:
  // Raw, user-specified params.
  const BackendParams backend_params_;
  const ImuParams imu_params_;
  const BackendOutputParams backend_output_params_;

  // State estimates.
  // TODO(Toni): bundle these in a VioNavStateTimestamped.
  Timestamp timestamp_lkf_;
  ImuBias imu_bias_lkf_;  //!< Most recent bias estimate..
  Vector3 W_Vel_B_lkf_;   //!< Velocity of body at k-1 in world coordinates
  Pose3 W_Pose_B_lkf_;    //!< Body pose at at k-1 in world coordinates.

  ImuBias imu_bias_prev_kf_;  //!< bias estimate at previous keyframe

  // State covariance. (initialize to zero)
  gtsam::Matrix state_covariance_lkf_ = Eigen::MatrixXd::Zero(15, 15);

  // Vision params.
  gtsam::SmartStereoProjectionParams smart_factors_params_;
  gtsam::SharedNoiseModel smart_noise_;

  // State.
  //!< current state of the system.
  gtsam::Values state_;

  // ISAM2 smoother
  std::unique_ptr<Smoother> smoother_;

  // Values
  //!< new states to be added
  gtsam::Values new_values_;

  // Factors.
  //!< New factors to be added
  gtsam::NonlinearFactorGraph new_imu_prior_and_other_factors_;
  //!< landmarkId -> {SmartFactorPtr}
  LandmarkIdSmartFactorMap new_smart_factors_;
  //!< landmarkId -> {SmartFactorPtr, SlotIndex}
  SmartFactorMap old_smart_factors_;
  // if SlotIndex is -1, means that the factor has not been inserted yet in the
  // graph

  // Data:
  // TODO grows unbounded currently, but it should be limited to time horizon.
  FeatureTracks feature_tracks_;

  // Counters.
  //! Last keyframe id.
  int last_kf_id_;
  //! Current keyframe id.
  int curr_kf_id_;

  // Imu Bias update callback. To be called as soon as we have a new IMU bias
  // update so that the frontend performs preintegration with the newest bias.
  ImuBiasCallback imu_bias_update_callback_;

  // Debug info.
  DebugVioInfo debug_info_;

  // To print smoother info, useful when looking for optimization bugs.
  bool debug_smoother_ = false;

 protected:
  //! No motion factors settings.
  gtsam::SharedNoiseModel zero_velocity_prior_noise_;
  gtsam::SharedNoiseModel no_motion_prior_noise_;
  gtsam::SharedNoiseModel constant_velocity_prior_noise_;

  //! Landmark count.
  int landmark_count_;

  //! Number of Cheirality exceptions
  size_t counter_of_exceptions_ = 0;

  //! Logger.
  const bool log_output_ = {false};
  std::unique_ptr<BackendLogger> logger_;
};

}  // namespace VIO
