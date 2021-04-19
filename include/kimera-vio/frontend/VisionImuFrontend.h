/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionImuFrontend.h
 * @brief  Class describing an abstract VIO Frontend
 * @author Marcus Abate
 */

#pragma once

#include <gflags/gflags.h>

#include <atomic>
#include <memory>

#include "kimera-vio/frontend/FrontendInputPacketBase.h"
#include "kimera-vio/frontend/FrontendOutputPacketBase.h"
#include "kimera-vio/frontend/OdometryParams.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontend.h"
#include "kimera-vio/imu-frontend/ImuFrontendParams.h"
#include "kimera-vio/initial/TimeAlignerBase.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/visualizer/Display-definitions.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

DECLARE_bool(visualize_feature_tracks);
DECLARE_bool(visualize_frontend_images);
DECLARE_bool(save_frontend_images);
DECLARE_bool(log_feature_tracks);
DECLARE_bool(log_mono_tracking_images);
DECLARE_bool(log_stereo_matching_images);

namespace VIO {

class VisionImuFrontend {
 public:
  KIMERA_POINTER_TYPEDEFS(VisionImuFrontend);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisionImuFrontend);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::function<void(double imu_time_shift_s)> ImuTimeShiftCallback;

 public:
  VisionImuFrontend(const ImuParams& imu_params,
                    const ImuBias& imu_initial_bias,
                    DisplayQueue* display_queue,
                    bool log_output,
                    boost::optional<OdometryParams> odom_params = boost::none);

  virtual ~VisionImuFrontend();

 public:
  FrontendOutputPacketBase::UniquePtr spinOnce(
      FrontendInputPacketBase::UniquePtr&& input);

  /* ------------------------------------------------------------------------ */
  // Update Imu Bias. This is thread-safe as imu_frontend_->updateBias is
  // thread-safe.
  inline void updateImuBias(const ImuBias& imu_bias) const {
    imu_frontend_->updateBias(imu_bias);
  }

  /* ------------------------------------------------------------------------ */
  /**
   * @brief isInitialized Returns whether the Frontend is initializing.
   * Needs to be Thread-Safe! Therefore, frontend_state_ is atomic.
   */
  inline bool isInitialized() const {
    return frontend_state_ != FrontendState::Bootstrap;
  }

  /* ------------------------------------------------------------------------ */
  // Get Imu Bias. This is thread-safe as imu_frontend_->getCurrentImuBias is
  // thread-safe.
  inline ImuBias getCurrentImuBias() const {
    return imu_frontend_->getCurrentImuBias();
  }

  /* ------------------------------------------------------------------------ */
  // Update Imu Bias and reset pre-integration during initialization.
  // This is not thread-safe! (no multi-thread during initialization)
  inline void updateAndResetImuBias(const ImuBias& imu_bias) const {
    imu_frontend_->updateBias(imu_bias);
    imu_frontend_->resetIntegrationWithCachedBias();
  }

  /* ------------------------------------------------------------------------ */
  // Get IMU Params for IMU Frontend.
  inline gtsam::PreintegratedImuMeasurements::Params getImuFrontendParams() {
    return imu_frontend_->getGtsamImuParams();
  }

  /* ------------------------------------------------------------------------ */
  static void printTrackingStatus(const TrackingStatus& status,
                                  const std::string& type);

  /* ------------------------------------------------------------------------ */
  // Get tracker info.
  inline DebugTrackerInfo getTrackerInfo() const {
    return tracker_->debug_info_;
  }

  /* ------------------------------------------------------------------------ */
  // register a callback for the frontend to update the imu time shift
  inline void registerImuTimeShiftUpdateCallback(
      const ImuTimeShiftCallback& callback) {
    imu_time_shift_update_callback_ = callback;
  }

 protected:
  virtual FrontendOutputPacketBase::UniquePtr bootstrapSpin(
      FrontendInputPacketBase::UniquePtr&& input) = 0;

  virtual FrontendOutputPacketBase::UniquePtr timeAlignmentSpin(
      FrontendInputPacketBase::UniquePtr&& input);

  virtual FrontendOutputPacketBase::UniquePtr nominalSpin(
      FrontendInputPacketBase::UniquePtr&& input) = 0;

  /* ------------------------------------------------------------------------ */
  // Reset ImuFrontend gravity. Trivial gravity is needed for initial alignment.
  // This is thread-safe as imu_frontend_->resetPreintegrationGravity is
  // thread-safe.
  inline void resetGravity(const gtsam::Vector3& reset_value) const {
    imu_frontend_->resetPreintegrationGravity(reset_value);
  }

  /* ------------------------------------------------------------------------ */
  // Get ImuFrontend gravity.
  // This is thread-safe as imu_frontend_->getPreintegrationGravity is
  // thread-safe.
  inline gtsam::Vector3 getGravity() const {
    return imu_frontend_->getPreintegrationGravity();
  }

  void outlierRejectionMono(const gtsam::Rot3& keyframe_R_cur_frame,
                            Frame* frame_lkf,
                            Frame* frame_k,
                            TrackingStatusPose* status_pose_mono);

  // can't be const (needs to cache keyframe odom if possible)
  inline boost::optional<gtsam::Pose3> getExternalOdometryRelativeBodyPose(
      FrontendInputPacketBase* input) {
    if (!odom_params_) {
      return boost::none;
    }

    // Past this point we are using external odometry
    CHECK(input);
    if (!input->world_NavState_odom_) {
      LOG(WARNING)
          << "Input packet did not contain valid external odometry measurement";
      return boost::none;
    }

    // First time getting a odometry measurement
    const gtsam::Pose3& odom_Pose_body =
        (*odom_params_).body_Pose_odom_.inverse();
    if (!world_Pose_lkf_body_) {
      world_Pose_lkf_body_ =
          (*input->world_NavState_odom_).pose().compose(odom_Pose_body);
      return boost::none;
    }

    gtsam::Pose3 world_Pose_kf_body =
        (*input->world_NavState_odom_).pose().compose(odom_Pose_body);
    gtsam::Pose3 lkf_body_Pose_kf_body =
        (*world_Pose_lkf_body_).between(world_Pose_kf_body);

    // We cache the current keyframe odometry for the next keyframe
    world_Pose_lkf_body_ = world_Pose_kf_body;
    return lkf_body_Pose_kf_body;
  }

  inline boost::optional<gtsam::Velocity3> getExternalOdometryWorldVelocity(
      FrontendInputPacketBase* input) const {
    if (!odom_params_) {
      return boost::none;
    }

    CHECK(input);
    if (!input->world_NavState_odom_) {
      // we could log here to, but RelativePose handles it...
      return boost::none;
    }

    // Pass the sensor velocity in the world frame if available
    // NOTE: typical odometry is not suitable for this since the vel estimate
    // in the world frame will not have a bounded error.
    return (*input->world_NavState_odom_).velocity();
  }

 protected:
  enum class FrontendState {
    Bootstrap = 0u,  //! Initialize Frontend
    InitialTimeAlignment =
        1u,       //! Optionally initialize IMU-Camera time alignment
    Nominal = 2u  //! Run Frontend
  };
  std::atomic<FrontendState> frontend_state_;

  // Counters.
  int frame_count_;
  int keyframe_count_;

  // Timestamp of last keyframe.
  Timestamp last_keyframe_timestamp_;

  // IMU Frontend.
  ImuFrontend::UniquePtr imu_frontend_;

  // Tracker
  Tracker::UniquePtr tracker_;
  TrackerStatusSummary tracker_status_summary_;

  // Display queue
  DisplayQueue* display_queue_;

  // Logger
  FrontendLogger::UniquePtr logger_;

  // Time alignment
  ImuTimeShiftCallback imu_time_shift_update_callback_;
  TimeAlignerBase::UniquePtr time_aligner_;

  // External odometry
  boost::optional<OdometryParams> odom_params_;
  // world_Pose_body for the last keyframe
  boost::optional<gtsam::Pose3> world_Pose_lkf_body_;
};

}  // namespace VIO
