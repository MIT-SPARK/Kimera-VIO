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
#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontend.h"
#include "kimera-vio/imu-frontend/ImuFrontendParams.h"
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
  
 public:
  VisionImuFrontend(const ImuParams& imu_params,
                 const ImuBias& imu_initial_bias,
                 DisplayQueue* display_queue,
                 bool log_output);

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

 protected:
  virtual FrontendOutputPacketBase::UniquePtr
      bootstrapSpin(FrontendInputPacketBase::UniquePtr&& input) = 0;

  virtual FrontendOutputPacketBase::UniquePtr
      nominalSpin(FrontendInputPacketBase::UniquePtr&& input) = 0;

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

  void outlierRejectionMono(
      const gtsam::Rot3& keyframe_R_cur_frame,
      Frame* frame_lkf,
      Frame* frame_k,
      TrackingStatusPose* status_pose_mono);

 protected:
  enum class FrontendState {
    Bootstrap = 0u,  //! Initialize Frontend
    Nominal = 1u     //! Run Frontend
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
};

}  // namespace VIO
