/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoVisionImuFrontend-definitions.h
 * @brief  Definitions for MonoVisionImuFrontend
 * @author Marcus Abate
 */

#pragma once

#include <gtsam/geometry/StereoPoint2.h>

#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/FrontendOutputPacketBase.h"
#include "kimera-vio/frontend/MonoImuSyncPacket.h"
#include "kimera-vio/frontend/VisionImuFrontendParams.h"

namespace VIO {

using MonoFrontendInputPayload = MonoImuSyncPacket;

using MonoMeasurement = std::pair<LandmarkId, gtsam::StereoPoint2>;
using MonoMeasurements = std::vector<MonoMeasurement>;
using MonoMeasurementsUniquePtr = std::unique_ptr<MonoMeasurements>;
using StatusMonoMeasurements =
    std::pair<TrackerStatusSummary, MonoMeasurements>;
using StatusMonoMeasurementsPtr = std::shared_ptr<StatusMonoMeasurements>;

struct MonoFrontendOutput : public FrontendOutputPacketBase {
 public:
  KIMERA_POINTER_TYPEDEFS(MonoFrontendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoFrontendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MonoFrontendOutput(
      const bool& is_keyframe,
      const StatusMonoMeasurementsPtr& status_mono_measurements,
      const gtsam::Pose3& b_Pose_cam_rect,
      const Frame& frame_lkf,
      const ImuFrontend::PimPtr& pim,
      const ImuAccGyrS& imu_acc_gyrs,
      const cv::Mat& feature_tracks,
      const DebugTrackerInfo& debug_tracker_info,
      boost::optional<gtsam::Pose3> lkf_body_Pose_kf_body = boost::none,
      boost::optional<gtsam::Velocity3> body_world_Vel_body = boost::none)
      : FrontendOutputPacketBase(frame_lkf.timestamp_,
                                 is_keyframe,
                                 FrontendType::kMonoImu,
                                 pim,
                                 imu_acc_gyrs,
                                 debug_tracker_info,
                                 lkf_body_Pose_kf_body,
                                 body_world_Vel_body),
        status_mono_measurements_(status_mono_measurements),
        b_Pose_cam_rect_(b_Pose_cam_rect),
        frame_lkf_(frame_lkf),
        feature_tracks_(feature_tracks) {}

  virtual ~MonoFrontendOutput() = default;

  virtual const Frame* getTrackingFrame() const override { return &frame_lkf_; }

  virtual const cv::Mat* getTrackingImage() const override {
    return &feature_tracks_;
  }

  virtual const gtsam::Pose3* getBodyPoseCam() const override {
    return &b_Pose_cam_rect_;
  }

  virtual const TrackerStatusSummary* getTrackerStatus() const override {
    return status_mono_measurements_ ? &(status_mono_measurements_->first)
                                     : nullptr;
  }

 public:
  const StatusMonoMeasurementsPtr status_mono_measurements_;
  const gtsam::Pose3 b_Pose_cam_rect_;
  // TODO(nathan) make this name consistent
  /**
   * This member is not necessarily a key-frame and can be one of two things:
   * - The last frame processed (is_keyframe_ = false)
   * - The newest keyframe (is_keyframe_ = true)
   */
  const Frame frame_lkf_;
  const cv::Mat feature_tracks_;
};

}  // namespace VIO
