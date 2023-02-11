/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdVisionImuFrontend-definitions.h
 * @brief  Definitions for RgbdVisionImuFrontend
 * @author Marcus Abate
 */

#pragma once

#include <gtsam/geometry/StereoPoint2.h>

#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/FrontendOutputPacketBase.h"
#include "kimera-vio/frontend/RgbdImuSyncPacket.h"
#include "kimera-vio/frontend/VisionImuFrontendParams.h"

namespace VIO {

using RgbdFrontendInputPayload = RgbdImuSyncPacket;
using RgbdFrontendParams = FrontendParams;

// TODO(marcus): need to match this with Rgbd Backend!
using RgbdMeasurement = std::pair<LandmarkId, gtsam::StereoPoint2>;
using RgbdMeasurements = std::vector<RgbdMeasurement>;
using RgbdMeasurementsUniquePtr = std::unique_ptr<RgbdMeasurements>;
using StatusRgbdMeasurements =
    std::pair<TrackerStatusSummary, RgbdMeasurements>;
using StatusRgbdMeasurementsPtr = std::shared_ptr<StatusRgbdMeasurements>;

struct RgbdFrontendOutput : public FrontendOutputPacketBase {
 public:
  KIMERA_POINTER_TYPEDEFS(RgbdFrontendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RgbdFrontendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RgbdFrontendOutput(const bool& is_keyframe,
                     const StatusRgbdMeasurementsPtr& status_rgbd_measurements,
                     const TrackingStatus& tracker_status,
                     const gtsam::Pose3& relative_pose_body_rgbd,
                     const gtsam::Pose3& b_Pose_cam_rect,
                     const RgbdFrame& rgbd_frame_lkf,
                     const ImuFrontend::PimPtr& pim,
                     const ImuAccGyrS& imu_acc_gyrs,
                     const cv::Mat& feature_tracks,
                     const DebugTrackerInfo& debug_tracker_info)
      : FrontendOutputPacketBase(rgbd_frame_lkf.timestamp_,
                                 is_keyframe,
                                 FrontendType::kRgbdImu,
                                 pim,
                                 imu_acc_gyrs,
                                 debug_tracker_info),
        status_rgbd_measurements(status_rgbd_measurements),
        tracker_status_(tracker_status),
        relative_pose_body_rgbd_(relative_pose_body_rgbd),
        b_Pose_cam_rect_(b_Pose_cam_rect),
        rgbd_frame_lkf_(rgbd_frame_lkf),
        feature_tracks_(feature_tracks) {}

  virtual ~RgbdFrontendOutput() = default;

 public:
  const StatusRgbdMeasurementsPtr status_rgbd_measurements;
  const TrackingStatus tracker_status_;
  const gtsam::Pose3 relative_pose_body_rgbd_;
  const gtsam::Pose3 b_Pose_cam_rect_;
  const RgbdFrame rgbd_frame_lkf_;
  const cv::Mat feature_tracks_;
};

}  // namespace VIO
