/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoVisionFrontEnd-definitions.h
 * @brief  Definitions for MonoVisionFrontEnd
 * @author Marcus Abate
 */

#pragma once

#include <gtsam/geometry/StereoPoint2.h>

#include "kimera-vio/frontend/MonoImuSyncPacket.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd.h"
#include "kimera-vio/pipeline/PipelinePayload.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

using MonoFrontEndInputPayload = MonoImuSyncPacket;
using MonoFrontendParams = FrontendParams;

// TODO(marcus): need to match this with mono backend!
using MonoMeasurement = std::pair<LandmarkId, gtsam::StereoPoint2>;
using MonoMeasurements = std::vector<MonoMeasurement>;
using MonoMeasurementsUniquePtr = std::unique_ptr<MonoMeasurements>;
using StatusMonoMeasurements =
    std::pair<TrackerStatusSummary, MonoMeasurements>;
using StatusMonoMeasurementsPtr = std::shared_ptr<StatusMonoMeasurements>;

struct MonoFrontendOutput : public PipelinePayload {
 public:
  KIMERA_POINTER_TYPEDEFS(MonoFrontendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoFrontendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MonoFrontendOutput(const bool is_keyframe,
                     const StatusMonoMeasurementsPtr& status_mono_measurements,
                     const TrackingStatus& tracker_status,
                     const gtsam::Pose3& relative_pose_body,
                     const gtsam::Pose3& b_Pose_cam_rect,
                     const Frame& frame_lkf,
                     const ImuFrontEnd::PimPtr& pim,
                     const ImuAccGyrS& imu_acc_gyrs,
                     const cv::Mat& feature_tracks,
                     const DebugTrackerInfo& debug_tracker_info)
      : PipelinePayload(frame_lkf.timestamp_),
        is_keyframe_(is_keyframe),
        status_mono_measurements_(status_mono_measurements),
        tracker_status_(tracker_status),
        relative_pose_body_(relative_pose_body),
        b_Pose_cam_rect_(b_Pose_cam_rect),
        frame_lkf_(frame_lkf),
        pim_(pim),
        imu_acc_gyrs_(imu_acc_gyrs),
        feature_tracks_(feature_tracks),
        debug_tracker_info_(debug_tracker_info) {}

  virtual ~MonoFrontendOutput() = default;

 public:
  const bool is_keyframe_;
  const StatusMonoMeasurementsPtr status_mono_measurements_;
  const TrackingStatus tracker_status_;
  const gtsam::Pose3 relative_pose_body_;
  const gtsam::Pose3 b_Pose_cam_rect_;
  const Frame frame_lkf_;
  const ImuFrontEnd::PimPtr pim_;
  const ImuAccGyrS imu_acc_gyrs_;
  const DebugTrackerInfo debug_tracker_info_;
  const cv::Mat feature_tracks_;

  inline DebugTrackerInfo getTrackerInfo() const { return debug_tracker_info_; }
};

}  // namespace VIO
