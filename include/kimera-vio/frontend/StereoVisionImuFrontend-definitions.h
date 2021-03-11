/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVisionImuFrontend-definitions.h
 * @brief  Definitions for StereoVisionImuFrontend
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/FrontendOutputPacketBase.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/VisionImuFrontend-definitions.h"

namespace VIO {

struct StereoFrontendOutput : public FrontendOutputPacketBase {
 public:
  KIMERA_POINTER_TYPEDEFS(StereoFrontendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoFrontendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StereoFrontendOutput(
      const bool is_keyframe,
      const StatusStereoMeasurementsPtr& status_stereo_measurements,
      const TrackingStatus& tracker_status,
      const gtsam::Pose3& relative_pose_body_stereo,
      const gtsam::Pose3& b_Pose_camL_rect,
      const gtsam::Pose3& b_Pose_camR_rect,
      const StereoFrame& stereo_frame_lkf,
      // Use rvalue reference: FrontendOutput owns pim now.
      const ImuFrontend::PimPtr& pim,
      const ImuAccGyrS& imu_acc_gyrs,
      const cv::Mat& feature_tracks,
      const DebugTrackerInfo& debug_tracker_info)
      : FrontendOutputPacketBase(stereo_frame_lkf.timestamp_,
                                 is_keyframe,
                                 FrontendType::kStereoImu,
                                 pim,
                                 imu_acc_gyrs,
                                 debug_tracker_info),
        status_stereo_measurements_(status_stereo_measurements),
        tracker_status_(tracker_status),
        relative_pose_body_stereo_(relative_pose_body_stereo),
        b_Pose_camL_rect_(b_Pose_camL_rect),
        b_Pose_camR_rect_(b_Pose_camR_rect),
        stereo_frame_lkf_(stereo_frame_lkf),
        feature_tracks_(feature_tracks) {}

  virtual ~StereoFrontendOutput() = default;

 public:
  const StatusStereoMeasurementsPtr status_stereo_measurements_;
  const TrackingStatus tracker_status_;
  const gtsam::Pose3 relative_pose_body_stereo_;
  const gtsam::Pose3 b_Pose_camL_rect_;
  const gtsam::Pose3 b_Pose_camR_rect_;
  const StereoFrame stereo_frame_lkf_;
  const cv::Mat feature_tracks_;
};

}  // namespace VIO
