/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVisionFrontEnd-definitions.h
 * @brief  Definitions for StereoVisionFrontEnd
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/FrontendOutputPacketBase.h"
#include "kimera-vio/frontend/StereoFrame-definitions.h"
#include "kimera-vio/frontend/Tracker.h"


namespace VIO {

// TODO(marcus): move this to VisionFrontEnd.h 
//  (raises weird errors, includes must be wrong somewhere).
enum class FrontendType {
  //! Frontend that works with Mono camera and Imu
  kMonoImu = 0,
  //! Frontend that works with Stereo camera and Imu
  kStereoImu = 1
};

struct StereoFrontendOutput : public FrontendOutputPacketBase {
 public:
  KIMERA_POINTER_TYPEDEFS(StereoFrontendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoFrontendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StereoFrontendOutput(const bool is_keyframe,
                 const StatusStereoMeasurementsPtr& status_stereo_measurements,
                 const TrackingStatus& tracker_status,
                 const gtsam::Pose3& relative_pose_body_stereo,
                 const gtsam::Pose3& b_Pose_camL_rect,
                 const StereoFrame& stereo_frame_lkf,
                 // Use rvalue reference: FrontendOutput owns pim now.
                 const ImuFrontEnd::PimPtr& pim,
                 const ImuAccGyrS& imu_acc_gyrs,
                 const cv::Mat& feature_tracks,
                 const DebugTrackerInfo& debug_tracker_info)
      : FrontendOutputPacketBase(stereo_frame_lkf.timestamp_,
                                 is_keyframe,
                                 pim,
                                 imu_acc_gyrs),
        status_stereo_measurements_(status_stereo_measurements),
        tracker_status_(tracker_status),
        relative_pose_body_stereo_(relative_pose_body_stereo),
        b_Pose_camL_rect_(b_Pose_camL_rect),
        stereo_frame_lkf_(stereo_frame_lkf),
        debug_tracker_info_(debug_tracker_info),
        feature_tracks_(feature_tracks) {}

  virtual ~StereoFrontendOutput() = default;

 public:
  const StatusStereoMeasurementsPtr status_stereo_measurements_;
  const TrackingStatus tracker_status_;
  const gtsam::Pose3 relative_pose_body_stereo_;
  const gtsam::Pose3 b_Pose_camL_rect_;
  const StereoFrame stereo_frame_lkf_;
  const DebugTrackerInfo debug_tracker_info_;
  const cv::Mat feature_tracks_;

  inline DebugTrackerInfo getTrackerInfo() const { return debug_tracker_info_; }
};

}  // namespace VIO
