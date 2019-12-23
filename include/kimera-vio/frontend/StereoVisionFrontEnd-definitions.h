/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVisionFrontEnd-definitions.h
 * @brief  Definitions for VioBackEnd
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/StereoFrame-definitions.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd.h"
#include "kimera-vio/pipeline/PipelinePayload.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

enum class FrontendType {
  //! Frontend that works with Stereo camera and Imu
  StereoImu = 0
};

struct FrontendOutput : public PipelinePayload {
 public:
  KIMERA_POINTER_TYPEDEFS(FrontendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(FrontendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FrontendOutput(const bool is_keyframe,
                 const StatusStereoMeasurementsPtr& status_stereo_measurements,
                 const TrackingStatus& tracker_status,
                 const gtsam::Pose3& relative_pose_body_stereo,
                 const StereoFrame& stereo_frame_lkf,
                 // Use rvalue reference: FrontendOutput owns pim now.
                 const ImuFrontEnd::PimPtr& pim,
                 const DebugTrackerInfo& debug_tracker_info)
      : PipelinePayload(stereo_frame_lkf.getTimestamp()),
        is_keyframe_(is_keyframe),
        status_stereo_measurements_(status_stereo_measurements),
        tracker_status_(tracker_status),
        relative_pose_body_stereo_(relative_pose_body_stereo),
        stereo_frame_lkf_(stereo_frame_lkf),
        pim_(pim),
        debug_tracker_info_(debug_tracker_info) {}

  virtual ~FrontendOutput() = default;

 public:
  const bool is_keyframe_;
  const StatusStereoMeasurementsPtr status_stereo_measurements_;
  const TrackingStatus tracker_status_;
  const gtsam::Pose3 relative_pose_body_stereo_;
  const StereoFrame stereo_frame_lkf_;
  const ImuFrontEnd::PimPtr pim_;
  const DebugTrackerInfo debug_tracker_info_;

  inline DebugTrackerInfo getTrackerInfo() { return debug_tracker_info_; }
};

}  // namespace VIO
