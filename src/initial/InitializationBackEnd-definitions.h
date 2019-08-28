/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   InitializationBackEnd-definitions.h
 * @brief  Derived class initialization input payload.
 * @author Antoni Rosinol
 * @author Sandro Berchier
 * @author Luca Carlone
 */

#pragma once

#include <gtsam/navigation/AHRSFactor.h>

#include "StereoVisionFrontEnd-definitions.h"

namespace VIO {

enum InitializationModes { GT, IMU, ALIGNMENT };

struct InitializationInputPayload : public StereoFrontEndOutputPayload {
 public:
  // Default constructor
  InitializationInputPayload(
      const bool is_keyframe,
      const StatusSmartStereoMeasurements& statusSmartStereoMeasurements,
      const TrackingStatus& tracker_status,
      const gtsam::Pose3& relative_pose_body_stereo,
      const StereoFrame& stereo_frame_lkf,
      const ImuFrontEnd::PreintegratedImuMeasurements& pim,
      const DebugTrackerInfo& debug_tracker_info,
      const gtsam::AHRSFactor::PreintegratedMeasurements& ahrs_pim =
          gtsam::AHRSFactor::PreintegratedMeasurements())
      : StereoFrontEndOutputPayload(is_keyframe,
                                    statusSmartStereoMeasurements,
                                    tracker_status,
                                    relative_pose_body_stereo,
                                    stereo_frame_lkf,
                                    pim,
                                    debug_tracker_info),
        ahrs_pim_(ahrs_pim) {}

 public:
  const gtsam::AHRSFactor::PreintegratedMeasurements ahrs_pim_;
};

}  // namespace VIO
