/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   timeAlignerBase.cpp
 * @brief  Class to estimate IMU to camera time offset
 * @author Nathan Hughes
 */

#include "kimera-vio/initial/TimeAlignerBase.h"
#include "kimera-vio/frontend/MonoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/StereoVisionImuFrontend-definitions.h"

namespace VIO {

TimeAlignerBase::Result TimeAlignerBase::estimateTimeAlignment(
    Tracker& tracker,
    const FrontendOutputPacketBase& output,
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_accgyrs) {
  Frame::UniquePtr curr_frame;
  switch (output.frontend_type_) {
    case FrontendType::kMonoImu:
      curr_frame = dynamic_cast<const MonoFrontendOutput&>(output)
                       .frame_lkf_.getRansacFrame();
      break;
    case FrontendType::kStereoImu:
      curr_frame = dynamic_cast<const StereoFrontendOutput&>(output)
                       .stereo_frame_lkf_.left_frame_.getRansacFrame();
      break;
    default:
      LOG(ERROR) << "Unknown frontend output type. Returning default value";
      return {true, 0.0};
  }

  if (!last_frame_) {
    VLOG(1) << "Initializing first frame for temporal calibration";
    last_frame_ = std::move(curr_frame);
    return {false, 0.0};
  }

  CHECK(last_frame_) << "last_frame_ is invalid";
  CHECK(curr_frame) << "curr_frame is invalid";
  TrackingStatusPose ransac_result = tracker.geometricOutlierRejectionMono(
      last_frame_.get(), curr_frame.get());
  auto timestamps_ref_curr =
      std::make_pair(last_frame_->timestamp_, curr_frame->timestamp_);
  last_frame_ = std::move(curr_frame);

  if (ransac_result.first == TrackingStatus::INVALID) {
    LOG(ERROR) << "Time alignment failed 5-pt RANSAC";
    return {true, 0.0};
  }
  if (ransac_result.first == TrackingStatus::DISABLED) {
    LOG(ERROR)
        << "5-pt RANSAC disabled for time-alignment. Returning default value";
    return {true, 0.0};
  }

  return attemptEstimation(timestamps_ref_curr, ransac_result.second, imu_stamps, imu_accgyrs);
}

}  // namespace VIO
