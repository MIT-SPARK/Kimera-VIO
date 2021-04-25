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

void TimeAlignerBase::mergeImuData(const ImuStampS& latest_stamps,
                                   const ImuAccGyrS& latest_accgyrs,
                                   ImuStampS* new_stamps,
                                   ImuAccGyrS* new_values) {
  CHECK(new_stamps);
  CHECK(new_values);
  if (imu_stamp_cache_.empty()) {
    *new_stamps = latest_stamps;
    *new_values = latest_accgyrs;
    return;
  }

  int total_cols = latest_stamps.cols();
  for (const auto& stamps : imu_stamp_cache_) {
    total_cols += stamps.cols();
  }

  *new_stamps = ImuStampS(1, total_cols);
  *new_values = ImuAccGyrS(6, total_cols);

  int col_offset = 0;
  for (size_t i = 0; i < imu_stamp_cache_.size(); ++i) {
    new_stamps->block(0, col_offset, 1, imu_stamp_cache_[i].cols()) =
        imu_stamp_cache_[i];
    new_values->block(0, col_offset, 6, imu_stamp_cache_[i].cols()) =
        imu_value_cache_[i];
    col_offset += imu_stamp_cache_[i].cols();
  }

  new_stamps->block(0, col_offset, 1, latest_stamps.cols()) = latest_stamps;
  new_values->block(0, col_offset, 6, latest_stamps.cols()) = latest_accgyrs;

  imu_stamp_cache_.clear();
  imu_value_cache_.clear();
}

TimeAlignerBase::Result TimeAlignerBase::estimateTimeAlignment(
    Tracker& tracker,
    const FrontendOutputPacketBase& output,
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_accgyrs,
    FrontendLogger* logger) {
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

  // cache timestamp (so we can figure out how many frames occurred
  // before RANSAC succeeded
  image_stamp_cache_.push_back(curr_frame->timestamp_);

  if (!last_frame_) {
    VLOG(1) << "Initializing first frame for temporal calibration";
    last_frame_ = std::move(curr_frame);
    return {false, 0.0};
  }

  CHECK(last_frame_) << "last_frame_ is invalid";
  CHECK(curr_frame) << "curr_frame is invalid";
  TrackingStatusPose ransac_result = tracker.geometricOutlierRejectionMono(
      last_frame_.get(), curr_frame.get());
  if (ransac_result.first == TrackingStatus::INVALID) {
    LOG(ERROR) << "Time alignment failed 5-pt RANSAC";
    return {true, 0.0};
  }
  if (ransac_result.first == TrackingStatus::DISABLED) {
    LOG(ERROR)
        << "5-pt RANSAC disabled for time-alignment. Returning default value";
    return {true, 0.0};
  }

  if (ransac_result.first != TrackingStatus::VALID) {
    VLOG(1) << "RANSAC for temporal sync failed due to "
            << (ransac_result.first == TrackingStatus::FEW_MATCHES
                    ? "too few matches"
                    : "too little disparity")
            << ". Caching IMU measurements";
    imu_stamp_cache_.push_back(imu_stamps);
    imu_value_cache_.push_back(imu_accgyrs);
    return {false, 0.0};
  }

  ImuStampS new_imu_stamps;
  ImuAccGyrS new_imu_accgyrs;
  mergeImuData(imu_stamps, imu_accgyrs, &new_imu_stamps, &new_imu_accgyrs);

  VLOG(5) << "temporal sync replacing frame #" << last_frame_->id_
          << " with frame #" << curr_frame->id_;
  last_frame_ = std::move(curr_frame);

  Result result = attemptEstimation(image_stamp_cache_,
                                    ransac_result.second,
                                    new_imu_stamps,
                                    new_imu_accgyrs,
                                    logger);
  image_stamp_cache_.clear();
  image_stamp_cache_.push_back(last_frame_->timestamp_);
  return result;
}

}  // namespace VIO
