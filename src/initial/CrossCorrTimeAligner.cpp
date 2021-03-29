/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   crossCorrTimeAligner.cpp
 * @brief  Class to estimate IMU to camera time offset by cross-correlation
 * @author Nathan Hughes
 */

#include "kimera-vio/initial/CrossCorrTimeAligner.h"

namespace VIO {

CrossCorrTimeAligner::CrossCorrTimeAligner(double imu_time_shift_est,
                                           bool should_estimate,
                                           size_t window_size)
    : TimeAlignerBase(imu_time_shift_est, should_estimate),
      window_size_(window_size),
      num_measurements_(0),
      curr_index_(0),
      vision_rotation_angles_(window_size),
      pim_rotation_angles_(window_size) {}

void CrossCorrTimeAligner::addNewImuData(const ImuStampS& imu_stamps_,
                                         const ImuAccGyrS& imu_acc_gyrs) {
  // TODO(nathan) ring buffer of IMU norms here
  // TODO(nathan) ring buffer of IMU timestamps here
}

TimeAlignerBase::Result CrossCorrTimeAligner::attemptEstimation(
    const FrontendOutputPacketBase& input) {
  // TODO(nathan) interpolate camera pose to IMU rate in ring buffer

  if (num_measurements_ < window_size_) {
    // we're still trying to accumulate enough measurements
    return {false, 0.0};
  }

  // TODO(nathan) cross correlation and delay calculation
  // vision_rotation_angles_(curr_index_) =
  // Rot3::Logmap((*input.stereo_ransac_body_pose_).rotation()).norm();
  // pim_rotation_angles_(curr_index_) =
  // Rot3::Logmap(input.pim_->deltaRij()).norm();
  // num_measurements_++;

  return {false, 0.0};
}

}  // namespace VIO
