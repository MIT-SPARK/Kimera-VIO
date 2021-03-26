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

TimeAlignerBase::Result CrossCorrTimeAligner::attemptEstimation(
    const BackendInput& input) {
  if (not input.stereo_ransac_body_pose_) {
    // TODO(nathan) log stuff about needing a pose estimate
    return {false, 0.0};
  }
  vision_rotation_angles_(curr_index_) =
      Rot3::Logmap((*input.stereo_ransac_body_pose_).rotation()).norm();
  pim_rotation_angles_(curr_index_) =
      Rot3::Logmap(input.pim_->deltaRij()).norm();
  num_measurements_++;

  // TODO(nathan) ring buffer here
  // TODO(nathan) cross correlation and timestamps here
  return {false, 0.0};
}

}  // namespace VIO
