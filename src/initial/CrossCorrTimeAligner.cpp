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

CrossCorrTimeAligner::CrossCorrTimeAligner(bool do_imu_rate_estimation,
                                           size_t window_size)
    : TimeAlignerBase(),
      do_imu_rate_estimation_(do_imu_rate_estimation),
      imu_buffer_(window_size),
      vision_buffer_(window_size) {}

void CrossCorrTimeAligner::addNewImuData(const ImuStampS& imu_stamps_,
                                         const ImuAccGyrS& imu_acc_gyrs) {}

TimeAlignerBase::Result CrossCorrTimeAligner::attemptEstimation(
    const FrontendOutputPacketBase& input) {
  // TODO(nathan) interpolate camera pose to IMU rate in ring buffer

  // TODO(nathan) cross correlation and delay calculation
  // vision_rotation_angles_(curr_index_) =
  // Rot3::Logmap((*input.stereo_ransac_body_pose_).rotation()).norm();
  // pim_rotation_angles_(curr_index_) =
  // Rot3::Logmap(input.pim_->deltaRij()).norm();
  // num_measurements_++;

  return {false, 0.0};
}

}  // namespace VIO
