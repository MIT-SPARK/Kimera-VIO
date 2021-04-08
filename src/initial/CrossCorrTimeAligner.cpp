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
                                           double imu_period_s,
                                           size_t window_size)
    : TimeAlignerBase(),
      do_imu_rate_estimation_(do_imu_rate_estimation),
      imu_period_s_(imu_period_s),
      imu_buffer_(window_size),
      vision_buffer_(window_size) {
  pim_params_.reset(new gtsam::PreintegratedRotationParams());
}

bool CrossCorrTimeAligner::add_new_imu_data_(Timestamp frame_timestamp,
                                             const ImuStampS& imu_stamps,
                                             const ImuAccGyrS& imu_acc_gyrs) {
  if (imu_stamps.cols() == 0) {
    // TODO(nathan) think about handling this better
    LOG(ERROR) << "addNewImuData called with no measurements";
    return false;
  }

  if (!do_imu_rate_estimation_) {
    gtsam::PreintegratedRotation rot_pim(pim_params_);
    for (int i = 0; i < imu_stamps.cols(); ++i) {
      // TODO(nathan) think about incorporating bias, though mean removal should
      // take care of it and not affect the cross-correlation
      rot_pim.integrateMeasurement(imu_acc_gyrs.block<3, 1>(0, i),
                                   Eigen::Vector3d::Zero(),
                                   imu_period_s_);
    }
    imu_buffer_.push(CrossCorrTimeAligner::Measurement(
        frame_timestamp, Rot3::Logmap(rot_pim.deltaRij()).norm()));
  } else {
    for (int i = 0; i < imu_stamps.cols(); ++i) {
      // TODO(nathan) think about multiplying by dt
      imu_buffer_.push(CrossCorrTimeAligner::Measurement(
          imu_stamps(0, i), imu_acc_gyrs.block<3, 1>(3, i).norm()));
    }
  }

  return true;
}

TimeAlignerBase::Result CrossCorrTimeAligner::attemptEstimation(
    const std::pair<Timestamp, Timestamp>& timestamps_ref_cur,
    const gtsam::Pose3& T_ref_cur,
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_acc_gyrs) {
  if (!add_new_imu_data_(timestamps_ref_cur.first, imu_stamps, imu_acc_gyrs)) {
    LOG(ERROR) << "Failed to add IMU data. Returning default estimate.";
    return {true, 0.0};
  }

  if (do_imu_rate_estimation_) {
  /*  size_t N = imu_buffer_.size() - vision_buffer_.size();*/
    //Eigen::Vector3d interp_angle = Rot3::Logmap(T_ref_cur.rotation()).norm() / N;
    //for (size_t i = 0; i < N; ++i) {
      //vision_buffer_.push(...);
    /*}*/
  } else {
    vision_buffer_.push(CrossCorrTimeAligner::Measurement(
        timestamps_ref_cur.first, Rot3::Logmap(T_ref_cur.rotation()).norm()));
  }

  if (!vision_buffer_.full()) {
    return {false, 0.0};
  }

  //TODO(nathan) cross correlation

  return {true, 0.0};
}

}  // namespace VIO
