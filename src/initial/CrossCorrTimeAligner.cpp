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

CrossCorrTimeAligner::CrossCorrTimeAligner(const ImuParams& params)
    : TimeAlignerBase(),
      do_imu_rate_estimation_(params.do_imu_rate_time_alignment_),
      imu_period_s_(params.nominal_sampling_time_s_),
      imu_variance_threshold_(3 * std::pow(params.gyro_noise_density_, 2.0)),
      imu_buffer_(params.time_alignment_window_size_),
      vision_buffer_(params.time_alignment_window_size_) {
  pim_params_.reset(new gtsam::PreintegratedRotationParams());
}

bool CrossCorrTimeAligner::addNewImuData_(Timestamp frame_timestamp,
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
      imu_buffer_.push(CrossCorrTimeAligner::Measurement(
          imu_stamps(0, i),
          imu_acc_gyrs.block<3, 1>(3, i).norm() * imu_period_s_));
    }
  }

  return true;
}

namespace {

double valueAccessor(const CrossCorrTimeAligner::Measurement& m) {
  return m.value;
}

}  // namespace

TimeAlignerBase::Result CrossCorrTimeAligner::attemptEstimation(
    const std::pair<Timestamp, Timestamp>& timestamps_ref_cur,
    const gtsam::Pose3& T_ref_cur,
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_acc_gyrs) {
  if (!addNewImuData_(timestamps_ref_cur.first, imu_stamps, imu_acc_gyrs)) {
    LOG(ERROR) << "Failed to add IMU data. Returning default estimate.";
    return {true, 0.0};
  }

  if (do_imu_rate_estimation_) {
    const size_t N = imu_stamps.cols();
    const double angle = Rot3::Logmap(T_ref_cur.rotation()).norm();
    double frame_diff = UtilsNumerical::NsecToSec(
        timestamps_ref_cur.first - vision_buffer_.back().timestamp);
    double imu_diff = UtilsNumerical::NsecToSec(
        imu_buffer_.back().timestamp -
        imu_buffer_[imu_buffer_.size() - N].timestamp);

    for (size_t i = 0; i < N; ++i) {
      const size_t index = imu_buffer_.size() - N + i;
      double ratio = UtilsNumerical::NsecToSec(
                         imu_buffer_[index].timestamp -
                         imu_buffer_[imu_buffer_.size() - N].timestamp) /
                     imu_diff;
      CHECK_GE(ratio, 0.0) << "Invalid ratio between imu timestamps: " << ratio;
      Timestamp new_timestamp = vision_buffer_.back().timestamp +
                                UtilsNumerical::SecToNsec(ratio * frame_diff);
      vision_buffer_.push(
          CrossCorrTimeAligner::Measurement(new_timestamp, angle * ratio));
    }
  } else {
    vision_buffer_.push(CrossCorrTimeAligner::Measurement(
        timestamps_ref_cur.first, Rot3::Logmap(T_ref_cur.rotation()).norm()));
  }

  if (!vision_buffer_.full()) {
    return {false, 0.0};
  }

  using std::placeholders::_1;
  double imu_variance =
      utils::variance(imu_buffer_, std::bind(valueAccessor, _1));
  if (imu_variance < imu_variance_threshold_) {
    return {false, 0.0};  // signal appears to mostly be noise
  }

  // TODO(nathan) check the vision variance as well

  std::vector<double> correlation = utils::crossCorrelation(
      vision_buffer_, imu_buffer_, std::bind(valueAccessor, _1));
  size_t max_idx =
      std::distance(correlation.begin(),
                    std::max_element(correlation.begin(), correlation.end()));
  int64_t offset = static_cast<int64_t>(vision_buffer_.size()) -
                   correlation.size() + max_idx;

  double timeshift = 0.0;
  if (max_idx >= vision_buffer_.size()) {
    timeshift =
        UtilsNumerical::NsecToSec(imu_buffer_[std::abs(offset)].timestamp -
                                  vision_buffer_.front().timestamp);
  } else {
    timeshift =
        UtilsNumerical::NsecToSec(vision_buffer_.front().timestamp -
                                  imu_buffer_[std::abs(offset)].timestamp);
  }
  LOG(WARNING) << "Computed timeshift of " << timeshift
               << "[s] (t_imu = t_cam + timeshift)";
  return {true, timeshift};
}

}  // namespace VIO
