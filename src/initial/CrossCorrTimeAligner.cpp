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
  CHECK_NE(imu_period_s_, 0.0) << "nominal_sampling_time_s_ must be non-zero";
  pim_params_.reset(new gtsam::PreintegratedRotationParams());
}

size_t CrossCorrTimeAligner::addNewImuData(
    const std::vector<Timestamp>& image_stamps,
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_acc_gyrs) {
  if (imu_stamps.cols() == 0) {
    // TODO(nathan) think about handling this better
    LOG(ERROR) << "addNewImuData called with no measurements";
    return 0;
  }

  if (!do_imu_rate_estimation_) {
    CHECK_GE(image_stamps.size(), 2u);
    size_t image_stamp_idx = 1;
    gtsam::PreintegratedRotation rot_pim(pim_params_);
    for (int i = 0; i < imu_stamps.cols(); ++i) {
      if (image_stamp_idx < image_stamps.size() - 1 &&
          image_stamps[image_stamp_idx] < imu_stamps(0, i)) {
        // we hit a boundary between the last frame and the
        // current one, so add a measurement and restart integration
        imu_buffer_.push(CrossCorrTimeAligner::Measurement(
            image_stamps[image_stamp_idx],
            Rot3::Logmap(rot_pim.deltaRij()).norm()));
        rot_pim = gtsam::PreintegratedRotation(pim_params_);
        image_stamp_idx++;
      }

      rot_pim.integrateMeasurement(imu_acc_gyrs.block<3, 1>(3, i),
                                   Eigen::Vector3d::Zero(),
                                   imu_period_s_);
    }
    imu_buffer_.push(CrossCorrTimeAligner::Measurement(
        image_stamps.back(), Rot3::Logmap(rot_pim.deltaRij()).norm()));

    // image_stamp_idx is equal to the number of measurements added
    return image_stamp_idx;
  }

  int start = 0;
  if (imu_buffer_.size() > 0 &&
      imu_buffer_.back().timestamp >= imu_stamps(0, 0)) {
    // this can occur normally  as interpUpperBorder in the DataProvider
    // gives us imu measurements in [prev, curr], so we don't warn here
    start = 1;
  }
  for (int i = start; i < imu_stamps.cols(); ++i) {
    // instantaneous rotation angle for single IMU measurement
    imu_buffer_.push(CrossCorrTimeAligner::Measurement(
        imu_stamps(0, i),
        imu_acc_gyrs.block<3, 1>(3, i).norm() * imu_period_s_));
  }

  return imu_stamps.cols() - start;
}

namespace {

double valueAccessor(const CrossCorrTimeAligner::Measurement& m) {
  return m.value;
}

}  // namespace

void CrossCorrTimeAligner::interpNewImageMeasurements(
    const std::vector<Timestamp>& image_stamps,
    const gtsam::Pose3& T_ref_cur,
    size_t num_new_imu_measurements) {
  const size_t N = num_new_imu_measurements;
  CHECK_LE(N, imu_buffer_.size())
      << "IMU buffer should contain at least the number of new measurements";

  const double angle = Rot3::Logmap(T_ref_cur.rotation()).norm();
  if (N == 1) {
    // we can't interpolate, so just push a single measurement
    vision_buffer_.push(
        CrossCorrTimeAligner::Measurement(image_stamps.back(), angle));
    return;
  }

  // calculate t_j - t_i and v_j - v_i for the images
  const double last_frame_angle =
      vision_buffer_.empty() ? 0.0 : vision_buffer_.back().value;
  const double frame_diff =
      UtilsNumerical::NsecToSec(image_stamps.back() - image_stamps.front());
  const double frame_value_diff = angle - last_frame_angle;

  // if we have no previous measurments, we have to start from the earliest
  // IMU measurement from the current packet. Otherwise, we use the latest
  // IMU measurement from the last packet for our reference for interpolation
  const Timestamp first_imu =
      (imu_buffer_.size() == N)
          ? imu_buffer_[0].timestamp
          : imu_buffer_[imu_buffer_.size() - N - 1].timestamp;
  const double imu_diff =
      UtilsNumerical::NsecToSec(imu_buffer_.back().timestamp - first_imu);
  CHECK_GT(imu_diff, 0.0) << "IMU timestamps did not increase over window!";

  for (size_t i = 0; i < N; ++i) {
    // linear interpolation based on IMU timestamps
    const double curr_diff = UtilsNumerical::NsecToSec(
        imu_buffer_[imu_buffer_.size() - N + i].timestamp - first_imu);
    const double ratio = curr_diff / imu_diff;
    CHECK_GE(ratio, 0.0) << "Invalid ratio between imu timestamps: " << ratio;

    Timestamp new_timestamp =
        image_stamps.front() + UtilsNumerical::SecToNsec(ratio * frame_diff);
    vision_buffer_.push(CrossCorrTimeAligner::Measurement(
        new_timestamp, last_frame_angle + frame_value_diff * ratio));
  }
}

double CrossCorrTimeAligner::getTimeShift() const {
  using std::placeholders::_1;
  std::vector<double> correlation = utils::crossCorrelation(
      vision_buffer_, imu_buffer_, std::bind(valueAccessor, _1));

  // we start in the middle to keep the time shift stable under low
  // correlation
  const size_t N = vision_buffer_.size();
  size_t max_idx = N;
  double max_corr = correlation[N];
  for (size_t i = 1; i < N; ++i) {
    // TODO(nathan) think about a ratio based test
    if (correlation[N - i] > max_corr) {
      max_idx = N - i;
      max_corr = correlation[max_idx];
    }
    if (correlation[N + i] > max_corr) {
      max_idx = N + i;
      max_corr = correlation[max_idx];
    }
  }

  int64_t offset = static_cast<int64_t>(vision_buffer_.size()) -
                   (correlation.size()) + max_idx;
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
  return timeshift;
}

void CrossCorrTimeAligner::logData(FrontendLogger* logger,
                                   size_t num_imu_added,
                                   bool not_enough_data,
                                   bool not_enough_variance,
                                   double result) {
  if (!logger) {
    return;
  }

  // this is a little awkward when doing things at imu rate,
  // as the result and status gets copied for each of the
  // samples even though the status is only is guaranteed to be
  // correct for the last. It's simple enough to piece together
  // later what happened though and this method is simpler
  for (size_t i = 0; i < num_imu_added; ++i) {
    size_t idx = imu_buffer_.size() - num_imu_added + i;
    logger->logFrontendTemporalCal(vision_buffer_[idx].timestamp,
                                   imu_buffer_[idx].timestamp,
                                   vision_buffer_[idx].value,
                                   imu_buffer_[idx].value,
                                   not_enough_data,
                                   not_enough_variance,
                                   result);
  }
}

TimeAlignerBase::Result CrossCorrTimeAligner::attemptEstimation(
    const std::vector<Timestamp>& image_stamps,
    const gtsam::Pose3& T_ref_cur,
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_acc_gyrs,
    FrontendLogger* logger) {
  size_t num_imu_added = addNewImuData(image_stamps, imu_stamps, imu_acc_gyrs);
  VLOG(5) << "Added " << num_imu_added
          << " relative angle(s) to imu buffer for temporal sync";
  if (num_imu_added == 0) {
    LOG(ERROR) << "Failed to add IMU data. Returning default estimate.";
    return {true, 0.0};
  }

  // this covers both the case of only having one measurement (frame rate
  // estimation) and multiple measurements (frame rate with RANSAC failing and
  // imu rate estimation)
  interpNewImageMeasurements(image_stamps, T_ref_cur, num_imu_added);

  if (!vision_buffer_.full()) {
    VLOG(1)
        << "Waiting for enough measurements to perform temporal calibration";
    logData(logger, num_imu_added, true, false, 0.0);
    return {false, 0.0};
  }

  using std::placeholders::_1;
  double imu_variance =
      utils::variance(imu_buffer_, std::bind(valueAccessor, _1));
  if (imu_variance < imu_variance_threshold_) {
    LOG(WARNING) << "Low gyro signal variance, delaying temporal calibration";
    logData(logger, num_imu_added, false, true, 0.0);
    return {false, 0.0};  // signal appears to mostly be noise
  }

  // TODO(nathan) check the vision variance as well

  double timeshift = getTimeShift();
  LOG(WARNING) << "Computed timeshift of " << timeshift
               << "[s] (t_imu = t_cam + timeshift)";
  logData(logger, num_imu_added, false, false, timeshift);
  return {true, timeshift};
}

}  // namespace VIO
