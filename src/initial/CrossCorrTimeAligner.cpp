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

#include "kimera-vio/utils/UtilsNumerical.h"

namespace VIO {

CrossCorrTimeAligner::CrossCorrTimeAligner(const ImuParams& params)
    : TimeAlignerBase(),
      do_imu_rate_estimation_(params.do_imu_rate_time_alignment_),
      imu_period_s_(params.nominal_sampling_time_s_),
      imu_variance_threshold_(std::pow(params.gyro_noise_density_, 2.0)),
      window_size_s_(params.time_alignment_window_size_s_) {
  imu_variance_threshold_ *= params.time_alignment_variance_threshold_scaling_;
}

void CrossCorrTimeAligner::doFirstFrameSetup(const Frame& frame) {
  size_t window_size;
  if (do_imu_rate_estimation_) {
    CHECK_NE(0.0, imu_period_s_);
    window_size = window_size_s_ / imu_period_s_;
  } else {
    // TODO(nathan) frame_rate_ is actually the period
    window_size = window_size_s_ / frame.cam_param_.frame_rate_;
  }
  CHECK_NE(0, window_size);
  imu_buffer_.reset(new Buffer(window_size));
  vision_buffer_.reset(new Buffer(window_size));
}

size_t CrossCorrTimeAligner::addNewImuDataImuRate(
    const std::vector<Timestamp>& image_stamps,
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_acc_gyrs) {
  size_t num_added = 0;
  for (int i = 0; i < imu_stamps.cols(); ++i) {
    if (imu_buffer_->size() > 0 &&
        imu_buffer_->back().timestamp >= imu_stamps(0, i)) {
      // this can occur normally  as interpUpperBorder in the DataProvider
      // gives us imu measurements in [prev, curr], so we don't warn here
      continue;
    }

    if (image_stamps.back() <= imu_stamps(0, i)) {
      // this can occur normally  as interpUpperBorder in the DataProvider
      // gives us imu measurements in [prev, curr], so we don't warn here
      break;
    }

    // instantaneous rotation angle for single IMU measurement
    imu_buffer_->push(CrossCorrTimeAligner::Measurement(
        imu_stamps(0, i),
        imu_acc_gyrs.block<3, 1>(3, i).norm() * imu_period_s_));
    num_added++;
  }

  return num_added;
}

using ParamType = gtsam::PreintegratedRotation::Params;

struct FakePreintegratedRotation : public gtsam::PreintegratedRotation {
  using gtsam::PreintegratedRotation::p_;
  using ParamPtr = decltype(p_);
};

gtsam::PreintegratedRotation makeNewRotPim() {
  FakePreintegratedRotation::ParamPtr params;
  params.reset(new ParamType());
  return gtsam::PreintegratedRotation(params);
}

size_t CrossCorrTimeAligner::addNewImuDataFrameRate(
    const std::vector<Timestamp>& image_stamps,
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_acc_gyrs) {
  CHECK_GE(image_stamps.size(), 2u);
  size_t image_stamp_idx = 1;
  auto rot_pim = makeNewRotPim();
  for (int i = 0; i < imu_stamps.cols() - 1; ++i) {
    if (image_stamp_idx < image_stamps.size() - 1 &&
        image_stamps[image_stamp_idx] < imu_stamps(0, i)) {
      // this IMU measurement is between the current and next frame
      // so add a measurement and restart integration
      imu_buffer_->push(CrossCorrTimeAligner::Measurement(
          image_stamps[image_stamp_idx - 1],
          Rot3::Logmap(rot_pim.deltaRij()).norm()));
      auto rot_pim = makeNewRotPim();
      image_stamp_idx++;
    }

    rot_pim.integrateMeasurement(
        imu_acc_gyrs.block<3, 1>(3, i), Eigen::Vector3d::Zero(), imu_period_s_);
  }

  const double last_dt = UtilsNumerical::NsecToSec(
      image_stamps.back() - imu_stamps[imu_stamps.cols() - 1]);
  if (last_dt > 0.0) {
    // integrate the last IMU measurement up to the frame timestamp
    rot_pim.integrateMeasurement(
        imu_acc_gyrs.block<3, 1>(3, imu_stamps.cols() - 1),
        Eigen::Vector3d::Zero(),
        last_dt);
  }

  imu_buffer_->push(CrossCorrTimeAligner::Measurement(
      image_stamps[image_stamp_idx - 1],
      Rot3::Logmap(rot_pim.deltaRij()).norm()));

  // image_stamp_idx is equal to the number of measurements added
  return image_stamp_idx;
}

size_t CrossCorrTimeAligner::addNewImuData(
    const std::vector<Timestamp>& image_stamps,
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_acc_gyrs) {
  if (imu_stamps.cols() == 0) {
    LOG(ERROR) << "addNewImuData called with no measurements";
    return 0;
  }

  if (do_imu_rate_estimation_) {
    return addNewImuDataImuRate(image_stamps, imu_stamps, imu_acc_gyrs);
  } else {
    return addNewImuDataFrameRate(image_stamps, imu_stamps, imu_acc_gyrs);
  }
}

namespace {

double valueAccessor(const CrossCorrTimeAligner::Measurement& m) {
  return m.value;
}

}  // namespace

void CrossCorrTimeAligner::interpNewImageMeasurements(
    const std::vector<Timestamp>& image_stamps,
    const gtsam::Pose3& T_ref_cur,
    const size_t& num_new_imu_measurements) {
  const size_t N = num_new_imu_measurements;
  CHECK_LE(N, imu_buffer_->size())
      << "IMU buffer should contain at least the number of new measurements";

  const double angle = Rot3::Logmap(T_ref_cur.rotation()).norm();
  if (N == 1) {
    // special case: keep things simple
    vision_buffer_->push(
        CrossCorrTimeAligner::Measurement(image_stamps.front(), angle));
    return;
  }

  // compute image quantities
  const double frame_diff =
      UtilsNumerical::NsecToSec(image_stamps.back() - image_stamps.front());
  const double frame_angle = angle / N;  // constant velocity assumption

  // compute IMU quantitities
  const Timestamp first_imu = (*imu_buffer_)[imu_buffer_->size() - N].timestamp;
  const Timestamp last_imu = imu_buffer_->back().timestamp;
  const double imu_diff = UtilsNumerical::NsecToSec(last_imu - first_imu);
  CHECK_GT(imu_diff, 0.0) << "IMU timestamps did not increase over window!";

  for (size_t i = 0; i < N; ++i) {
    // linear interpolation based on IMU timestamps
    const double curr_diff = UtilsNumerical::NsecToSec(
        (*imu_buffer_)[imu_buffer_->size() - N + i].timestamp - first_imu);
    const double ratio = curr_diff / imu_diff;
    CHECK_GE(ratio, 0.0) << "Invalid ratio between imu timestamps: " << ratio;

    Timestamp new_timestamp =
        image_stamps.front() + UtilsNumerical::SecToNsec(ratio * frame_diff);
    vision_buffer_->push(
        CrossCorrTimeAligner::Measurement(new_timestamp, frame_angle));
  }
}

template <typename T>
std::string getBufferStr(const T& input) {
  std::stringstream ss;
  ss << "[";

  auto iter = input.begin();
  while (iter != input.end()) {
    ss << *iter;
    ++iter;
    if (iter != input.end()) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

size_t CrossCorrTimeAligner::getMaxFromN(const std::vector<double>& values,
                                         size_t N) {
  CHECK_LT(N, values.size());

  // we either need to traverse values.size() - N - 1 values to cover the entire
  // right-hand side of the buffer or N values to cover the entire left-hand
  // side of the buffer, depending on whether N is closer to 0 or to
  // values.size()
  size_t limit = std::max(values.size() - N, N + 1);

  // we step uniformly from N to make sure that we pick the nearest duplicate
  // maxium value to N
  size_t max_idx = N;
  double max_value = values[N];
  for (size_t i = 1; i < limit; ++i) {
    if (i <= N && values[N - i] > max_value) {
      max_idx = N - i;
      max_value = values[max_idx];
    }

    if (N + i < values.size() && values[N + i] > max_value) {
      max_idx = N + i;
      max_value = values[max_idx];
    }
  }

  VLOG(5) << "Max index: " << max_idx << " with value: " << max_value;
  return max_idx;
}

double CrossCorrTimeAligner::getTimeShift() const {
  using std::placeholders::_1;
  const auto correlation = utils::crossCorrelation(
      *vision_buffer_, *imu_buffer_, std::bind(valueAccessor, _1));

  if (VLOG_IS_ON(5)) {
    VLOG(5) << "Vision: " << getBufferStr(*vision_buffer_);
    VLOG(5) << "IMU: " << getBufferStr(*imu_buffer_);
    VLOG(5) << "Correlation: " << getBufferStr(correlation);
    VLOG(5) << "IMU size: " << imu_buffer_->size();
    VLOG(5) << "Vision size: " << vision_buffer_->size();
  }

  const size_t max_idx = getMaxFromN(correlation, vision_buffer_->size());
  int64_t offset = static_cast<int64_t>(vision_buffer_->size()) -
                   (correlation.size()) + max_idx;
  double timeshift = 0.0;
  if (max_idx >= vision_buffer_->size()) {
    timeshift =
        UtilsNumerical::NsecToSec((*imu_buffer_)[std::abs(offset)].timestamp -
                                  vision_buffer_->front().timestamp);
  } else {
    timeshift =
        UtilsNumerical::NsecToSec(vision_buffer_->front().timestamp -
                                  (*imu_buffer_)[std::abs(offset)].timestamp);
  }
  return timeshift;
}

void CrossCorrTimeAligner::logData(FrontendLogger* logger,
                                   const size_t& num_imu_added,
                                   const bool& not_enough_data,
                                   const bool& not_enough_variance,
                                   const double& result) {
  if (!logger) {
    return;
  }

  // this is a little awkward when doing things at imu rate,
  // as the result and status gets copied for each of the
  // samples even though the status is only is guaranteed to be
  // correct for the last. It's simple enough to piece together
  // later what happened though and this method is simpler
  for (size_t i = 0; i < num_imu_added; ++i) {
    size_t idx = imu_buffer_->size() - num_imu_added + i;
    logger->logFrontendTemporalCal((*vision_buffer_)[idx].timestamp,
                                   (*imu_buffer_)[idx].timestamp,
                                   (*vision_buffer_)[idx].value,
                                   (*imu_buffer_)[idx].value,
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
  CHECK(vision_buffer_) << "Initial setup not performed";
  CHECK(imu_buffer_) << "Initial setup not performed";
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

  if (!vision_buffer_->full()) {
    VLOG(1)
        << "Waiting for enough measurements to perform temporal calibration";
    logData(logger, num_imu_added, true, false, 0.0);
    return {false, 0.0};
  }

  // RANSAC should match up pretty well to the IMU, so we only check the
  // variance of the IMU signal (as expected variance of RANSAC is harder to
  // capture)
  using std::placeholders::_1;
  double imu_variance =
      utils::variance(*imu_buffer_, std::bind(valueAccessor, _1));
  VLOG(1) << "Computed IMU variance of " << imu_variance
          << "(threshold: " << imu_variance_threshold_ << ")";
  if (imu_variance < imu_variance_threshold_) {
    LOG(WARNING) << "Low gyro signal variance, delaying temporal calibration";
    logData(logger, num_imu_added, false, true, 0.0);
    return {false, 0.0};  // signal appears to mostly be noise
  }

  double timeshift = getTimeShift();
  LOG(WARNING) << "Computed timeshift of " << timeshift
               << "[s] (t_imu = t_cam + timeshift)";
  logData(logger, num_imu_added, false, false, timeshift);
  return {true, timeshift};
}

}  // namespace VIO
