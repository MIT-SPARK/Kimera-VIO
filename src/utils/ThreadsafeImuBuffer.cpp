/*******************************************************************************
 Copyright 2017 Autonomous Systems Lab, ETH Zurich, Switzerland

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*******************************************************************************/

/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ThreadsafeImuBuffer.cpp
 * @brief  Threadsafe Imu Buffer with timestamp lookup.
 * @author Antoni Rosinol
 */

#include <chrono>
#include <iostream>

#include <algorithm>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/utils/ThreadsafeImuBuffer.h"
#include "kimera-vio/utils/Timer.h"

namespace VIO {

namespace utils {

template <template <typename, typename> class Container, typename Type>
using Aligned = Container<Type, Eigen::aligned_allocator<Type>>;

ThreadsafeImuBuffer::QueryResult ThreadsafeImuBuffer::isDataAvailableUpToImpl(
    const Timestamp& timestamp_ns_from,
    const Timestamp& timestamp_ns_to) const {
  CHECK_LT(timestamp_ns_from, timestamp_ns_to);

  if (shutdown_) {
    return QueryResult::kQueueShutdown;
  }

  if (buffer_.empty()) {
    return QueryResult::kDataNotYetAvailable;
  }

  ImuMeasurement value;
  if (buffer_.getNewestValue(&value) && value.timestamp_ < timestamp_ns_to) {
    // This is triggered if the timestamp_ns_to requested exceeds the newest
    // IMU measurement, meaning that there is data still to arrive to reach the
    // requested point in time.
    return QueryResult::kDataNotYetAvailable;
  }

  if (buffer_.getOldestValue(&value) && timestamp_ns_from < value.timestamp_) {
    // This is triggered if the user requests data previous to the oldest IMU
    // measurement present in the buffer, meaning that there is missing data
    // from the timestamp_ns_from requested to the oldest stored timestamp.
    return QueryResult::kDataNeverAvailable;
  }
  return QueryResult::kDataAvailable;
}

void ThreadsafeImuBuffer::linearInterpolate(const Timestamp& t0,
                                            const ImuAccGyr& y0,
                                            const Timestamp& t1,
                                            const ImuAccGyr& y1,
                                            const Timestamp& t,
                                            ImuAccGyr* y) {
  CHECK_NOTNULL(y);
  CHECK_LE(t0, t);
  CHECK_LE(t, t1);
  *y = t0 == t1 ? y0 :  // y0 if t0 == t1, interpolate otherwise:
           y0 + (y1 - y0) * static_cast<double>(t - t0) /
                    static_cast<double>(t1 - t0);
}

ThreadsafeImuBuffer::QueryResult ThreadsafeImuBuffer::getImuDataBtwTimestamps(
    const Timestamp& timestamp_ns_from,
    const Timestamp& timestamp_ns_to,
    ImuStampS* imu_timestamps,
    ImuAccGyrS* imu_measurements,
    bool get_lower_bound) {
  CHECK_NOTNULL(imu_timestamps);
  CHECK_NOTNULL(imu_measurements);
  DCHECK_LT(timestamp_ns_from, timestamp_ns_to);
  QueryResult query_result =
      isDataAvailableUpToImpl(timestamp_ns_from, timestamp_ns_to);
  if (query_result != QueryResult::kDataAvailable) {
    imu_timestamps->resize(Eigen::NoChange, 0);
    imu_measurements->resize(Eigen::NoChange, 0);
    return query_result;
  }

  // TODO can we avoid copying and have the temporal buffer store pairs of
  // timestamps/accgyr data?
  // Copy the data with timestamp_up_to <= timestamps_buffer from the buffer.
  Aligned<std::vector, ImuMeasurement> between_values;
  CHECK(buffer_.getValuesBetweenTimes(timestamp_ns_from, timestamp_ns_to,
                                      &between_values, get_lower_bound));

  if (between_values.empty()) {
    LOG(WARNING) << "No IMU measurements available strictly between time "
                 << timestamp_ns_from << "[ns] and " << timestamp_ns_to
                 << "[ns].";
    imu_timestamps->resize(Eigen::NoChange, 0);
    imu_measurements->resize(Eigen::NoChange, 0);
    return QueryResult::kTooFewMeasurementsAvailable;
  }

  const size_t num_measurements = between_values.size();
  imu_timestamps->resize(Eigen::NoChange, num_measurements);
  imu_measurements->resize(Eigen::NoChange, num_measurements);

  for (size_t idx = 0u; idx < num_measurements; ++idx) {
    (*imu_timestamps)(idx) = between_values[idx].timestamp_;
    (*imu_measurements).col(idx) = between_values[idx].acc_gyr_;
  }

  return query_result;
}

ThreadsafeImuBuffer::QueryResult
ThreadsafeImuBuffer::getImuDataInterpolatedUpperBorder(
    const Timestamp& timestamp_ns_from,
    const Timestamp& timestamp_ns_to,
    ImuStampS* imu_timestamps,
    ImuAccGyrS* imu_measurements) {
  CHECK_NOTNULL(imu_timestamps);
  CHECK_NOTNULL(imu_measurements);
  DCHECK_LT(timestamp_ns_from, timestamp_ns_to);
  // Get data.
  QueryResult query_result = getImuDataBtwTimestamps(
      timestamp_ns_from, timestamp_ns_to, imu_timestamps, imu_measurements,
      true);  // Get lower bound.
  // Early exit if there is no data.
  if (query_result != QueryResult::kDataAvailable) {
    imu_timestamps->resize(Eigen::NoChange, 0);
    imu_measurements->resize(Eigen::NoChange, 0);
    return query_result;
  }

  // Interpolate upper border.
  ImuAccGyr interpolated_upper_border;
  interpolateValueAtTimestamp(timestamp_ns_to, &interpolated_upper_border);

  DCHECK_EQ(imu_timestamps->rows(), 1);
  DCHECK_EQ(imu_measurements->rows(), 6);
  // The last measurement will correspond to the interpolated data.
  const size_t num_measurements = imu_timestamps->cols() + 1u;
  imu_timestamps->conservativeResize(Eigen::NoChange, num_measurements);
  imu_measurements->conservativeResize(Eigen::NoChange, num_measurements);
  // Append upper border.
  imu_timestamps->rightCols<1>()(0) = timestamp_ns_to;
  imu_measurements->rightCols<1>() = interpolated_upper_border;

  return query_result;
}

ThreadsafeImuBuffer::QueryResult
ThreadsafeImuBuffer::getImuDataInterpolatedBorders(
    const Timestamp& timestamp_ns_from,
    const Timestamp& timestamp_ns_to,
    ImuStampS* imu_timestamps,
    ImuAccGyrS* imu_measurements) {
  CHECK_NOTNULL(imu_timestamps);
  CHECK_NOTNULL(imu_measurements);
  // Get data.
  ImuStampS imu_timestamps_tmp;
  ImuAccGyrS imu_measurements_tmp;
  QueryResult query_result =
      getImuDataBtwTimestamps(timestamp_ns_from, timestamp_ns_to,
                              &imu_timestamps_tmp, &imu_measurements_tmp);

  // Early exit if there is no data.
  if (query_result != QueryResult::kDataAvailable) {
    imu_timestamps->resize(Eigen::NoChange, 0);
    imu_measurements->resize(Eigen::NoChange, 0);
    return query_result;
  }

  // Interpolate lower border.
  ImuAccGyr interpolated_lower_border;
  interpolateValueAtTimestamp(timestamp_ns_from, &interpolated_lower_border);
  // Interpolate upper border.
  ImuAccGyr interpolated_upper_border;
  interpolateValueAtTimestamp(timestamp_ns_to, &interpolated_upper_border);

  DCHECK_EQ(imu_timestamps->rows(), 1);
  DCHECK_EQ(imu_measurements->rows(), 6);
  // The first and last measurements will correspond to the interpolated data.
  const size_t num_measurements = imu_timestamps_tmp.cols() + 2u;
  imu_timestamps->resize(Eigen::NoChange, num_measurements);
  imu_measurements->resize(Eigen::NoChange, num_measurements);
  // Prepend lower border.
  imu_timestamps->leftCols<1>()(0) = timestamp_ns_from;
  imu_measurements->leftCols<1>() = interpolated_lower_border;
  // Add measurements.
  imu_timestamps->middleCols(1, imu_timestamps_tmp.cols()) = imu_timestamps_tmp;
  imu_measurements->middleCols(1, imu_measurements_tmp.cols()) =
      imu_measurements_tmp;
  // Append upper border.
  imu_timestamps->rightCols<1>()(0) = timestamp_ns_to;
  imu_measurements->rightCols<1>() = interpolated_upper_border;

  return query_result;
}

void ThreadsafeImuBuffer::interpolateValueAtTimestamp(
    const Timestamp& timestamp_ns,
    ImuAccGyr* interpolated_imu_measurement) {
  CHECK_NOTNULL(interpolated_imu_measurement);
  Timestamp pre_border_timestamp, post_border_timestamp;
  ImuMeasurement pre_border_value, post_border_value;
  CHECK(buffer_.getValueAtOrBeforeTime(timestamp_ns, &pre_border_timestamp,
                                       &pre_border_value))
      << "The IMU buffer seems not to contain measurements at or before time: "
      << timestamp_ns;
  CHECK_EQ(pre_border_timestamp, pre_border_value.timestamp_);
  CHECK(buffer_.getValueAtOrAfterTime(timestamp_ns, &post_border_timestamp,
                                      &post_border_value))
      << "The IMU buffer seems not to contain measurements at or after time: "
      << timestamp_ns;
  CHECK_EQ(post_border_timestamp, post_border_value.timestamp_);
  linearInterpolate(pre_border_value.timestamp_,
                    pre_border_value.acc_gyr_,
                    post_border_value.timestamp_,
                    post_border_value.acc_gyr_,
                    timestamp_ns,
                    interpolated_imu_measurement);
}

ThreadsafeImuBuffer::QueryResult
ThreadsafeImuBuffer::getImuDataInterpolatedBordersBlocking(
    const Timestamp& timestamp_ns_from,
    const Timestamp& timestamp_ns_to,
    const Timestamp& wait_timeout_nanoseconds,
    ImuStampS* imu_timestamps,
    ImuAccGyrS* imu_measurements) {
  CHECK_NOTNULL(imu_timestamps);
  CHECK_NOTNULL(imu_measurements);

  // Wait for the IMU buffer to contain the required measurements within a
  // timeout.
  auto tic = Timer::tic();
  QueryResult query_result;
  {
    std::unique_lock<std::mutex> lock(m_buffer_);
    while ((query_result =
                isDataAvailableUpToImpl(timestamp_ns_from, timestamp_ns_to)) !=
           QueryResult::kDataAvailable) {
      cv_new_measurement_.wait_for(
          lock, std::chrono::nanoseconds(wait_timeout_nanoseconds));

      if (shutdown_) {
        imu_timestamps->resize(Eigen::NoChange, 0);
        imu_measurements->resize(Eigen::NoChange, 0);
        return QueryResult::kQueueShutdown;
      }

      // Check if we hit the max. time allowed to wait for the required data.
      auto toc = Timer::toc<std::chrono::nanoseconds>(tic);
      if (toc.count() >= wait_timeout_nanoseconds) {
        imu_timestamps->resize(Eigen::NoChange, 0);
        imu_measurements->resize(Eigen::NoChange, 0);
        LOG(WARNING) << "Timeout reached while trying to get the requested "
                     << "IMU data. Requested range: " << timestamp_ns_from
                     << " to " << timestamp_ns_to << ".";
        if (query_result == QueryResult::kDataNotYetAvailable) {
          LOG(WARNING) << "The relevant IMU data is not yet available.";
        } else if (query_result == QueryResult::kDataNeverAvailable) {
          LOG(WARNING) << "The relevant IMU data will never be available. "
                       << "Either the buffer is too small or a sync issue "
                       << "occurred.";
        } else {
          LOG(FATAL) << "Unknown query result error.";
        }
        return query_result;
      }
    }
  }
  return getImuDataInterpolatedBorders(timestamp_ns_from, timestamp_ns_to,
                                       imu_timestamps, imu_measurements);
}

}  // namespace utils

}  // namespace VIO
