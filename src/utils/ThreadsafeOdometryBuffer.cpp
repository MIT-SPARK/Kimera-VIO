/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ThreadsafeOdometryBuffer.cpp
 * @brief  Threadsafe buffer to store additional odometry measurements
 * @author Nathan Hughes
 */

#include "kimera-vio/utils/ThreadsafeOdometryBuffer.h"
#include "kimera-vio/utils/UtilsNumerical.h"

namespace VIO {

ThreadsafeOdometryBuffer::ThreadsafeOdometryBuffer(const Timestamp& buffer_length_ns)
    : buffer_(buffer_length_ns) {}

void ThreadsafeOdometryBuffer::add(const Timestamp& time,
                                   const gtsam::NavState& odometry) {
  Odometry to_add;
  to_add.timestamp = time;
  to_add.value = odometry;
  buffer_.addValue(time, to_add);
}

ThreadsafeOdometryBuffer::QueryResult ThreadsafeOdometryBuffer::getNearest(
    const Timestamp& timestamp,
    gtsam::NavState* odometry) {
  if (buffer_.empty()) {
    return QueryResult::DataNotYetAvailable;
  }

  Odometry oldest;
  CHECK(buffer_.getOldestValue(&oldest))
      << "odomety buffer failed to get oldest value";
  if (oldest.timestamp > timestamp) {
    return QueryResult::DataNeverAvailable;
  }

  Odometry newest;
  CHECK(buffer_.getNewestValue(&newest))
      << "odometry buffer failed to get newest value";

  if (newest.timestamp < timestamp) {
    return QueryResult::DataNotYetAvailable;
  }

  Odometry m_nearest;
  buffer_.getNearestValueToTime(timestamp, &m_nearest);
  VLOG(10) << "Found odom measurement with t="
           << UtilsNumerical::NsecToSec(m_nearest.timestamp) << " which is "
           << UtilsNumerical::NsecToSec(m_nearest.timestamp - timestamp)
           << " seconds away from t=" << UtilsNumerical::NsecToSec(timestamp);

  *odometry = m_nearest.value;
  return QueryResult::DataAvailable;
}

}  // namespace VIO
