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
 * @file   ThreadsafeGnssBuffer.cpp
 * @brief  Threadsafe Gnss Buffer with timestamp lookup.
 * @author Antoni Rosinol
 * @author Elizaveta Karaseva
 */

#include "kimera-vio/utils/ThreadsafeGnssBuffer.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <algorithm>
#include <chrono>
#include <iostream>

#include "kimera-vio/utils/Timer.h"

namespace VIO {

namespace utils {

template <template <typename, typename> class Container, typename Type>
using Aligned = Container<Type, Eigen::aligned_allocator<Type>>;

bool ThreadsafeGnssBuffer::getNewestGnssMeasurement(GnssMeasurement* value) {
  CHECK_NOTNULL(value);
  return buffer_.getNewestValue(value);
}

void ThreadsafeGnssBuffer::linearInterpolate(const Timestamp& t0,
                                             const GnssPoint& y0,
                                             const Timestamp& t1,
                                             const GnssPoint& y1,
                                             const Timestamp& t,
                                             GnssPoint* y) {
  CHECK_NOTNULL(y);
  CHECK_LE(t0, t);
  CHECK_LE(t, t1);
  *y = t0 == t1 ? y0 :  // y0 if t0 == t1, interpolate otherwise:
           y0 + (y1 - y0) * static_cast<double>(t - t0) /
                    static_cast<double>(t1 - t0);
}

ThreadsafeGnssBuffer::QueryResult ThreadsafeGnssBuffer::getInterpolatedValue(
    const Timestamp& timestamp_ns,
    // const uint64_t gnss_period_ns,
    Timestamp* gnss_timestamp,
    GnssPoint* gnss_point) const {
  CHECK_NOTNULL(gnss_timestamp);
  CHECK_NOTNULL(gnss_point);

  if (buffer_.empty()) {
    LOG(WARNING) << "GNSS buffer is empty.";
    return QueryResult::kTooFewMeasurementsAvailable;
  }

  GnssMeasurement oldest;
  CHECK(buffer_.getOldestValue(&oldest))
      << "Gnss buffer failed to get oldest value";
  if (oldest.timestamp_ > timestamp_ns) {
    return QueryResult::kDataNeverAvailable;
  }
  Timestamp pre_border_timestamp, post_border_timestamp;
  GnssMeasurement pre_border_value, post_border_value;
  if (!buffer_.getValueAtOrBeforeTime(
          timestamp_ns, &pre_border_timestamp, &pre_border_value)) {
    LOG(WARNING)
        << "The GNSS buffer seems not to contain points at or before time: "
        << timestamp_ns;
    return QueryResult::kTooFewMeasurementsAvailable;
  }
  CHECK_EQ(pre_border_timestamp, pre_border_value.timestamp_);
  if (!buffer_.getValueAtOrAfterTime(
          timestamp_ns, &post_border_timestamp, &post_border_value)) {
    LOG(WARNING)
        << "The GNSS buffer seems not to contain points at or after time: "
        << timestamp_ns;
    return QueryResult::kTooFewMeasurementsAvailable;
  }
  CHECK_EQ(post_border_timestamp, post_border_value.timestamp_);
  GnssPoint interpolated_gnss_point;
  linearInterpolate(pre_border_value.timestamp_,
                    pre_border_value.point_,
                    post_border_value.timestamp_,
                    post_border_value.point_,
                    timestamp_ns,
                    &interpolated_gnss_point);

  *gnss_timestamp = timestamp_ns;
  *gnss_point = interpolated_gnss_point;
  return QueryResult::kDataAvailable;
}

}  // namespace utils

}  // namespace VIO
