/********************************************************************************
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
*********************************************************************************/

/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ThreadsafeGnssBuffer.h
 * @brief  Threadsafe Gnss Buffer with timestamp lookup.
 * @author Antoni Rosinol
 * @author Elizaveta Karaseva
 */

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <utility>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/Gnss.h"
#include "kimera-vio/utils/ThreadsafeTemporalBuffer.h"

namespace VIO {

namespace utils {

/// \class ThreadsafeGnssBuffer
/// This buffering class can be used to store a history of Gnss measurements. It
/// allows to
/// retrieve the nearest measurement up to a given timestamp. The data is stored
/// in the order
/// it is added. So make sure to add it in correct time-wise order.
class ThreadsafeGnssBuffer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class QueryResult {
    /// Query was successful and the data is available.
    kDataAvailable,
    /// The required data is not (yet) available. The query timestamp is above
    /// the last GNSS sample's time.
    kDataNotYetAvailable,
    /// The queried timestamp lies before the first GNSS sample in the buffer.
    /// This request will never succeed considering chronological ordering
    /// of the buffer input data.
    kDataNeverAvailable,
    /// Queue shutdown.
    kQueueShutdown,
    kTooFewMeasurementsAvailable
  };

  explicit ThreadsafeGnssBuffer(const Timestamp& buffer_length_ns)
      : buffer_(buffer_length_ns), shutdown_(false) {}

  ~ThreadsafeGnssBuffer() { shutdown(); }

  /// Shutdown the queue and release all blocked waiters.
  inline void shutdown();
  inline size_t size() const;
  inline void clear();

  /// Add GNSS measurement.
  inline void addMeasurement(const Timestamp& timestamp_nanoseconds,
                             const GnssPoint& gnss_point);

  /// Get interpolated value by given timestamp
  QueryResult getInterpolatedValue(const Timestamp& timestamp_ns,
                                   Timestamp* gnss_timestamp,
                                   GnssPoint* gnss_point) const;

  /// Get the most recently pushed Gnss measurement
  /// Returns true unless the buffer is empty
  bool getNewestGnssMeasurement(GnssMeasurement* value);

  /// Linear interpolation between two gnss points.
  static void linearInterpolate(const Timestamp& x0,
                                const GnssPoint& y0,
                                const Timestamp& x1,
                                const GnssPoint& y1,
                                const Timestamp& x,
                                GnssPoint* y);

 private:
  typedef std::pair<const Timestamp, GnssMeasurement> BufferElement;
  typedef Eigen::aligned_allocator<BufferElement> BufferAllocator;
  typedef ThreadsafeTemporalBuffer<GnssMeasurement, BufferAllocator> Buffer;

  Buffer buffer_;
  std::condition_variable cv_new_measurement_;
  std::atomic<bool> shutdown_;
};

}  // namespace utils

}  // namespace VIO

#include "./ThreadsafeGnssBuffer-inl.h"
