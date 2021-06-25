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
 * @file   ThreadsafeImuBuffer.h
 * @brief  Threadsafe Imu Buffer with timestamp lookup.
 * @author Antoni Rosinol
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <utility>

#include <functional>
#include <map>

#include <Eigen/Dense>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/utils/ThreadsafeTemporalBuffer.h"

namespace VIO {

namespace utils {

/// \class ThreadsafeImuBuffer
/// This buffering class can be used to store a history of IMU measurements. It
/// allows to
/// retrieve a list  of measurements up to a given timestamp. The data is stored
/// in the order
/// it is added. So make sure to add it in correct time-wise order.
class ThreadsafeImuBuffer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class QueryResult {
    /// Query was successful and the data is available.
    kDataAvailable,
    /// The required data is not (yet) available. The query timestamp is above
    /// the last IMU sample's time.
    kDataNotYetAvailable,
    /// The queried timestamp lies before the first IMU sample in the buffer.
    /// This request will never succeed considering chronological ordering
    /// of the buffer input data.
    kDataNeverAvailable,
    /// Queue shutdown.
    kQueueShutdown,
    kTooFewMeasurementsAvailable
  };

  explicit ThreadsafeImuBuffer(const Timestamp& buffer_length_ns)
      : buffer_(buffer_length_ns), shutdown_(false) {}

  ~ThreadsafeImuBuffer() { shutdown(); }

  /// Shutdown the queue and release all blocked waiters.
  inline void shutdown();
  inline size_t size() const;
  inline void clear();

  /// Add IMU measurement in IMU frame.
  /// (Ordering: accelerations [m/s^2], angular velocities [rad/s])
  inline void addMeasurement(const Timestamp& timestamp_nanoseconds,
                             const ImuAccGyr& imu_measurement);
  inline void addMeasurements(const ImuStampS& timestamps_nanoseconds,
                              const ImuAccGyrS& imu_measurements);

  // Get IMU data strictly between Timestamps.
  // Example: content: 2 3 4 5
  //      getImuDataStrictlyBtwTiemstamps(2, 5, ...) returns elements at 3, 4.
  // Alternatively, the user might ask for the lower bound as well.
  //      getImuDataStrictlyBtwTiemstamps(2, 5, ..., true)
  // returns elements at 2, 3, 4.
  // by setting the parameter get_lower_bound to true.
  QueryResult getImuDataBtwTimestamps(const Timestamp& timestamp_ns_from,
                                      const Timestamp& timestamp_ns_to,
                                      ImuStampS* imu_timestamps,
                                      ImuAccGyrS* imu_measurements,
                                      bool get_lower_bound = false);

  /// \brief Return a list of the IMU measurements between the specified
  /// timestamps. The IMU values get interpolated if the queried timestamp does
  /// not match a measurement.
  /// @param[in] timestamp_from Try to get the IMU measurements from this time
  /// [ns].
  /// @param[in] timestamp_to Try to get the IMU measurements up this time [ns].
  /// @param[out] imu_timestamps List of timestamps. [ns]
  /// @param[out] imu_measurements List of IMU measurements. (Order: [acc,
  /// gyro])
  /// @return The return code signals if the buffer does not contain data
  /// up to the requested timestamp.
  /// In this case the output matrices will be of size 0.
  QueryResult getImuDataInterpolatedBorders(const Timestamp& timestamp_from,
                                            const Timestamp& timestamp_to,
                                            ImuStampS* imu_timestamps,
                                            ImuAccGyrS* imu_measurements);

  /// \brief Return a list of the IMU measurements between the specified
  /// timestamps. ONLY the IMU newest value gets interpolated (if the queried
  /// timestamp does not match a measurement, otw no interpolation).
  /// The oldest IMU value is always an actual measurement
  /// (aka no interpolation for the lower border, ever).
  /// @param[in] timestamp_from Get the IMU measurements from this time
  /// [ns].
  /// @param[in] timestamp_to Try to get the IMU measurements up this time [ns].
  /// @param[out] imu_timestamps List of timestamps. [ns]
  /// @param[out] imu_measurements List of IMU measurements. (Order: [acc,
  /// gyro])
  /// @return The return code signals if the buffer does not contain data
  /// surrounding the requested timestamps
  /// (check function isDataAvailableUpToImpl).
  /// In this case the output matrices will be of size 0.
  QueryResult getImuDataInterpolatedUpperBorder(
      const Timestamp& timestamp_ns_from,
      const Timestamp& timestamp_ns_to,
      ImuStampS* imu_timestamps,
      ImuAccGyrS* imu_measurements);

  // Interpolates an IMU measurement at timestamp by taking previous and
  // posterior measurements to the given timestamp.
  // WARNING: the user must make sure the buffer has enough elements.
  // This can be done be checked using the isDataAvailableUpToImpl function.
  void interpolateValueAtTimestamp(const Timestamp& timestamp_ns,
                                   ImuAccGyr* interpolated_imu_measurement);

  /// Try to pop the requested IMU measurements for the duration of
  /// wait_timeout_nanoseconds.
  /// If the requested data is still not available when timeout has been
  /// reached, the method
  /// will return false and no data will be removed from the buffer.
  QueryResult getImuDataInterpolatedBordersBlocking(
      const Timestamp& timestamp_ns_from,
      const Timestamp& timestamp_ns_to,
      const Timestamp& wait_timeout_nanoseconds,
      ImuStampS* imu_timestamps,
      ImuAccGyrS* imu_measurements);

  /// Get the most recently pushed IMU measurement
  /// Returns true unless the buffer is empty
  bool getNewestImuMeasurement(ImuMeasurement* value);

  /// Linear interpolation between two imu measurements.
  static void linearInterpolate(const Timestamp& x0,
                                const ImuAccGyr& y0,
                                const Timestamp& x1,
                                const ImuAccGyr& y1,
                                const Timestamp& x,
                                ImuAccGyr* y);

 private:
  /// TODO I think this comment is deprecated:
  /// Is data available up to this timestamp? Note this function does not lock
  /// the buffers, the caller must hold the lock.
  /// TODO I believe it should be:
  /// a) checks that the requested interval of data is a subset of the data in
  ///  the IMU buffer, returns:
  ///       Ex. IMU buffer holds timestamps 2 3 4 5, then
  ///       i) Query timestamp_ns_to > 5 (e.g. 7)
  ///           , returns kDataNotYetAvailable.
  ///       ii) Query from timestamp < 2 (e.g. 0) or timestamp to < 2 (e.g. 1)
  ///           , returns kDataNeverAvailable.
  ///       iii) Query from timestamp 3 to 4, returns kDataAvailable.
  /// And I don't think there is a need to lock any mutex, as buffer_ is already
  /// threadsafe...
  QueryResult isDataAvailableUpToImpl(const Timestamp& timestamp_ns_from,
                                      const Timestamp& timestamp_ns_to) const;

  typedef std::pair<const Timestamp, ImuMeasurement> BufferElement;
  typedef Eigen::aligned_allocator<BufferElement> BufferAllocator;
  typedef ThreadsafeTemporalBuffer<ImuMeasurement, BufferAllocator> Buffer;

  Buffer buffer_;
  mutable std::mutex m_buffer_;
  std::condition_variable cv_new_measurement_;
  std::atomic<bool> shutdown_;
  std::atomic<bool> have_warned_user_;
};

}  // namespace utils

}  // namespace VIO

#include "./ThreadsafeImuBuffer-inl.h"
