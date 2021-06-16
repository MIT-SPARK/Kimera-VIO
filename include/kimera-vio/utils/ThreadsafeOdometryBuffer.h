/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ThreadsafeOdometryBuffer.h
 * @brief  Threadsafe buffer to store additional odometry measurements
 * @author Nathan Hughes
 */

#include <gtsam/navigation/NavState.h>
#include "kimera-vio/utils/ThreadsafeTemporalBuffer.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class ThreadsafeOdometryBuffer {
 public:
  KIMERA_POINTER_TYPEDEFS(ThreadsafeOdometryBuffer);

  enum class QueryResult {
    DataNotYetAvailable,
    DataNeverAvailable,
    DataAvailable,
  };

  explicit ThreadsafeOdometryBuffer(const Timestamp& buffer_length_ns);

  void add(const Timestamp& time, const gtsam::NavState& odometry);

  ThreadsafeOdometryBuffer::QueryResult getNearest(const Timestamp& timestamp,
                                                   gtsam::NavState* odometry);

 private:
  struct Odometry {
    Timestamp timestamp;
    gtsam::NavState value;
  };
  utils::ThreadsafeTemporalBuffer<Odometry> buffer_;
};

}  // namespace VIO
