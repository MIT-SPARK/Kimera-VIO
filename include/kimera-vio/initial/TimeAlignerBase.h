/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   timeAlignerBase.h
 * @brief  Class to estimate IMU to camera time offset
 * @author Nathan Hughes
 */

#pragma once
#include "kimera-vio/backend/VioBackend-definitions.h"

namespace VIO {

class TimeAlignerBase {
 public:
  struct Result {
    bool valid;             //<! Whether or not the estimate is valid
    double imu_time_shift;  //<! Defined as t_imu = t_cam + imu_shift
  };

  TimeAlignerBase(double imu_time_shift_est = 0.0, bool should_estimate = false)
      : imu_time_shift_est_(imu_time_shift_est),
        should_estimate_(should_estimate) {}

  virtual ~TimeAlignerBase() = default;

  Result estimateTimeAlignment(const BackendInput& input) {
    if (not should_estimate_) {
      return {true, imu_time_shift_est_};
    }

    // TODO(nathan) this could maybe also be a PIMPL pattern if we really cared enough
    // to do that
    return attemptEstimation(input);
  }

 protected:
  double imu_time_shift_est_;  //<! Defined as t_imu = t_cam + imu_shift

  virtual Result attemptEstimation(const BackendInput& input) = 0;

 private:
  bool should_estimate_;  //<! whether or not estimation is enabled
};

}  // namespace VIO
