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
#include "kimera-vio/frontend/FrontendInputPacketBase.h"
#include "kimera-vio/frontend/FrontendOutputPacketBase.h"

namespace VIO {

class TimeAlignerBase {
 public:
  typedef std::unique_ptr<TimeAlignerBase> UniquePtr;

  struct Result {
    bool valid;             //<! Whether or not the estimate is valid
    double imu_time_shift;  //<! Defined as t_imu = t_cam + imu_shift
  };

  TimeAlignerBase() = default;

  virtual ~TimeAlignerBase() = default;

  virtual void addNewImuData(const ImuStampS& imu_stamps,
                             const ImuAccGyrS& imu_accgyrs) = 0;

  inline Result estimateTimeAlignment(const FrontendOutputPacketBase& input) {
    // TODO(nathan) there might be an advantage to putting some extra logic here
    return attemptEstimation(input);
  }

 protected:
  virtual Result attemptEstimation(const FrontendOutputPacketBase& input) = 0;
};

}  // namespace VIO
