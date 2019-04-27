/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontEnd.h
 * @brief  Class managing sequences of IMU measurements.
 * @author Antoni Rosinol, Luca Carlone
 */

#pragma once

#include <map>
#include <string>
#include <tuple>
#include <thread>
#include <utility>
#include <mutex>

#include <Eigen/Dense>

#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/ImuBias.h>

#include "ImuFrontEnd-definitions.h"

#include <gtsam/geometry/Pose3.h>
#include "utils/ThreadsafeImuBuffer.h"

namespace VIO {

/*
 * Class describing the imu parameters and data.
 */
class ImuData {
public:
  // Imu buffer with virtually infinite memory.
  ImuData()
    : imu_buffer_(-1) {}

  // Imu parameters.
  gtsam::Pose3 body_Pose_cam_;
  double imu_rate_;
  double nominal_imu_rate_;
  double imu_rate_std_;
  double imu_rate_maxMismatch_;
  double gyro_noise_;
  double gyro_walk_;
  double acc_noise_;
  double acc_walk_;

  // Imu data.
  utils::ThreadsafeImuBuffer imu_buffer_;

public:
  void print() const;
};

} // End of VIO namespace.
