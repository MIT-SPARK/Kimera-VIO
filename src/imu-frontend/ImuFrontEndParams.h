/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontEndParams.h
 * @brief  Params for ImuFrontEnd
 * @author Antoni Rosinol
 */

#pragma once

#include <gtsam/base/Vector.h>

namespace VIO {

struct ImuParams {
 public:
  double gyro_noise_;
  double gyro_walk_;
  double acc_noise_;
  double acc_walk_;
  double imu_shift_;  // Defined as t_imu = t_cam + imu_shift

  // TODO: n_gravity_ should not be in ImuParams
  gtsam::Vector3 n_gravity_;
  double imu_integration_sigma_;

 public:
  void print() const;
};

}  // namespace VIO
