/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontEndParams.cpp
 * @brief  Params for ImuFrontEnd.
 * @author Antoni Rosinol
 */

#include "imu-frontend/ImuFrontEndParams.h"

#include <glog/logging.h>

namespace VIO {

void ImuParams::print() const {
  LOG(INFO) << "------------ ImuParams::print -------------\n"
            << "gyroscope_noise_density: " << gyro_noise_ << '\n'
            << "gyroscope_random_walk: " << gyro_walk_ << '\n'
            << "accelerometer_noise_density: " << acc_noise_ << '\n'
            << "accelerometer_random_walk: " << acc_walk_ << '\n'
            << "n_gravity: " << n_gravity_ << '\n'
            << "imu_integration_sigma: " << imu_integration_sigma_;
}

}  // namespace VIO
