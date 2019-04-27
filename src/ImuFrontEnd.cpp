/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontEnd.cpp
 * @brief  Class managing sequences of IMU measurements.
 * @author Antoni Rosinol, Luca Carlone
 */
#include "ImuFrontEnd.h"

#include <glog/logging.h>

namespace VIO {

////////////////////////////////////////////////////////////////////////////////
//////////////// FUNCTIONS OF THE CLASS ImuData                       //////////
////////////////////////////////////////////////////////////////////////////////
/* -------------------------------------------------------------------------- */
void ImuData::print() const {
  LOG(INFO) << "------------ ImuData::print    -------------";
  body_Pose_cam_.print("body_Pose_cam_: \n");
  LOG(INFO) << "\n nominal_imu_rate: " << nominal_imu_rate_ << '\n'
            << "imu_rate: " << imu_rate_ << '\n'
            << "imu_rate_std: " << imu_rate_std_ << '\n'
            << "imu_rate_maxMismatch: " << imu_rate_maxMismatch_ << '\n'
            << "gyroscope_noise_density: " << gyro_noise_ << '\n'
            << "gyroscope_random_walk: " << gyro_walk_ << '\n'
            << "accelerometer_noise_density: " << acc_noise_ << '\n'
            << "accelerometer_random_walk: " << acc_walk_ << '\n'
            << "nr of imu measurements: " << imu_buffer_.size();
}

} // End of VIO namespace.
