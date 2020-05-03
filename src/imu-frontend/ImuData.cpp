/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuData.cpp
 * @brief  Class to store Inertial data
 * @author Antoni Rosinol
 */

#include "kimera-vio/imu-frontend/ImuData.h"

#include <glog/logging.h>

namespace VIO {

void ImuData::print() const {
  LOG(INFO) << "------------ ImuData::print -------------\n"
            << "Number of IMU measurements: " << imu_buffer_.size();
}

}  // namespace VIO
