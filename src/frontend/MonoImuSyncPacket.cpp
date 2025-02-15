/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoImuSyncPacket.cpp
 * @brief  Class describing the minimum input for VIO to run
 * Contains a Monocular Frame with Imu data synchronized from last
 * Keyframe timestamp to the current frame timestamp.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/MonoImuSyncPacket.h"

#include <utility>

namespace VIO {

MonoImuSyncPacket::MonoImuSyncPacket(Frame::UniquePtr frame,
                                     const ImuStampS& imu_stamps,
                                     const ImuAccGyrS& imu_accgyrs,
                                     std::optional<gtsam::NavState> external_odometry)
    : FrontendInputPacketBase(frame->timestamp_,
                              imu_stamps,
                              imu_accgyrs,
                              external_odometry),
      frame_(std::move(frame)) {
  CHECK_GT(imu_stamps_.cols(), 0u);
  CHECK_EQ(frame_->timestamp_, imu_stamps_(imu_stamps_.cols() - 1));
}

void MonoImuSyncPacket::print() const {
  LOG(INFO) << "Mono Frame timestamp: " << frame_->timestamp_ << '\n'
            << "STAMPS IMU rows : \n"
            << imu_stamps_.rows() << '\n'
            << "STAMPS IMU cols : \n"
            << imu_stamps_.cols() << '\n'
            << "STAMPS IMU: \n"
            << imu_stamps_ << '\n'
            << "ACCGYR IMU rows : \n"
            << imu_accgyrs_.rows() << '\n'
            << "ACCGYR IMU cols : \n"
            << imu_accgyrs_.cols() << '\n'
            << "ACCGYR IMU: \n"
            << imu_accgyrs_;
}

}  // namespace VIO
