/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoImuSyncPacket.h
 * @brief  Class describing the minimum input for VIO to run
 * Contains a Mono Frame with Imu data synchronized from last
 * Keyframe timestamp to the current frame timestamp.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/MonoImuSyncPacket.h"
#include "kimera-vio/frontend/Frame.h"

#include <utility>

namespace VIO {
MonoImuSyncPacket::MonoImuSyncPacket(Frame::UniquePtr frame,
                                     const ImuStampS& imu_stamps,
                                     const ImuAccGyrS& imu_accgyr)
    : PipelinePayload(frame->timestamp_),
      frame_(std::move(frame)),
      imu_stamps_(imu_stamps),
      imu_accgyr_(imu_accgyr) {
  CHECK(frame_);
  CHECK_GT(imu_stamps_.cols(), 0u);
  CHECK_GT(imu_accgyr_.cols(), 0u);
  CHECK_EQ(imu_stamps_.cols(), imu_accgyr_.cols());
  // The timestamp of the last IMU measurement must correspond to the timestamp
  // of the frame. In case there is no IMU measurement with exactly
  // the same timestamp as the frame, the user should interpolate
  // IMU measurements to get a value at the time of the frame.
  CHECK_EQ(frame_->timestamp_, imu_stamps_(imu_stamps_.cols() - 1));
}

void MonoImuSyncPacket::print() const {
  CHECK(frame_);
  LOG(INFO) << "Mono Frame timestamp: " << frame_->timestamp_ << '\n'
            << "STAMPS IMU rows : \n"
            << imu_stamps_.rows() << '\n'
            << "STAMPS IMU cols : \n"
            << imu_stamps_.cols() << '\n'
            << "STAMPS IMU: \n"
            << imu_stamps_ << '\n'
            << "ACCGYR IMU rows : \n"
            << imu_accgyr_.rows() << '\n'
            << "ACCGYR IMU cols : \n"
            << imu_accgyr_.cols() << '\n'
            << "ACCGYR IMU: \n"
            << imu_accgyr_;
}

}  // namespace VIO
