/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoImuSyncPacket.h
 * @brief  Class describing the minimum input for VIO to run
 * Contains a Stereo Frame with Imu data synchronized from last
 * Keyframe timestamp to the current stereo frame timestamp.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/StereoImuSyncPacket.h"

#include <utility>

namespace VIO {
StereoImuSyncPacket::StereoImuSyncPacket(const StereoFrame& stereo_frame,
                                         const ImuStampS& imu_stamps,
                                         const ImuAccGyrS& imu_accgyr,
                                         const ReinitPacket& reinit_packet)
    : PipelinePayload(stereo_frame.getTimestamp()),
      stereo_frame_(stereo_frame),
      imu_stamps_(imu_stamps),
      imu_accgyr_(imu_accgyr),
      reinit_packet_(reinit_packet) {
  CHECK_GT(imu_stamps_.cols(), 0u);
  CHECK_GT(imu_accgyr_.cols(), 0u);
  CHECK_EQ(imu_stamps_.cols(), imu_accgyr_.cols());
  // The timestamp of the last IMU measurement must correspond to the timestamp
  // of the stereo frame. In case there is no IMU measurement with exactly
  // the same timestamp as the stereo frame, the user should interpolate
  // IMU measurements to get a value at the time of the stereo_frame.
  CHECK_EQ(stereo_frame_.getTimestamp(), imu_stamps_(imu_stamps_.cols() - 1));
  // TODO: Add check on ReinitPacket
}

void StereoImuSyncPacket::print() const {
  LOG(INFO) << "Stereo Frame timestamp: " << stereo_frame_.getTimestamp()
            << '\n'
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
  if (reinit_packet_.getReinitFlag() == true) {
    LOG(INFO) << "\nVIO Re-Initialized at: " << reinit_packet_.getReinitTime()
              << '\n'
              << "POSE : \n"
              << reinit_packet_.getReinitPose() << '\n'
              << "VELOCITY : \n"
              << reinit_packet_.getReinitVel() << '\n'
              << "BIAS : \n"
              << reinit_packet_.getReinitBias();
  }
}

}  // namespace VIO
