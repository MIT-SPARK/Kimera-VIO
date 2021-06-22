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

using OptOdom = boost::optional<gtsam::NavState>;

StereoImuSyncPacket::StereoImuSyncPacket(const StereoFrame& stereo_frame,
                                         const ImuStampS& imu_stamps,
                                         const ImuAccGyrS& imu_accgyrs,
                                         OptOdom external_odometry,
                                         const ReinitPacket& reinit_packet)
    : FrontendInputPacketBase(stereo_frame.timestamp_,
                              imu_stamps,
                              imu_accgyrs,
                              external_odometry),
      stereo_frame_(stereo_frame),
      reinit_packet_(reinit_packet) {
  // The timestamp of the last IMU measurement must correspond to the timestamp
  // of the stereo frame. In case there is no IMU measurement with exactly
  // the same timestamp as the stereo frame, the user should interpolate
  // IMU measurements to get a value at the time of the stereo_frame.
  CHECK_GT(imu_stamps_.cols(), 0);
  CHECK_EQ(stereo_frame_.timestamp_, imu_stamps_(imu_stamps_.cols() - 1));
  // TODO: Add check on ReinitPacket
}

void StereoImuSyncPacket::print() const {
  LOG(INFO) << "Stereo Frame timestamp: " << stereo_frame_.timestamp_ << '\n'
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
