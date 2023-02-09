/**
 * @file   RgbdImuSyncPacket.h
 * @brief  Class describing the minimum input for VIO to run
 * Contains a RGB Frame + a Depth Frame with Imu data synchronized from last
 * Keyframe timestamp to the current stereo frame timestamp.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/RgbdImuSyncPacket.h"

namespace VIO {

RgbdImuSyncPacket::RgbdImuSyncPacket(const RgbdFrame& rgbd_frame,
                                     const ImuStampS& imu_stamps,
                                     const ImuAccGyrS& imu_accgyr)
    : FrontendInputPacketBase(rgbd_frame.timestamp_, imu_stamps, imu_accgyr),
      rgbd_frame_(rgbd_frame) {}

void RgbdImuSyncPacket::print() const {
  LOG(INFO) << "Stereo Frame timestamp: " << rgbd_frame_.timestamp_
            << '\n'
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
