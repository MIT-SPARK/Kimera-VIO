/**
 * @file   RgbdImuSyncPacket.h
 * @brief  Class describing the minimum input for VIO to run
 * Contains a RGB Frame + a Depth Frame with Imu data synchronized from last
 * Keyframe timestamp to the current stereo frame timestamp.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/rgbd/RgbdImuSyncPacket.h"

namespace VIO {

RgbdImuSyncPacket::RgbdImuSyncPacket(const Timestamp& timestamp,
                                     RgbdFrame::UniquePtr rgbd_frame,
                                     const ImuStampS& imu_stamps,
                                     const ImuAccGyrS& imu_accgyr)
    : FrontendInputPacketBase(rgbd_frame->timestamp_, imu_stamps, imu_accgyr),
      rgbd_frame_(std::move(rgbd_frame)) {}

void RgbdImuSyncPacket::print() const {
  // TODO
}

}  // namespace VIO
