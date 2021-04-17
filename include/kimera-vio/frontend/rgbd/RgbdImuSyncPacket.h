/**
 * @file   RgbdImuSyncPacket.h
 * @brief  Class describing the minimum input for VIO to run
 * Contains a RGB Frame + a Depth Frame with Imu data synchronized from last
 * Keyframe timestamp to the current stereo frame timestamp.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/FrontendInputPacketBase.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/frontend/rgbd/RgbdFrame.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class RgbdImuSyncPacket : public FrontendInputPacketBase {
 public:
  KIMERA_POINTER_TYPEDEFS(RgbdImuSyncPacket);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RgbdImuSyncPacket);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RgbdImuSyncPacket(const Timestamp& timestamp,
                    RgbdFrame::UniquePtr rgbd_frame,
                    const ImuStampS& imu_stamps,
                    const ImuAccGyrS& imu_accgyr);
  virtual ~RgbdImuSyncPacket() = default;

  void print() const;

 public:
  RgbdFrame::UniquePtr rgbd_frame_;
};

}  // namespace VIO
