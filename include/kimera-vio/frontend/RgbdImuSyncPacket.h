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
#include "kimera-vio/frontend/RgbdFrame.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class RgbdImuSyncPacket : public FrontendInputPacketBase {
 public:
  KIMERA_POINTER_TYPEDEFS(RgbdImuSyncPacket);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RgbdImuSyncPacket);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RgbdImuSyncPacket() = delete;
  RgbdImuSyncPacket(const RgbdFrame& rgbd_frame,
                    const ImuStampS& imu_stamps,
                    const ImuAccGyrS& imu_accgyr);
  ~RgbdImuSyncPacket() = default;

  // Careful, returning references to members can lead to dangling refs.
  inline const RgbdFrame& getRgbdFrame() const { return rgbd_frame_; }
  inline const ImuStampS& getImuStamps() const { return imu_stamps_; }
  inline const ImuAccGyrS& getImuAccGyrs() const { return imu_accgyrs_; }

  void print() const;

 public:
  RgbdFrame rgbd_frame_;
};

}  // namespace VIO
