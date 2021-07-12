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
 * Contains a Monocular Frame with Imu data synchronized from last
 * Keyframe timestamp to the current frame timestamp.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/FrontendInputPacketBase.h"
#include "kimera-vio/frontend/Frame.h"
namespace VIO {

class MonoImuSyncPacket : public FrontendInputPacketBase {
 public:
  KIMERA_POINTER_TYPEDEFS(MonoImuSyncPacket);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoImuSyncPacket);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MonoImuSyncPacket() = delete;
  MonoImuSyncPacket(Frame::UniquePtr frame,
                    const ImuStampS& imu_stamps,
                    const ImuAccGyrS& imu_accgyrs,
                    boost::optional<gtsam::NavState> external_odometry = boost::none);

  virtual ~MonoImuSyncPacket() = default;

  inline const Frame& getFrame() const { return *frame_; }
  inline const ImuStampS& getImuStamps() const { return imu_stamps_; }
  inline const ImuAccGyrS& getImuAccGyrs() const { return imu_accgyrs_; }
  void print() const;

  Frame::UniquePtr frame_;
};

}  // namespace VIO
