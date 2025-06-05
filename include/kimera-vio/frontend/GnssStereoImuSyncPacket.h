/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GnssStereoImuSyncPacket.h
 * @brief  Class describing the minimum input for Gnss VIO to run
 * Contains a Stereo Frame with Imu data and Gnss point synchronized from last
 * Keyframe timestamp to the current stereo frame timestamp.
 * @author Antoni Rosinol
 * @author Elizaveta Karaseva
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include <optional>

#include "kimera-vio/frontend/FrontendInputPacketBase.h"
#include "kimera-vio/frontend/Gnss.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"

namespace VIO {

class GnssStereoImuSyncPacket : public FrontendInputPacketBase {
 public:
  KIMERA_POINTER_TYPEDEFS(GnssStereoImuSyncPacket);
  KIMERA_DELETE_COPY_CONSTRUCTORS(GnssStereoImuSyncPacket);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GnssStereoImuSyncPacket() = delete;
  GnssStereoImuSyncPacket(
      const StereoFrame& stereo_frame,
      const ImuStampS& imu_stamps,
      const ImuAccGyrS& imu_accgyr,
      const GnssPoint& gnss_point,
      std::optional<gtsam::NavState> external_odometry = std::nullopt,
      const VIO::ReinitPacket& reinit_packet = ReinitPacket());
  ~GnssStereoImuSyncPacket() = default;

  inline const StereoFrame& getStereoFrame() const { return stereo_frame_; }
  inline const ImuStampS& getImuStamps() const { return imu_stamps_; }
  inline const ImuAccGyrS& getImuAccGyrs() const { return imu_accgyrs_; }
  inline const ReinitPacket& getReinitPacket() const { return reinit_packet_; }
  inline bool getReinitFlag() const { return reinit_packet_.getReinitFlag(); }
  inline const GnssPoint& getGnssPoint() const { return gnss_point_; }

  void print() const;

 private:
  const StereoFrame stereo_frame_;
  const GnssPoint gnss_point_;
  const ReinitPacket reinit_packet_;
};

}  // namespace VIO
