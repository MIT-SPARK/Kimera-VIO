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

#pragma once

#include "StereoFrame.h"
#include "ImuFrontEnd.h"

namespace VIO {

class StereoImuSyncPacket {
public:
  StereoImuSyncPacket() = delete;
  StereoImuSyncPacket(StereoFrame stereo_frame,
                      ImuStamps imu_stamps,
                      ImuAccGyr imu_accgyr);
  ~StereoImuSyncPacket() = default;

  // Careful, returning references to members can lead to dangling refs.
  inline const StereoFrame& getStereoFrame() const {return stereo_frame_;}
  inline const ImuStamps& getImuStamps() const {return imu_stamps_;}
  inline const ImuAccGyr& getImuAccGyr() const {return imu_accgyr_;}

private:
  StereoFrame stereo_frame_;
  ImuStamps imu_stamps_;
  ImuAccGyr imu_accgyr_;
};

} // End of VIO namespace.
