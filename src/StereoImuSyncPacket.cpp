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

#include "StereoImuSyncPacket.h"
#include <utility>

namespace VIO {
StereoImuSyncPacket::StereoImuSyncPacket(StereoFrame stereo_frame,
                                         ImuStampS imu_stamps,
                                         ImuAccGyrS imu_accgyr)
  : stereo_frame_(std::move(stereo_frame)),
    imu_stamps_(std::move(imu_stamps)),
    imu_accgyr_(std::move(imu_accgyr)) {}

} // End of VIO namespace.
