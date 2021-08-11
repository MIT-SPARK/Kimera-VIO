/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FrontendType
 * @brief  Frontend type enum definition
 * @author Marcus Abate
 */

#pragma once

namespace VIO {

enum class FrontendType {
  //! Frontend that works with Mono camera and Imu
  kMonoImu = 0,
  //! Frontend that works with Stereo camera and Imu
  kStereoImu = 1,
  //! Frontend that works with RGB + Depth camera and Imu
  kRgbdImu = 2,
  //! Placeholder for parsing
  kUnknown = 3,
};

}  // namespace VIO
