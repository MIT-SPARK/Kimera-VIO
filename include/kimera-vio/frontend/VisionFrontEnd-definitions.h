/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionFrontEnd-definitions.h
 * @brief  Definitions for VisionFrontEnd
 * @author Marcus Abate
 */

#pragma once

namespace VIO {

enum class FrontendType {
  //! Frontend that works with Mono camera and Imu
  kMonoImu = 0,
  //! Frontend that works with Stereo camera and Imu
  kStereoImu = 1
};

}  // namespace VIO
