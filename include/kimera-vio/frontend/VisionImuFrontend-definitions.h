/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionImuFrontend-definitions.h
 * @brief  Definitions for VisionImuFrontend
 * @author Marcus Abate
 */

#pragma once

#include <gtsam/navigation/NavState.h>

#include "kimera-vio/common/vio_types.h"

namespace VIO {

enum class FrontendType {
  //! Frontend that works with Mono camera and Imu
  kMonoImu = 0,
  //! Frontend that works with Stereo camera and Imu
  kStereoImu = 1,
  //! Frontend that works with RGB + Depth camera and Imu
  kRgbdImu = 2
};


// TODO(marcus): this could probably move somewhere else once refactor happens
struct ExternalOdomMeasurement {
  ExternalOdomMeasurement() = default;
  ExternalOdomMeasurement(const Timestamp& timestamp,
                          const gtsam::NavState& odom_data)
      : timestamp_(timestamp), odom_data_(odom_data) {}
  ExternalOdomMeasurement(Timestamp&& timestamp, gtsam::NavState&& odom_data)
      : timestamp_(std::move(timestamp)), odom_data_(std::move(odom_data)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Timestamp timestamp_;
  gtsam::NavState odom_data_;
};

}  // namespace VIO
