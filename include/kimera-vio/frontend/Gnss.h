/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Gnss.h
 * @brief  Class describing Gnss Point
 * @author Elizaveta Karaseva
 */

#pragma once

#include <Eigen/Core>
#include <utility>  // for move

#include "kimera-vio/common/vio_types.h"

namespace VIO {

using GnssPoint = gtsam::Point3;

struct GnssMeasurement {
  GnssMeasurement() = default;
  GnssMeasurement(const Timestamp& timestamp, const GnssPoint& point)
    : timestamp_(timestamp), point_(point) {}
  GnssMeasurement(Timestamp&& timestamp, GnssPoint&& point)
  : timestamp_(std::move(timestamp)), point_(std::move(point)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Timestamp timestamp_;
  GnssPoint point_;
};
}  // namespace VIO
