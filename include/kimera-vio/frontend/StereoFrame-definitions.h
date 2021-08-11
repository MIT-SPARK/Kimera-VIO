/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoFrame-definitions.h
 * @brief  Definitions for StereoFrame
 * @author Antoni Rosinol
 */

#pragma once

#include <gtsam/geometry/StereoPoint2.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/Tracker-definitions.h"

namespace VIO {

using StereoMeasurement = std::pair<LandmarkId, gtsam::StereoPoint2>;
using StereoMeasurements = std::vector<StereoMeasurement>;
using StatusStereoMeasurements =
    std::pair<TrackerStatusSummary, StereoMeasurements>;
using StatusStereoMeasurementsPtr = std::shared_ptr<StatusStereoMeasurements>;

// TODO make enum class.
enum Mesh2Dtype { VALIDKEYPOINTS, DENSE };

////////////////////////////////////////////////////////////////////////////////
struct KeypointWithDepth {
 public:
  KeypointWithDepth() = default;
  KeypointWithDepth(const KeypointCV& p, const double& d) : px(p), depth(d) {}

 public:
  KeypointCV px;
  double depth;
};
using KeypointsWithDepth = std::vector<KeypointWithDepth>;

// Definitions relevant to StereoFrame type.
using Points3d = std::vector<Vector3, Eigen::aligned_allocator<Vector3>>;

}  // namespace VIO
