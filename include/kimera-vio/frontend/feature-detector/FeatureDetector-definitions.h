/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FeatureDetector-definitions.h
 * @brief  Definitions for feature detection.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/feature-detector/anms/anms.h"  // REMOVE

namespace VIO {

// GFTT is goodFeaturesToTrack detector.
enum class FeatureDetectorType { FAST = 0, ORB = 1, AGAST = 2, GFTT = 3 };

}  // namespace VIO
