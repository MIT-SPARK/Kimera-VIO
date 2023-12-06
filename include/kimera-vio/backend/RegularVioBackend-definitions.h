/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackend-definitions.h
 * @brief  Definitions for VioBackend.
 * @author Antoni Rosinol
 */

#pragma once

#include <vector>

#include <glog/logging.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/StereoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontend.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace VIO {

enum class RegularBackendModality {
  //! Only use structureless factors, equiv to normal Vio.
  STRUCTURELESS = 0,
  //! Converts all structureless factors to projection factors
  PROJECTION = 1,
  //! Projection factors used for regularities.
  STRUCTURELESS_AND_PROJECTION = 2,
  //! Projection Vio + regularity factors.
  PROJECTION_AND_REGULARITY = 3,
  //! All types of factors used.
  STRUCTURELESS_PROJECTION_AND_REGULARITY = 4
};

}  // namespace VIO
