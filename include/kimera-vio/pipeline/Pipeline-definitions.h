/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Pipeline-definitions.h
 * @brief  Definition for VIO pipeline.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/backend/VioBackEndParams.h"
#include "kimera-vio/frontend/VioFrontEndParams.h"
#include "kimera-vio/imu-frontend/ImuFrontEndParams.h"
#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"

namespace VIO {

struct PipelineParams {
  VioFrontEndParams frontend_params_;
  VioBackEndParamsPtr backend_params_;
  ImuParams imu_params_;
  LoopClosureDetectorParams lcd_params_;
  int backend_type_;
  bool parallel_run_;
};

}  // namespace VIO
