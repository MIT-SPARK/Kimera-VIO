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

#include "VioBackEndParams.h"
#include "VioFrontEndParams.h"
#include "imu-frontend/ImuFrontEndParams.h"

namespace VIO {

struct PipelineParams {
  VioFrontEndParams frontend_params_;
  VioBackEndParamsPtr backend_params_;
  ImuParams imu_params_;
  int backend_type_;
  bool parallel_run_;
};

}  // namespace VIO
