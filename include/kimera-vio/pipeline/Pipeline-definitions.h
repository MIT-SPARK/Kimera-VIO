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

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/backend/VioBackEndParams.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd-definitions.h"
#include "kimera-vio/frontend/VioFrontEndParams.h"
#include "kimera-vio/imu-frontend/ImuFrontEndParams.h"
#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"

namespace VIO {

struct VioParams {
  //! Sensor parameters
  ImuParams imu_params_;
  MultiCameraParams camera_params_;
  //! Pipeline Modules paramters
  VioFrontEndParams frontend_params_;
  VioBackEndParams::Ptr backend_params_;
  LoopClosureDetectorParams lcd_params_;
  //! General Pipeline parameters
  FrontendType frontend_type_;
  BackendType backend_type_;
  bool parallel_run_;
};

}  // namespace VIO
