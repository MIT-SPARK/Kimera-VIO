/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionFrontEndFactory.h
 * @brief  Factory of vision frontends.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/StereoVisionFrontEnd-definitions.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd-definitions.h"

namespace VIO {

class VisionFrontEndFactory {
 public:
  KIMERA_POINTER_TYPEDEFS(VisionFrontEndFactory);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisionFrontEndFactory);
  VisionFrontEndFactory() = delete;
  virtual ~VisionFrontEndFactory() = default;

  static StereoVisionFrontEnd::UniquePtr createFrontend(
      const FrontendType& frontend_type,
      const ImuParams& imu_params,
      const ImuBias& imu_initial_bias,
      const VioFrontEndParams& frontend_params,
      bool log_output);
};

}  // namespace VIO
