/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  VisionImuFrontendModule.cpp
 * @brief
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/VisionImuFrontendModule.h"

namespace VIO {

VisionImuFrontendModule::VisionImuFrontendModule(
    InputQueue* input_queue,
    bool parallel_run,
    VisionImuFrontend::UniquePtr vio_frontend)
    : SIMO(input_queue, "VioFrontend", parallel_run),
      vio_frontend_(std::move(vio_frontend)) {
  CHECK(vio_frontend_);
}

FrontendOutputPacketBase::UniquePtr VisionImuFrontendModule::spinOnce(
    FrontendInputPacketBase::UniquePtr input) {
  CHECK(input);
  return vio_frontend_->spinOnce(std::move(input));
}

}  // namespace VIO
