/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  VisionFrontEndModule.cpp
 * @brief
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/VisionFrontEndModule.h"

namespace VIO {

VisionFrontEndModule::VisionFrontEndModule(
    InputQueue* input_queue,
    bool parallel_run,
    VisionFrontEnd::UniquePtr vio_frontend)
    : SIMO(input_queue, "VioFrontEnd", parallel_run),
      vio_frontend_(std::move(vio_frontend)) {
  CHECK(vio_frontend_);
}

FrontendOutputPacketBase::UniquePtr VisionFrontEndModule::spinOnce(
    FrontendInputPacketBase::UniquePtr input) {
  CHECK(input);
  return vio_frontend_->spinOnce(std::move(input));
}

}  // namespace VIO
