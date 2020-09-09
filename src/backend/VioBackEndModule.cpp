/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackEndModule.cpp
 * @brief  Pipeline module for the backend.
 *
 * @author Antoni Rosinol
 */

#include "kimera-vio/backend/VioBackEndModule.h"

namespace VIO {

VioBackEndModule::VioBackEndModule(InputQueue* input_queue,
                                   bool parallel_run,
                                   VioBackEnd::UniquePtr vio_backend)
    : SIMO(input_queue, "VioBackEnd", parallel_run),
      vio_backend_(std::move(vio_backend)) {
  CHECK(vio_backend_);
}

VioBackEndModule::OutputUniquePtr VioBackEndModule::spinOnce(
    BackendInput::UniquePtr input) {
  CHECK(input);
  CHECK(vio_backend_);
  OutputUniquePtr output = vio_backend_->spinOnce(*input);
  if (!output) {
    LOG(ERROR) << "Backend did not return an output: shutting down backend.";
    shutdown();
  }
  return output;
}

void VioBackEndModule::registerImuBiasUpdateCallback(
    const VioBackEnd::ImuBiasCallback& imu_bias_update_callback) {
  CHECK(vio_backend_);
  vio_backend_->registerImuBiasUpdateCallback(imu_bias_update_callback);
}

}  // namespace VIO
