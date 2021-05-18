/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackendModule.cpp
 * @brief  Pipeline module for the Backend.
 *
 * @author Antoni Rosinol
 */

#include "kimera-vio/backend/VioBackendModule.h"

namespace VIO {

VioBackendModule::VioBackendModule(InputQueue* input_queue,
                                   bool parallel_run,
                                   VioBackend::UniquePtr vio_backend)
    : SIMO(input_queue, "VioBackend", parallel_run),
      vio_backend_(std::move(vio_backend)) {
  CHECK(vio_backend_);
}

VioBackendModule::OutputUniquePtr VioBackendModule::spinOnce(
    BackendInput::UniquePtr input) {
  CHECK(input);
  CHECK(vio_backend_);
  OutputUniquePtr output = vio_backend_->spinOnce(*input);
  if (!output) {
    LOG(ERROR) << "Backend did not return an output: shutting down Backend.";
    shutdown();
  }
  return output;
}

void VioBackendModule::registerImuBiasUpdateCallback(
    const VioBackend::ImuBiasCallback& imu_bias_update_callback) {
  CHECK(vio_backend_);
  vio_backend_->registerImuBiasUpdateCallback(imu_bias_update_callback);
}

void VioBackendModule::registerMapUpdateCallback(
    const VioBackend::MapCallback& map_update_callback) {
  CHECK(vio_backend_);
  vio_backend_->registerMapUpdateCallback(map_update_callback);
}

}  // namespace VIO
