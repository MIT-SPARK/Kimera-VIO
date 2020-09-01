/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoDataProviderModule.cpp
 * @brief  Pipeline Module that takes care of providing RBGD + IMU data to the
 * VIO pipeline.
 * @author Antoni Rosinol
 */

#include <utility>  // for move

#include "kimera-vio/dataprovider/MonoDataProviderModule.h"
#include "kimera-vio/frontend/MonoImuSyncPacket.h"

namespace VIO {

MonoDataProviderModule::MonoDataProviderModule(OutputQueue* output_queue,
                                               const std::string& name_id,
                                               const bool& parallel_run)
    : DataProviderModule<MonoImuSyncPacket, MonoImuSyncPacket>(output_queue,
                                                               name_id,
                                                               parallel_run) {}

MonoDataProviderModule::InputUniquePtr
MonoDataProviderModule::getInputPacket() {
  if (!MISO::shutdown_) {
    MonoImuSyncPacket::UniquePtr mono_imu_sync_packet =getMonoImuSyncPacket();
    if (!mono_imu_sync_packet) return nullptr;
    
    CHECK(vio_pipeline_callback_);
    vio_pipeline_callback_(std::move(mono_imu_sync_packet));
  } else {
    return nullptr;
  }
  return nullptr;
  // Push the synced messages to the frontend's input queue
  // TODO(Toni): should be a return like that, so that we pass the info to the
  // queue... Right now we use a callback bcs otw I need to fix all
  // initialization which is a lot to be fixed.
  // Actually, right now we don't even use a callback since mono VIO is not
  // implemented...
}

}  // namespace VIO
