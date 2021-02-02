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
    : DataProviderModule(output_queue,
                         name_id,
                         parallel_run),
      left_frame_queue_("data_provider_left_frame_queue") {}

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
}

MonoImuSyncPacket::UniquePtr MonoDataProviderModule::getMonoImuSyncPacket() {
  //! Retrieve left frame data.
  Frame::UniquePtr left_frame_payload = getLeftFramePayload();
  if (!left_frame_payload) {
    return nullptr;
  }

  //! Retrieve IMU data.
  const Timestamp& timestamp = left_frame_payload->timestamp_;
  ImuMeasurements imu_meas;
  if (!getTimeSyncedImuMeasurements(timestamp, &imu_meas)) {
    return nullptr;
  }

  //! Send synchronized left frame and IMU data.
  return VIO::make_unique<MonoImuSyncPacket>(
      std::move(left_frame_payload), imu_meas.timestamps_, imu_meas.acc_gyr_);
}

Frame::UniquePtr MonoDataProviderModule::getLeftFramePayload() {
  bool queue_state = false;
  Frame::UniquePtr left_frame_payload = nullptr;
  if (MISO::parallel_run_) {
    queue_state = left_frame_queue_.popBlocking(left_frame_payload);
  } else {
    queue_state = left_frame_queue_.pop(left_frame_payload);
  }

  if (!queue_state) {
    LOG_IF(WARNING, MISO::parallel_run_)
        << "Module: " << MISO::name_id_ << " - queue is down";
    VLOG_IF(1, !MISO::parallel_run_)
        << "Module: " << MISO::name_id_ << " - queue is empty or down";
    return nullptr;
  }
  CHECK(left_frame_payload);

  return left_frame_payload;
}

void MonoDataProviderModule::shutdownQueues() {
  left_frame_queue_.shutdown();
  DataProviderModule::shutdownQueues();
}

}  // namespace VIO
