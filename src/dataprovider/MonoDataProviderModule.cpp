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
      left_frame_queue_("data_provider_left_frame_queue"),
      cached_left_frame_(nullptr) {}

MonoDataProviderModule::InputUniquePtr
MonoDataProviderModule::getInputPacket() {
  if (!MISO::shutdown_) {
    MonoImuSyncPacket::UniquePtr mono_imu_sync_packet = getMonoImuSyncPacket();
    if (!mono_imu_sync_packet) return nullptr;

    CHECK(vio_pipeline_callback_);
    vio_pipeline_callback_(std::move(mono_imu_sync_packet));
  } else {
    return nullptr;
  }
  return nullptr;
}

MonoImuSyncPacket::UniquePtr MonoDataProviderModule::getMonoImuSyncPacket(
    bool cache_timestamp) {
  // Retrieve left frame data.
  Frame::UniquePtr left_frame_payload;
  if (cached_left_frame_) {
    repeated_frame_ = true;
    left_frame_payload = std::move(cached_left_frame_);
  } else {
    repeated_frame_ = false;
    left_frame_payload = getLeftFramePayload();
  }

  if (!left_frame_payload) {
    return nullptr;
  }

  if (timestamp_last_frame_ >= left_frame_payload->timestamp_) {
    LOG(WARNING) << "Dropping frame: "
                 << UtilsNumerical::NsecToSec(left_frame_payload->timestamp_)
                 << " (curr) <= "
                 << UtilsNumerical::NsecToSec(timestamp_last_frame_)
                 << " (last)";
    return nullptr;
  }

  //! Retrieve IMU data.
  const Timestamp& timestamp = left_frame_payload->timestamp_;
  ImuMeasurements imu_meas;
  FrameAction action = getTimeSyncedImuMeasurements(timestamp, &imu_meas);
  switch (action) {
    case FrameAction::Use:
      break;
    case FrameAction::Wait:
      cached_left_frame_ = std::move(left_frame_payload);
      return nullptr;
    case FrameAction::Drop:
      return nullptr;
  }

  bool odometry_valid = false;
  gtsam::NavState external_odometry;
  if (external_odometry_buffer_) {
    ThreadsafeOdometryBuffer::QueryResult result =
        external_odometry_buffer_->getNearest(timestamp, &external_odometry);
    switch (result) {
      case ThreadsafeOdometryBuffer::QueryResult::DataNotYetAvailable:
        // TODO(nathan) consider increasing verbosity here
        VLOG(10) << "Odometry data not available yet, spinning";
        cached_left_frame_ =
            std::move(left_frame_payload);  // we need to spin some more
        return nullptr;
      case ThreadsafeOdometryBuffer::QueryResult::DataNeverAvailable:
        odometry_valid = false;
        break;
      case ThreadsafeOdometryBuffer::QueryResult::DataAvailable:
        odometry_valid = true;
        break;
    }
  }

  if (cache_timestamp) {
    timestamp_last_frame_ = timestamp;
  }

  if (odometry_valid) {
    // return synchronized left frame, IMU data and external odometry
    return VIO::make_unique<MonoImuSyncPacket>(std::move(left_frame_payload),
                                               imu_meas.timestamps_,
                                               imu_meas.acc_gyr_,
                                               external_odometry);
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
    LOG_IF(WARNING, MISO::parallel_run_ && !MISO::shutdown_)
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
