/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoDataProviderModule.cpp
 * @brief  Pipeline Module that takes care of providing data to the VIO
 * pipeline.
 * @author Antoni Rosinol
 */

#include "kimera-vio/dataprovider/GnssStereoDataProviderModule.h"

#include <glog/logging.h>

#include <memory>   // for make_unique
#include <string>   // for string
#include <utility>  // for move

#include "kimera-vio/frontend/Gnss.h"
#include "kimera-vio/frontend/GnssStereoImuSyncPacket.h"
#include "kimera-vio/frontend/MonoImuSyncPacket.h"

namespace VIO {

GnssStereoDataProviderModule::GnssStereoDataProviderModule(
    OutputQueue* output_queue,
    const std::string& name_id,
    const bool& parallel_run,
    const StereoMatchingParams& stereo_matching_params,
    const GnssParams& gnss_params)
    : MonoDataProviderModule(output_queue, name_id, parallel_run),
      right_frame_queue_("data_provider_right_frame_queue"),
      stereo_matching_params_(stereo_matching_params),
      gnss_params_(gnss_params) {}

GnssStereoDataProviderModule::InputUniquePtr
GnssStereoDataProviderModule::getInputPacket() {
  //! Get left image + IMU data
  MonoImuSyncPacket::UniquePtr mono_imu_sync_packet =
      getMonoImuSyncPacket(false);
  if (!mono_imu_sync_packet) {
    return nullptr;
  }

  const Timestamp& timestamp = mono_imu_sync_packet->timestamp_;
  const FrameId& left_frame_id = mono_imu_sync_packet->frame_->id_;

  //! Get right image data
  // This search might not be successful if the data_provider did not push
  // to
  // the right queue (perhaps fast enough).
  Frame::UniquePtr right_frame_payload = nullptr;
  if (!MISO::syncQueue(timestamp, &right_frame_queue_, &right_frame_payload)) {
    // Dropping this message because of missing left/right stereo synced
    // frames.
    LOG(ERROR) << "Missing right frame for left frame with id " << left_frame_id
               << ", dropping this frame.";
    return nullptr;
  }
  CHECK(right_frame_payload);
  //! Retrieve IMU data.
  GnssMeasurement gnss_meas;
  FrameAction action =
      getTimeSyncedGnssMeasurement(timestamp, gnss_params_, &gnss_meas);
  switch (action) {
    case FrameAction::Use:
      break;
    case FrameAction::Wait:
      // return nullptr;
      break;
    case FrameAction::Drop:
      // return nullptr;
      break;
  }

  timestamp_last_frame_ = timestamp;
  if (!shutdown_) {
    auto packet = std::make_unique<GnssStereoImuSyncPacket>(
        StereoFrame(left_frame_id,
                    timestamp,
                    *mono_imu_sync_packet->frame_,  // this copies...
                    *right_frame_payload),          // this copies...
        // be given in PipelineParams.
        mono_imu_sync_packet->imu_stamps_,
        mono_imu_sync_packet->imu_accgyrs_,
        gnss_meas.point_);
    if (send_packet_) {
      CHECK(vio_pipeline_callback_);
      vio_pipeline_callback_(std::move(packet));
    } else {
      return packet;
    }
  }
  return nullptr;
}

void GnssStereoDataProviderModule::shutdownQueues() {
  right_frame_queue_.shutdown();
  MonoDataProviderModule::shutdownQueues();
}

}  // namespace VIO
