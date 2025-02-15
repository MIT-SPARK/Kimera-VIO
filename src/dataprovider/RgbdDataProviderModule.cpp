/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdDataProviderModule.cpp
 * @brief  Pipeline Module that takes care of providing RBGD + IMU data to the
 * VIO pipeline.
 * @author Antoni Rosinol
 */

#include "kimera-vio/dataprovider/RgbdDataProviderModule.h"

#include "kimera-vio/frontend/MonoImuSyncPacket.h"
#include "kimera-vio/frontend/RgbdImuSyncPacket.h"

#include <gflags/gflags.h>

DEFINE_string(depth_image_mask,
              "",
              "mask for depth image to remove invalid regions");

namespace VIO {

RgbdDataProviderModule::RgbdDataProviderModule(OutputQueue* output_queue,
                                               const std::string& name_id,
                                               const bool& parallel_run)
    : MonoDataProviderModule(output_queue, name_id, parallel_run),
      depth_frame_queue_("data_provider_depth_frame_queue") {
  if (!FLAGS_depth_image_mask.empty()) {
    depth_mask_ = cv::imread(FLAGS_depth_image_mask, cv::IMREAD_GRAYSCALE);
    if (depth_mask_.empty()) {
      LOG(WARNING) << "Invalid mask path: " << FLAGS_depth_image_mask
                   << ". Defaulting to not using a mask";
    }
  }
}

RgbdDataProviderModule::InputUniquePtr
RgbdDataProviderModule::getInputPacket() {
  //! Get left frame + IMU synchronized data
  MonoImuSyncPacket::UniquePtr mono_imu_sync_packet = getMonoImuSyncPacket();
  if (!mono_imu_sync_packet) {
    return nullptr;
  }

  const Timestamp& timestamp = mono_imu_sync_packet->timestamp_;
  const FrameId& left_frame_id = mono_imu_sync_packet->frame_->id_;

  //! Get the synchronized depth frame inside the queue.
  // This search might not be successful if the data_provider did not push
  // to the depth queue (perhaps fast enough).
  DepthFrame::UniquePtr depth_frame_payload = nullptr;
  if (!PIO::syncQueue(timestamp, &depth_frame_queue_, &depth_frame_payload)) {
    // Dropping this message because of missing left/depth stereo synced frames.
    LOG(ERROR) << "Missing depth frame for left frame with id " << left_frame_id
               << ", dropping this frame.";
    return nullptr;
  }
  CHECK(depth_frame_payload);

  if (!depth_mask_.empty()) {
    cv::Mat result(depth_frame_payload->depth_img_.rows,
                   depth_frame_payload->depth_img_.cols,
                   depth_frame_payload->depth_img_.type());
    result.setTo(0);

    cv::bitwise_or(depth_frame_payload->depth_img_,
                   depth_frame_payload->depth_img_,
                   result,
                   depth_mask_);
    const auto id = depth_frame_payload->id_;
    const auto stamp = depth_frame_payload->timestamp_;
    depth_frame_payload.reset(new DepthFrame(id, stamp, result));
  }

  if (!shutdown_) {
    CHECK(vio_pipeline_callback_);
    vio_pipeline_callback_(std::make_unique<RgbdImuSyncPacket>(
        timestamp,
        std::make_unique<RgbdFrame>(left_frame_id,
                                    timestamp,
                                    *mono_imu_sync_packet->frame_,
                                    *depth_frame_payload),
        mono_imu_sync_packet->imu_stamps_,
        mono_imu_sync_packet->imu_accgyrs_,
        mono_imu_sync_packet->world_NavState_ext_odom_));
  }

  // Push the synced messages to the Frontend's input queue
  // TODO(Toni): should be a return like that, so that we pass the info to the
  // queue... Right now we use a callback bcs otw I need to fix all
  // initialization which is a lot to be fixed.
  // return std::make_unique<StereoImuSyncPacket>(
  //    StereoFrame(
  //        left_frame_payload->id_,
  //        timestamp,
  //        *left_frame_payload,
  //        *right_frame_payload,
  //        stereo_matching_params_),  // TODO(Toni): these params should
  //                                   // be given in PipelineParams.
  //    imu_meas.timestamps_,
  //    imu_meas.acc_gyr_);
  return nullptr;
}

void RgbdDataProviderModule::shutdownQueues() {
  depth_frame_queue_.shutdown();
  MonoDataProviderModule::shutdownQueues();
}

}  // namespace VIO
