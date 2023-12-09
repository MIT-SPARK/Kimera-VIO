/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MesherModule.cpp
 * @brief  Build 3D mesh from 2D mesh.
 * @author Antoni Rosinol
 */

#include "kimera-vio/mesh/MesherModule.h"

namespace VIO {

MesherModule::MesherModule(bool parallel_run, Mesher::UniquePtr mesher)
    : MIMOPipelineModule<MesherInput, MesherOutput>("Mesher", parallel_run),
      frontend_payload_queue_("mesher_frontend"),
      backend_payload_queue_("mesher_backend"),
      mesher_(std::move(mesher)) {}

MesherModule::InputUniquePtr MesherModule::getInputPacket() {
  MesherBackendInput backend_payload = nullptr;
  bool queue_state = false;
  if (PIO::parallel_run_) {
    queue_state = backend_payload_queue_.popBlocking(backend_payload);
  } else {
    // TODO(Toni): can't you always do popBlocking?
    queue_state = backend_payload_queue_.pop(backend_payload);
  }

  if (!queue_state) {
    LOG_IF(WARNING, PIO::parallel_run_)
        << "Module: " << name_id_ << " - Backend queue is down";
    VLOG_IF(1, !PIO::parallel_run_)
        << "Module: " << name_id_ << " - Backend queue is empty or down";
    return nullptr;
  }

  CHECK(backend_payload);
  const Timestamp& timestamp = backend_payload->timestamp_;

  // Look for the synchronized packet in Frontend payload queue
  // This should always work, because it should not be possible to have
  // a Backend payload without having a Frontend one first!
  MesherFrontendInput frontend_payload = nullptr;
  PIO::syncQueue(timestamp, &frontend_payload_queue_, &frontend_payload);
  CHECK(frontend_payload);
  CHECK(frontend_payload->is_keyframe_);

  return std::make_unique<MesherInput>(
      timestamp, frontend_payload, backend_payload);
}

MesherModule::OutputUniquePtr MesherModule::spinOnce(
    MesherInput::UniquePtr input) {
  CHECK(input);
  return mesher_->spinOnce(*input);
}

void MesherModule::shutdownQueues() {
  LOG(INFO) << "Shutting down queues for: " << name_id_;
  frontend_payload_queue_.shutdown();
  backend_payload_queue_.shutdown();
};

bool MesherModule::hasWork() const {
  // We don't check Frontend queue because it runs faster than Backend queue.
  return !backend_payload_queue_.empty();
};

}  // namespace VIO
