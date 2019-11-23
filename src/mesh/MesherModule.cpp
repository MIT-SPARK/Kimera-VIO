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
    : MIMOPipelineModule<MesherInput, MesherOutput>("MesherModule",
                                                    parallel_run),
      frontend_payload_queue_("mesher_frontend"),
      backend_payload_queue_("mesher_backend"),
      mesher_(std::move(mesher)) {}

MesherModule::InputPtr MesherModule::getInputPacket() {
  MesherBackendInput backend_payload = nullptr;
  bool queue_state = false;
  if (PIO::parallel_run_) {
    queue_state = backend_payload_queue_.popBlocking(backend_payload);
  } else {
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

  // Look for the synchronized packet in frontend payload queue
  // This should always work, because it should not be possible to have
  // a backend payload without having a frontend one first!
  // TODO(Toni): make a getSyncedPayloadFromQueue(timestamp, &payload);
  // utility function in pipeline module. Will need a payload baseclass
  // that has a timestamp as a member.
  Timestamp frontend_payload_timestamp = std::numeric_limits<Timestamp>::max();
  MesherFrontendInput frontend_payload = nullptr;
  while (timestamp != frontend_payload_timestamp) {
    if (!frontend_payload_queue_.pop(frontend_payload)) {
      // We had a backend input but no frontend input, something's wrong.
      LOG(ERROR) << name_id_ << "'s frontend payload queue is empty or "
                 << "has been shutdown.";
      return nullptr;
    } else {
      VLOG(5) << "Popping frontend_payload.";
    }
    if (frontend_payload) {
      frontend_payload_timestamp = frontend_payload->timestamp_;
    } else {
      LOG(WARNING) << "Missing frontend payload for Module: " << name_id_;
    }
  }
  CHECK(frontend_payload);

  return VIO::make_unique<MesherInput>(
      timestamp, frontend_payload, backend_payload);
}

MesherModule::OutputPtr MesherModule::spinOnce(const MesherInput& input) {
  return mesher_->spinOnce(input);
}

void MesherModule::shutdownQueues() {
  LOG(INFO) << "Shutting down queues for: " << name_id_;
  frontend_payload_queue_.shutdown();
  backend_payload_queue_.shutdown();
};

bool MesherModule::hasWork() const {
  // We don't check frontend queue because it runs faster than backend queue.
  return backend_payload_queue_.empty();
};

}  // namespace VIO
