/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Visualizer3DModule.cpp
 * @brief  Pipeline Module for the 3D visualizer.
 * @author Antoni Rosinol
 */

#include "kimera-vio/visualizer/Visualizer3DModule.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace VIO {

VisualizerModule::VisualizerModule(bool parallel_run,
                                   Visualizer3D::UniquePtr visualizer)
    : MIMOPipelineModule<VisualizerInput, VisualizerOutput>("Visualizer",
                                                            parallel_run),
      frontend_queue_(""),
      backend_queue_(""),
      mesher_queue_(""),
      visualizer_(std::move(visualizer)){};

VisualizerModule::InputPtr VisualizerModule::getInputPacket() {
  bool queue_state = false;
  VizMesherInput mesher_payload = nullptr;
  if (PIO::parallel_run_) {
    queue_state = mesher_queue_.popBlocking(mesher_payload);
  } else {
    queue_state = mesher_queue_.pop(mesher_payload);
  }
  if (!queue_state) {
    LOG_IF(WARNING, PIO::parallel_run_)
        << "Module: " << name_id_ << " - Mesher queue is down";
    VLOG_IF(1, !PIO::parallel_run_)
        << "Module: " << name_id_ << " - Mesher queue is empty or down";
    return nullptr;
  }
  CHECK(mesher_payload);
  const Timestamp& timestamp = mesher_payload->timestamp_;

  // Look for the synchronized packet in frontend payload queue
  // This should always work, because it should not be possible to have
  // a backend payload without having a frontend one first!
  Timestamp frontend_payload_timestamp = std::numeric_limits<Timestamp>::max();
  VizFrontendInput frontend_payload = nullptr;
  while (timestamp != frontend_payload_timestamp) {
    // Pop will remove messages until the queue is empty.
    // This assumes the mesher ends processing after the frontend queue
    // has been filled (it could happen that frontend_queue_ didn't receive
    // the msg before mesher finishes, but that is very unlikely, unless
    // the queue is blocked by someone...)
    if (!frontend_queue_.pop(frontend_payload)) {
      // We had a mesher input but no frontend input, something's wrong.
      // We assume mesher runs after frontend.
      LOG(ERROR) << name_id_
                 << "'s frontend payload queue is empty or "
                    "has been shutdown.";
      return nullptr;
    }
    if (frontend_payload) {
      frontend_payload_timestamp =
          frontend_payload->stereo_frame_lkf_.getTimestamp();
    } else {
      LOG(WARNING) << "Missing frontend payload for Module: " << name_id_;
    }
  }
  CHECK(frontend_payload);

  Timestamp backend_payload_timestamp = std::numeric_limits<Timestamp>::max();
  VizBackendInput backend_payload = nullptr;
  while (timestamp != backend_payload_timestamp) {
    if (!backend_queue_.pop(backend_payload)) {
      // We had a mesher input but no backend input, something's wrong.
      // We assume mesher runs after backend.
      LOG(ERROR) << "Visualizer's backend payload queue is empty or "
                    "has been shutdown.";
      return nullptr;
    }
    if (backend_payload) {
      backend_payload_timestamp = backend_payload->W_State_Blkf_.timestamp_;
    } else {
      LOG(WARNING) << "Missing backend payload for Module: " << name_id_;
    }
  }
  CHECK(backend_payload);

  // Push the synced messages to the visualizer's input queue
  const StereoFrame& stereo_keyframe = frontend_payload->stereo_frame_lkf_;
  // TODO(TONI): store the payloads' pointers in the visualizer payload
  // so that no copies are done, nor we have dangling references!
  return VIO::make_unique<VisualizerInput>(
      timestamp, mesher_payload, backend_payload, frontend_payload);
}

VisualizerModule::OutputPtr VisualizerModule::spinOnce(
    const VisualizerInput& input) {
  return visualizer_->spinOnce(input);
}

void VisualizerModule::shutdownQueues() {
  LOG(INFO) << "Shutting down queues for: " << name_id_;
  frontend_queue_.shutdown();
  backend_queue_.shutdown();
  mesher_queue_.shutdown();
};

//! Checks if the module has work to do (should check input queues are empty)
bool VisualizerModule::hasWork() const {
  LOG_IF(WARNING, mesher_queue_.empty() && !backend_queue_.empty())
      << "Mesher queue is empty, yet backend queue is not!"
         "This should not happen since Mesher runs at Backend pace!";
  // We don't check frontend queue because it runs faster than the other two
  // queues.
  return mesher_queue_.empty();
};

}  // namespace VIO
