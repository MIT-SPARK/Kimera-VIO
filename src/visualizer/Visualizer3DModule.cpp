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
      frontend_queue_("visualizer_frontend_queue"),
      backend_queue_("visualizer_backend_queue"),
      mesher_queue_("visualizer_mesher_queue"),
      visualizer_(std::move(visualizer)){};

VisualizerModule::InputUniquePtr VisualizerModule::getInputPacket() {
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
  VizFrontendInput frontend_payload = nullptr;
  PIO::syncQueue(timestamp, &frontend_queue_, &frontend_payload);
  CHECK(frontend_payload);
  CHECK(frontend_payload->is_keyframe_);

  VizBackendInput backend_payload = nullptr;
  PIO::syncQueue(timestamp, &backend_queue_, &backend_payload);
  CHECK(backend_payload);

  // Push the synced messages to the visualizer's input queue
  return VIO::make_unique<VisualizerInput>(
      timestamp, mesher_payload, backend_payload, frontend_payload);
}

VisualizerModule::OutputUniquePtr VisualizerModule::spinOnce(
    VisualizerInput::UniquePtr input) {
  CHECK(input);
  return visualizer_->spinOnce(*input);
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
  return !mesher_queue_.empty();
};

}  // namespace VIO
