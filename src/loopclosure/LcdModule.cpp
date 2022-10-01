/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LcdModule.cpp
 * @brief  Pipeline module for LoopClosureDetector
 * @author Marcus Abate
 * @author Luca Carlone
 */

#include "kimera-vio/loopclosure/LcdModule.h"

namespace VIO {

LcdModule::LcdModule(bool parallel_run, LoopClosureDetector::UniquePtr lcd)
    : MIMOPipelineModule<LcdInput, LcdOutput>("Lcd", parallel_run),
      frontend_queue_("lcd_frontend_queue"),
      backend_queue_("lcd_backend_queue"),
      lcd_(std::move(lcd)) {
  CHECK(lcd_);
  lcd_->registerIsBackendQueueFilledCallback(
      std::bind(&LcdModule::hasWork, this));
}

LcdModule::InputUniquePtr LcdModule::getInputPacket() {
  // TODO(X): this is the same or very similar to the Mesher getInputPacket.
  LcdBackendInput backend_payload;
  bool queue_state = false;
  if (PIO::parallel_run_) {
    queue_state = backend_queue_.popBlocking(backend_payload);
  } else {
    queue_state = backend_queue_.pop(backend_payload);
  }
  if (!queue_state) {
    LOG_IF(WARNING, PIO::parallel_run_)
        << "Module: " << name_id_ << " - Backend queue is down";
    VLOG_IF(1, !PIO::parallel_run_)
        << "Module: " << name_id_ << " - Backend queue is empty or down";
    return nullptr;
  }
  CHECK(backend_payload);
  const Timestamp& timestamp = backend_payload->W_State_Blkf_.timestamp_;

  // Look for the synchronized packet in Frontend payload queue
  // This should always work, because it should not be possible to have
  // a Backend payload without having a Frontend one first!
  LcdFrontendInput frontend_payload = nullptr;
  PIO::syncQueue(timestamp, &frontend_queue_, &frontend_payload);
  CHECK(frontend_payload);
  CHECK(frontend_payload->is_keyframe_);
  CHECK_EQ(timestamp, frontend_payload->timestamp_);

  // Push the synced messages to the lcd's input queue
  const gtsam::Pose3& body_pose = backend_payload->W_State_Blkf_.pose_;
  return VIO::make_unique<LcdInput>(timestamp,
                                    frontend_payload,
                                    backend_payload->cur_kf_id_,
                                    backend_payload->landmarks_with_id_map_,
                                    body_pose);
}

}  // namespace VIO
