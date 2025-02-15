/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LcdModule.h
 * @brief  Pipeline module for LoopClosureDetector
 * @author Marcus Abate
 * @author Luca Carlone
 */

#pragma once

#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"
#include "kimera-vio/loopclosure/LoopClosureDetector.h"
#include "kimera-vio/pipeline/PipelineModule.h"

namespace VIO {

class LcdModule : public MIMOPipelineModule<LcdInput, LcdOutput> {
 public:
  KIMERA_POINTER_TYPEDEFS(LcdModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LcdModule);
  using LcdFrontendInput = FrontendOutputPacketBase::Ptr;
  using LcdBackendInput = BackendOutput::Ptr;

  LcdModule(bool parallel_run, LoopClosureDetector::UniquePtr lcd);
  virtual ~LcdModule() = default;

  inline void fillFrontendQueue(const LcdFrontendInput& frontend_payload) {
    if (!frontend_payload || !frontend_payload->is_keyframe_) {
      return;
    }

    frontend_queue_.push(frontend_payload);
  }

  inline void fillBackendQueue(const LcdBackendInput& backend_payload) {
    backend_queue_.push(backend_payload);
  }

  LoopResult registerFrames(FrameId first_kf, FrameId second_kf) {
    std::unique_lock<std::mutex> lock(mutex_);
    return lcd_->registerFrames(first_kf, second_kf);
  }

 protected:
  //! Synchronize input queues.
  InputUniquePtr getInputPacket() override;

  OutputUniquePtr spinOnce(LcdInput::UniquePtr input) override {
    std::unique_lock<std::mutex> lock(mutex_);
    return lcd_->spinOnce(*input);
  }

  //! Called when general shutdown of PipelineModule is triggered.
  void shutdownQueues() override {
    LOG(INFO) << "Shutting down queues for: " << name_id_;
    frontend_queue_.shutdown();
    backend_queue_.shutdown();
  }

  //! Checks if the module has work to do (should check input queues are empty)
  bool hasWork() const override {
    // We don't check Frontend queue because it runs faster than Backend queue.
    return !backend_queue_.empty();
  }

 private:
  //! Input Queues
  ThreadsafeQueue<LcdFrontendInput> frontend_queue_;
  ThreadsafeQueue<LcdBackendInput> backend_queue_;

  //! Lcd implementation
  LoopClosureDetector::UniquePtr lcd_;

  // handles access to underlying lcd detector
  mutable std::mutex mutex_;
};

}  // namespace VIO
