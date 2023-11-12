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

#pragma once

#include <glog/logging.h>

#include <vector>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"
#include "kimera-vio/visualizer/Visualizer3D.h"

namespace VIO {

class VisualizerModule
    : public MISOPipelineModule<VisualizerInput, DisplayInputBase> {
 public:
  KIMERA_POINTER_TYPEDEFS(VisualizerModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisualizerModule);
  using MISO = MISOPipelineModule<VisualizerInput, DisplayInputBase>;

  using VizFrontendInput = FrontendOutputPacketBase::Ptr;
  using VizBackendInput = BackendOutput::Ptr;
  using VizMesherInput = MesherOutput::Ptr;

  VisualizerModule(OutputQueue* output_queue,
                   bool parallel_run,
                   bool use_lcd,
                   Visualizer3D::UniquePtr visualizer);
  virtual ~VisualizerModule() = default;

  inline void fillFrontendQueue(const VizFrontendInput& frontend_payload) {
    if (!frontend_payload || !frontend_payload->is_keyframe_) {
      return;
    }

    frontend_queue_.push(frontend_payload);
  }

  inline void fillBackendQueue(const VizBackendInput& backend_payload) {
    backend_queue_.push(backend_payload);
  }

  void fillMesherQueue(const VizMesherInput& mesher_payload);

  inline void disableMesherQueue() { mesher_queue_.reset(nullptr); }

 protected:
  //! Synchronize input queues. Currently doing it in a crude way:
  //! Pop blocking the payload that should be the last to be computed,
  //! then loop over the other queues until you get a payload that has exactly
  //! the same timestamp. Guaranteed to sync messages unless the assumption
  //! on the order of msg generation is broken.
  inline InputUniquePtr getInputPacket() override;

  OutputUniquePtr spinOnce(VisualizerInput::UniquePtr input) override;

  //! Called when general shutdown of PipelineModule is triggered.
  void shutdownQueues() override;

  //! Checks if the module has work to do (should check input queues are empty)
  bool hasWork() const override;

 private:
  //! Input Queues
  ThreadsafeQueue<VizFrontendInput> frontend_queue_;
  ThreadsafeQueue<VizBackendInput> backend_queue_;
  /// Mesher queue is optional, therefore it is a unique ptr (nullptr if unused)
  ThreadsafeQueue<VizMesherInput>::UniquePtr mesher_queue_;

  //! Visualizer implementation
  Visualizer3D::UniquePtr visualizer_;
};

}  // namespace VIO
