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

#include <vector>

#include <glog/logging.h>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Visualizer3D.h"

namespace VIO {

class VisualizerModule
    : public MIMOPipelineModule<VisualizerInput, VisualizerOutput> {
 public:
  KIMERA_POINTER_TYPEDEFS(VisualizerModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisualizerModule);
  using VizFrontendInput = FrontendOutput::Ptr;
  using VizBackendInput = BackendOutput::Ptr;
  using VizMesherInput = MesherOutput::Ptr;

  VisualizerModule(bool parallel_run, Visualizer3D::UniquePtr visualizer);
  virtual ~VisualizerModule() = default;

  //! Callbacks to fill queues: they should be all lighting fast.
  inline void fillFrontendQueue(const VizFrontendInput& frontend_payload) {
    frontend_queue_.push(frontend_payload);
  }
  inline void fillBackendQueue(const VizBackendInput& backend_payload) {
    backend_queue_.push(backend_payload);
  }
  inline void fillMesherQueue(const VizMesherInput& mesher_payload) {
    mesher_queue_.push(mesher_payload);
  }

 protected:
  //! Synchronize input queues. Currently doing it in a crude way:
  //! Pop blocking the payload that should be the last to be computed,
  //! then loop over the other queues until you get a payload that has exactly
  //! the same timestamp. Guaranteed to sync messages unless the assumption
  //! on the order of msg generation is broken.
  virtual inline InputUniquePtr getInputPacket() override;

  virtual OutputUniquePtr spinOnce(VisualizerInput::UniquePtr input) override;

  //! Called when general shutdown of PipelineModule is triggered.
  virtual void shutdownQueues() override;

  //! Checks if the module has work to do (should check input queues are empty)
  virtual bool hasWork() const override;

 private:
  //! Input Queues
  ThreadsafeQueue<VizFrontendInput> frontend_queue_;
  ThreadsafeQueue<VizBackendInput> backend_queue_;
  ThreadsafeQueue<VizMesherInput> mesher_queue_;

  //! Visualizer implementation
  Visualizer3D::UniquePtr visualizer_;
};

}  // namespace VIO
