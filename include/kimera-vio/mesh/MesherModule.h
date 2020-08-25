/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MesherModule.h
 * @brief  Pipeline Module for the Mesher.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/mesh/Mesher.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class MesherModule : public MIMOPipelineModule<MesherInput, MesherOutput> {
 public:
  KIMERA_POINTER_TYPEDEFS(MesherModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MesherModule);
  using MesherFrontendInput = StereoFrontendOutput::Ptr;
  using MesherBackendInput = BackendOutput::Ptr;
  // TODO(Toni): using this callback generates copies...
  using MesherOutputCallback = std::function<void(const MesherOutput& output)>;

  MesherModule(bool parallel_run, Mesher::UniquePtr mesher);
  virtual ~MesherModule() = default;

  //! Callbacks to fill queues: they should be all lighting fast.
  inline void fillFrontendQueue(const MesherFrontendInput& frontend_payload) {
    frontend_payload_queue_.push(frontend_payload);
  }
  inline void fillBackendQueue(const MesherBackendInput& backend_payload) {
    backend_payload_queue_.push(backend_payload);
  }

 protected:
  //! Synchronize input queues. Currently doing it in a crude way:
  //! Pop blocking the payload that should be the last to be computed,
  //! then loop over the other queues until you get a payload that has exactly
  //! the same timestamp. Guaranteed to sync messages unless the assumption
  //! on the order of msg generation is broken.
  InputUniquePtr getInputPacket() override;

  OutputUniquePtr spinOnce(MesherInput::UniquePtr input) override;

 protected:
  //! Called when general shutdown of PipelineModule is triggered.
  void shutdownQueues() override;

  //! Checks if the module has work to do (should check input queues are empty)
  bool hasWork() const override;

 private:
  //! Input Queues
  ThreadsafeQueue<MesherFrontendInput> frontend_payload_queue_;
  ThreadsafeQueue<MesherBackendInput> backend_payload_queue_;

  //! Mesher implementation
  Mesher::UniquePtr mesher_;
};

}  // namespace VIO
