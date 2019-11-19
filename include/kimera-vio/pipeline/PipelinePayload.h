/**
 * @file   PipelinePayload.h
 * @brief  Base class for the payloads shared between pipeline modules.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

struct PipelinePayload {
  KIMERA_POINTER_TYPEDEFS(PipelinePayload);
  KIMERA_DELETE_COPY_CONSTRUCTORS(PipelinePayload);
  explicit PipelinePayload(const Timestamp& timestamp);
  virtual ~PipelinePayload() = default;

  // Untouchable timestamp of the payload.
  const Timestamp timestamp_;
};

}  // namespace VIO
