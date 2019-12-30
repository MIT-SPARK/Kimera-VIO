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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit PipelinePayload(const Timestamp& timestamp);
  virtual ~PipelinePayload() = default;

  // Untouchable timestamp of the payload.
  const Timestamp timestamp_;
};

/**
 * @brief The NullPipelinePayload is an empty payload, used for those modules
 * that do not return a payload, such as the display module, which only
 * displays images and returns nothing.
 */
struct NullPipelinePayload : public PipelinePayload {
  KIMERA_POINTER_TYPEDEFS(NullPipelinePayload);
  KIMERA_DELETE_COPY_CONSTRUCTORS(NullPipelinePayload);
  explicit NullPipelinePayload() : PipelinePayload(Timestamp()) {}
  virtual ~NullPipelinePayload() = default;
};

}  // namespace VIO
