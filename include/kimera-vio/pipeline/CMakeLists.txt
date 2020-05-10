### Add source code for stereoVIO
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/Pipeline.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/Pipeline-definitions.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/PipelinePayload.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/PipelineModule.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/PipelineParams.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/QueueSynchronizer.h"
)
