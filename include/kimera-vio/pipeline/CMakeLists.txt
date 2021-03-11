### Add source code for stereoVIO
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/MonoImuPipeline.h"
  "${CMAKE_CURRENT_LIST_DIR}/Pipeline.h"
  "${CMAKE_CURRENT_LIST_DIR}/Pipeline-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/PipelinePayload.h"
  "${CMAKE_CURRENT_LIST_DIR}/PipelineModule.h"
  "${CMAKE_CURRENT_LIST_DIR}/PipelineParams.h"
  "${CMAKE_CURRENT_LIST_DIR}/QueueSynchronizer.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoImuPipeline.h"
)
