### Add source code for stereoVIO
target_sources(KimeraVIO PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/Pipeline.h"
  "${CMAKE_CURRENT_LIST_DIR}/Pipeline-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/PipelinePayload.h"
  "${CMAKE_CURRENT_LIST_DIR}/PipelineModule.h"
)

target_include_directories(KimeraVIO PRIVATE ${CMAKE_CURRENT_LIST_DIR})

