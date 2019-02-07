### Add source code for stereoVIO
target_sources(sparkvio
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/vio_types.h"
)
target_include_directories(sparkvio PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
