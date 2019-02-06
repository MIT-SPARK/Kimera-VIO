### Add source code for stereoVIO
target_sources(stereoVIO
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/vio_types.h"
)
target_include_directories(stereoVIO PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
