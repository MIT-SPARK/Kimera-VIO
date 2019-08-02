### Add source code for stereoVIO
target_sources(SparkVio
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/vio_types.h"
    "${CMAKE_CURRENT_LIST_DIR}/FilesystemUtils.h"
)
target_include_directories(SparkVio PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
