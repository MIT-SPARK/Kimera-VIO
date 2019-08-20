### Add source code for stereoVIO
target_sources(SparkVio
    PRIVATE
      "${CMAKE_CURRENT_LIST_DIR}/Logger.cpp"
      "${CMAKE_CURRENT_LIST_DIR}/Logger.h"
)
target_include_directories(SparkVio PRIVATE ${CMAKE_CURRENT_LIST_DIR})

