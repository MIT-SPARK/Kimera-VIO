### Add source code for stereoVIO
target_sources(stereoVIO
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/ThreadsafeTemporalBuffer.h"
    "${CMAKE_CURRENT_LIST_DIR}/ThreadsafeQueue.h"
    "${CMAKE_CURRENT_LIST_DIR}/ThreadsafeImuBuffer.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ThreadsafeImuBuffer.h"
    "${CMAKE_CURRENT_LIST_DIR}/Timer.h"
)
target_include_directories(stereoVIO PRIVATE ${CMAKE_CURRENT_LIST_DIR})

