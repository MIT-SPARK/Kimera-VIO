### Add source code for stereoVIO
target_sources(sparkvio
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/ThreadsafeTemporalBuffer.h"
    "${CMAKE_CURRENT_LIST_DIR}/ThreadsafeQueue.h"
    "${CMAKE_CURRENT_LIST_DIR}/ThreadsafeImuBuffer.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ThreadsafeImuBuffer.h"
    "${CMAKE_CURRENT_LIST_DIR}/Timer.h"
    "${CMAKE_CURRENT_LIST_DIR}/Accumulator.h"
    "${CMAKE_CURRENT_LIST_DIR}/Statistics.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Statistics.h"
)
target_include_directories(sparkvio PRIVATE ${CMAKE_CURRENT_LIST_DIR})

