### Add includes for stereoVIO
target_sources(KimeraVIO
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/Accumulator.h"
    "${CMAKE_CURRENT_LIST_DIR}/ThreadsafeQueue.h"
    "${CMAKE_CURRENT_LIST_DIR}/ThreadsafeTemporalBuffer.h"
    "${CMAKE_CURRENT_LIST_DIR}/Timer.h"
)
target_include_directories(KimeraVIO PRIVATE ${CMAKE_CURRENT_LIST_DIR})

