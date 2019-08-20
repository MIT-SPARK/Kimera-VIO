### Add source code for LoopClosureDetector
target_sources(SparkVio
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/LoopClosureDetector.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/LoopClosureDetector.h"
    "${CMAKE_CURRENT_LIST_DIR}/LoopClosureDetectorParams.h"
    "${CMAKE_CURRENT_LIST_DIR}/LoopClosureDetector-definitions.h"
)
target_include_directories(SparkVio PRIVATE ${CMAKE_CURRENT_LIST_DIR})
