### Add source code for stereoVIO
target_sources(SparkVio
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/Pipeline.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/Pipeline.h"
)
target_include_directories(SparkVio PRIVATE ${CMAKE_CURRENT_LIST_DIR})

