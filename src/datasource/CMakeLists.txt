### Add source code for stereoVIO
target_sources(SparkVio
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/DataSource-definitions.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/DataSource.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ETH_parser.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/KittiDataSource.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/DataSource-definitions.h"
    "${CMAKE_CURRENT_LIST_DIR}/DataSource.h"
    "${CMAKE_CURRENT_LIST_DIR}/ETH_parser.h"
    "${CMAKE_CURRENT_LIST_DIR}/KittiDataSource.h"
)
target_include_directories(SparkVio PRIVATE ${CMAKE_CURRENT_LIST_DIR})
