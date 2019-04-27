### Add source code for stereoVIO
target_sources(stereoVIO
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/DataSource.cpp"
    PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/DataSource.h"
)
