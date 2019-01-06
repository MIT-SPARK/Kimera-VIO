### Add source code for stereoVIO
target_sources(stereoVIO
    PRIVATE
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/Pipeline.h"
        "${CMAKE_CURRENT_LIST_DIR}/ProcessControl.h"
        "${CMAKE_CURRENT_LIST_DIR}/BufferControl.h"
)
