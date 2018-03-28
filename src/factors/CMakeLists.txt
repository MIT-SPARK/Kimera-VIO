### Add source code for stereoVIO
target_sources(stereoVIO
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/ParallelPlaneRegularFactor.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/PointPlaneFactor.cpp"
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/ParallelPlaneRegularFactor.h"
        "${CMAKE_CURRENT_LIST_DIR}/PointPlaneFactor.h"
)
