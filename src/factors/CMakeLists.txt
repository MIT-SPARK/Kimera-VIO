### Add source code for stereoVIO
target_sources(stereoVIO
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/ParallelPlaneRegularFactor.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/PointPlaneFactor.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ThreePointsPlaneFactor.cpp"
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/ParallelPlaneRegularFactor.h"
        "${CMAKE_CURRENT_LIST_DIR}/PointPlaneFactor.h"
        "${CMAKE_CURRENT_LIST_DIR}/ThreePointsPlaneFactor.h"
)
