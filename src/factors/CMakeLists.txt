### Add source code for stereoVIO
target_sources(sparkvio
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/ParallelPlaneRegularFactor.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/PointPlaneFactor.cpp"
)
target_include_directories(sparkvio PRIVATE "${CMAKE_CURRENT_LIST_DIR}")

