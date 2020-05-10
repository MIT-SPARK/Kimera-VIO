### Add source code for stereoVIO
target_sources(kimera_vio
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/ParallelPlaneRegularFactor.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/PointPlaneFactor.cpp"
)

