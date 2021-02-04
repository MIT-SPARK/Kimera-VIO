### Add source code for stereoVIO
target_sources(kimera_vio
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/MonoImuPipeline.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/PipelineModule.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/PipelinePayload.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/PipelineParams.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Pipeline.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Pipeline-definitions.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/QueueSynchronizer.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/StereoImuPipeline.cpp"
)
