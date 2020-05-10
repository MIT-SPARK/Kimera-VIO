### Add source code for stereoVIO
target_sources(kimera_vio
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/Pipeline.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/PipelineModule.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/PipelinePayload.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/PipelineParams.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Pipeline-definitions.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/QueueSynchronizer.cpp"
)
