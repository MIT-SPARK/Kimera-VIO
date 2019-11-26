### Add source code for stereoVIO
target_sources(KimeraVIO
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/Pipeline.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/PipelineModule.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/PipelinePayload.cpp"
)
