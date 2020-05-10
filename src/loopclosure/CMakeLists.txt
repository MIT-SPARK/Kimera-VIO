### Add source code for LoopClosureDetector
target_sources(kimera_vio
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/LoopClosureDetector.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/LcdThirdPartyWrapper.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/LoopClosureDetectorParams.cpp"
)
