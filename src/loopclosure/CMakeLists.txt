### Add source code for LoopClosureDetector
target_sources(kimera_vio
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/LcdThirdPartyWrapper.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/LcdModule.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/LcdOutputPacket.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/LoopClosureDetector.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/LoopClosureDetectorParams.cpp"
)
