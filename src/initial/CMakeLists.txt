### Add source code for stereoVIO
target_sources(kimera_vio
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/InitializationFromImu.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/InitializationBackend.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/OnlineGravityAlignment.cpp"
)
