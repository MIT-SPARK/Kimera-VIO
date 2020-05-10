### Add source code for stereoVIO
target_sources(kimera_vio
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/InitializationFromImu.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/InitializationBackEnd.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/OnlineGravityAlignment.cpp"
)
