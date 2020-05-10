### Add source code for stereoVIO
target_sources(kimera_vio
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/DataProviderInterface-definitions.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/DataProviderInterface.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/DataProviderModule.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/EurocDataProvider.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/KittiDataProvider.cpp"
)
