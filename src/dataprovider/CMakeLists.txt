### Add source code for stereoVIO
target_sources(kimera_vio
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/DataProviderInterface-definitions.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/DataProviderInterface.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/DataProviderModule.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/MonoDataProviderModule.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/RgbdDataProviderModule.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/StereoDataProviderModule.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/EurocDataProvider.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/KittiDataProvider.cpp"
)
