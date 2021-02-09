### Add source code for stereoVIO
target_sources(kimera_vio
  PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/Camera.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoCamera.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoMatcher.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/UndistorterRectifier.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CameraParams.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoFrame.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoMatchingParams.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoImuSyncPacket.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoVisionImuFrontend.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/MonoImuSyncPacket.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/MonoVisionImuFrontend.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VisionImuFrontend.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VisionImuFrontendModule.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VisionImuFrontendParams.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Tracker.cpp"
)

add_subdirectory(feature-detector)
add_subdirectory(rgbd)
add_subdirectory(optical-flow)


