### Add source code for stereoVIO
target_sources(kimera_vio
  PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/CameraParams.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/StereoFrame.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/StereoMatchingParams.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/StereoImuSyncPacket.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/StereoVisionFrontEnd.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/VisionFrontEndModule.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/VisionFrontEndFactory.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/VisionFrontEndParams.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Tracker.cpp"
)

add_subdirectory(feature-detector)

