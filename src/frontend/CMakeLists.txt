### Add source code for stereoVIO
target_sources(kimera_vio
  PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/Camera.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoCamera.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/UndistorterRectifier.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CameraParams.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoFrame.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoMatchingParams.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoImuSyncPacket.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoVisionFrontEnd.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VisionFrontEndModule.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VisionFrontEndFactory.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VisionFrontEndParams.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Tracker.cpp"
)

add_subdirectory(feature-detector)

