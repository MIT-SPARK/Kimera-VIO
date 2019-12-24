### Add source code for stereoVIO
target_sources(kimera_vio
  PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/CameraParams.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/FeatureSelector.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoFrame.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoImuSyncPacket.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/StereoVisionFrontEnd.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VisionFrontEndModule.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VisionFrontEndFactory.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/FrontendParams.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Tracker.cpp"
)

