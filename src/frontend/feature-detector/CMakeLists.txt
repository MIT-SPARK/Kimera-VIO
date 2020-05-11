### Add source code for stereoVIO
target_sources(kimera_vio
  PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/FeatureDetectorParams.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/FeatureDetector.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NonMaximumSuppression.cpp"
)

add_subdirectory(anms)
