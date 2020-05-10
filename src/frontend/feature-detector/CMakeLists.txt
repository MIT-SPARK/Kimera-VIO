### Add source code for stereoVIO
target_sources(kimera_vio
  PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetectorParams.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetector.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/NonMaximumSuppression.cpp"
)

add_subdirectory(anms)
