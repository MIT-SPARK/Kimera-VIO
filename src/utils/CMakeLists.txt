### Add source code for stereoVIO
target_sources(kimera_vio
  PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/ThreadsafeImuBuffer.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Statistics.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Histogram.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/UtilsGeometry.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/UtilsOpenCV.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/UtilsNumerical.cpp"
)
