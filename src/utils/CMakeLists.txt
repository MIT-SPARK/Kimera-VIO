### Add source code for stereoVIO
target_sources(kimera_vio
  PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/Histogram.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/FilesystemUtils.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Statistics.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ThreadsafeImuBuffer.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ThreadsafeOdometryBuffer.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/UtilsGeometry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/UtilsNumerical.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/UtilsOpenCV.cpp"
)
