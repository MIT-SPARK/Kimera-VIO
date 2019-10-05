### Add source code for stereoVIO
target_sources(KimeraVIO
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/ThreadsafeImuBuffer.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Statistics.cpp"
)
