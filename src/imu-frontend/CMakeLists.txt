### Add source code for kimera_vio
target_sources(kimera_vio
    PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/ImuFrontEnd.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/ImuFrontEndParams.cpp"
)

