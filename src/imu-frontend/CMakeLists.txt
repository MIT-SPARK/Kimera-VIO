### Add source code for kimera_vio
target_sources(kimera_vio
    PRIVATE
      "${CMAKE_CURRENT_LIST_DIR}/ImuFrontend.cpp"
      "${CMAKE_CURRENT_LIST_DIR}/ImuFrontendParams.cpp"
)

