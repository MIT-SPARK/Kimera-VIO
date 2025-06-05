### Add source code
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/VioBackendModule.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackend.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackendParams.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackend.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackendParams.cpp"
### GNSS
  "${CMAKE_CURRENT_LIST_DIR}/GnssVioBackend.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/GnssVioBackendParams.cpp"
)
