### Add source code for stereoVIO
target_sources(sparkvio
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/Mesh.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Mesher.cpp"
)
target_include_directories(sparkvio
  PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)
