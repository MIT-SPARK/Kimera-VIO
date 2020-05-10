### Add source code for stereoVIO
target_sources(kimera_vio
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/Mesh.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Mesher.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/MesherModule.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/MesherFactory.cpp"
)
