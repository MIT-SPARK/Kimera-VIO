### Add source code for stereoVIO
target_sources(KimeraVIO
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/Mesh.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Mesher.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/MesherModule.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/MesherFactory.cpp"
)
