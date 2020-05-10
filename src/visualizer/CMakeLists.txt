### Add source code
target_sources(kimera_vio
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/Visualizer3D.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Visualizer3DModule.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Visualizer3DFactory.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Display-definitions.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Display.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/DisplayModule.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/DisplayFactory.cpp"
)
