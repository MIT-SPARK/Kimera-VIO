### Add source code for distortion models
target_sources(kimera_vio
  PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/DistortionModel.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RadTan4.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RadTan8.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Equidistant4.cpp"
)

