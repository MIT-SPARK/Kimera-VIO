### Add source code
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/VioBackEndModule.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/VioBackEnd.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/VioBackEndParams.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/RegularVioBackEnd.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/RegularVioBackEndParams.cpp"
)
