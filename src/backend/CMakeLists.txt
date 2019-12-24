### Add source code
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackEnd.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackEndParams.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackEnd.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackEndParams.cpp"
)
