### Add source code for stereoVIO
target_sources(stereoVIO
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/Mesh.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Mesher.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Mesher_cgal.h"
  PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/Mesh.h"
    "${CMAKE_CURRENT_LIST_DIR}/Mesher.h"
)
target_include_directories(stereoVIO PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
