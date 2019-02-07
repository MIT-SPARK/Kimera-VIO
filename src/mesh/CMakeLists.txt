### Add source code for stereoVIO
target_sources(sparkvio
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/Mesh.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Mesher.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Mesher_cgal.h"
    "${CMAKE_CURRENT_LIST_DIR}/Mesh.h"
    "${CMAKE_CURRENT_LIST_DIR}/Mesher.h"
)
target_include_directories(sparkvio PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
