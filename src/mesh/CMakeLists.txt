### Add source code for stereoVIO
target_sources(stereoVIO
  PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/Mesh3D.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Mesher.cpp"
  PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}/Mesh3D.h"
  "${CMAKE_CURRENT_LIST_DIR}/Mesher.h"
  "${CMAKE_CURRENT_LIST_DIR}/Mesher_cgal.h"
)
