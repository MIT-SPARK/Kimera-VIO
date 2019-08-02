### Add source code for stereoVIO
target_sources(SparkVio
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/Mesh.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Mesher.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Mesher_cgal.h"
)
target_include_directories(SparkVio
  PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"
)
