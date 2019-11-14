### Add source code for stereoVIO
target_sources(KimeraVIO PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/Mesh.h"
  "${CMAKE_CURRENT_LIST_DIR}/Mesher.h"
  "${CMAKE_CURRENT_LIST_DIR}/Mesher_cgal.h"
)
target_include_directories(KimeraVIO PUBLIC ${CMAKE_CURRENT_LIST_DIR})
