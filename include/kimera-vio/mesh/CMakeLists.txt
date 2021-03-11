### Add source code for stereoVIO
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/Mesh.h"
  "${CMAKE_CURRENT_LIST_DIR}/MeshUtils.h"
  "${CMAKE_CURRENT_LIST_DIR}/Mesher.h"
  "${CMAKE_CURRENT_LIST_DIR}/MesherModule.h"
  "${CMAKE_CURRENT_LIST_DIR}/MesherFactory.h"
  "${CMAKE_CURRENT_LIST_DIR}/Mesher-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/MeshOptimization.h"
  "${CMAKE_CURRENT_LIST_DIR}/MeshOptimization-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/Mesher_cgal.h"
)
