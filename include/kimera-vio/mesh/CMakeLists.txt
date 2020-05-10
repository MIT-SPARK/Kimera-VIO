### Add source code for stereoVIO
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/Mesh.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/Mesher.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/MesherModule.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/MesherFactory.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/Mesher-definitions.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/Mesher_cgal.h"
)
