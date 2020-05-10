### Add source code just for IDEs
target_sources(kimera_vio
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/vio_types.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/VioNavState.h"
)
