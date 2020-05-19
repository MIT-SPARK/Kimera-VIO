### Add source code just for IDEs
target_sources(kimera_vio
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/vio_types.h"
    "${CMAKE_CURRENT_LIST_DIR}/VioNavState.h"
)
