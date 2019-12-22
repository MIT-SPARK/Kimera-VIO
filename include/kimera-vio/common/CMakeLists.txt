### Add source code just for IDEs
target_sources(KimeraVIO
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/vio_types.h"
    "${CMAKE_CURRENT_LIST_DIR}/FilesystemUtils.h"
)
