### Add source code for stereoVIO
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/InitializationBackend-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/InitializationBackend.h"
  "${CMAKE_CURRENT_LIST_DIR}/InitializationFromImu.h"
  "${CMAKE_CURRENT_LIST_DIR}/OnlineGravityAlignment-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/OnlineGravityAlignment.h"
)
