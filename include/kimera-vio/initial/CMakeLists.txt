### Add source code for stereoVIO
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/InitializationBackEnd-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/InitializationBackEnd.h"
  "${CMAKE_CURRENT_LIST_DIR}/InitializationFromImu.h"
  "${CMAKE_CURRENT_LIST_DIR}/OnlineGravityAlignment-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/OnlineGravityAlignment.h"
)
