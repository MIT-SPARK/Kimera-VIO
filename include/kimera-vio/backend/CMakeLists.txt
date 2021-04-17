### Add source code just for IDEs
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/FactorGraphManagement.h"
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackend-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackend.h"
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackendParams.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackend-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackend.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackendParams.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackendModule.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackendFactory.h"
)
