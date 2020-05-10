### Add source code for IDEs
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/DataProviderInterface-definitions.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/DataProviderModule.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/DataProviderInterface.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/EurocDataProvider.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/KittiDataProvider.h"
  )
