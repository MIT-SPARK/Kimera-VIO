### Add source code for IDEs
target_sources(KimeraVIO PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/DataProviderInterface-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/DataProviderModule.h"
  "${CMAKE_CURRENT_LIST_DIR}/DataProviderInterface.h"
  "${CMAKE_CURRENT_LIST_DIR}/EurocDataProvider.h"
  "${CMAKE_CURRENT_LIST_DIR}/KittiDataProvider.h"
  )
