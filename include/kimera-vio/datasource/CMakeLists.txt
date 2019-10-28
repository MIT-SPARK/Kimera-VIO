### Add source code for IDEs
target_sources(KimeraVIO PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/DataSource-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/DataSource.h"
  "${CMAKE_CURRENT_LIST_DIR}/ETH_parser.h"
  "${CMAKE_CURRENT_LIST_DIR}/KittiDataSource.h"
  )
target_include_directories(KimeraVIO PRIVATE ${CMAKE_CURRENT_LIST_DIR})
