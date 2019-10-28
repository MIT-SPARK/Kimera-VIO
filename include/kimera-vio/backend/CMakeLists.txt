### Add source code just for IDEs
target_sources(KimeraVIO PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackEnd.h"
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackEndParams.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackEnd-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackEnd.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackEndParams.h"
)
### Add include dirs
target_include_directories(KimeraVIO PRIVATE ${CMAKE_CURRENT_LIST_DIR})
