### Add source code just for IDEs
target_sources(KimeraVIO PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackEnd-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackEnd.h"
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackEndParams.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackEnd-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackEnd.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackEndParams.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackEndModule.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackEndFactory.h"
)
### Add include dirs
target_include_directories(KimeraVIO PRIVATE ${CMAKE_CURRENT_LIST_DIR})
