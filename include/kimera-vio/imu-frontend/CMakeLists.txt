### Add source code for KimeraVIO
target_sources(KimeraVIO PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/ImuFrontEnd-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/ImuFrontEnd.h"
  "${CMAKE_CURRENT_LIST_DIR}/ImuFrontEndParams.h"
)
target_include_directories(KimeraVIO PRIVATE ${CMAKE_CURRENT_LIST_DIR})

