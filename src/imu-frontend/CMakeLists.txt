### Add source code for SparkVio
target_sources(SparkVio
    PRIVATE
      "${CMAKE_CURRENT_LIST_DIR}/ImuFrontEnd.cpp"
      "${CMAKE_CURRENT_LIST_DIR}/ImuFrontEnd.h"
      "${CMAKE_CURRENT_LIST_DIR}/ImuFrontEndParams.cpp"
      "${CMAKE_CURRENT_LIST_DIR}/ImuFrontEndParams.h"
      "${CMAKE_CURRENT_LIST_DIR}/ImuFrontEnd-definitions.h"
)
target_include_directories(SparkVio PRIVATE ${CMAKE_CURRENT_LIST_DIR})

