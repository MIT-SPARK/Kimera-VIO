### Add source code for LoopClosureDetector
target_sources(KimeraVIO PRIVATE
 "${CMAKE_CURRENT_LIST_DIR}/LoopClosureDetector-definitions.h"
 "${CMAKE_CURRENT_LIST_DIR}/LoopClosureDetector.h"
 "${CMAKE_CURRENT_LIST_DIR}/LoopClosureDetectorParams.h"
 "${CMAKE_CURRENT_LIST_DIR}/LcdThirdPartyWrapper.h"
)
target_include_directories(KimeraVIO PRIVATE ${CMAKE_CURRENT_LIST_DIR})
