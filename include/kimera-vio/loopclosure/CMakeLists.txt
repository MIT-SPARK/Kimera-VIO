### Add source code for LoopClosureDetector
target_sources(kimera_vio PRIVATE
"${CMAKE_CURRENT_LIST_DIR}/LcdThirdPartyWrapper.h"
"${CMAKE_CURRENT_LIST_DIR}/LcdModule.h"
"${CMAKE_CURRENT_LIST_DIR}/LcdFactory.h"
 "${CMAKE_CURRENT_LIST_DIR}/LoopClosureDetector-definitions.h"
 "${CMAKE_CURRENT_LIST_DIR}/LoopClosureDetector.h"
 "${CMAKE_CURRENT_LIST_DIR}/LoopClosureDetectorParams.h"
)
