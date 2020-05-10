### Add source code just for IDEs
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/FactorGraphManagement.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/RegularVioBackEnd-definitions.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/RegularVioBackEnd.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/RegularVioBackEndParams.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/VioBackEnd-definitions.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/VioBackEnd.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/VioBackEndParams.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/VioBackEndModule.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/VioBackEndFactory.h"
)
