### Add source code for gtsam distortion models
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/DistortionModel.h"
  "${CMAKE_CURRENT_LIST_DIR}/RadTan4.h"
  "${CMAKE_CURRENT_LIST_DIR}/RadTan8.h"
  "${CMAKE_CURRENT_LIST_DIR}/Equidistant4.h"
)
