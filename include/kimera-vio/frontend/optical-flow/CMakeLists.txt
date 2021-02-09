### Add source code for IDEs
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/OpticalFlowPredictor.h"
  "${CMAKE_CURRENT_LIST_DIR}/OpticalFlowPredictorFactory.h"
  "${CMAKE_CURRENT_LIST_DIR}/OpticalFlowPredictor-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/OpticalFlowVisualizer.h"
)
