### Add source code for IDEs
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/FeatureDetector.h"
  "${CMAKE_CURRENT_LIST_DIR}/FeatureDetectorParams.h"
  "${CMAKE_CURRENT_LIST_DIR}/FeatureDetector-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/NonMaximumSuppression.h"
)

add_subdirectory(anms)
