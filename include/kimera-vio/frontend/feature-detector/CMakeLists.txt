### Add source code for IDEs
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetector.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetectorParams.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetector-definitions.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/NonMaximumSuppression.h"
)

add_subdirectory(anms)
