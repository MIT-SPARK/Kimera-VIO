### Add source code for IDEs
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/Camera.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/CameraParams.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/StereoMatchingParams.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/Frame.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/StereoFrame-definitions.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/StereoFrame.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/StereoImuSyncPacket.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/StereoVisionFrontEnd-definitions.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/StereoVisionFrontEnd.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/VisionFrontEndParams.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/VisionFrontEndModule.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/VisionFrontEndFactory.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/Tracker-definitions.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/Tracker.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/OpticalFlowPredictor-definitions.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/OpticalFlowPredictor.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/OpticalFlowPredictorFactory.h"
)

add_subdirectory(feature-detector)
