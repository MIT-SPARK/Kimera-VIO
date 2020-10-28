### Add source code for IDEs
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/Camera.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoCamera.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoMatcher.h"
  "${CMAKE_CURRENT_LIST_DIR}/UndistorterRectifier.h"
  "${CMAKE_CURRENT_LIST_DIR}/CameraParams.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoMatchingParams.h"
  "${CMAKE_CURRENT_LIST_DIR}/Frame.h"
  "${CMAKE_CURRENT_LIST_DIR}/FrontendInputPacketBase.h"
  "${CMAKE_CURRENT_LIST_DIR}/FrontendOutputPacketBase.h"
  "${CMAKE_CURRENT_LIST_DIR}/MonoVisionFrontEnd.h"
  "${CMAKE_CURRENT_LIST_DIR}/MonoVisionFrontEnd-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoFrame-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoFrame.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoImuSyncPacket.h"
  "${CMAKE_CURRENT_LIST_DIR}/MonoImuSyncPacket.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoVisionFrontEnd-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoVisionFrontEnd.h"
  "${CMAKE_CURRENT_LIST_DIR}/VisionFrontEnd.h"
  "${CMAKE_CURRENT_LIST_DIR}/VisionFrontEnd-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/VisionFrontEndParams.h"
  "${CMAKE_CURRENT_LIST_DIR}/VisionFrontEndModule.h"
  "${CMAKE_CURRENT_LIST_DIR}/VisionFrontEndFactory.h"
  "${CMAKE_CURRENT_LIST_DIR}/Tracker-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/Tracker.h"
  "${CMAKE_CURRENT_LIST_DIR}/OpticalFlowPredictor-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/OpticalFlowPredictor.h"
  "${CMAKE_CURRENT_LIST_DIR}/OpticalFlowPredictorFactory.h"
)

add_subdirectory(feature-detector)
add_subdirectory(rgbd)
