### Add source code for IDEs
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/Camera.h"
  "${CMAKE_CURRENT_LIST_DIR}/CameraParams.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoMatchingParams.h"
  "${CMAKE_CURRENT_LIST_DIR}/FeatureSelector.h"
  "${CMAKE_CURRENT_LIST_DIR}/Frame.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoFrame-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoFrame.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoImuSyncPacket.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoVisionFrontEnd-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/StereoVisionFrontEnd.h"
  "${CMAKE_CURRENT_LIST_DIR}/VisionFrontEndModule.h"
  "${CMAKE_CURRENT_LIST_DIR}/VisionFrontEndFactory.h"
  "${CMAKE_CURRENT_LIST_DIR}/Tracker-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/Tracker.h"
  "${CMAKE_CURRENT_LIST_DIR}/VioFrontEndParams.h"
)
