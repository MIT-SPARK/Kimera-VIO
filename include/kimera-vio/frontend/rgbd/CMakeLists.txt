### Add source code for IDEs
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/DepthFrame.h"
  "${CMAKE_CURRENT_LIST_DIR}/RgbdCamera.h"
  "${CMAKE_CURRENT_LIST_DIR}/RgbdFrame.h"
  "${CMAKE_CURRENT_LIST_DIR}/RgbdImuSyncPacket.h"
)
