### Add includes
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/Visualizer3D-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/Visualizer3DModule.h"
  "${CMAKE_CURRENT_LIST_DIR}/Visualizer3DFactory.h"
  "${CMAKE_CURRENT_LIST_DIR}/Visualizer3D.h"
  "${CMAKE_CURRENT_LIST_DIR}/OpenCvVisualizer3D.h"
  "${CMAKE_CURRENT_LIST_DIR}/DisplayParams.h"
  "${CMAKE_CURRENT_LIST_DIR}/Display-definitions.h"
  "${CMAKE_CURRENT_LIST_DIR}/DisplayModule.h"
  "${CMAKE_CURRENT_LIST_DIR}/DisplayFactory.h"
  "${CMAKE_CURRENT_LIST_DIR}/Display.h"
  "${CMAKE_CURRENT_LIST_DIR}/OpenCvDisplay.h"
  "${CMAKE_CURRENT_LIST_DIR}/OpenCvDisplayParams.h"
)

if(Pangolin_FOUND)
  target_sources(kimera_vio PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/PangolinDisplay.h"
  )
endif(Pangolin_FOUND)
