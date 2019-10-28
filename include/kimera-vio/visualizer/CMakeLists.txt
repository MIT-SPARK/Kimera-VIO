### Add includes
target_sources(KimeraVIO PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/Visualizer3D.h"
)
target_include_directories(KimeraVIO PRIVATE ${CMAKE_CURRENT_LIST_DIR})

