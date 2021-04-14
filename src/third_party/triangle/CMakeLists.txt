add_library(triangle STATIC
  #"${CMAKE_CURRENT_SOURCE_DIR}/include/triangle/triangle.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/triangle.c"
)
# TODO(nathan) pick up -march=native from top-level file
target_compile_options(triangle
    PRIVATE -march=native -Wno-format)
target_include_directories(triangle PRIVATE "${PROJECT_SOURCE_DIR}/include")
set_target_properties(triangle PROPERTIES POSITION_INDEPENDENT_CODE ON)
