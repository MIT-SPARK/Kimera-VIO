cmake_minimum_required (VERSION 3.5)
project(triangle VERSION 0.1.6 LANGUAGES CXX C)

add_library(triangle STATIC "${CMAKE_CURRENT_LIST_DIR}/src/triangle.c")
target_include_directories(triangle PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>)
set_target_properties(triangle PROPERTIES POSITION_INDEPENDENT_CODE ON)
#TODO(someone) consider removing march=native here
target_compile_options(triangle PRIVATE -march=native -Wno-format)

add_library(triangle::triangle ALIAS triangle)

install(TARGETS triangle EXPORT kimera_vio-export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  COMPONENT library
)
