get_filename_component(KimeraVIO_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

list(APPEND CMAKE_MODULE_PATH ${KimeraVIO_CMAKE_DIR})

find_dependency(Boost REQUIRED)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES
  INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()
find_dependency(Gflags REQUIRED)
find_dependency(Glog REQUIRED)
find_dependency(GTSAM REQUIRED)
find_dependency(OpenCV REQUIRED)
find_dependency(opengv REQUIRED)
find_dependency(DBoW2 REQUIRED)
find_dependency(KimeraRPGO REQUIRED)

# We should set those right?
#find_dependency(Thread REQUIRED)

list(REMOVE_AT CMAKE_MODULE_PATH -1)

if(NOT TARGET kimera_vio)
  include("${KimeraVIO_CMAKE_DIR}/kimera_vioTargets.cmake")
endif()
