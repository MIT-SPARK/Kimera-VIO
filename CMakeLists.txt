cmake_minimum_required (VERSION 3.1)
project (spark_vio)
# set(CMAKE_CXX_FLAGS "-std=c++11") #  -stdlib=libc++
# set(CMAKE_CXX_FLAGS "-stdlib=libc++")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -pipe")

### Include the boost directory
find_package(Boost COMPONENTS filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
message(STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")

### Find the GTSAM package
find_package(GTSAM REQUIRED)
include_directories(BEFORE "${GTSAM_INCLUDE_DIR}")
set (LIBS "${LIBS}" gtsam gtsam_unstable)

### Find the OpenGV package
message(STATUS "Setting OpenGV on Luca's machine")
include_directories(/usr/local/include//opengv/)
set (LIBS "${LIBS}" /usr/local/lib/libopengv.a)

### Find opencv
find_package(OpenCV 3.3.1 REQUIRED)
if (OpenCV_FOUND)
  message(STATUS "FOUND OpenCV! ${OpenCV_INCLUDE_DIRS}")
  message(STATUS "LIB_PATH = ${OpenCV_LIBS}")
else (OpenCV_FOUND)
  message(STATUS "CANNOT FOUND OpenCV!")
endif (OpenCV_FOUND)
include_directories(${OpenCV_INCLUDE_DIRS})
set (LIBS "${LIBS}" ${OpenCV_LIBS})

### External Dependencies
include(ExternalProject)

### Include the gflags directory
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gflags.cmake)
set(LIBS "${LIBS}" ${GFLAGS_LIBRARIES})

### Include the glog directory
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/glog.cmake)
set(LIBS "${LIBS}" ${GLOG_LIBRARIES})

### Display all libs
message(STATUS "ALL LIBS: ${LIBS}")

### Compile the code
add_library(stereoVIO SHARED
  src/ETH_parser.cpp
  src/StereoVisionFrontEnd.cpp
  src/Tracker.cpp
  src/StereoFrame.cpp
  src/StereoImuSyncPacket.cpp
  src/UtilsGeometry.cpp
  src/UtilsOpenCV.cpp
  src/ImuFrontEnd.cpp
  src/VioBackEnd.cpp
  src/RegularVioBackEnd.cpp
  src/Histogram.cpp
  src/LoggerMatlab.cpp
  src/FeatureSelector.cpp
  src/VioBackEndParams.h
  src/VioFrontEndParams.h
  src/RegularVioBackEndParams.h
  src/CameraParams.cpp
  src/Frame.h
  src/Visualizer3D.cpp "")

### Add source code for data provider.
include(src/datasource/CMakeLists.txt)
### Add source code for factors
include(src/factors/CMakeLists.txt)
### Add source code for mesh
include(src/mesh/CMakeLists.txt)
### Add source code for utils
include(src/utils/CMakeLists.txt)
### Add source code for pipeline
include(src/pipeline/CMakeLists.txt)
### Add source code for common
include(src/common/CMakeLists.txt)

if (DEFINED external_project_dependencies)
  add_dependencies(stereoVIO ${external_project_dependencies})
endif()
target_link_libraries(stereoVIO PUBLIC ${LIBS})
target_include_directories(stereoVIO
  PUBLIC
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  "src")

add_executable(stereoVIOEuroc ./examples/stereoVIOEuroc.cpp)
target_link_libraries(stereoVIOEuroc PRIVATE stereoVIO)

add_executable(semanticVIOEuroc ./examples/semanticVIOEuroc.cpp)
target_link_libraries(semanticVIOEuroc PRIVATE stereoVIO)

add_executable(stereoVIOSimulation ./examples/stereoVIOSimulation.cpp)
target_link_libraries(stereoVIOSimulation PRIVATE stereoVIO)

### Include the micro for adding test
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GtsamTesting.cmake)

### Get all the cpp files in ./src except the main file
set(VIO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(cpp_file_pattern "${VIO_SRC_DIR}/*.cpp" "${VIO_SRC_DIR}/factors/*.cpp"
  "${VIO_SRC_DIR}/mesh/*.cpp")
file(GLOB src_cpp_files ${cpp_file_pattern})

#set(main_file_pattern "${VIO_SRC_DIR}/stereoVIOExample.cpp" "${VIO_SRC_DIR}/#stereoVIOExampleSimulation.cpp" "${VIO_SRC_DIR}/stereoVIOPlainBackend.cpp")
#file(GLOB main_files ${main_file_pattern})
#list(REMOVE_ITEM src_cpp_files ${main_files})
message(STATUS "src_cpp_files = ${src_cpp_files}")

### Add testing
add_subdirectory(tests)
#####################################################################################

### Add examples
# add_subdirectory(examples)

## Export stereoVIO library for others to use. Should fix the PUBLIC PRIVATE PARTS.
#target_include_directories(stereoVIO PUBLIC
#  "./src"
#  ${Boost_INCLUDE_DIRS}
#  ${GTSAM_INCLUDE_DIR}
#  "/usr/local/include/opengv/"
#  ${OpenCV_INCLUDE_DIRS}
#  ${GFLAGS_INCLUDE_DIRS}
#  ${GLOG_INCLUDE_DIRS})

#export(TARGETS SparkVio FILE SparkVio.cmake)
#
#
#install(TARGETS SparkVio EXPORT stereoVioTargets
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib
#  RUNTIME DESTINATION bin
#  INCLUDES DESTINATION include
#  )
#install(EXPORT SparkVioTargets
#  FILE SparkVioTargets.cmake
#  NAMESPACE stereoVio::
#  DESTINATION lib/cmake/SparkVio
#  )

#include(CMakePackageConfigHelpers)
#write_basic_package_version_file("SparkVioConfigVersion.cmake"
#  VERSION ${SparkVio_VERSION}
#  COMPATIBILITY SameMajorVersion
#)

##############################################
## Installation instructions
#include(GNUInstallDirs)
#set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/SparkVio)
#
## First of all, we need to install the library itself.
#install(TARGETS sparkvio
#    EXPORT sparkvio-export
#    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#    #   INCLUDES DESTINATION include # We need this right?
#)
## This is required so that the exported target has the name SparkVio and not sparkvio
#set_target_properties(sparkvio PROPERTIES EXPORT_NAME SparkVio)
#
#install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#
## We install the export that we defined above
## Export the targets to a script
## This will install the import script SparkVioTargets.cmake
## When findpackaged by other scripts, it will load the targets defined
## in the export sparkvio-export.
#install(EXPORT sparkvio-export
#  FILE SparkVioTargets.cmake
#  NAMESPACE SparkVio::
#  DESTINATION ${INSTALL_CONFIGDIR}
#)
#
## Create a ConfigVersion.cmake file
#include(CMakePackageConfigHelpers)
#write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/SparkVioConfigVersion.cmake
#    VERSION ${PROJECT_VERSION}
#    COMPATIBILITY AnyNewerVersion
#)
#
## Update SparkVioConfig.cmake with extra info from SparkVioConfig.cmake.in
#configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/SparkVioConfig.cmake.in
#    ${CMAKE_CURRENT_BINARY_DIR}/SparkVioConfig.cmake
#    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
#)
#
## Install the config and configversion
## Here you could also install custom cmake find modules
#install(FILES
#    ${CMAKE_CURRENT_BINARY_DIR}/SparkVioConfig.cmake
#    ${CMAKE_CURRENT_BINARY_DIR}/SparkVioConfigVersion.cmake
#    DESTINATION ${INSTALL_CONFIGDIR}
#)
