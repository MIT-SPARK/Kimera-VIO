cmake_minimum_required (VERSION 3.5)
project(SparkVio VERSION 1.0 LANGUAGES CXX)

# Make sure that custom modules like FindXXXX are found
# Currently we do not have any custom find module...
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

### Include the boost directory
find_package(Boost REQUIRED COMPONENTS filesystem)
message(STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")

### Eigen
# Although we are using eigen ourselves, if we link against
# one that is different from GTSAM's one, we end up with
# issue described in :
# https://bitbucket.org/gtborg/gtsam/issues/295
# If you have multiple installed version of Eigen,
#  you can pick your favorite one by setting the Eigen3_DIR
#  cmake's variable to the respective path containing
#  the Eigen3*.cmake files.
# Recommended to use GTSAM's one, others should work as well.
set(EIGEN3_INCLUDE_DIR "" CACHE PATH
  "User settable path to eigen's include directory. (e.g. /usr/local/eigen3)")
if(NOT EIGEN3_INCLUDE_DIR) # User did not specify eigen, using system wide one.
  message(STATUS "Using system-wide Eigen.")
  find_package(Eigen3 3.2.5 REQUIRED NO_MODULE) # 3.2.5 for MKL for gtsam apparently
else() # ALLOW the user to specify EIGEN3_INCLUDE_DIR directly
  message(STATUS "Using user-specified Eigen.")
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()
message(STATUS "EIGEN3_INCLUDE_DIR = ${EIGEN3_INCLUDE_DIR}")

### Find the GTSAM package
find_package(GTSAM REQUIRED)

### Find the OpenGV package
message(STATUS "Setting OpenGV on Luca's machine")
# TODO REMOVE HARDCODED

### Find opencv
find_package(OpenCV 3.3.1 REQUIRED)
if (OpenCV_FOUND)
  message(STATUS "FOUND OpenCV! ${OpenCV_INCLUDE_DIRS}")
  message(STATUS "LIB_PATH = ${OpenCV_LIBS}")
else (OpenCV_FOUND)
  message(STATUS "CANNOT FOUND OpenCV!")
endif (OpenCV_FOUND)

### External Dependencies
include(ExternalProject)

### Include the gflags directory
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gflags.cmake)

### Include the glog directory
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/glog.cmake)

### Display all libs
message(STATUS "ALL LIBS: ${LIBS}")

### Compile the code
add_library(sparkvio SHARED
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

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(SparkVio::sparkvio ALIAS sparkvio)

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
  add_dependencies(sparkvio ${external_project_dependencies})
endif()

target_link_libraries(sparkvio
  PUBLIC
  /usr/local/lib/libopengv.a
  Boost::boost
  Boost::filesystem
  ${OpenCV_LIBS}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  gtsam
  gtsam_unstable
  )
target_include_directories(sparkvio
  PRIVATE
    ${GTSAM_INCLUDE_DIR}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${GFLAGS_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${GLOG_INCLUDE_DIRS}>

    $<INSTALL_INTERFACE:src>
    $<INSTALL_INTERFACE:${GFLAGS_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:${GLOG_INCLUDE_DIRS}>
  )
target_compile_options(sparkvio PRIVATE -Wall -pipe) # Remove these
# We would just need to say cxx_std_11 if we were using cmake 3.8
target_compile_features(sparkvio PUBLIC
        cxx_auto_type cxx_constexpr cxx_range_for cxx_nullptr cxx_override ) # And many more

add_executable(stereoVIOEuroc ./examples/stereoVIOEuroc.cpp)
target_link_libraries(stereoVIOEuroc PUBLIC SparkVio::sparkvio PRIVATE Eigen3::Eigen)

add_executable(semanticVIOEuroc ./examples/semanticVIOEuroc.cpp)
target_link_libraries(semanticVIOEuroc PUBLIC SparkVio::sparkvio PRIVATE Eigen3::Eigen)

add_executable(stereoVIOSimulation ./examples/stereoVIOSimulation.cpp)
target_link_libraries(stereoVIOSimulation PUBLIC SparkVio::sparkvio PRIVATE Eigen3::Eigen)

### Include the micro for adding test
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GtsamTesting.cmake)

### Get all the cpp files in ./src except the main file
set(VIO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(cpp_file_pattern
  "${VIO_SRC_DIR}/*.cpp"
  "${VIO_SRC_DIR}/*/*.cpp")
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
#
## First of all, we need to install the library itself.
#install(TARGETS sparkvio
#    EXPORT sparkvio-export
#    LIBRARY DESTINATION lib
#    ARCHIVE DESTINATION lib
#    INCLUDES DESTINATION include # We need this right?
#)

## This is required so that the exported target has the name SparkVio and not sparkvio
#set_target_properties(sparkvio PROPERTIES EXPORT_NAME SparkVio)

## TODO THIS SHOULD BE only the headers... aka include/
##install(DIRECTORY src/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#
## Install headers of our dependencies, not sure if it works...
#install(DIRECTORY ${GTSAM_INCLUDE_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#install(DIRECTORY ${GFLAGS_INCLUDE_DIRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#install(DIRECTORY ${GLOG_INCLUDE_DIRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#
## CMAKE_INSTALL_LIBDIR is lib usually
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/SparkVio)

## We install the export that we defined above
## Export the targets to a script
## This will install the import script SparkVioTargets.cmake
## When findpackaged by other scripts, it will load the targets defined
## in the export sparkvio-export.
#install(EXPORT sparkvio-export
#  FILE SparkVioTargets.cmake
#  NAMESPACE SparkVio::
## It's usually a good idea to give CMake access to the version,
## so that find_package can have a version specified.
## Create a ConfigVersion.cmake file
## It's usually a good idea to give CMake access to the version,
## so that find_package can have a version specified.
## Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/SparkVioConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Create SparkVioConfig.cmake with extra info from SparkVioConfig.cmake.in
# This file is necessary to find_package the library sparkvio.
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/SparkVioConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/SparkVioConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)
#
## Install the config and configversion
## Here you could also install custom cmake find modules
#install(FILES
#    ${CMAKE_CURRENT_BINARY_DIR}/SparkVioConfig.cmake
#    ${CMAKE_CURRENT_BINARY_DIR}/SparkVioConfigVersion.cmake
#    DESTINATION ${INSTALL_CONFIGDIR}
#)

export(TARGETS sparkvio
       NAMESPACE SparkVio::
       FILE SparkVioTargets.cmake)
export(PACKAGE SparkVio)

### SIYI's way of exporting.
##target_include_directories(stereoVIO PUBLIC "./src" ${Boost_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} "/usr/local/include/opengv/" ${OpenCV_INCLUDE_DIRS} ${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})
##export(TARGETS stereoVIO FILE stereoVIO.cmake)
