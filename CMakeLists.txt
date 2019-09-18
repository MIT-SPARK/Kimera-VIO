cmake_minimum_required (VERSION 3.5)
project(SparkVio VERSION 1.0 LANGUAGES CXX)

# Make sure that custom modules like FindXXXX are found
# Currently we do not have any custom find module...
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "===============================================================")
message(STATUS "====================  Dependencies ============================")

### System Dependencies
find_package(Boost REQUIRED COMPONENTS filesystem serialization regex timer date_time thread)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES
  INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()

find_package(GTSAM REQUIRED)
find_package(opengv REQUIRED)
find_package(OpenCV 3.3.1 REQUIRED)

### External Dependencies
include(ExternalProject)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gflags.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/glog.cmake)

### Compile the code
add_library(SparkVio STATIC
  src/StereoVisionFrontEnd.cpp
  src/Tracker.cpp
  src/StereoFrame.cpp
  src/StereoImuSyncPacket.cpp
  src/UtilsGeometry.cpp
  src/UtilsOpenCV.cpp
  src/VioBackEnd.cpp
  src/RegularVioBackEnd.cpp
  src/Histogram.cpp
  src/FeatureSelector.cpp
  src/YamlParser.h
  src/VioBackEndParams.h
  src/VioFrontEndParams.h
  src/RegularVioBackEndParams.h
  src/CameraParams.cpp
  src/Frame.h
  src/Visualizer3D.cpp "")

# Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(SparkVio::SparkVio ALIAS SparkVio)

### Add source code for data provider.
include(src/datasource/CMakeLists.txt)
### Add source code for factors
include(src/factors/CMakeLists.txt)
### Add source code for mesh
include(src/mesh/CMakeLists.txt)
### Add source code for utils
include(src/utils/CMakeLists.txt)
### Add source code for initialization
include(src/initial/CMakeLists.txt)
### Add source code for pipeline
include(src/pipeline/CMakeLists.txt)
### Add source code for common
include(src/common/CMakeLists.txt)
### Add source code for logging
include(src/logging/CMakeLists.txt)
### Add source code for imu frontend
include(src/imu-frontend/CMakeLists.txt)

target_link_libraries(SparkVio
  PRIVATE
    opengv
  PUBLIC
    Boost::boost
    ${OpenCV_LIBRARIES} # TODO: modernize
    gflags::gflags
    glog::glog
    gtsam
    gtsam_unstable
)
target_include_directories(SparkVio
  PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:src>
  )

target_compile_options(SparkVio
  PRIVATE -Wall -pipe
  PRIVATE -march=native)

# We would just need to say cxx_std_11 if we were using cmake 3.8
target_compile_features(SparkVio PUBLIC
        cxx_auto_type cxx_constexpr cxx_range_for cxx_nullptr cxx_override ) # And many more

add_executable(stereoVIOEuroc ./examples/SparkVio.cpp)
target_link_libraries(stereoVIOEuroc PUBLIC SparkVio::SparkVio)

### Add testing
# Download and unpack googletest at configure time
# TODO Consider doing the same for glog, gflags, although it might
# make the command `cmake ..` a bit slow.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gtest.cmake external/googletest-download/CMakeLists.txt)
execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/external/googletest-download" )
execute_process(COMMAND "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/external/googletest-download" )

# Prevent GoogleTest from overriding our compiler/linker options
# when building with Visual Studio
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This adds
# the following targets: gtest, gtest_main, gmock
# and gmock_main
add_subdirectory("${CMAKE_BINARY_DIR}/external/googletest-src"
                 "${CMAKE_BINARY_DIR}/external/googletest-build")

# The gtest/gmock targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if(CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include"
                        "${gmock_SOURCE_DIR}/include")
endif()

# Generate gtests.
include(CTest)
add_executable(testSparkVio
  tests/testSparkVio.cpp
  tests/testCameraParams.cpp
  tests/testCodesignIdeas.cpp
  tests/testFeatureSelector.cpp
  tests/testFrame.cpp
  tests/testGeneralParallelPlaneRegularBasicFactor.cpp
  tests/testGeneralParallelPlaneRegularTangentSpaceFactor.cpp
  tests/testImuFrontEnd.cpp
  tests/testKittiDataProvider.cpp # TODO
  tests/testLogger.cpp
  # tests/testMesher.cpp # rotten
  tests/testParallelPlaneRegularBasicFactor.cpp
  tests/testParallelPlaneRegularTangentSpaceFactor.cpp
  tests/testPointPlaneFactor.cpp
  #tests/testRegularVioBackEnd.cpp # rotten
  tests/testRegularVioBackEndParams.cpp
  tests/testStereoFrame.cpp
  tests/testStereoVisionFrontEnd.cpp
  tests/testThreadsafeImuBuffer.cpp
  tests/testThreadsafeQueue.cpp
  tests/testThreadsafeTemporalBuffer.cpp
  tests/testTimer.cpp
  tests/testTracker.cpp
  tests/testUtilsOpenCV.cpp
  tests/testInitializationFromImu.cpp
  tests/testVioBackEnd.cpp
  tests/testVioBackEndParams.cpp
  tests/testVioFrontEndParams.cpp
  tests/testVisualizer3D.cpp
  )

target_link_libraries(testSparkVio gtest SparkVio::SparkVio)

if(CMAKE_VERSION VERSION_LESS 3.10)
  enable_testing()
  include(FindGTest)
  gtest_add_tests(testSparkVio "" AUTO)
else()
  include(GoogleTest)
  gtest_discover_tests(testSparkVio)
endif()

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
#install(TARGETS SparkVio
#    EXPORT SparkVio-export
#    LIBRARY DESTINATION lib
#    ARCHIVE DESTINATION lib
#    INCLUDES DESTINATION include # We need this right?
#)

## This is required so that the exported target has the name SparkVio and not SparkVio
#set_target_properties(SparkVio PROPERTIES EXPORT_NAME SparkVio)

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
## in the export SparkVio-export.
#install(EXPORT SparkVio-export
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
# This file is necessary to find_package the library SparkVio.
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

set(EXPORTED_DEPENDENCIES "")
list(APPEND EXPORTED_DEPENDENCIES "glog::glog")
list(APPEND EXPORTED_DEPENDENCIES "gflags::gflags")

export(TARGETS SparkVio ${EXPORTED_DEPENDENCIES}
       FILE SparkVioTargets.cmake)
export(PACKAGE SparkVio)

### SIYI's way of exporting.
##target_include_directories(stereoVIO PUBLIC "./src" ${Boost_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} "/usr/local/include/opengv/" ${OpenCV_INCLUDE_DIRS} ${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})
##export(TARGETS stereoVIO FILE stereoVIO.cmake)

###############################################################################
# Print configuration variables
message(STATUS "===============================================================")
message(STATUS "================  Configuration Options  ======================")
message(STATUS "CMAKE_CXX_COMPILER_ID type                : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMAKE_CXX_COMPILER_VERSION                : ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Build flags                                               ")
if(NOT MSVC AND NOT XCODE_VERSION)
    message(STATUS "  Build type                              : ${CMAKE_BUILD_TYPE}")
    if (NOT ${CMAKE_BUILD_TYPE} STREQUAL "Release")
      message(WARNING "  Build type is not in RELEASE, this will slow down the code.")
    endif()
    message(STATUS "  C compilation flags (Release)           : ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE}")
    message(STATUS "  C++ compilation flags (Release)         : ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
endif()
