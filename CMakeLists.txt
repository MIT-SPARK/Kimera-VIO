cmake_minimum_required (VERSION 3.5)
project(KimeraVIO VERSION 1.0 LANGUAGES CXX)

# Make sure that custom modules like FindXXXX are found
# Currently we do not have any custom find module...
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

IF(APPLE)
    # Fix linking on 10.14+. See https://stackoverflow.com/questions/54068035
    LINK_DIRECTORIES(/usr/local/lib)
ENDIF()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "==============================================================")
message(STATUS "====================  Dependencies ===========================")

### System Dependencies
find_package(Boost REQUIRED COMPONENTS
  filesystem serialization regex timer date_time thread)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES
  INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()

find_package(GTSAM REQUIRED)
find_package(opengv REQUIRED)
find_package(OpenCV 3.3.1 REQUIRED)
find_package(DBoW2 REQUIRED)
if(NOT TARGET DBoW2::DBoW2)
  add_library(DBoW2::DBoW2 INTERFACE IMPORTED)
  set_target_properties(DBoW2::DBoW2 PROPERTIES
  INTERFACE_LINK_LIBRARIES "${DBoW2_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${DBoW2_INCLUDE_DIRS}")
endif()
find_package(KimeraRPGO REQUIRED)

### Unzip the vocabularly file
message(STATUS "Unpacking BoW Vocabulary File.")
execute_process(
  COMMAND ${CMAKE_COMMAND} -E tar xzf ${CMAKE_SOURCE_DIR}/vocabulary/ORBvoc.zip
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/vocabulary
)

### External Dependencies
include(ExternalProject)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gflags.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/glog.cmake)

### Compile the code
add_library(KimeraVIO SHARED "")
# Add an alias so that library can be used inside the build tree,
# e.g. when testing
add_library(KimeraVIO::KimeraVIO ALIAS KimeraVIO)

### Add source code for data provider.
include(include/kimera-vio/datasource/CMakeLists.txt)
include(src/datasource/CMakeLists.txt)
### Add source code for frontend.
include(include/kimera-vio/frontend/CMakeLists.txt)
include(src/frontend/CMakeLists.txt)
### Add source code for backend.
include(include/kimera-vio/backend/CMakeLists.txt)
include(src/backend/CMakeLists.txt)
### Add source code for factors
include(include/kimera-vio/factors/CMakeLists.txt)
include(src/factors/CMakeLists.txt)
### Add source code for mesh
include(include/kimera-vio/mesh/CMakeLists.txt)
include(src/mesh/CMakeLists.txt)
### Add source code for initialization
include(include/kimera-vio/initial/CMakeLists.txt)
include(src/initial/CMakeLists.txt)
### Add source code for utils
include(include/kimera-vio/utils/CMakeLists.txt)
include(src/utils/CMakeLists.txt)
### Add source code for pipeline
include(include/kimera-vio/pipeline/CMakeLists.txt)
include(src/pipeline/CMakeLists.txt)
### Add source code for common
include(include/kimera-vio/common/CMakeLists.txt)
include(src/common/CMakeLists.txt)
### Add source code for loopclosure
include(include/kimera-vio/loopclosure/CMakeLists.txt)
include(src/loopclosure/CMakeLists.txt)
### Add source code for logging
include(include/kimera-vio/logging/CMakeLists.txt)
include(src/logging/CMakeLists.txt)
### Add source code for imu frontend
include(include/kimera-vio/imu-frontend/CMakeLists.txt)
include(src/imu-frontend/CMakeLists.txt)
### Add source code for visualizer.
include(include/kimera-vio/visualizer/CMakeLists.txt)
include(src/visualizer/CMakeLists.txt)

target_link_libraries(KimeraVIO
  PRIVATE
    opengv
    DBoW2::DBoW2
    KimeraRPGO
  PUBLIC
    Boost::boost
    ${OpenCV_LIBRARIES}
    gflags::gflags
    glog::glog
    gtsam
    gtsam_unstable
 )
target_include_directories(KimeraVIO
  PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(KimeraVIO
  PRIVATE -Wall -pipe
  PRIVATE -march=native
)

# We would just need to say cxx_std_11 if we were using cmake 3.8
target_compile_features(KimeraVIO PUBLIC
        cxx_auto_type cxx_constexpr cxx_range_for cxx_nullptr cxx_override)

add_executable(stereoVIOEuroc ./examples/KimeraVIO.cpp)
target_link_libraries(stereoVIOEuroc PUBLIC KimeraVIO::KimeraVIO)

### Add testing
# Download and unpack googletest at configure time
# TODO Consider doing the same for glog, gflags, although it might
# make the command `cmake ..` a bit slow.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gtest.cmake
  external/googletest-download/CMakeLists.txt)
execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/external/googletest-download"
    OUTPUT_QUIET)
execute_process(COMMAND "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/external/googletest-download"
    OUTPUT_QUIET)

# Prevent GoogleTest from overriding our compiler/linker options
# when building with Visual Studio
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This adds
# the following targets: gtest, gtest_main, gmock
# and gmock_main
# Exclude these targets from installation with EXCLUDE_FROM_ALL
# Check issue: https://github.com/google/googletest/issues/868
add_subdirectory("${CMAKE_BINARY_DIR}/external/googletest-src"
                 "${CMAKE_BINARY_DIR}/external/googletest-build"
                 EXCLUDE_FROM_ALL)

# The gtest/gmock targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if(CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include"
                        "${gmock_SOURCE_DIR}/include")
endif()

# Generate gtests.
include(CTest)
add_executable(testKimeraVIO
  tests/testKimeraVIO.cpp
  tests/testCameraParams.cpp
  tests/testCodesignIdeas.cpp
  tests/testFeatureSelector.cpp
  tests/testFrame.cpp # NEEDS UPDATE
  tests/testGeneralParallelPlaneRegularBasicFactor.cpp
  tests/testGeneralParallelPlaneRegularTangentSpaceFactor.cpp
  tests/testImuFrontEnd.cpp
  tests/testKittiDataProvider.cpp # TODO
  tests/testLoopClosureDetector.cpp
  tests/testLogger.cpp
  tests/testMesher.cpp # rotten
  tests/testParallelPlaneRegularBasicFactor.cpp
  tests/testParallelPlaneRegularTangentSpaceFactor.cpp
  tests/testPointPlaneFactor.cpp
  #tests/testRegularVioBackEnd.cpp # rotten
  tests/testRegularVioBackEndParams.cpp
  tests/testStereoFrame.cpp # NEEDS UPDATE
  tests/testStereoVisionFrontEnd.cpp # NEEDS UPDATE
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
  tests/testVisualizer3D.cpp # NEEDS UPDATE
  tests/testOnlineAlignment.cpp
  )

target_link_libraries(testKimeraVIO gtest KimeraVIO::KimeraVIO)

if(CMAKE_VERSION VERSION_LESS 3.10)
  enable_testing()
  include(FindGTest)
  gtest_add_tests(testKimeraVIO "" AUTO)
else()
  include(GoogleTest)
  gtest_discover_tests(testKimeraVIO)
endif()

#export(TARGETS KimeraVIO FILE KimeraVIO.cmake)
#
#
#install(TARGETS KimeraVIO EXPORT stereoVioTargets
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib
#  RUNTIME DESTINATION bin
#  INCLUDES DESTINATION include
#  )
#install(EXPORT KimeraVIOTargets
#  FILE KimeraVIOTargets.cmake
#  NAMESPACE stereoVio::
#  DESTINATION lib/cmake/KimeraVIO
#  )

#include(CMakePackageConfigHelpers)
#write_basic_package_version_file("KimeraVIOConfigVersion.cmake"
#  VERSION ${KimeraVIO_VERSION}
#  COMPATIBILITY SameMajorVersion
#)

##############################################
## Installation instructions
#include(GNUInstallDirs)
#
## First of all, we need to install the library itself.
#install(TARGETS KimeraVIO
#    EXPORT KimeraVIO-export
#    LIBRARY DESTINATION lib
#    ARCHIVE DESTINATION lib
#    INCLUDES DESTINATION include # We need this right?
#)

## This is required so that the exported target has the name KimeraVIO and not KimeraVIO
#set_target_properties(KimeraVIO PROPERTIES EXPORT_NAME KimeraVIO)

## TODO THIS SHOULD BE only the headers... aka include/
##install(DIRECTORY src/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#
## Install headers of our dependencies, not sure if it works...
#install(DIRECTORY ${GTSAM_INCLUDE_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#install(DIRECTORY ${GFLAGS_INCLUDE_DIRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#install(DIRECTORY ${GLOG_INCLUDE_DIRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#
## CMAKE_INSTALL_LIBDIR is lib usually
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/KimeraVIO)

## We install the export that we defined above
## Export the targets to a script
## This will install the import script KimeraVIOTargets.cmake
## When findpackaged by other scripts, it will load the targets defined
## in the export KimeraVIO-export.
#install(EXPORT KimeraVIO-export
#  FILE KimeraVIOTargets.cmake
#  NAMESPACE KimeraVIO::
## It's usually a good idea to give CMake access to the version,
## so that find_package can have a version specified.
## Create a ConfigVersion.cmake file
## It's usually a good idea to give CMake access to the version,
## so that find_package can have a version specified.
## Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/KimeraVIOConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Create KimeraVIOConfig.cmake with extra info from KimeraVIOConfig.cmake.in
# This file is necessary to find_package the library KimeraVIO.
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/KimeraVIOConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/KimeraVIOConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)
#
## Install the config and configversion
## Here you could also install custom cmake find modules
#install(FILES
#    ${CMAKE_CURRENT_BINARY_DIR}/KimeraVIOConfig.cmake
#    ${CMAKE_CURRENT_BINARY_DIR}/KimeraVIOConfigVersion.cmake
#    DESTINATION ${INSTALL_CONFIGDIR}
#)

set(EXPORTED_DEPENDENCIES "")
list(APPEND EXPORTED_DEPENDENCIES "glog::glog")
list(APPEND EXPORTED_DEPENDENCIES "gflags::gflags")

export(TARGETS KimeraVIO ${EXPORTED_DEPENDENCIES}
       FILE KimeraVIOTargets.cmake)
export(PACKAGE KimeraVIO)

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

#include(GNUInstallDirs)
#install(TARGETS KimeraVIO
#    EXPORT kimera-vio-export
#    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#)
#install(EXPORT kimera-vio-export
#  FILE
#    KimeraVIOTargets.cmake
#    #NAMESPACE
#    #NAKimeraVIO::
#  DESTINATION
#    ${CMAKE_INSTALL_LIBDIR}/cmake/KimeraVIO
#)
