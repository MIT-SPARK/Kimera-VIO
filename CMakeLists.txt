cmake_minimum_required (VERSION 3.5)
project(kimera_vio VERSION 1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Make sure that custom modules like FindXXXX are found
# Prefer to use system installation of gflags/glog
set(GFLAGS_PREFER_EXPORTED_GFLAGS_CMAKE_CONFIGURATION TRUE)
set(GLOG_PREFER_EXPORTED_GLOG_CMAKE_CONFIGURATION TRUE)
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

find_package(Gflags REQUIRED)
find_package(Glog 0.3.5 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(opengv REQUIRED)
find_package(OpenCV REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(KimeraRPGO REQUIRED)
# Pangolin is optional
find_package(Pangolin QUIET)

include(VerifyGtsamConfig)
option(KIMERA_VERIFY_GTSAM_CONFIG "Check that GTSAM was compiled with the right options" ON)
if (KIMERA_VERIFY_GTSAM_CONFIG)
    verify_gtsam_config()
endif()

### External Dependencies
include(ExternalProject)

### Download and unzip the vocabularly file
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.yml)
  message(STATUS "Downloading vocabulary file from dropbox.")
  file(DOWNLOAD
       http://www.dropbox.com/s/lyo0qgbdxn6eg6o/ORBvoc.zip?dl=1
       ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.zip
       SHOW_PROGRESS
       STATUS orbvoc_download_success
       TIMEOUT 60)
  if(orbvoc_download_success)
    message(STATUS "Unzipping vocabulary file.")

    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.zip
                            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/)
    message(STATUS "Moving vocabulary file.")
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory
                    ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc/
                    ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/)
    execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc/)
  else(orbvoc_download_success)
    message(STATUS "Failed to download vocabulary file. Please download manually.")
  endif(orbvoc_download_success)
else()
  message(STATUS "Vocabulary file exists, will not download.")
endif()

### Handle third party libraries
add_subdirectory(third_party)

### Compile the code
add_library(${PROJECT_NAME} SHARED "")

### Add source code for data provider.
add_subdirectory(include/kimera-vio/dataprovider)
add_subdirectory(src/dataprovider)
### Add source code for data provider.
add_subdirectory(include/kimera-vio/playground)
add_subdirectory(src/playground)
### Add source code for Frontend.
add_subdirectory(include/kimera-vio/frontend)
add_subdirectory(src/frontend)
### Add source code for Backend.
add_subdirectory(include/kimera-vio/backend)
add_subdirectory(src/backend)
### Add source code for factors
add_subdirectory(include/kimera-vio/factors)
add_subdirectory(src/factors)
### Add source code for mesh
add_subdirectory(include/kimera-vio/mesh)
add_subdirectory(src/mesh)
### Add source code for initialization
add_subdirectory(include/kimera-vio/initial)
add_subdirectory(src/initial)
### Add source code for utils
add_subdirectory(include/kimera-vio/utils)
add_subdirectory(src/utils)
### Add source code for pipeline
add_subdirectory(include/kimera-vio/pipeline)
add_subdirectory(src/pipeline)
### Add source code for common
add_subdirectory(include/kimera-vio/common)
add_subdirectory(src/common)
### Add source code for loopclosure
add_subdirectory(include/kimera-vio/loopclosure)
add_subdirectory(src/loopclosure)
### Add source code for logging
add_subdirectory(include/kimera-vio/logging)
add_subdirectory(src/logging)
### Add source code for imu Frontend
add_subdirectory(include/kimera-vio/imu-frontend)
add_subdirectory(src/imu-frontend)
### Add source code for visualizer.
add_subdirectory(include/kimera-vio/visualizer)
add_subdirectory(src/visualizer)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    KimeraRPGO
    triangle::triangle
  PUBLIC
    ${OpenCV_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${DBoW2_LIBRARIES}
    gtsam
    gtsam_unstable
    opengv
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${DBoW2_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GFLAGS_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Pangolin is optional
if(Pangolin_FOUND)
  target_link_libraries(${PROJECT_NAME}
    PUBLIC
      ${Pangolin_LIBRARIES}
  )
  target_include_directories(${PROJECT_NAME}
    PUBLIC
      ${Pangolin_INCLUDE_DIRS}
  )
  target_compile_definitions(${PROJECT_NAME} PRIVATE Pangolin_FOUND=1)
else(Pangolin_FOUND)
  message(STATUS "Pangolin not found.")
endif(Pangolin_FOUND)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wno-unused-parameter -pipe)

# We would just need to say cxx_std_11 if we were using cmake 3.8
target_compile_features(${PROJECT_NAME} PUBLIC
        cxx_auto_type cxx_constexpr cxx_range_for cxx_nullptr cxx_override)

# Add an alias so that library can be used inside the build tree,
# e.g. when testing
add_library(kimera_vio::kimera_vio ALIAS kimera_vio)

add_executable(stereoVIOEuroc ./examples/KimeraVIO.cpp)
target_link_libraries(stereoVIOEuroc PUBLIC kimera_vio::kimera_vio)

############################### TESTS ##########################################
### Add testing
option(KIMERA_BUILD_TESTS "Build tests" ON)
if(KIMERA_BUILD_TESTS)
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
    tests/testStereoImuPipeline.cpp
    tests/testEurocPlayground.cpp
    tests/testCamera.cpp # NEEDS UPDATE
    tests/testCrossCorrelation.cpp
    tests/testDepthFrame.cpp
    tests/testStereoCamera.cpp # NEEDS UPDATE
    tests/testCameraParams.cpp
    tests/testCodesignIdeas.cpp
    tests/testExternalOdometryFrontend.cpp
    tests/testFrame.cpp # NEEDS UPDATE
    tests/testFrameCache.cpp
    tests/testGeneralParallelPlaneRegularBasicFactor.cpp
    tests/testGeneralParallelPlaneRegularTangentSpaceFactor.cpp
    tests/testImuFrontend.cpp
    tests/testImuParams.cpp
    # tests/testKittiDataProvider.cpp # TODO
    tests/testLoopClosureDetector.cpp
    tests/testLogger.cpp
    tests/testMesher.cpp # rotten
    tests/testMesh.cpp
    tests/testMeshUtils.cpp
    tests/testMeshOptimization.cpp
    tests/testMonoProvider.cpp
    tests/testOdomParams.cpp
    tests/testParallelMonoProvider.cpp
    tests/testParallelPlaneRegularBasicFactor.cpp
    tests/testParallelPlaneRegularTangentSpaceFactor.cpp
    tests/testParallelStereoProvider.cpp
    tests/testPointPlaneFactor.cpp
    #tests/testRegularVioBackend.cpp # rotten
    tests/testRegularVioBackendParams.cpp
    tests/testRgbdCamera.cpp
    tests/testRgbdFrame.cpp
    tests/testRgbdVisionImuFrontend.cpp
    tests/testStereoFrame.cpp # NEEDS UPDATE
    tests/testStereoMatcher.cpp
    tests/testStereoProvider.cpp
    tests/testStereoVisionImuFrontend.cpp # NEEDS UPDATE
    tests/testTemporalCalibration.cpp
    tests/testUndistortRectifier.cpp
    tests/testThreadsafeImuBuffer.cpp
    tests/testThreadsafeOdometryBuffer.cpp
    tests/testThreadsafeQueue.cpp
    tests/testThreadsafeTemporalBuffer.cpp
    tests/testTimer.cpp
    tests/testTracker.cpp # NEEDS UPDATE
    tests/testUtilsOpenCV.cpp
    tests/testUtilsNumerical.cpp
    tests/testInitializationFromImu.cpp
    tests/testVioBackend.cpp
    tests/testVioBackendParams.cpp
    tests/testVioParams.cpp
    tests/testVisionImuFrontendParams.cpp
    tests/testFeatureDetectorParams.cpp
    tests/testFeatureDetector.cpp
    tests/testVisualizer3D.cpp # NEEDS UPDATE
    tests/testOnlineAlignment.cpp
    tests/testOpticalFlowPredictor.cpp
  )
  target_include_directories(testKimeraVIO PUBLIC tests/include)
  target_link_libraries(testKimeraVIO gtest gmock kimera_vio::kimera_vio)

  include(GoogleTest)
  gtest_discover_tests(testKimeraVIO PRE_TEST)
endif(KIMERA_BUILD_TESTS)

############################### INSTALL/EXPORT #################################
## We install the export that we defined above
## Export the targets to a script
## This will install the import script kimera_vioTargets.cmake
## When findpackaged by other scripts, it will load the targets defined
## in the export kimera_vio-export.
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_vioConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# Create kimera_vioConfig.cmake with extra info from kimera_vioConfig.cmake.in
# This file is necessary to find_package the library kimera_vio.
set(INSTALL_CONFIGDIR lib/cmake/kimera_vio)
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/kimera_vioConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_vioConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

option(EXPORT_KIMERA "Export kimera_vio instead of installing it." OFF)
if(EXPORT_KIMERA)
  export(TARGETS ${PROJECT_NAME}
         FILE kimera_vioTargets.cmake)
       export(PACKAGE ${PROJECT_NAME})
else(EXPORT_KIMERA)
  include(GNUInstallDirs)
  ## First of all, we need to install the library itself.
  install(TARGETS ${PROJECT_NAME}
      EXPORT kimera_vio-export
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      # Perhaps use this, instead of installing include dir
      #PUBLIC_HEADER DESTINATION include
      INCLUDES DESTINATION include # We need this right?
      RUNTIME DESTINATION bin
  )
  install(EXPORT kimera_vio-export
    FILE
      kimera_vioTargets.cmake
    DESTINATION
      ${INSTALL_CONFIGDIR}
  )
  # Install header files
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h")

  ## Install the config and configversion
  install(FILES
      ${CMAKE_CURRENT_BINARY_DIR}/kimera_vioConfig.cmake
      ${CMAKE_CURRENT_BINARY_DIR}/kimera_vioConfigVersion.cmake
      ${CMAKE_CURRENT_LIST_DIR}/cmake/FindGflags.cmake
      ${CMAKE_CURRENT_LIST_DIR}/cmake/FindGlog.cmake
      DESTINATION ${INSTALL_CONFIGDIR}
  )
endif(EXPORT_KIMERA)

################################################################################
# Print configuration variables
message(STATUS "===============================================================")
message(STATUS "================  Configuration Options  ======================")
message(STATUS "CMAKE_CXX_COMPILER_ID type                : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMAKE_CXX_COMPILER_VERSION                : ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Build flags                                               ")
if(NOT MSVC AND NOT XCODE_VERSION)
  message(STATUS "  Build type                              : ${CMAKE_BUILD_TYPE}")
  if (NOT (${CMAKE_BUILD_TYPE} STREQUAL "Release" OR ${CMAKE_BUILD_TYPE} STREQUAL "RelWithDebInfo"))
    message(WARNING "  Build type is not in RELEASE, this will slow down the code.")
  endif()
  message(STATUS "  C compilation flags (Release)           : ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE}")
  message(STATUS "  C++ compilation flags (Release)         : ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
endif()
