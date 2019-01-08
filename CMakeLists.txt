cmake_minimum_required (VERSION 3.1)
project (VIO CXX C)
# set(CMAKE_CXX_FLAGS "-std=c++11") #  -stdlib=libc++
# set(CMAKE_CXX_FLAGS "-stdlib=libc++")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O2 -Wall -pipe")

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

### Find CGAL (not REQUIRED)
message(STATUS "Setting CGAL on Luca's machine")
find_package(CGAL COMPONENTS Core)
set (CGAL_DIR /home/luca/Desktop/code/CGAL-4.11/build/)
if ( CGAL_FOUND )
  include( ${CGAL_USE_FILE} )
  include( CGAL_CreateSingleSourceCGALProgram )
  include_directories (BEFORE "../../include")
  set (LIBS "${LIBS}" ${CGAL_LIBS})
else()
    message(STATUS "CGAL not found - code will work, but flag USE_CGAL = false.")
endif()

### External Dependencies
include(ExternalProject)

### Include the gflags directory
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gflags.cmake)
list(APPEND VIO_INCLUDE_DIRS PUBLIC ${GFLAGS_INCLUDE_DIRS})
set(LIBS "${LIBS}" ${GFLAGS_LIBRARIES})

### Include the glog directory
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/glog.cmake)
list(APPEND VIO_INCLUDE_DIRS PUBLIC ${GLOG_INCLUDE_DIRS})
set(LIBS "${LIBS}" ${GLOG_LIBRARIES})

#include_directories($CGAL_DIR)
#find_package(CGAL REQUIRED COMPONENTS Core)
#include( ${CGAL_USE_FILE} )
#if (CGAL_FOUND)
#  message(STATUS "FOUND CGAL! ${CGAL_INCLUDE_DIRS}")
#else (CGAL_FOUND)
#  message(STATUS "CANNOT FOUND CGAL!")
#endif (CGAL_FOUND)
#include_directories(${CGAL_INCLUDE_DIRS})
#set (LIBS "${LIBS}" ${CGAL_LIBS})
#include_directories (BEFORE "../../include")
#message(STATUS BEFORE "../../include")
#include_directories (BEFORE "include")
#message(STATUS BEFORE "include")
#include( CGAL_CreateSingleSourceCGALProgram )

### Display all libs
message(STATUS "ALL LIBS: ${LIBS}")

### Compile the code
include_directories("./src")

add_library(stereoVIO SHARED ./src/ETH_parser.cpp ./src/StereoVisionFrontEnd.cpp
                      ./src/Tracker.cpp ./src/StereoFrame.cpp
                      ./src/UtilsGeometry.cpp ./src/UtilsOpenCV.cpp
                      ./src/ImuFrontEnd.cpp ./src/VioBackEnd.cpp
                      ./src/RegularVioBackEnd.cpp
                      ./src/Histogram.cpp ./src/LoggerMatlab.cpp
                      ./src/FeatureSelector.h
                      ./src/VioBackEndParams.h ./src/VioFrontEndParams.h
                      ./src/RegularVioBackEndParams.h
                      ./src/CameraParams.h ./src/Frame.h
                      ./src/Visualizer3D.h
                      ./src/Histogram.h
                      "")

### Add source code for factors
include(src/factors/CMakeLists.txt)
### Add source code for mesh
include(src/mesh/CMakeLists.txt)

if (DEFINED external_project_dependencies)
  add_dependencies(stereoVIO ${external_project_dependencies})
endif()
target_link_libraries(stereoVIO ${LIBS})
target_include_directories(stereoVIO ${VIO_INCLUDE_DIRS})

add_executable(stereoVIOEuroc ./examples/stereoVIOEuroc.cpp)
target_link_libraries (stereoVIOEuroc stereoVIO)

add_executable(stereoVIOSimulation ./examples/stereoVIOSimulation.cpp)
target_link_libraries (stereoVIOSimulation stereoVIO)

add_executable(regularFactorGraph ./examples/regularFactorGraphExample.cpp)
target_link_libraries(regularFactorGraph stereoVIO)

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

### Add examples
# add_subdirectory(examples)
