cmake_minimum_required (VERSION 2.6)
# set(CMAKE_CXX_FLAGS "-std=c++11") #  -stdlib=libc++
# set(CMAKE_CXX_FLAGS "-stdlib=libc++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project (VIO CXX C)

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
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
  message(STATUS "FOUND OpenCV! ${OpenCV_INCLUDE_DIRS}")
  message(STATUS "LIB_PATH = ${OpenCV_LIBS}")
else (OpenCV_FOUND)
  message(STATUS "CANNOT FOUND OpenCV!")
endif (OpenCV_FOUND)
include_directories(${OpenCV_INCLUDE_DIRS})
set (LIBS "${LIBS}" ${OpenCV_LIBS})

message(STATUS "ALL LIBS: ${LIBS}")

### Compile the code
include_directories("./src")

add_executable(stereoVIOEuroc ./examples/stereoVIOEuroc.cpp 
    ./src/ETH_parser.cpp ./src/StereoVisionFrontEnd.cpp  ./src/Tracker.cpp ./src/StereoFrame.cpp ./src/ImuFrontEnd.cpp ./src/VioBackEnd.cpp)
target_link_libraries (stereoVIOEuroc ${LIBS})

add_executable(stereoVIOSimulation ./examples/stereoVIOSimulation.cpp 
    ./src/ETH_parser.cpp ./src/StereoVisionFrontEnd.cpp  ./src/Tracker.cpp ./src/StereoFrame.cpp ./src/ImuFrontEnd.cpp ./src/VioBackEnd.cpp)
target_link_libraries (stereoVIOSimulation ${LIBS})

### Include the micro for adding test
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GtsamTesting.cmake)

### Get all the cpp files in ./src except the main file
set(VIO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(cpp_file_pattern "${VIO_SRC_DIR}/*.cpp")
file(GLOB src_cpp_files ${cpp_file_pattern})

#set(main_file_pattern "${VIO_SRC_DIR}/stereoVIOExample.cpp" "${VIO_SRC_DIR}/#stereoVIOExampleSimulation.cpp" "${VIO_SRC_DIR}/stereoVIOPlainBackend.cpp")
#file(GLOB main_files ${main_file_pattern})
#list(REMOVE_ITEM src_cpp_files ${main_files})
message(STATUS "src_cpp_files = ${src_cpp_files}")

### Add testing
add_subdirectory(tests)

### Add examples
# add_subdirectory(examples)
