cmake_minimum_required(VERSION 2.8.2)

project(googletest-download NONE)

include(ExternalProject)
ExternalProject_Add(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.10.x # Last working version
    SOURCE_DIR "${CMAKE_BINARY_DIR}/external/googletest-src"
    BINARY_DIR "${CMAKE_BINARY_DIR}/external/googletest-build"
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    TEST_COMMAND ""
)
