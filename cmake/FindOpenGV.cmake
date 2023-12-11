
# This is FindOPENGV.cmake
# CMake module to locate the OPENGV package
#
# The following cache variables may be set before calling this script:
#
# OPENGV_DIR (or OPENGV_ROOT): (Optional) The install prefix OR source tree of opengv (e.g. /usr/local or src/opengv)
# OPENGV_BUILD_NAME:          (Optional) If compiling against a source tree, the name of the build directory
#                            within it (e.g build-debug).  Without this defined, this script tries to
#                            intelligently find the build directory based on the project's build directory name
#                            or based on the build type (Debug/Release/etc).
#
# The following variables will be defined:
#
# OPENGV_FOUND          : TRUE if the package has been successfully found
# OPENGV_INCLUDE_DIR    : paths to OPENGV's INCLUDE directories
# OPENGV_LIBS           : paths to OPENGV's libraries
#
# NOTES on compiling against an uninstalled OPENGV build tree:
# - A OPENGV source tree will be automatically searched for in the directory
#   'opengv' next to your project directory, after searching
#   CMAKE_INSTALL_PREFIX and $HOME, but before searching /usr/local and /usr.
# - The build directory will be searched first with the same name as your
#   project's build directory, e.g. if you build from 'MyProject/build-optimized',
#   'opengv/build-optimized' will be searched first.  Next, a build directory for
#   your project's build type, e.g. if CMAKE_BUILD_TYPE in your project is
#   'Release', then 'opengv/build-release' will be searched next.  Finally, plain
#   'opengv/build' will be searched.
# - You can control the opengv build directory name directly by defining the CMake
#   cache variable 'OPENGV_BUILD_NAME', then only 'opengv/${OPENGV_BUILD_NAME} will
#   be searched.
# - Use the standard CMAKE_PREFIX_PATH, or OPENGV_DIR, to find a specific opengv
#   directory.

# Get path suffixes to help look for opengv
if(OPENGV_BUILD_NAME)
  set(opengv_build_names "${OPENGV_BUILD_NAME}/opengv")
else()
  # lowercase build type
  string(TOLOWER "${CMAKE_BUILD_TYPE}" build_type_suffix)
  # build suffix of this project
  get_filename_component(my_build_name "${CMAKE_BINARY_DIR}" NAME)
  
  set(opengv_build_names "${my_build_name}/opengv" "build-${build_type_suffix}/opengv" "build/opengv" "build/lib")
endif()

# Use OPENGV_ROOT or OPENGV_DIR equivalently
if(OPENGV_ROOT AND NOT OPENGV_DIR)
  set(OPENGV_DIR "${OPENGV_ROOT}")
endif()

if(OPENGV_DIR)
  # Find include dirs
  find_path(OPENGV_INCLUDE_DIR opengv/types.hpp
    PATHS "${OPENGV_DIR}/include" "${OPENGV_DIR}" NO_DEFAULT_PATH
    DOC "OPENGV include directories")

  # Find libraries
  find_library(OPENGV_LIBS NAMES opengv
    HINTS "${OPENGV_DIR}/lib" "${OPENGV_DIR}" NO_DEFAULT_PATH
    PATH_SUFFIXES ${opengv_build_names}
    DOC "OPENGV libraries")
else()
  # Find include dirs
  set(extra_include_paths ${CMAKE_INSTALL_PREFIX}/include "$ENV{HOME}/include" "${PROJECT_SOURCE_DIR}/../opengv" /usr/local/include /usr/include)
  find_path(OPENGV_INCLUDE_DIR opengv/types.hpp
    PATHS ${extra_include_paths}
    DOC "OPENGV include directories")
  if(NOT OPENGV_INCLUDE_DIR)
    message(STATUS "Searched for opengv headers in default paths plus ${extra_include_paths}")
  endif()

  # Find libraries
  find_library(OPENGV_LIBS NAMES opengv
    HINTS ${CMAKE_INSTALL_PREFIX}/lib "$ENV{HOME}/lib" "${PROJECT_SOURCE_DIR}/../opengv" /usr/local/lib /usr/lib
    PATH_SUFFIXES ${opengv_build_names}
    DOC "OPENGV libraries")
endif()

# handle the QUIETLY and REQUIRED arguments and set OPENGV_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OPENGV DEFAULT_MSG
                                  OPENGV_LIBS OPENGV_INCLUDE_DIR)
