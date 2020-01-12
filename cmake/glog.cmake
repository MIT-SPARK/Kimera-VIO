# glog depends on gflags
include("cmake/gflags.cmake")

if (NOT __GLOG_INCLUDED)
  set(__GLOG_INCLUDED TRUE)

  # Try the system-wide glog first.
  # Note: Glog < 0.3.5 doesn't include CMake support, and system-wide glog causes linker issues on current Ubuntu LTS
  #find_package(glog QUIET)
  if (glog_FOUND)
    message(STATUS "FOUND glog!")
  else()
    # Fetch and build glog from github
    message(STATUS "NOT FOUND glog! Will be downloaded from github.")

    # build directory
    set(GLOG_PREFIX ${CMAKE_BINARY_DIR}/external/glog-prefix)
    # install directory
    set(GLOG_INSTALL ${CMAKE_BINARY_DIR}/external/glog-install)

    # we build glog statically, but want to link it into the caffe shared library
    # this requires position-independent code
    if (UNIX)
      set(GLOG_EXTRA_COMPILER_FLAGS "-fPIC")
    endif()

    set(GLOG_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GLOG_EXTRA_COMPILER_FLAGS}")
    set(GLOG_C_FLAGS "${CMAKE_C_FLAGS} ${GLOG_EXTRA_COMPILER_FLAGS}")

    # depend on gflags if we're also building it
    if (gflags_FOUND)
      set(GLOG_DEPENDS gflags::gflags)
    endif()

    ExternalProject_Add(glog
      DEPENDS ${GLOG_DEPENDS}
      PREFIX ${GLOG_PREFIX}
      GIT_REPOSITORY "https://github.com/google/glog"
      GIT_TAG "v0.4.0"
      UPDATE_COMMAND ""
      INSTALL_DIR ${GLOG_INSTALL}
      CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                 -DCMAKE_INSTALL_PREFIX=${GLOG_INSTALL}
                 -DBUILD_SHARED_LIBS=OFF
                 -DBUILD_TESTING=OFF
                 -DCMAKE_C_FLAGS=${GLOG_C_FLAGS}
                 -DCMAKE_CXX_FLAGS=${GLOG_CXX_FLAGS}
      LOG_DOWNLOAD 1
      LOG_INSTALL 1
      )

    set(glog_FOUND TRUE)
    set(GLOG_INCLUDE_DIR ${GLOG_INSTALL}/include)
    set(GLOG_LIBRARIES ${GLOG_INSTALL}/lib/libglog.a)
    set(GLOG_LIBRARY_DIRS ${GLOG_INSTALL}/lib)
    # HACK to avoid interface library glog::glog to complain that
    # INTERFACE_INCLUDE_DIRECTORIES does not exist the first time we run cmake before build.
    file(MAKE_DIRECTORY ${GLOG_INCLUDE_DIR})
  endif()

  if (NOT glog_FOUND)
    message(FATAL_ERROR "Error: glog not found.")
  else()
    # Create interface library to link against glog.
    if(NOT TARGET glog::glog)
      message(STATUS "Create glog::glog.")
      add_library(glog::glog INTERFACE IMPORTED GLOBAL)
      set_target_properties(glog::glog PROPERTIES
        INTERFACE_LINK_LIBRARIES "${GLOG_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIR}")
      if(TARGET glog)
        add_dependencies(glog::glog glog)
      endif()
    else()
      message(STATUS "Using system-wide glog.")
      set(GLOG_SYSTEM_WIDE TRUE)
    endif()
  endif()
endif()
