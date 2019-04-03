# glog depends on gflags
include("cmake/gflags.cmake")

if (NOT __GLOG_INCLUDED)
  set(__GLOG_INCLUDED TRUE)

  # try the system-wide glog first
  find_package(glog QUIET)
  if (GLOG_FOUND)
      message("FOUND glog!")
      set(GLOG_EXTERNAL FALSE)
  else()
    message("Did not find glog! Building from github.")
    # fetch and build glog from github

    # build directory
    set(glog_PREFIX ${CMAKE_BINARY_DIR}/external/glog-prefix)
    # install directory
    set(glog_INSTALL ${CMAKE_BINARY_DIR}/external/glog-install)

    # we build glog statically, but want to link it into the caffe shared library
    # this requires position-independent code
    if (UNIX)
      set(GLOG_EXTRA_COMPILER_FLAGS "-fPIC")
    endif()

    set(GLOG_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GLOG_EXTRA_COMPILER_FLAGS}")
    set(GLOG_C_FLAGS "${CMAKE_C_FLAGS} ${GLOG_EXTRA_COMPILER_FLAGS}")

    # depend on gflags if we're also building it
    if (GFLAGS_EXTERNAL)
      set(GLOG_DEPENDS gflags)
    endif()

    ExternalProject_Add(glog
      DEPENDS ${GLOG_DEPENDS}
      PREFIX ${glog_PREFIX}
      GIT_REPOSITORY "https://github.com/google/glog"
      GIT_TAG "v0.3.4"
      UPDATE_COMMAND ""
      INSTALL_DIR ${glog_INSTALL}
      PATCH_COMMAND autoreconf -i ${glog_PREFIX}/src/glog
      CONFIGURE_COMMAND env "CFLAGS=${GLOG_C_FLAGS}" "CXXFLAGS=${GLOG_CXX_FLAGS}" ${glog_PREFIX}/src/glog/configure --prefix=${glog_INSTALL} --enable-shared=no --enable-static=yes --with-gflags=${GFLAGS_LIBRARY_DIRS}/..
      LOG_DOWNLOAD 1
      LOG_CONFIGURE 1
      LOG_INSTALL 1
      )

    set(GLOG_FOUND TRUE)
    set(GLOG_INCLUDE_DIRS ${glog_INSTALL}/include)
    set(GLOG_LIBRARIES ${glog_INSTALL}/lib/libglog.a)
    set(GLOG_LIBRARY_DIRS ${glog_INSTALL}/lib)
    set(GLOG_EXTERNAL TRUE)
  endif()


  if (NOT GLOG_FOUND)
    message("Error: glog not found.")
  else()
    # Create interface library to link against glog.
    if(NOT TARGET glog::glog)
      message("Creating glog::glog!")
      add_library(glog::glog INTERFACE IMPORTED GLOBAL)
      set_target_properties(glog::glog PROPERTIES
        INTERFACE_LINK_LIBRARIES "${GLOG_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIRS}")
    endif()
  endif()


endif()
