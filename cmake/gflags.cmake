if (NOT __GFLAGS_INCLUDED) # guard against multiple includes
  set(__GFLAGS_INCLUDED TRUE)

  # Try the system-wide gflags first.
  # Note: System-wide gflags causes linker issues on current Ubuntu LTS
  #find_package(gflags QUIET)
  if (gflags_FOUND)
    message(STATUS "FOUND gflags!")
  else()
    message(STATUS "NOT FOUND gflags! Will be downloaded from github.")

    # gflags will use pthreads if it's available in the system, so we must link with it
    find_package(Threads)

    # build directory
    set(gflags_PREFIX ${CMAKE_BINARY_DIR}/external/gflags-prefix)
    # install directory
    set(gflags_INSTALL ${CMAKE_BINARY_DIR}/external/gflags-install)

    # we build gflags statically, but want to link it into the caffe shared library
    # this requires position-independent code
    if (UNIX)
        set(GFLAGS_EXTRA_COMPILER_FLAGS "-fPIC")
    endif()

    set(GFLAGS_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GFLAGS_EXTRA_COMPILER_FLAGS}")

    ExternalProject_Add(gflags
      PREFIX ${gflags_PREFIX}
      GIT_REPOSITORY "https://github.com/gflags/gflags.git"
      GIT_TAG "v2.2.2"
      UPDATE_COMMAND ""
      INSTALL_DIR ${gflags_INSTALL}
      CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                 -DCMAKE_INSTALL_PREFIX=${gflags_INSTALL}
                 -DBUILD_STATIC_LIBS=ON
                 -DGFLAGS_NAMESPACE=google
                 -DBUILD_PACKAGING=OFF
                 -DBUILD_TESTING=OFF
                 -DINSTALL_HEADERS=ON
                 -DCMAKE_CXX_FLAGS=${GFLAGS_CXX_FLAGS}
      LOG_DOWNLOAD 1
      LOG_INSTALL 1
      )

    set(gflags_FOUND TRUE)
    set(GFLAGS_INCLUDE_DIR ${gflags_INSTALL}/include)
    set(GFLAGS_LIBRARIES ${gflags_INSTALL}/lib/libgflags.a ${CMAKE_THREAD_LIBS_INIT})
    # HACK to avoid interface library gflags::gflags to complain that
    # INTERFACE_INCLUDE_DIRECTORIES does not exist the first time we run cmake before build.
    file(MAKE_DIRECTORY ${GFLAGS_INCLUDE_DIR})
  endif()

  if (NOT gflags_FOUND)
    message(FATAL_ERROR "Error: gflags not found.")
  else()
    # Create interface library to link against gflags.
    if(NOT TARGET gflags::gflags)
      add_library(gflags::gflags INTERFACE IMPORTED GLOBAL)
      set_target_properties(gflags::gflags PROPERTIES
        INTERFACE_LINK_LIBRARIES "${GFLAGS_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${GFLAGS_INCLUDE_DIR}")
      if(TARGET gflags)
        # This is to avoid sparkvio library to build before gflags
        add_dependencies(gflags::gflags gflags)
      endif()
    else()
      message(STATUS "Using system-wide gflags.")
      set(GFLAGS_SYSTEM_WIDE TRUE)
    endif()
  endif()

endif()
