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

    # we build gflags shared, this requires position-independent code
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
                 -DBUILD_gflags_LIB=ON
                 -DBUILD_gflags_nothreads_LIB=ON
                 -DBUILD_SHARED_LIBS=ON
                 -DINSTALL_SHARED_LIBS=ON
                 -DBUILD_STATIC_LIBS=OFF
                 -DINSTALL_STATIC_LIBS=OFF
                 -DBUILD_PACKAGING=OFF
                 -DBUILD_TESTING=OFF
                 -DINSTALL_HEADERS=ON
                 -DREGISTER_BUILD_DIR=OFF
                 # Register for glog to find gflags, otw send gflags lib dir
                 # This is because glog calls find_package(gflags) internally
                 -DREGISTER_INSTALL_PREFIX=ON
                 -DCMAKE_CXX_FLAGS=${GFLAGS_CXX_FLAGS}
                 -DGFLAGS_NAMESPACE=google
      LOG_DOWNLOAD 1
      LOG_INSTALL 1
      )

    set(gflags_FOUND TRUE)
    set(GFLAGS_INCLUDE_DIR ${gflags_INSTALL}/include)
    set(GFLAGS_LIBRARIES
      "${gflags_INSTALL}/lib/libgflags$<$<CONFIG:Debug>:_debug>.dylib"
      ${CMAKE_THREAD_LIBS_INIT})
    # HACK to avoid interface library gflags::gflags to complain that
    # INTERFACE_INCLUDE_DIRECTORIES does not exist the first time we run cmake before build.
    file(MAKE_DIRECTORY ${GFLAGS_INCLUDE_DIR})
  endif()

  if (NOT gflags_FOUND)
    message(FATAL_ERROR "Error: gflags not found.")
  else()
    # Create interface library to link against gflags.
    if(NOT TARGET gflags::gflags)
      message(STATUS "Create gflags::gflags.")
      add_library(gflags::gflags INTERFACE IMPORTED GLOBAL)
      set_target_properties(gflags::gflags PROPERTIES
        INTERFACE_LINK_LIBRARIES "${GFLAGS_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${GFLAGS_INCLUDE_DIR}")
      if(TARGET gflags)
        # This is to avoid KimeraVIO library to build before gflags
        add_dependencies(gflags::gflags gflags)
      endif()
    else()
      message(STATUS "Using system-wide gflags.")
      set(GFLAGS_SYSTEM_WIDE TRUE)
    endif()
  endif()

endif()
