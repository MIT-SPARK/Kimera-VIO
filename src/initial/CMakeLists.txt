### Add source code for stereoVIO
target_sources(sparkvio
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/InitializationBackEnd-definitions.h"
        "${CMAKE_CURRENT_LIST_DIR}/InitializationBackEnd.h"
        "${CMAKE_CURRENT_LIST_DIR}/InitializationBackEnd.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/OnlineGravityAlignment-definitions.h"
        "${CMAKE_CURRENT_LIST_DIR}/OnlineGravityAlignment.h"
	"${CMAKE_CURRENT_LIST_DIR}/OnlineGravityAlignment.cpp"
)
target_include_directories(sparkvio PRIVATE ${CMAKE_CURRENT_LIST_DIR})
