### Add source code for stereoVIO
target_sources(KimeraVIO
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/DataSource-definitions.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/DataSource.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ETH_parser.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/KittiDataSource.cpp"
)
