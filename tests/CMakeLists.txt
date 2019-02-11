# exclude certain files
# note the source dir on each
set(tests_exclude "testMesher.cpp" "testRegularVioBackEnd.cpp")
# Extra tests that do not pass if we link eigen as PRIVATE to sparkvio
# and link eigen directly to executables.
set(tests_exclude ${tests_exclude} "testFeatureSelector.cpp" "testTracker.cpp")

set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
configure_file(test_config.h.in test_config.h)
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# Build tests
gtsamAddTestsGlob(VIOTests "test*.cpp" "${tests_exclude}" SparkVio::sparkvio)
