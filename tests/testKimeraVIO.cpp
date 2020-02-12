#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

DEFINE_string(test_data_path, "../tests/data", "Path to data for unit tests.");

int main(int argc, char **argv) {
  // Initialize Google's testing library.
  ::testing::InitGoogleTest(&argc, argv);
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}
