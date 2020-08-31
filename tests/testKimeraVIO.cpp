#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

DEFINE_string(test_data_path, "../tests/data", "Path to data for unit tests.");

DEFINE_bool(display, false, "Display test results.");

int main(int argc, char **argv) {
  // Initialize Google's testing library.
  ::testing::InitGoogleTest(&argc, argv);
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  return RUN_ALL_TESTS();
}
