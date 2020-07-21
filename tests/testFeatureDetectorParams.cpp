/**
 * @file   testFeatureDetectorParams.h
 * @brief  test FeatureDetectorParams
 * @author Antoni Rosinol
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector-definitions.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetectorParams.h"

DECLARE_string(test_data_path);
DECLARE_bool(fast);
DECLARE_bool(faster);

namespace VIO {

static const double tol = 1e-7;

TEST(testFeatureDetectorParams, FeatureDetectorParamParseYAML) {
  // check that YAML is parsed correctly

  // Test parseYAML
  FeatureDetectorParams tp;
  tp.parseYAML(FLAGS_test_data_path + "/ForTracker/trackerParameters.yaml");

  SubPixelCornerFinderParams expected_subpixel_params;
  expected_subpixel_params.term_criteria_.type =
      CV_TERMCRIT_EPS + CV_TERMCRIT_ITER;
  expected_subpixel_params.term_criteria_.maxCount = 42;
  expected_subpixel_params.term_criteria_.epsilon = 0.201;
  expected_subpixel_params.window_size_ = cv::Size(12, 12);
  expected_subpixel_params.zero_zone_ = cv::Size(2, 2);

  // Compare results!
  EXPECT_EQ(VIO::to_underlying(tp.feature_detector_type_), 0);
  EXPECT_EQ(tp.max_features_per_frame_, 200);
  EXPECT_EQ(tp.enable_subpixel_corner_refinement_, true);
  EXPECT_TRUE(
      tp.subpixel_corner_finder_params_.equals(expected_subpixel_params));
  EXPECT_EQ(tp.enable_non_max_suppression_, true);
  EXPECT_EQ(VIO::to_underlying(tp.non_max_suppression_type_), 4);
  EXPECT_EQ(tp.quality_level_, 0.5);
  EXPECT_EQ(tp.block_size_, 3);
  EXPECT_EQ(tp.use_harris_corner_detector_, 0);
  EXPECT_EQ(tp.k_, 0.04);
  EXPECT_EQ(tp.fast_thresh_, 52);
}

TEST(testFeatureDetectorParams, equals) {
  FeatureDetectorParams tp = FeatureDetectorParams();
  EXPECT_TRUE(tp.equals(tp));
}

TEST(testFeatureDetectorParams, FastAndFasterModifications) {
  // check that fast and faster appropriately adjust the number of features
  int default_value = 0;
  {  // default parameter closure
    FeatureDetectorParams tp;
    tp.parseYAML(FLAGS_test_data_path + "/ForTracker/trackerParameters.yaml");
    default_value = tp.max_features_per_frame_;
  }

  int fast_value = 0;
  {  // fast parameter closure
    google::FlagSaver saver;
    FLAGS_fast = true;
    FeatureDetectorParams tp;
    tp.parseYAML(FLAGS_test_data_path + "/ForTracker/trackerParameters.yaml");
    fast_value = tp.max_features_per_frame_;
  }

  int faster_value = 0;
  {  // faster parameter closure
    google::FlagSaver saver;
    FLAGS_faster = true;
    FeatureDetectorParams tp;
    tp.parseYAML(FLAGS_test_data_path + "/ForTracker/trackerParameters.yaml");
    faster_value = tp.max_features_per_frame_;
  }
  EXPECT_NE(default_value, 0);
  EXPECT_NE(fast_value, 0);
  EXPECT_NE(faster_value, 0);
  EXPECT_LE(fast_value, default_value);
  EXPECT_LE(faster_value, fast_value);
  double fast_to_default_ratio =
      fast_value / static_cast<double>(default_value);
  double faster_to_fast_ratio = faster_value / static_cast<double>(fast_value);
  EXPECT_NEAR(fast_to_default_ratio, faster_to_fast_ratio, 1.0e-2);
}

TEST(testFeatureDetectorParams, FastAndFasterConflict) {
  // check that fast and faster together results in faster configuration
  int faster_value = 0;
  {  // fast parameter closure
    google::FlagSaver saver;
    FLAGS_faster = true;
    FeatureDetectorParams tp;
    tp.parseYAML(FLAGS_test_data_path + "/ForTracker/trackerParameters.yaml");
    faster_value = tp.max_features_per_frame_;
  }

  int combined_value = 0;
  {  // faster parameter closure
    google::FlagSaver saver;
    FLAGS_fast = true;
    FLAGS_faster = true;
    FeatureDetectorParams tp;
    tp.parseYAML(FLAGS_test_data_path + "/ForTracker/trackerParameters.yaml");
    combined_value = tp.max_features_per_frame_;
  }
  EXPECT_EQ(faster_value, combined_value);
}

}  // namespace VIO
