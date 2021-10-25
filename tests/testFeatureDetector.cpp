/**
 * @file   testFeatureDetector.h
 * @brief  test FeatureDetector
 * @author Luca Carlone
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector-definitions.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetectorParams.h"

DECLARE_string(test_data_path);

using namespace std;
using namespace VIO;

namespace VIO {

static const double tol = 1e-7;

TEST(FeatureDetector, FeatureDetector) {
  // check that YAML is parsed correctly

  // parseYAML with detector params
  FeatureDetectorParams tp;
  tp.parseYAML(FLAGS_test_data_path + "/ForFeatureDetector/frontendParams-noNMS.yaml");

  tp.print();

  // create a frame
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName = string(FLAGS_test_data_path) + "/chessboard.png";
  Frame f(id, tmp, CameraParams(), UtilsOpenCV::ReadAndConvertToGrayScale(imgName));

  cv::imshow("img_", f.img_);
  cv::waitKey(0);

  // perform feature detection
  FeatureDetector feature_detector(tp);
  feature_detector.featureDetection(&f);

//  SubPixelCornerFinderParams expected_subpixel_params;
//  expected_subpixel_params.term_criteria_.type =
//      cv::TermCriteria::EPS + cv::TermCriteria::COUNT;
//  expected_subpixel_params.term_criteria_.maxCount = 42;
//  expected_subpixel_params.term_criteria_.epsilon = 0.201;
//  expected_subpixel_params.window_size_ = cv::Size(12, 12);
//  expected_subpixel_params.zero_zone_ = cv::Size(2, 2);
//
//  // Compare results!
//  EXPECT_EQ(VIO::to_underlying(tp.feature_detector_type_), 0);
//  EXPECT_EQ(tp.max_features_per_frame_, 200);
//  EXPECT_EQ(tp.enable_subpixel_corner_refinement_, true);
//  EXPECT_TRUE(tp.subpixel_corner_finder_params_.equals(expected_subpixel_params));
//  EXPECT_EQ(tp.enable_non_max_suppression_, true);
//  EXPECT_EQ(VIO::to_underlying(tp.non_max_suppression_type_), 4);
//  EXPECT_EQ(tp.quality_level_, 0.5);
//  EXPECT_EQ(tp.block_size_, 3);
//  EXPECT_EQ(tp.use_harris_corner_detector_, 0);
//  EXPECT_EQ(tp.k_, 0.04);
//  EXPECT_EQ(tp.fast_thresh_, 52);
}


}  // namespace VIO
