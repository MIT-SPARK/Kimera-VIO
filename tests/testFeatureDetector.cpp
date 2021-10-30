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

/* ************************************************************************* */
TEST(FeatureDetector, FeatureDetectorNoNonMaxSuppression) {
  // parseYAML with detector params
  FeatureDetectorParams tp;
  tp.parseYAML(FLAGS_test_data_path + "/ForFeatureDetector/frontendParams-noNMS.yaml");

  // create a frame
  CameraParams cam_params;
  cam_params.parseYAML(FLAGS_test_data_path + "/sensor.yaml");
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName = string(FLAGS_test_data_path) + "/ForStereoFrame/left_fisheye_img_0.png";
  Frame::Ptr f = std::make_shared<Frame>(id, tmp, cam_params, UtilsOpenCV::ReadAndConvertToGrayScale(imgName));

  // perform feature detection
  FeatureDetector feature_detector(tp);
  feature_detector.featureDetection(f.get());

  // Compare results!
  EXPECT_EQ(f->keypoints_.size(), 393);
  // Note: while the yaml file specify that we want 300 features, since we do not apply non-max
  // suppression, we end up with more than 300 features
}

/* ************************************************************************* */
TEST(FeatureDetector, FeatureDetector_ANMS_TopN) {
  // parseYAML with detector params
  FeatureDetectorParams tp;
  tp.parseYAML(FLAGS_test_data_path + "/ForFeatureDetector/frontendParams-NMS-TopN.yaml");

  // create a frame
  CameraParams cam_params;
  cam_params.parseYAML(FLAGS_test_data_path + "/sensor.yaml");
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName = string(FLAGS_test_data_path) + "/ForStereoFrame/left_fisheye_img_0.png";
  Frame::Ptr f = std::make_shared<Frame>(id, tmp, cam_params, UtilsOpenCV::ReadAndConvertToGrayScale(imgName));

  // perform feature detection
  FeatureDetector feature_detector(tp);
  feature_detector.featureDetection(f.get());

  // Compare results: we get exactly as many keypoints as we asked for in the yaml
  EXPECT_EQ(f->keypoints_.size(), 300);
}

/* ************************************************************************* */
TEST(FeatureDetector, FeatureDetector_ANMS_Binning) {
  // parseYAML with detector params
  FeatureDetectorParams tp;
  tp.parseYAML(FLAGS_test_data_path + "/ForFeatureDetector/frontendParams-NMS-Binning.yaml");

  tp.print();

  // create a frame
  CameraParams cam_params;
  cam_params.parseYAML(FLAGS_test_data_path + "/sensor.yaml");
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName = string(FLAGS_test_data_path) + "/ForStereoFrame/left_fisheye_img_0.png";
  Frame::Ptr f = std::make_shared<Frame>(id, tmp, cam_params, UtilsOpenCV::ReadAndConvertToGrayScale(imgName));

  // perform feature detection
  FeatureDetector feature_detector(tp);
  feature_detector.featureDetection(f.get());

  // Compare results: we get exactly as many keypoints as we asked for in the yaml
  EXPECT_EQ(f->keypoints_.size(), 300);
}



}  // namespace VIO
