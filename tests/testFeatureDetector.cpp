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
  tp.parseYAML(FLAGS_test_data_path +
               "/ForFeatureDetector/frontendParams-noNMS.yaml");

  // create a frame
  CameraParams cam_params;
  cam_params.parseYAML(FLAGS_test_data_path + "/sensor.yaml");
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName =
      string(FLAGS_test_data_path) + "/ForStereoFrame/left_fisheye_img_0.png";
  Frame::Ptr f = std::make_shared<Frame>(
      id, tmp, cam_params, UtilsOpenCV::ReadAndConvertToGrayScale(imgName));

  // perform feature detection
  FeatureDetector feature_detector(tp);
  feature_detector.featureDetection(f.get());

  // Compare results!
  EXPECT_EQ(f->keypoints_.size(), 393);
  // Note: while the yaml file specify that we want 300 features, since we do
  // not apply non-max suppression, we end up with more than 300 features
}

/* ************************************************************************* */
TEST(FeatureDetector, FeatureDetectorNoNonMaxSuppression2) {
  // parseYAML with detector params
  FeatureDetectorParams tp;
  tp.parseYAML(FLAGS_test_data_path +
               "/ForFeatureDetector/frontendParams-noNMS.yaml");
  tp.quality_level_ = 1e-10;  // we accept the max number of features

  //   create a frame
  CameraParams cam_params;
  cam_params.parseYAML(FLAGS_test_data_path + "/sensor.yaml");
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName =
      string(FLAGS_test_data_path) + "/ForStereoFrame/left_fisheye_img_0.png";
  Frame::Ptr f = std::make_shared<Frame>(
      id, tmp, cam_params, UtilsOpenCV::ReadAndConvertToGrayScale(imgName));

  // perform feature detection
  FeatureDetector feature_detector(tp);
  feature_detector.featureDetection(f.get());

  // Compare results!
  EXPECT_EQ(f->keypoints_.size(), 400);
  // Note: while the yaml file specify that we want 300 features, since we do
  // not apply non-max suppression, we end up with more than 300 features
  // (corresponding to max_nr_keypoints_before_anms = 400)
}

/* ************************************************************************* */
TEST(FeatureDetector, FeatureDetector_ANMS_TopN) {
  // parseYAML with detector params
  FeatureDetectorParams tp;
  tp.parseYAML(FLAGS_test_data_path +
               "/ForFeatureDetector/frontendParams-NMS-TopN.yaml");

  // create a frame
  CameraParams cam_params;
  cam_params.parseYAML(FLAGS_test_data_path + "/sensor.yaml");
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName =
      string(FLAGS_test_data_path) + "/ForStereoFrame/left_fisheye_img_0.png";
  Frame::Ptr f = std::make_shared<Frame>(
      id, tmp, cam_params, UtilsOpenCV::ReadAndConvertToGrayScale(imgName));

  // perform feature detection
  FeatureDetector feature_detector(tp);
  feature_detector.featureDetection(f.get());

  // Compare results: we get exactly as many keypoints as we asked for in the
  // yaml
  EXPECT_EQ(f->keypoints_.size(), 300);
}

/* ************************************************************************* */
TEST(FeatureDetector, FeatureDetector_ANMS_Binning) {
  // parseYAML with detector params
  FeatureDetectorParams tp;
  tp.parseYAML(FLAGS_test_data_path +
               "/ForFeatureDetector/frontendParams-NMS-Binning.yaml");

  // create a frame
  CameraParams cam_params;
  cam_params.parseYAML(FLAGS_test_data_path + "/sensor.yaml");
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName =
      string(FLAGS_test_data_path) + "/ForStereoFrame/left_fisheye_img_0.png";
  Frame::Ptr f = std::make_shared<Frame>(
      id, tmp, cam_params, UtilsOpenCV::ReadAndConvertToGrayScale(imgName));

  // perform feature detection
  FeatureDetector feature_detector(tp);
  feature_detector.featureDetection(f.get());

  // Compare results: since we specify features per bin, we might get a bit less
  // than desired (e.g. a big might have low response)
  EXPECT_EQ(f->keypoints_.size(), 20);  // 1 feature per bin

  // check that frequency of features per bin is correct
  float binRowSize = float(f->img_.rows) / float(tp.nr_vertical_bins_);
  float binColSize = float(f->img_.cols) / float(tp.nr_horizontal_bins_);

  gtsam::Matrix keypointPerBinCount = gtsam::Matrix::Zero(
      tp.nr_vertical_bins_,
      tp.nr_horizontal_bins_);  // grid of bins according to the yaml
  for (int i = 0; i < f->keypoints_.size(); i++) {
    const size_t binRowInd =
        static_cast<size_t>(f->keypoints_[i].y / binRowSize);
    const size_t binColInd =
        static_cast<size_t>(f->keypoints_[i].x / binColSize);
    keypointPerBinCount(binRowInd, binColInd) += 1;
  }
  // we expect 1 feature per bin
  CHECK(gtsam::assert_equal(
      keypointPerBinCount, gtsam::Matrix::Ones(5, 4), 1e-9));
}

/* ************************************************************************* */
TEST(FeatureDetector, FeatureDetector_ANMS_Binning2) {
  // parseYAML with detector params
  FeatureDetectorParams tp;
  tp.parseYAML(FLAGS_test_data_path +
               "/ForFeatureDetector/frontendParams-NMS-Binning.yaml");
  tp.max_features_per_frame_ =
      200;  // since we have 20 bins, we expect 4 kpts per bin
  tp.quality_level_ = 1e-10;
  tp.enable_subpixel_corner_refinement_ =
      0;  // otherwise the kpts might end up moving across bins

  // create a frame
  CameraParams cam_params;
  cam_params.parseYAML(FLAGS_test_data_path + "/sensor.yaml");
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName =
      string(FLAGS_test_data_path) + "/ForStereoFrame/left_fisheye_img_0.png";
  Frame::Ptr f = std::make_shared<Frame>(
      id, tmp, cam_params, UtilsOpenCV::ReadAndConvertToGrayScale(imgName));

  // perform feature detection
  FeatureDetector feature_detector(tp);
  feature_detector.featureDetection(f.get());

  // Compare results: since we specify features per bin, we might get a bit less
  // than desired (e.g. a big might have low response)
  EXPECT_EQ(f->keypoints_.size(), 200);  // we extract the number we specified

  // check that frequency of features per bin is correct
  float binRowSize = float(f->img_.rows) / float(tp.nr_vertical_bins_);
  float binColSize = float(f->img_.cols) / float(tp.nr_horizontal_bins_);

  gtsam::Matrix keypointPerBinCount = gtsam::Matrix::Zero(
      tp.nr_vertical_bins_,
      tp.nr_horizontal_bins_);  // grid of bins according to the yaml
  for (int i = 0; i < f->keypoints_.size(); i++) {
    const size_t binRowInd =
        static_cast<size_t>(f->keypoints_[i].y / binRowSize);
    const size_t binColInd =
        static_cast<size_t>(f->keypoints_[i].x / binColSize);
    keypointPerBinCount(binRowInd, binColInd) += 1;
  }

  // we expect 1 feature per bin
  CHECK(gtsam::assert_equal(
      keypointPerBinCount, 10 * gtsam::Matrix::Ones(5, 4), 1e-9));
}

/* ************************************************************************* */
TEST(FeatureDetector, FeatureDetector_ANMS_Binning3) {
  // parseYAML with detector params
  FeatureDetectorParams tp;
  tp.parseYAML(FLAGS_test_data_path +
               "/ForFeatureDetector/frontendParams-NMS-Binning2.yaml");
  tp.quality_level_ = 1e-10;
  tp.enable_subpixel_corner_refinement_ =
      0;  // otherwise the kpts might end up moving across bins

  // create a frame
  CameraParams cam_params;
  cam_params.parseYAML(FLAGS_test_data_path + "/sensor.yaml");
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName =
      string(FLAGS_test_data_path) + "/ForStereoFrame/left_fisheye_img_0.png";
  Frame::Ptr f = std::make_shared<Frame>(
      id, tmp, cam_params, UtilsOpenCV::ReadAndConvertToGrayScale(imgName));

  // perform feature detection
  FeatureDetector feature_detector(tp);
  feature_detector.featureDetection(f.get());

  // Compare results: since we specify features per bin, we might get a bit less
  // than desired (e.g. a big might have low response)
  EXPECT_EQ(f->keypoints_.size(), 140);  // we extract the number we specified

  // check that frequency of features per bin is correct
  float binRowSize = float(f->img_.rows) / float(tp.nr_vertical_bins_);
  float binColSize = float(f->img_.cols) / float(tp.nr_horizontal_bins_);

  gtsam::Matrix keypointPerBinCount = gtsam::Matrix::Zero(
      tp.nr_vertical_bins_,
      tp.nr_horizontal_bins_);  // grid of bins according to the yaml
  for (int i = 0; i < f->keypoints_.size(); i++) {
    const size_t binRowInd =
        static_cast<size_t>(f->keypoints_[i].y / binRowSize);
    const size_t binColInd =
        static_cast<size_t>(f->keypoints_[i].x / binColSize);
    keypointPerBinCount(binRowInd, binColInd) += 1;
  }

  double expecteNrFeaturesPerBin =
      140.0 / 14.0;  // 140 features over 14 valid bins

  Eigen::MatrixXd expectedBinCount =
      expecteNrFeaturesPerBin * Eigen::MatrixXd::Ones(5, 4);
  expectedBinCount(0, 0) = 0;
  expectedBinCount(0, 1) = 0;
  expectedBinCount(0, 2) = 0;
  expectedBinCount(0, 3) = 0;
  expectedBinCount(3, 0) = 0;
  expectedBinCount(3, 2) = 0;

  // we expect 1 feature per bin
  CHECK(gtsam::assert_equal(keypointPerBinCount, expectedBinCount, 1e-9));
}

}  // namespace VIO
