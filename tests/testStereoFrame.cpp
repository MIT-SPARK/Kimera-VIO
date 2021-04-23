/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testStereoFrame.h
 * @brief  test StereoFrame
 * @author Antoni Rosinol
 * @author Luca Carlone
 * @author Marcus Abate
 */

#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <gtsam/geometry/StereoCamera.h>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/frontend/VisionImuFrontendParams.h"
#include "kimera-vio/utils/Timer.h"

DECLARE_string(test_data_path);

using namespace gtsam;
using namespace std;
using namespace VIO;

// Data
static const double tol = 1e-7;
static const FrameId id = 0;
static const int64_t timestamp = 1;
static const string stereo_FLAGS_test_data_path(FLAGS_test_data_path +
                                                string("/ForStereoFrame/"));
static const string left_image_name = "left_img_0.png";
static const string right_image_name = "right_img_0.png";

void initializeData() {}

class StereoFrameFixture : public ::testing::Test {
 public:
  StereoFrameFixture() : cam_params_left(), cam_params_right(), tp() {
    cam_params_left.parseYAML(stereo_FLAGS_test_data_path + "/sensorLeft.yaml");
    cam_params_right.parseYAML(stereo_FLAGS_test_data_path +
                               "/sensorRight.yaml");

    // construct stereo camera
    sf = std::make_shared<StereoFrame>(
        id,
        timestamp,
        Frame(id,
              timestamp,
              cam_params_left,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  stereo_FLAGS_test_data_path + left_image_name,
                  tp.stereo_matching_params_.equalize_image_)),
        Frame(id,
              timestamp,
              cam_params_right,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  stereo_FLAGS_test_data_path + right_image_name,
                  tp.stereo_matching_params_.equalize_image_)));

    stereo_camera_ =
        std::make_shared<VIO::StereoCamera>(cam_params_left, cam_params_right);
    stereo_matcher_ = VIO::make_unique<StereoMatcher>(
        stereo_camera_, tp.stereo_matching_params_);

    // stereo_matcher_->sparseStereoReconstruction(sf.get());
    // sf->getLeftImgRectified().copyTo(left_image_rectified);
    // sf->getRightImgRectified().copyTo(right_image_rectified);
    P1 = stereo_camera_->getP1();
    P2 = stereo_camera_->getP2();

    initializeDataStereo();
  }

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  // Helper function
  void initializeDataStereo() {
    cam_params_left.parseYAML(stereo_FLAGS_test_data_path + 
                              "/sensorLeft.yaml");
    cam_params_right.parseYAML(stereo_FLAGS_test_data_path +
                               "/sensorRight.yaml");

    // construct stereo camera
    FrontendParams tp;
    sfnew = std::make_shared<StereoFrame>(
        id,
        timestamp,
        Frame(id,
              timestamp,
              cam_params_left,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  stereo_FLAGS_test_data_path + left_image_name,
                  tp.stereo_matching_params_.equalize_image_)),
        Frame(id,
              timestamp,
              cam_params_right,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  stereo_FLAGS_test_data_path + right_image_name,
                  tp.stereo_matching_params_.equalize_image_)));

    Frame* left_frame = &sfnew->left_frame_;
    UtilsOpenCV::ExtractCorners(left_frame->img_, &left_frame->keypoints_);
    left_frame->versors_.reserve(sfnew->left_frame_.keypoints_.size());
    int landmark_count_ = 0;
    for (size_t i = 0; i < sfnew->left_frame_.keypoints_.size(); i++) {
      sfnew->left_frame_.landmarks_.push_back(landmark_count_);
      sfnew->left_frame_.landmarks_age_.push_back(
          5 * landmark_count_);  // seen in a single (key)frame
      sfnew->left_frame_.scores_.push_back(10 * landmark_count_);
      sfnew->left_frame_.versors_.push_back(
          UndistorterRectifier::UndistortKeypointAndGetVersor(
              sfnew->left_frame_.keypoints_.at(i),
              sfnew->left_frame_.cam_param_));
      ++landmark_count_;
    }

    // do sparse stereo
    stereo_matcher_->sparseStereoReconstruction(sfnew.get());
  }

  FrontendParams tp;
  CameraParams cam_params_left;
  CameraParams cam_params_right;
  VIO::StereoCamera::ConstPtr stereo_camera_;
  VIO::StereoMatcher::UniquePtr stereo_matcher_;
  StereoFrame::Ptr sf;
  StereoFrame::Ptr sfnew;
  cv::Mat left_image_rectified, right_image_rectified;
  cv::Mat P1, P2;
};

static cv::Mat cvTranslateImageX(cv::Mat img, double dist) {
  cv::Mat result = cv::Mat(img.rows, img.cols, img.type());
  cv::Mat translation_mat = cv::Mat::eye(3, 3, CV_64F);
  translation_mat.at<double>(0, 2) = dist;
  cv::warpPerspective(
      img, result, translation_mat, img.size(), cv::INTER_NEAREST);
  return result;
}

TEST_F(StereoFrameFixture, setIsKeyframe) {
  // all false by default
  EXPECT_TRUE(!sf->isKeyframe());
  EXPECT_TRUE(!sf->left_frame_.isKeyframe_);
  EXPECT_TRUE(!sf->right_frame_.isKeyframe_);
  // Check that are correctly changed to true.
  sf->setIsKeyframe(true);
  EXPECT_TRUE(sf->isKeyframe());
  EXPECT_TRUE(sf->left_frame_.isKeyframe_);
  EXPECT_TRUE(sf->right_frame_.isKeyframe_);
}

/* ************************************************************************* */
// TODO(marcus): why is this here? not a funtion in StereoFrame...
// TEST_F(StereoFrameFixture, matchTemplate) {
//   cv::Mat left_img = sf->left_frame_.img_;
//   // Getting right (rectified) images by translating left image
//   double offset = 10;
//   cv::Mat right_img = cvTranslateImageX(left_img, (int)offset);

//   // parameters used in findMatchingKeypointRectified
//   // int templ_cols = 101, templ_rows = 11;
//   // int stripe_cols = left_img.cols;

//   int templ_cols = 31, templ_rows = 11;
//   int stripe_cols = 61;
//   int stripe_rows = templ_rows + 4;

//   // template
//   int temp_corner_x = 100;
//   int temp_corner_y = 100;
//   cv::Rect templ_selector =
//       cv::Rect(temp_corner_x, temp_corner_y, templ_cols, templ_rows);
//   cv::Mat templ(left_img, templ_selector);

//   // stripe
//   int stripe_corner_x = 100;
//   int stripe_corner_y = 100;
//   cv::Rect stripe_selector =
//       cv::Rect(stripe_corner_x, stripe_corner_y, stripe_cols, stripe_rows);
//   cv::Mat stripe(right_img, stripe_selector);

//   size_t nrTests = 100;
//   ///////////////////////////////////////////////////////////////////////////////////////////////
//   auto tic = utils::Timer::tic();
//   cv::Mat result1;
//   // cout << "templ \n" << templ << endl;
//   // cout << "stripe \n" << stripe << endl;
//   for (size_t i = 0; i < nrTests; i++) {
//     matchTemplate(stripe, templ, result1, CV_TM_SQDIFF_NORMED);
//   }
//   double timeMatching1 = utils::Timer::toc<std::chrono::seconds>(tic).count();
//   std::cout << "timeMatching 1: " << timeMatching1 / double(nrTests) << endl;

//   ///////////////////////////////////////////////////////////////////////////////////////////////
//   tic = utils::Timer::tic();
//   cv::Mat result2;
//   templ = cv::Mat(left_img, templ_selector);
//   stripe = cv::Mat(right_img, stripe_selector);

//   for (size_t t = 0; t < nrTests; t++) {
//     UtilsOpenCV::PlainMatchTemplate(stripe, templ, result2);
//   }
//   double timeMatching2 = utils::Timer::toc<std::chrono::seconds>(tic).count();
//   std::cout << "timeMatching 2: " << timeMatching2 / double(nrTests) << endl;

//   // check that results are the same
//   // cout << result1 << "\n" << result2 << endl;
//   EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(result2, result1, 1e-3));
// }
