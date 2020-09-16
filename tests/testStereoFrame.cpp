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
#include "kimera-vio/frontend/VisionFrontEndParams.h"
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
          Frame::calibratePixel(sfnew->left_frame_.keypoints_.at(i),
                                sfnew->left_frame_.cam_param_));
      ++landmark_count_;
    }

    // do sparse stereo
    stereo_matcher_->sparseStereoReconstruction(sfnew.get());
  }

  FrontendParams tp;
  CameraParams cam_params_left;
  CameraParams cam_params_right;
  VIO::StereoCamera::Ptr stereo_camera_;
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

// TODO(marcus): move this to the testStereoMatcher file.
// TEST_F(StereoFrameFixture, findMatchingKeypointRectified) {
//   // Synthetic experiments for findMatchingKeypointRectified

//   // Extract keypoints from the left img!
//   Frame* left_frame = &sf->left_frame_;
//   UtilsOpenCV::ExtractCorners(left_frame->img_, &left_frame->keypoints_);
//   cv::Mat left_img = sf->left_frame_.img_;
//   const KeypointsCV& left_keypoints = sf->left_frame_.keypoints_;
//   const int num_points = left_keypoints.size();

//   // we offset left image artificially (to get right image) in order to have
//   // ground truth for matching keypoint set of offsets we are going to test:
//   double offset_range_array[] = {-(double)left_img.cols / 2.0,
//                                  (double)-left_img.cols / 4.0,
//                                  -10.0,
//                                  0.0,
//                                  10.0,
//                                  (double)left_img.cols / 4.0,
//                                  (double)left_img.cols / 2.0};
//   vector<double> offset_range(offset_range_array, offset_range_array + 7);
//   const int num_offset_test = offset_range.size();

//   int countValid = 0;
//   int totalKeypointsTested = 0;
//   for (int i = 0; i < num_offset_test; i++) {
//     // Getting right (rectified) images by translating left image
//     double offset = offset_range[i];
//     cv::Mat right_img = cvTranslateImageX(left_img, (int)offset);

//     for (auto left_pt : left_keypoints) {  // for each left keypoint

//       totalKeypointsTested += 1;

//       double tol_corr;
//       for (int t = 0; t < 2; t++) {
//         // t = 0: test on keypoints in float coordinates, tolerate larger
//         // matching error t = 1: test on rounded keypoints, expect very small
//         // matching error
//         if (t == 1) {
//           left_pt.x = round(left_pt.x);
//           left_pt.y = round(left_pt.y);
//           tol_corr = 1e-6;
//         } else {
//           tol_corr = 5e-3;
//         }
//         StatusKeypointCV right_pt;
//         // parameters used in findMatchingKeypointRectified
//         int templ_cols = 101, templ_rows = 11;
//         int stripe_cols = left_img.cols;
//         int stripe_rows = templ_rows + 4;
//         double matchingVal_LR;
//         // actual keypoint
//         tie(right_pt, matchingVal_LR) =
//             sf->findMatchingKeypointRectified(left_img,
//                                               left_pt,
//                                               right_img,
//                                               templ_cols,
//                                               templ_rows,
//                                               stripe_cols,
//                                               stripe_rows,
//                                               tol_corr,
//                                               false);

//         // Judging the correctness of the matches.
//         double y_left = left_pt.y;
//         double x_exp = left_pt.x + offset;
//         double x_actual = right_pt.second.x;

//         // If left point is too close to boundary, we set right to be
//         // NO_RIGHT_RECT
//         if (y_left <= (stripe_rows - 1) / 2 ||
//             y_left + (stripe_rows - 1) / 2 >= left_img.rows) {
//           EXPECT_EQ(right_pt.first, KeypointStatus::NO_RIGHT_RECT);
//           EXPECT_DOUBLE_EQ(matchingVal_LR, -1);
//         } else if (x_exp >= (templ_cols - 1) / 2 &&
//                    x_exp + (templ_cols - 1) / 2 < left_img.cols) {
//           // The match should be marked valid!
//           EXPECT_EQ(right_pt.first, KeypointStatus::VALID);
//           EXPECT_LT(matchingVal_LR, tol_corr);
//           EXPECT_NEAR(x_exp, x_actual, 0.5);
//           EXPECT_NEAR(left_pt.y, right_pt.second.y, 0.5);
//         } else if (x_exp < 0 ||
//                    x_exp >= left_img.cols) {  // if true right match is invalid
//                                               // and left is outside image
//           // The match should be marked invalid!
//           EXPECT_EQ(right_pt.first, KeypointStatus::NO_RIGHT_RECT);
//           EXPECT_GE(matchingVal_LR, tol_corr);
//         } else {
//           // Corner case: The matching may be affected by the image border
//           EXPECT_TRUE(
//               (abs(x_exp - x_actual) < 0.5 && matchingVal_LR < tol_corr) ||
//               (abs(x_exp - x_actual) >= 0.5 && matchingVal_LR >= tol_corr));

//           countValid += 1;
//           if (!((abs(x_exp - x_actual) < 0.5 && matchingVal_LR < tol_corr) ||
//                 (abs(x_exp - x_actual) >= 0.5 && matchingVal_LR >= tol_corr))) {
//             cout << "IF WE GET TO THIS POINT, THE TEST FAILED - we visualize "
//                     "wrong matches:"
//                  << endl;
//             KeypointsCV left_corners, right_corners;
//             left_corners.push_back(left_pt);
//             right_corners.push_back(right_pt.second);
//             std::vector<cv::DMatch> matches;
//             matches.push_back(cv::DMatch(0, 0, 0));
//             cv::Mat canvas = UtilsOpenCV::DrawCornersMatches(
//                 left_img, left_corners, right_img, right_corners, matches);
//             // imshow("matched keypoints", canvas);
//             // waitKey(0);
//           }
//         }
//       }
//     }
//   }
//   // make sure that at least some keypoints were valid (these are the ones we
//   // are interested in)
//   EXPECT_EQ(82, countValid);
//   // 7 is the number of offsets we try
//   EXPECT_EQ(num_points * 7, totalKeypointsTested);
// }

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

/* ************************************************************************* */
// TODO(marcus): decide where this test should go
// TEST_F(StereoFrameFixture, getDepthFromRectifiedMatches) {
//   // depth = fx * b / disparity
//   // Synthetic keypoints depth set
//   std::vector<double> depth2baseline_ratio_set;
//   depth2baseline_ratio_set.push_back(0.5);
//   depth2baseline_ratio_set.push_back(1.0);
//   depth2baseline_ratio_set.push_back(2.0);
//   depth2baseline_ratio_set.push_back(3.0);
//   depth2baseline_ratio_set.push_back(5.0);
//   depth2baseline_ratio_set.push_back(10.0);
//   depth2baseline_ratio_set.push_back(15.0);
//   depth2baseline_ratio_set.push_back(20.0);
//   depth2baseline_ratio_set.push_back(30.0);
//   depth2baseline_ratio_set.push_back(50.0);
//   depth2baseline_ratio_set.push_back(100.0);

//   // synthetic keypoints location set: x coordinates
//   std::vector<double> x2depth_set;
//   x2depth_set.push_back(-0.2);
//   x2depth_set.push_back(0);
//   x2depth_set.push_back(0.2);

//   // synthetic keypoints location set: y coordinates
//   std::vector<double> y2depth_set;
//   y2depth_set.push_back(-0.2);
//   y2depth_set.push_back(0);
//   y2depth_set.push_back(0.2);

//   // synthesize left_rectified_keypoints, right_rectified_keypoints by
//   // projecting 3D point
//   StatusKeypointsCV left_keypoints_rectified, right_keypoints_rectified;
//   std::vector<double> depth_expected;

//   for (auto depth2baseline_ratio : depth2baseline_ratio_set) {
//     for (auto x2depth : x2depth_set) {
//       for (auto y2depth : y2depth_set) {
//         double depth = depth2baseline_ratio * stereo_camera_->getBaseline();
//         double x_loc = x2depth * depth;
//         double y_loc = y2depth * depth;

//         cv::Mat pt_mat_left = P1 * (cv::Mat_<double>(4, 1) << x_loc,
//                                     y_loc,
//                                     depth,
//                                     1);  // project to left
//         cv::Mat pt_mat_right = P2 * (cv::Mat_<double>(4, 1) << x_loc,
//                                      y_loc,
//                                      depth,
//                                      1);  // project to right
//         pt_mat_left =
//             pt_mat_left / pt_mat_left.at<double>(2, 0);  // express as pixels
//         pt_mat_right =
//             pt_mat_right / pt_mat_right.at<double>(2, 0);  // express as pixels
//         cv::Point2f pt_left(pt_mat_left.at<double>(0, 0),
//                             pt_mat_left.at<double>(1, 0));  // express as pixels
//         cv::Point2f pt_right(
//             pt_mat_right.at<double>(0, 0),
//             pt_mat_right.at<double>(1, 0));  // express as pixels
//         left_keypoints_rectified.push_back(
//             make_pair(KeypointStatus::VALID, pt_left));
//         right_keypoints_rectified.push_back(
//             make_pair(KeypointStatus::VALID, pt_right));
//         depth_expected.push_back(depth);
//       }
//     }
//   }

//   // Add a few invalid keypoints to the test.
//   left_keypoints_rectified.push_back(
//       make_pair(KeypointStatus::VALID, cv::Point2f(1.0, 2.0)));
//   right_keypoints_rectified.push_back(
//       make_pair(KeypointStatus::NO_RIGHT_RECT, cv::Point2f(1.0, 2.0)));
//   depth_expected.push_back(0);

//   left_keypoints_rectified.push_back(
//       make_pair(KeypointStatus::NO_LEFT_RECT, cv::Point2f(1.0, 2.0)));
//   right_keypoints_rectified.push_back(
//       make_pair(KeypointStatus::VALID, cv::Point2f(1.0, 2.0)));
//   depth_expected.push_back(0);

//   left_keypoints_rectified.push_back(
//       make_pair(KeypointStatus::NO_DEPTH, cv::Point2f(1.0, 2.0)));
//   right_keypoints_rectified.push_back(
//       make_pair(KeypointStatus::FAILED_ARUN, cv::Point2f(1.0, 2.0)));
//   depth_expected.push_back(0);

//   // Add a test case with negative disparity
//   left_keypoints_rectified.push_back(
//       make_pair(KeypointStatus::NO_DEPTH, cv::Point2f(3.0, 2.0)));
//   right_keypoints_rectified.push_back(
//       make_pair(KeypointStatus::FAILED_ARUN, cv::Point2f(1.0, 2.0)));
//   depth_expected.push_back(0);

//   // Call StereoFrame::getDepthFromRectifiedMatches to get the actual depth!
//   std::vector<double> depth_actual =
//       sf->getDepthFromRectifiedMatches(left_keypoints_rectified,
//                                        right_keypoints_rectified,
//                                        P1.at<double>(0, 0),
//                                        stereo_camera_->getBaseline(),
//                                        tp.stereo_matching_params_);
//   for (int i = 0; i < depth_actual.size(); i++) {
//     // if depth is outside the valid range, it is conventionally set to zero
//     if (depth_expected[i] < tp.stereo_matching_params_.min_point_dist_ ||
//         depth_expected[i] > tp.stereo_matching_params_.max_point_dist_) {
//       EXPECT_NEAR(depth_actual[i], 0, 1e-3);
//     } else {  // check value
//       EXPECT_NEAR(depth_expected[i], depth_actual[i], 1e-3);
//     }
//   }
// }

/* *************************************************************************
TEST_F(StereoFrameFixture, sparseStereoMatching_v2) {
  // this should be enabled if lines after 66 are uncommented
  // create a brand new stereo frame
  initializeDataStereo();

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // check that data is correctly populated:
  EXPECT_NEAR(0.110078, sfnew->baseline(), 1e-5);
  EXPECT_NEAR(100, sfnew->right_frame_.keypoints_.size(), 1e-5);
  EXPECT_NEAR(0, sfnew->right_frame_.scores_.size(), 1e-5);  //
  // scores do not get populated
  EXPECT_NEAR(0, sfnew->right_frame_.landmarks_.size(),
              1e-5);  // landmarks_ do not get populated
  EXPECT_NEAR(0, sfnew->right_frame_.landmarksAge_.size(),
              1e-5);  // landmarksAges do not get populated
  EXPECT_NEAR(0, sfnew->right_frame_.versors_.size(),
              1e-5);  // landmarksAges do not get populated
  EXPECT_NEAR(100, sfnew->keypoints_depth_.size(), 1e-5);
  EXPECT_NEAR(100, sfnew->keypoints_3d_.size(), 1e-5);
  EXPECT_NEAR(100, sfnew->right_keypoints_status_.size(), 1e-5);

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // check that 3d point is consistent with the left versor and the depth
  int nrValid = 0;
  for (size_t i = 0; i < sfnew->keypoints_3d_.size(); i++) {
    if (sfnew->right_keypoints_status_.at(i) == Kstatus::VALID) {
      nrValid += 1;
      double depth = sfnew->keypoints_depth_.at(i);
      gtsam::Vector3 versorActual = sfnew->keypoints_3d_.at(i) / depth;
      gtsam::Vector3 versorExpected = sfnew->left_frame_.versors_.at(i) /
                               sfnew->left_frame_.versors_.at(i)[2];
      EXPECT_TRUE(assert_equal(versorExpected, versorActual));
    } else {
      // when invalid 3D point is set to zero..
      gtsam::Vector3 pointActual = sfnew->keypoints_3d_.at(i);
      gtsam::Vector3 pointExpected = gtsam::Vector3::Zero();
      EXPECT_TRUE(assert_equal(pointExpected, pointActual));
      // and the corresponding depth is set to zero too
      EXPECT_NEAR(0.0, sfnew->keypoints_depth_.at(i), 1e-5);
    }
  }
  EXPECT_NEAR(93, nrValid, 1e-5);

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // check that 3D point reprojects correctly to the two cameras:
  for (size_t i = 0; i < sfnew->keypoints_3d_.size(); i++) {
    if (sfnew->right_keypoints_status_.at(i) == Kstatus::VALID) {
      // versor is wrt rectified frame
      gtsam::Vector3 versor_i_rect = sfnew->left_frame_.versors_.at(i);
      versor_i_rect = versor_i_rect / versor_i_rect(2);  // set last element to
      1, instead of norm 1

      // TEST: check rotation due to rectification (important when projecting
      points later on) gtsam::Rot3 expected_camL_R_camLrect =
      sfnew->left_frame_.cam_param_.body_Pose_cam_.rotation().between(sfnew->getBPoseCamLRect().rotation());
      // camL_R_camLrect gtsam::Rot3 actual_camL_R_camLrect =
      UtilsOpenCV::Cvmat2rot(sfnew->left_frame_.cam_param_.R_rectify_)
          .inverse();
      EXPECT_TRUE(
          assert_equal(expected_camL_R_camLrect, actual_camL_R_camLrect, 1e-6));

      // TEST: uncalibrateDistUnrect(versor) = original distorted unrectified
      point(CHECK DIST UNRECT CALIBRATION WORKS) KeypointCV kp_i_distUnrect =
          sfnew->left_frame_.keypoints_.at(i);
      // after stereo matching, versor will be in rectified frame, so to
      // compare with unrect measurements we have to compensate rectification
      gtsam::Vector3 versor_i_unRect = actual_camL_R_camLrect.matrix() *
versor_i_rect; versor_i_unRect = versor_i_unRect / versor_i_unRect(2); Point2
kp_i_distUnrect_gtsam =
          sfnew->left_frame_.cam_param_.calibration_.uncalibrate(
              Point2(versor_i_unRect(0), versor_i_unRect(1)));
      EXPECT_TRUE(assert_equal(Point2(kp_i_distUnrect.x, kp_i_distUnrect.y),
                               kp_i_distUnrect_gtsam, 1));

      // TEST: uncalibrateUndistRect(versor) = original distorted unrectified
      point(CHECK UNDIST RECT CALIBRATION WORKS) KeypointCV kp_i_undistRect =
          sfnew->left_keypoints_rectified_.at(i);
      Cal3_S2 KundistRect(sfnew->left_undistRectCameraMatrix_.fx(),
                          sfnew->left_undistRectCameraMatrix_.fy(),
                          sfnew->left_undistRectCameraMatrix_.skew(),
                          sfnew->left_undistRectCameraMatrix_.px(),
                          sfnew->left_undistRectCameraMatrix_.py());
      Point2 kp_i_undistRect_gtsam =
          KundistRect.uncalibrate(Point2(versor_i_rect(0), versor_i_rect(1)));
      EXPECT_TRUE(assert_equal(Point2(kp_i_undistRect.x, kp_i_undistRect.y),
                               kp_i_undistRect_gtsam, 1));

      // TEST: distortUnrectify(undistRectified) = original distorted
      unrectified point(CHECK UNDISTORTION WORKS) KeypointsCV dup;
      vector<Kstatus> statuses;
      StatusKeypointsCV urp;
      urp.push_back(make_pair(Kstatus::VALID, kp_i_undistRect));
      tie(dup, statuses) = StereoFrame::DistortUnrectifyPoints(
          urp, sfnew->left_frame_.cam_param_.undistRect_map_x_,
          sfnew->left_frame_.cam_param_.undistRect_map_y_);
      EXPECT_TRUE(assert_equal(Point2(kp_i_distUnrect.x, kp_i_distUnrect.y),
                               Point2(dup.at(0).x, dup.at(0).y), 1));

      // TEST: projecting 3d point to left camera (undist and rectified) =
      original undistorted rectified point(CHECK BACKPROJECTION WORKS)
          PinholeCamera<Cal3_S2>
              leftCam_undistRect(gtsam::Pose3(), KundistRect);
      Point3 point3d_rect = sfnew->keypoints_3d_.at(i);
      Point2 p2_undistRect = leftCam_undistRect.project(point3d_rect);
      EXPECT_TRUE(assert_equal(Point2(kp_i_undistRect.x, kp_i_undistRect.y),
                               p2_undistRect, 1));

      // TEST: projecting 3d point to left camera (distorted and unrectified)
      = original distorted unrectified point(CHECK BACKPROJECTION WORKS)
          Point3 point3d_unRect =
              actual_camL_R_camLrect.rotate(point3d_rect);  //
      compensate for the rotation induced by rectification Cal3DS2 KdistUnrect
      = sfnew->left_frame_.cam_param_.calibration_;
      PinholeCamera<Cal3DS2> leftCam_distUnrect(gtsam::Pose3(), KdistUnrect);
      Point2 p2_distUnrect = leftCam_distUnrect.project(point3d_unRect);
      EXPECT_TRUE(assert_equal(Point2(kp_i_distUnrect.x, kp_i_distUnrect.y),
                               p2_distUnrect, 1));

      // TEST: projecting 3d point to stereo camera
      // reproject to camera and check that matches corresponding rectified
      // TODO: don't explicitly call new, use make_shared
      pixels Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(
          sfnew->left_undistRectCameraMatrix_.fx(),
          sfnew->left_undistRectCameraMatrix_.fy(),
          sfnew->left_undistRectCameraMatrix_.skew(),
          sfnew->left_undistRectCameraMatrix_.px(),
          sfnew->left_undistRectCameraMatrix_.py(), sfnew->baseline_));
      // Note: camera pose is the identity (instead of
      sfnew->getBPoseCamLRect()) since the 3D point is in the left camera
      frame StereoCamera stereoCam = StereoCamera(gtsam::Pose3(),K);
      StereoPoint2 sp2 = stereoCam.project(point3d_rect);
      EXPECT_DOUBLE_EQ(sp2.uL(), sfnew->left_keypoints_rectified_.at(i).x, 1);
      EXPECT_DOUBLE_EQ(sp2.v(), sfnew->left_keypoints_rectified_.at(i).y, 1);
      EXPECT_DOUBLE_EQ(sp2.uR(), sfnew->right_keypoints_rectified_.at(i).x, 1);
      EXPECT_DOUBLE_EQ(sp2.v(), sfnew->right_keypoints_rectified_.at(i).y,
                       3);  // slightly larger errors
    }
  }
}
*/
