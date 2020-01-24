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
 */

#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <gtsam/geometry/StereoCamera.h>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/VioFrontEndParams.h"

DECLARE_string(test_data_path);

using namespace gtsam;
using namespace std;
using namespace VIO;
using namespace cv;

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
  StereoFrameFixture() : cam_params_left(), cam_params_right() {
    cam_params_left.parseYAML(stereo_FLAGS_test_data_path + "/sensorLeft.yaml");
    cam_params_right.parseYAML(stereo_FLAGS_test_data_path +
                               "/sensorRight.yaml");

    // construct stereo camera
    VioFrontEndParams tp;  // only to get default stereo matching params
    sf = std::make_shared<StereoFrame>(
        id,
        timestamp,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            stereo_FLAGS_test_data_path + left_image_name,
            tp.stereo_matching_params_.equalize_image_),
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            stereo_FLAGS_test_data_path + right_image_name,
            tp.stereo_matching_params_.equalize_image_),
        cam_params_right,
        tp.stereo_matching_params_);

    CHECK(sf->isRectified());

    sf->left_img_rectified_.copyTo(left_image_rectified);
    sf->right_img_rectified_.copyTo(right_image_rectified);
    P1 = sf->getLeftFrame().cam_param_.P_;
    P2 = sf->getRightFrame().cam_param_.P_;

    initializeDataStereo();
  }

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  // Helper function
  void initializeDataStereo() {
    cam_params_left.parseYAML(stereo_FLAGS_test_data_path + "/sensorLeft.yaml");
    cam_params_right.parseYAML(stereo_FLAGS_test_data_path +
                               "/sensorRight.yaml");

    // construct stereo camera
    VioFrontEndParams tp;
    sfnew = std::make_shared<StereoFrame>(
        id,
        timestamp,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            stereo_FLAGS_test_data_path + left_image_name,
            tp.stereo_matching_params_.equalize_image_),
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            stereo_FLAGS_test_data_path + right_image_name,
            tp.stereo_matching_params_.equalize_image_),
        cam_params_right,
        tp.stereo_matching_params_);

    sfnew->getLeftFrameMutable()->extractCorners();
    sfnew->getLeftFrameMutable()->versors_.reserve(
        sfnew->getLeftFrame().keypoints_.size());
    int landmark_count_ = 0;
    for (size_t i = 0; i < sfnew->getLeftFrame().keypoints_.size(); i++) {
      sfnew->getLeftFrameMutable()->landmarks_.push_back(landmark_count_);
      sfnew->getLeftFrameMutable()->landmarksAge_.push_back(
          5 * landmark_count_);  // seen in a single (key)frame
      sfnew->getLeftFrameMutable()->scores_.push_back(10 * landmark_count_);
      sfnew->getLeftFrameMutable()->versors_.push_back(
          Frame::calibratePixel(sfnew->getLeftFrame().keypoints_.at(i),
                                sfnew->getLeftFrame().cam_param_));
      ++landmark_count_;
    }

    // do sparse stereo
    sfnew->sparseStereoMatching();
  }

  CameraParams cam_params_left;
  CameraParams cam_params_right;
  std::shared_ptr<StereoFrame> sf;
  std::shared_ptr<StereoFrame> sfnew;
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
  EXPECT_TRUE(!sf->getLeftFrame().isKeyframe_);
  EXPECT_TRUE(!sf->getRightFrame().isKeyframe_);
  // Check that are correctly changed to true.
  sf->setIsKeyframe(true);
  EXPECT_TRUE(sf->isKeyframe());
  EXPECT_TRUE(sf->getLeftFrame().isKeyframe_);
  EXPECT_TRUE(sf->getRightFrame().isKeyframe_);
}

TEST_F(StereoFrameFixture, rectification) {
  // Rectification is already called in initializeData(), to avoid calling it in
  // every function

  // Verify the correctness of the rectification matrix!
  CameraParams left_camera_info = sf->getLeftFrame().cam_param_;
  CameraParams right_camera_info = sf->getRightFrame().cam_param_;

  // THIS TEST IS REDUNDANT AS IT REPLICATES A SANITY CHECK ALREADY PRESENT IN
  // THE CODE Make sure that relative pose between camera AFTER rectification
  // satisfies: 1) no rotation 2) translation only along x axis = baseline
  // Compensate for the fact that opencv works on the inverse of rotations
  gtsam::Rot3 camL_Rot_camLrect =
      UtilsOpenCV::cvMatToGtsamRot3(left_camera_info.R_rectify_).inverse();
  gtsam::Pose3 camL_Pose_camLrect = gtsam::Pose3(camL_Rot_camLrect, Point3());

  // B_Pose_camLrect
  gtsam::Pose3 B_Pose_camLrect_expect =
      (left_camera_info.body_Pose_cam_).compose(camL_Pose_camLrect);
  sf->setIsRectified(true);  // Fake that the camera is rectified.
  EXPECT_TRUE(sf->getBPoseCamLRect().equals(B_Pose_camLrect_expect));

  // Right camera pose after rectification
  gtsam::Rot3 camR_Rot_camRrect =
      UtilsOpenCV::cvMatToGtsamRot3(right_camera_info.R_rectify_).inverse();
  gtsam::Pose3 camR_Pose_camRrect = gtsam::Pose3(camR_Rot_camRrect, Point3());

  // B_Pose_camRrect
  gtsam::Pose3 B_Pose_camRrect =
      right_camera_info.body_Pose_cam_.compose(camR_Pose_camRrect);
  // Relative pose
  sf->setIsRectified(true);  // Fake that the camera is rectified.
  gtsam::Pose3 camLrect_Pose_camRrect =
      sf->getBPoseCamLRect().between(B_Pose_camRrect);

  // Verify the quality of the rectification!
  // Baseline
  double baseline_expect =
      left_camera_info.body_Pose_cam_.between(right_camera_info.body_Pose_cam_)
          .translation()
          .norm();
  // Make sure that it is compatible with the baseline used in the test data
  EXPECT_TRUE(baseline_expect >= 0.10 && baseline_expect <= 0.12);
  EXPECT_NEAR(baseline_expect, sf->getBaseline(), 1e-5);
  // check condition 1) on Rotation
  double rotation_deviation =
      gtsam::Rot3::Logmap(camLrect_Pose_camRrect.rotation()).norm();
  EXPECT_LT(rotation_deviation, tol);
  // check condition 2) on Translation
  double y_deviation = camLrect_Pose_camRrect.translation().y();
  double z_deviation = camLrect_Pose_camRrect.translation().z();
  EXPECT_LT(y_deviation, tol);
  EXPECT_LT(z_deviation, tol);

  // Check that the intrinsics of P1 and P2 are exactly the same!
  EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(P1(cv::Rect(0, 0, 3, 3)),
                                                P2(cv::Rect(0, 0, 3, 3))));

  // Test projection using 3x4 (rectified) camera matrix
  Mat P2_T_expect = P1(cv::Rect(0, 0, 3, 3)) *
                    (cv::Mat_<double>(3, 1) << -baseline_expect, 0, 0);
  P2_T_expect = P2_T_expect.clone();
  Mat P2_T_actual = P2(cv::Rect(3, 0, 1, 3)).clone();
  EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(P2_T_expect, P2_T_actual));
}

TEST_F(StereoFrameFixture, cloneRectificationParameters) {
  // construct stereo camera
  VioFrontEndParams tp;
  StereoFrame* sf2 =
      new StereoFrame(id,
                      timestamp,
                      UtilsOpenCV::ReadAndConvertToGrayScale(
                          stereo_FLAGS_test_data_path + left_image_name,
                          tp.stereo_matching_params_.equalize_image_),
                      cam_params_left,
                      UtilsOpenCV::ReadAndConvertToGrayScale(
                          stereo_FLAGS_test_data_path + right_image_name,
                          tp.stereo_matching_params_.equalize_image_),
                      cam_params_right,
                      tp.stereo_matching_params_);
  // clone
  sf2->cloneRectificationParameters(*sf);
  // make sure everything was copied correctly
  EXPECT_TRUE(
      sf2->getLeftFrame().cam_param_.equals(sf->getLeftFrame().cam_param_));
  EXPECT_TRUE(
      sf2->getRightFrame().cam_param_.equals(sf->getRightFrame().cam_param_));
}

TEST_F(StereoFrameFixture, findMatchingKeypointRectified) {
  // Synthetic experiments for findMatchingKeypointRectified

  // Extract keypoints from the left img!
  sf->getLeftFrameMutable()->extractCorners();
  Mat left_img = sf->getLeftFrame().img_;
  const KeypointsCV& left_keypoints = sf->getLeftFrame().keypoints_;
  const int num_points = left_keypoints.size();

  // we offset left image artificially (to get right image) in order to have
  // ground truth for matching keypoint set of offsets we are going to test:
  double offset_range_array[] = {-(double)left_img.cols / 2.0,
                                 (double)-left_img.cols / 4.0,
                                 -10.0,
                                 0.0,
                                 10.0,
                                 (double)left_img.cols / 4.0,
                                 (double)left_img.cols / 2.0};
  vector<double> offset_range(offset_range_array, offset_range_array + 7);
  const int num_offset_test = offset_range.size();

  int countValid = 0;
  int totalKeypointsTested = 0;
  for (int i = 0; i < num_offset_test; i++) {
    // Getting right (rectified) images by translating left image
    double offset = offset_range[i];
    Mat right_img = cvTranslateImageX(left_img, (int)offset);

    for (auto left_pt : left_keypoints) {  // for each left keypoint

      totalKeypointsTested += 1;

      double tol_corr;
      for (int t = 0; t < 2; t++) {
        // t = 0: test on keypoints in float coordinates, tolerate larger
        // matching error t = 1: test on rounded keypoints, expect very small
        // matching error
        if (t == 1) {
          left_pt.x = round(left_pt.x);
          left_pt.y = round(left_pt.y);
          tol_corr = 1e-6;
        } else {
          tol_corr = 5e-3;
        }
        StatusKeypointCV right_pt;
        // parameters used in findMatchingKeypointRectified
        int templ_cols = 101, templ_rows = 11;
        int stripe_cols = left_img.cols;
        int stripe_rows = templ_rows + 4;
        double matchingVal_LR;
        // actual keypoint
        tie(right_pt, matchingVal_LR) =
            sf->findMatchingKeypointRectified(left_img,
                                              left_pt,
                                              right_img,
                                              templ_cols,
                                              templ_rows,
                                              stripe_cols,
                                              stripe_rows,
                                              tol_corr,
                                              false);

        // Judging the correctness of the matches.
        double y_left = left_pt.y;
        double x_exp = left_pt.x + offset;
        double x_actual = right_pt.second.x;

        // If left point is too close to boundary, we set right to be
        // NO_RIGHT_RECT
        if (y_left <= (stripe_rows - 1) / 2 ||
            y_left + (stripe_rows - 1) / 2 >= left_img.rows) {
          EXPECT_EQ(right_pt.first, KeypointStatus::NO_RIGHT_RECT);
          EXPECT_DOUBLE_EQ(matchingVal_LR, -1);
        } else if (x_exp >= (templ_cols - 1) / 2 &&
                   x_exp + (templ_cols - 1) / 2 < left_img.cols) {
          // The match should be marked valid!
          EXPECT_EQ(right_pt.first, KeypointStatus::VALID);
          EXPECT_LT(matchingVal_LR, tol_corr);
          EXPECT_NEAR(x_exp, x_actual, 0.5);
          EXPECT_NEAR(left_pt.y, right_pt.second.y, 0.5);
        } else if (x_exp < 0 ||
                   x_exp >= left_img.cols) {  // if true right match is invalid
                                              // and left is outside image
          // The match should be marked invalid!
          EXPECT_EQ(right_pt.first, KeypointStatus::NO_RIGHT_RECT);
          EXPECT_GE(matchingVal_LR, tol_corr);
        } else {
          // Corner case: The matching may be affected by the image border
          EXPECT_TRUE(
              (abs(x_exp - x_actual) < 0.5 && matchingVal_LR < tol_corr) ||
              (abs(x_exp - x_actual) >= 0.5 && matchingVal_LR >= tol_corr));

          countValid += 1;
          if (!((abs(x_exp - x_actual) < 0.5 && matchingVal_LR < tol_corr) ||
                (abs(x_exp - x_actual) >= 0.5 && matchingVal_LR >= tol_corr))) {
            cout << "IF WE GET TO THIS POINT, THE TEST FAILED - we visualize "
                    "wrong matches:"
                 << endl;
            KeypointsCV left_corners, right_corners;
            left_corners.push_back(left_pt);
            right_corners.push_back(right_pt.second);
            std::vector<cv::DMatch> matches;
            matches.push_back(cv::DMatch(0, 0, 0));
            Mat canvas = UtilsOpenCV::DrawCornersMatches(
                left_img, left_corners, right_img, right_corners, matches);
            // imshow("matched keypoints", canvas);
            // waitKey(0);
          }
        }
      }
    }
  }
  // make sure that at least some keypoints were valid (these are the ones we
  // are interested in)
  EXPECT_NEAR(84, countValid, 1e-5);
  // 7 is the number of offsets we try
  EXPECT_NEAR(num_points * 7, totalKeypointsTested, 1e-5);
}

/* ************************************************************************* */
TEST_F(StereoFrameFixture, matchTemplate) {
  Mat left_img = sf->getLeftFrame().img_;
  // Getting right (rectified) images by translating left image
  double offset = 10;
  Mat right_img = cvTranslateImageX(left_img, (int)offset);

  // parameters used in findMatchingKeypointRectified
  // int templ_cols = 101, templ_rows = 11;
  // int stripe_cols = left_img.cols;

  int templ_cols = 31, templ_rows = 11;
  int stripe_cols = 61;
  int stripe_rows = templ_rows + 4;

  // template
  int temp_corner_x = 100;
  int temp_corner_y = 100;
  Rect templ_selector =
      Rect(temp_corner_x, temp_corner_y, templ_cols, templ_rows);
  Mat templ(left_img, templ_selector);

  // stripe
  int stripe_corner_x = 100;
  int stripe_corner_y = 100;
  Rect stripe_selector =
      Rect(stripe_corner_x, stripe_corner_y, stripe_cols, stripe_rows);
  Mat stripe(right_img, stripe_selector);

  size_t nrTests = 100;
  ///////////////////////////////////////////////////////////////////////////////////////////////
  double timeBefore = UtilsOpenCV::GetTimeInSeconds();
  Mat result1;
  // cout << "templ \n" << templ << endl;
  // cout << "stripe \n" << stripe << endl;
  for (size_t i = 0; i < nrTests; i++) {
    matchTemplate(stripe, templ, result1, CV_TM_SQDIFF_NORMED);
  }
  double timeMatching1 = UtilsOpenCV::GetTimeInSeconds() - timeBefore;
  cout << "timeMatching 1: " << timeMatching1 / double(nrTests) << endl;

  ///////////////////////////////////////////////////////////////////////////////////////////////
  timeBefore = UtilsOpenCV::GetTimeInSeconds();
  Mat result2;
  templ = Mat(left_img, templ_selector);
  stripe = Mat(right_img, stripe_selector);

  for (size_t t = 0; t < nrTests; t++) {
    UtilsOpenCV::PlainMatchTemplate(stripe, templ, result2);
  }
  double timeMatching2 = UtilsOpenCV::GetTimeInSeconds() - timeBefore;
  cout << "timeMatching 2: " << timeMatching2 / double(nrTests) << endl;

  // check that results are the same
  // cout << result1 << "\n" << result2 << endl;
  EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(result2, result1, 1e-3));
}

/* ************************************************************************* */
TEST_F(StereoFrameFixture, DistortUnrectifyPoints) {
  // Prepare the input data, on a grid!
  const int numRows = 8;
  const int numCols = 10;
  StatusKeypointsCV keypoints_rectified;
  for (int r = 0; r < numRows; r++) {
    for (int c = 0; c < numCols; c++) {
      int y = left_image_rectified.rows / (numRows - 1) * r;
      int x = left_image_rectified.cols / (numCols - 1) * c;
      KeypointStatus st = KeypointStatus::VALID;
      if ((r + c) % 2 == 0) {  // set some of them to NO_RIGHT_RECT
        st = KeypointStatus::NO_RIGHT_RECT;
      }
      keypoints_rectified.push_back(make_pair(st, Point2f(x, y)));
    }
  }

  KeypointsCV keypoints_unrectified;
  std::vector<KeypointStatus> keypoints_status;

  tie(keypoints_unrectified, keypoints_status) =
      StereoFrame::distortUnrectifyPoints(
          keypoints_rectified,
          sf->getLeftFrame().cam_param_.undistRect_map_x_,
          sf->getLeftFrame().cam_param_.undistRect_map_y_);

  // Manually compute the expected distorted/unrectified keypoints!
  for (int i = 0; i < keypoints_unrectified.size(); i++) {  // for each keypoint

    // Status should be the same
    EXPECT_EQ(keypoints_status[i], keypoints_rectified[i].first);
    if (keypoints_status[i] !=
        KeypointStatus::VALID) {  // if status is not valid, keypoint is
                                  // conventionally
                                  // set to (0,0)
      EXPECT_TRUE(keypoints_unrectified[i].x == 0 &&
                  keypoints_unrectified[i].y == 0);
      continue;
    }

    // if it is VALID, compare actual pixel value against manually computed
    // expected value
    Mat pt = (cv::Mat_<double>(3, 1) << keypoints_rectified[i].second.x,
              keypoints_rectified[i].second.y,
              1);  // homogeneous pixel
    Mat P1_inv = P1(cv::Rect(0, 0, 3, 3)).inv();
    Mat R = sf->getLeftFrame().cam_param_.R_rectify_;
    Mat xn = R.t() * P1_inv * pt;
    Point2 pt_in = Point2(
        xn.at<double>(0, 0) / xn.at<double>(2, 0),
        xn.at<double>(1, 0) /
            xn.at<double>(
                2, 0));  // convert back to pixel (non-homogeneous coordinates)
    Point2 pt_expected =
        sf->getLeftFrame().cam_param_.distortion_->uncalibrate(pt_in);

    // actual value
    Point2 pt_actual =
        Point2(keypoints_unrectified[i].x, keypoints_unrectified[i].y);

    // Comparison!
    EXPECT_TRUE(assert_equal(pt_expected, pt_actual, 1e-3));
  }
}

/* ************************************************************************* */
TEST_F(StereoFrameFixture, undistortRectifyPoints) {
  // get image rows and cols
  const int img_rows = sf->getLeftFrame().img_.rows;
  const int img_cols = sf->getLeftFrame().img_.cols;

  // generate keypoints regularly spaced along a grid of size numRows x numRows
  KeypointsCV keypoints_unrectified_gt;
  const int numRows = 8;
  const int numCols = 10;
  for (int r = 0; r < numRows; r++) {
    for (int c = 0; c < numCols; c++) {
      int y = img_rows / (numRows - 1) * r;
      int x = img_cols / (numCols - 1) * c;
      keypoints_unrectified_gt.push_back(cv::Point2f(x, y));
    }
  }

  // Actual value
  StatusKeypointsCV keypoints_rectified;
  sf->undistortRectifyPoints(keypoints_unrectified_gt,
                             sf->getLeftFrame().cam_param_,
                             sf->getLeftUndistRectCamMat(),
                             &keypoints_rectified);

  // Map back!
  KeypointsCV keypoints_unrectified_actual;
  std::vector<KeypointStatus> status_unrectified;
  tie(keypoints_unrectified_actual, status_unrectified) =
      StereoFrame::distortUnrectifyPoints(
          keypoints_rectified,
          sf->getLeftFrame().cam_param_.undistRect_map_x_,
          sf->getLeftFrame().cam_param_.undistRect_map_y_);

  // Comparision
  for (int i = 0; i < keypoints_unrectified_actual.size(); i++) {
    if (keypoints_rectified[i].first != KeypointStatus::VALID) continue;
    // compare pixel coordinates of valid points
    EXPECT_NEAR(
        keypoints_unrectified_actual[i].x, keypoints_unrectified_gt[i].x, 1);
    EXPECT_NEAR(
        keypoints_unrectified_actual[i].y, keypoints_unrectified_gt[i].y, 1);
  }
}

/* ************************************************************************* */
TEST_F(StereoFrameFixture, getDepthFromRectifiedMatches) {
  // depth = fx * b / disparity
  // Synthetic keypoints depth set
  std::vector<double> depth2baseline_ratio_set;
  depth2baseline_ratio_set.push_back(0.5);
  depth2baseline_ratio_set.push_back(1.0);
  depth2baseline_ratio_set.push_back(2.0);
  depth2baseline_ratio_set.push_back(3.0);
  depth2baseline_ratio_set.push_back(5.0);
  depth2baseline_ratio_set.push_back(10.0);
  depth2baseline_ratio_set.push_back(15.0);
  depth2baseline_ratio_set.push_back(20.0);
  depth2baseline_ratio_set.push_back(30.0);
  depth2baseline_ratio_set.push_back(50.0);
  depth2baseline_ratio_set.push_back(100.0);

  // synthetic keypoints location set: x coordinates
  std::vector<double> x2depth_set;
  x2depth_set.push_back(-0.2);
  x2depth_set.push_back(0);
  x2depth_set.push_back(0.2);

  // synthetic keypoints location set: y coordinates
  std::vector<double> y2depth_set;
  y2depth_set.push_back(-0.2);
  y2depth_set.push_back(0);
  y2depth_set.push_back(0.2);

  // synthesize left_rectified_keypoints, right_rectified_keypoints by
  // projecting 3D point
  StatusKeypointsCV left_keypoints_rectified, right_keypoints_rectified;
  std::vector<double> depth_expected;

  for (auto depth2baseline_ratio : depth2baseline_ratio_set) {
    for (auto x2depth : x2depth_set) {
      for (auto y2depth : y2depth_set) {
        double depth = depth2baseline_ratio * sf->getBaseline();
        double x_loc = x2depth * depth;
        double y_loc = y2depth * depth;

        Mat pt_mat_left = P1 * (cv::Mat_<double>(4, 1) << x_loc,
                                y_loc,
                                depth,
                                1);  // project to left
        Mat pt_mat_right = P2 * (cv::Mat_<double>(4, 1) << x_loc,
                                 y_loc,
                                 depth,
                                 1);  // project to right
        pt_mat_left =
            pt_mat_left / pt_mat_left.at<double>(2, 0);  // express as pixels
        pt_mat_right =
            pt_mat_right / pt_mat_right.at<double>(2, 0);  // express as pixels
        Point2f pt_left(pt_mat_left.at<double>(0, 0),
                        pt_mat_left.at<double>(1, 0));  // express as pixels
        Point2f pt_right(pt_mat_right.at<double>(0, 0),
                         pt_mat_right.at<double>(1, 0));  // express as pixels
        left_keypoints_rectified.push_back(
            make_pair(KeypointStatus::VALID, pt_left));
        right_keypoints_rectified.push_back(
            make_pair(KeypointStatus::VALID, pt_right));
        depth_expected.push_back(depth);
      }
    }
  }

  // Add a few invalid keypoints to the test.
  left_keypoints_rectified.push_back(
      make_pair(KeypointStatus::VALID, Point2f(1.0, 2.0)));
  right_keypoints_rectified.push_back(
      make_pair(KeypointStatus::NO_RIGHT_RECT, Point2f(1.0, 2.0)));
  depth_expected.push_back(0);

  left_keypoints_rectified.push_back(
      make_pair(KeypointStatus::NO_LEFT_RECT, Point2f(1.0, 2.0)));
  right_keypoints_rectified.push_back(
      make_pair(KeypointStatus::VALID, Point2f(1.0, 2.0)));
  depth_expected.push_back(0);

  left_keypoints_rectified.push_back(
      make_pair(KeypointStatus::NO_DEPTH, Point2f(1.0, 2.0)));
  right_keypoints_rectified.push_back(
      make_pair(KeypointStatus::FAILED_ARUN, Point2f(1.0, 2.0)));
  depth_expected.push_back(0);

  // Add a test case with negative disparity
  left_keypoints_rectified.push_back(
      make_pair(KeypointStatus::NO_DEPTH, Point2f(3.0, 2.0)));
  right_keypoints_rectified.push_back(
      make_pair(KeypointStatus::FAILED_ARUN, Point2f(1.0, 2.0)));
  depth_expected.push_back(0);

  // Call StereoFrame::getDepthFromRectifiedMatches to get the actual depth!
  std::vector<double> depth_actual =
      sf->getDepthFromRectifiedMatches(left_keypoints_rectified,
                                       right_keypoints_rectified,
                                       P1.at<double>(0, 0),
                                       sf->getBaseline());
  for (int i = 0; i < depth_actual.size(); i++) {
    // if depth is outside the valid range, it is conventionally set to zero
    if (depth_expected[i] < sf->getSparseStereoParams().min_point_dist_ ||
        depth_expected[i] > sf->getSparseStereoParams().max_point_dist_) {
      EXPECT_NEAR(depth_actual[i], 0, 1e-3);
    } else {  // check value
      EXPECT_NEAR(depth_expected[i], depth_actual[i], 1e-3);
    }
  }
}

/* ************************************************************************* */
TEST_F(StereoFrameFixture, getRightKeypointsRectified) {
  // Extract keypoints from the left img!
  sf->getLeftFrameMutable()->extractCorners();
  Mat left_img = sf->getLeftFrame().img_;
  const KeypointsCV& left_keypoints = sf->getLeftFrame().keypoints_;
  const int num_points = left_keypoints.size();

  // we offset left image artificially (to get right image) in order to have
  // ground truth for matching keypoint set of offsets we are going to test:
  // disparity = left_px.x - right_px.x, hence we check: right_px.x < left_px.x
  // => offset must be nonpositive
  std::vector<double> offset_range;
  offset_range.push_back(-20);
  offset_range.push_back(-10);
  offset_range.push_back(-5);
  const int num_offset_test = offset_range.size();

  int countValid = 0;
  int totalKeypointsTested = 0;
  for (int i = 0; i < num_offset_test; i++) {
    // Getting right (rectified) images by translating left image
    double offset = offset_range[i];
    Mat right_img = cvTranslateImageX(left_img, (int)offset);

    double tol_corr;
    // create left keypoints with status
    StatusKeypointsCV left_keypoints_rectified;
    for (int t = 0; t < 2; t++) {
      // t = 0: test on keypoints in float coordinates, tolerate larger matching
      // error t = 1: test on rounded keypoints, expect very small matching
      // error
      if (t == 1) {
        for (const auto& left_px : left_keypoints)
          left_keypoints_rectified.push_back(
              make_pair(KeypointStatus::VALID,
                        KeypointCV(round(left_px.x), round(left_px.y))));
        tol_corr = 1e-6;
      } else {
        for (const auto& left_px : left_keypoints)
          left_keypoints_rectified.push_back(
              make_pair(KeypointStatus::VALID, left_px));
        tol_corr = 5e-3;
      }

      // get actual matches
      StatusKeypointsCV right_keypoints_rectified =
          sf->getRightKeypointsRectified(left_img,
                                         right_img,
                                         left_keypoints_rectified,
                                         458.654,
                                         sf->getBaseline());

      for (size_t i = 0; i < left_keypoints_rectified.size();
           i++) {  // for each right status keypoint

        totalKeypointsTested += 1;
        StatusKeypointCV left_pt = left_keypoints_rectified.at(i);
        StatusKeypointCV right_pt = right_keypoints_rectified.at(i);

        // Judging the correctness of the matches.
        double y_left = left_pt.second.y;
        double x_exp = left_pt.second.x + offset;
        double x_actual = right_pt.second.x;

        int templ_cols = 101, templ_rows = 11;
        int stripe_rows = templ_rows + 4;

        // If left point is too close to boundary, we set right to be
        // NO_RIGHT_RECT
        if (y_left <= (stripe_rows - 1) / 2 ||
            y_left + (stripe_rows - 1) / 2 >= left_img.rows) {
          EXPECT_EQ(right_pt.first, KeypointStatus::NO_RIGHT_RECT);
        } else if (x_exp >= (templ_cols - 1) / 2 &&
                   x_exp + (templ_cols - 1) / 2 <
                       left_img.cols) {  // inside the frame
          // The match should be marked valid!
          EXPECT_EQ(right_pt.first, KeypointStatus::VALID);
          EXPECT_NEAR(x_exp, x_actual, 0.5);
          EXPECT_NEAR(left_pt.second.y, right_pt.second.y, 0.5);
          countValid += 1;
        } else if (x_exp < 0 ||
                   x_exp >= left_img.cols) {  // if true right match is invalid
                                              // and left is outside image
          // The match should be marked invalid!
          // no real way to detect this, besides checking disparity
          // EXPECT_EQ(right_pt.first, Kstatus::NO_RIGHT_RECT);
        } else {
          // Corner case: The matching may be affected by the image border
          EXPECT_TRUE((abs(x_exp - x_actual) < 0.5) ||
                      (abs(x_exp - x_actual) >= 0.5));
          if (!((abs(x_exp - x_actual) < 0.5) ||
                (abs(x_exp - x_actual) >= 0.5))) {
            cout << "IF WE GET TO THIS POINT, THE TEST FAILED - we visualize "
                    "wrong matches:"
                 << endl;
            KeypointsCV left_corners, right_corners;
            left_corners.push_back(left_pt.second);
            right_corners.push_back(right_pt.second);
            std::vector<cv::DMatch> matches;
            matches.push_back(cv::DMatch(0, 0, 0));
            Mat canvas = UtilsOpenCV::DrawCornersMatches(
                left_img, left_corners, right_img, right_corners, matches);
            // imshow("matched keypoints", canvas);
            // waitKey(0);
          }
        }
      }
    }
  }
  // make sure that at least some keypoints were valid (these are the ones we
  // are interested in)
  EXPECT_NEAR(858, countValid, 1e-5);
  // 7 is the number of offsets we try
  EXPECT_NEAR(900, totalKeypointsTested, 1e-5);
}

/* ************************************************************************* */
TEST_F(StereoFrameFixture, sparseStereoMatching) {
  // create a brand new stereo frame
  initializeDataStereo();

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // check that data is correctly populated:
  EXPECT_NEAR(0.110078, sfnew->getBaseline(), 1e-5);
  EXPECT_NEAR(100, sfnew->getRightFrame().keypoints_.size(), 1e-5);
  EXPECT_NEAR(0,
              sfnew->getRightFrame().scores_.size(),
              1e-5);  // scores do not get populated
  EXPECT_NEAR(0,
              sfnew->getRightFrame().landmarks_.size(),
              1e-5);  // landmarks_ do not get populated
  EXPECT_NEAR(0,
              sfnew->getRightFrame().landmarksAge_.size(),
              1e-5);  // landmarksAges do not get populated
  EXPECT_NEAR(0,
              sfnew->getRightFrame().versors_.size(),
              1e-5);  // landmarksAges do not get populated
  EXPECT_NEAR(100, sfnew->keypoints_depth_.size(), 1e-5);
  EXPECT_NEAR(100, sfnew->keypoints_3d_.size(), 1e-5);
  EXPECT_NEAR(100, sfnew->right_keypoints_status_.size(), 1e-5);

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // check that 3d point is consistent with the left versor and the depth
  // IMPORTANT: versors and keypoints3D are in different ref frames (former in
  // camL, latter in camLrect) TEST: check rotation due to rectification
  // (important when projecting points later on)
  sfnew->setIsRectified(true);  // Fake that the camera is rectified.
  gtsam::Rot3 expected_camL_R_camLrect =
      sfnew->getLeftFrame().cam_param_.body_Pose_cam_.rotation().between(
          sfnew->getBPoseCamLRect().rotation());  // camL_R_camLrect
  gtsam::Rot3 actual_camL_R_camLrect =
      UtilsOpenCV::cvMatToGtsamRot3(sfnew->getLeftFrame().cam_param_.R_rectify_)
          .inverse();
  EXPECT_TRUE(
      assert_equal(expected_camL_R_camLrect, actual_camL_R_camLrect, 1e-4));

  int nrValid = 0;
  for (size_t i = 0; i < sfnew->keypoints_3d_.size(); i++) {
    if (sfnew->right_keypoints_status_.at(i) == KeypointStatus::VALID) {
      nrValid += 1;
      double depth = sfnew->keypoints_depth_.at(i);
      gtsam::Vector3 versorActual = sfnew->keypoints_3d_.at(i) / depth;
      gtsam::Vector3 versorExpected =
          actual_camL_R_camLrect.inverse().matrix() *
          sfnew->getLeftFrame().versors_.at(i);
      versorExpected = versorExpected / versorExpected[2];
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
  // TODO(Toni): sometimes it is 68(lambda), sometimes 92 (Jenkins)?
  EXPECT_NEAR(92, nrValid, 1e-5);

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // check that 3D point reprojects correctly to the two cameras:
  for (size_t i = 0; i < sfnew->keypoints_3d_.size(); i++) {
    if (sfnew->right_keypoints_status_.at(i) == KeypointStatus::VALID) {
      // TEST: uncalibrateDistUnrect(versor) = original distorted unrectified
      // point (CHECK DIST UNRECT CALIBRATION WORKS)
      KeypointCV kp_i_distUnrect = sfnew->getLeftFrame().keypoints_.at(i);
      gtsam::Vector3 versor_i = sfnew->getLeftFrame().versors_.at(i);
      versor_i =
          versor_i / versor_i(2);  // set last element to 1, instead of norm 1
      Point2 kp_i_distUnrect_gtsam =
          sfnew->getLeftFrame().cam_param_.distortion_->uncalibrate(
              Point2(versor_i(0), versor_i(1)));
      EXPECT_TRUE(assert_equal(Point2(kp_i_distUnrect.x, kp_i_distUnrect.y),
                               kp_i_distUnrect_gtsam,
                               1));

      // TEST: uncalibrateUndistRect(versor) = original distorted unrectified
      // point (CHECK UNDIST RECT CALIBRATION WORKS)
      KeypointCV kp_i_undistRect = sfnew->left_keypoints_rectified_.at(i);
      Cal3_S2 sfnew_left_undist_rect_cam_mat = sfnew->getLeftUndistRectCamMat();
      Cal3_S2 KundistRect(sfnew_left_undist_rect_cam_mat.fx(),
                          sfnew_left_undist_rect_cam_mat.fy(),
                          sfnew_left_undist_rect_cam_mat.skew(),
                          sfnew_left_undist_rect_cam_mat.px(),
                          sfnew_left_undist_rect_cam_mat.py());
      versor_i = actual_camL_R_camLrect.inverse().matrix() *
                 versor_i;  // compensate for rotation due to rectification
      versor_i =
          versor_i / versor_i(2);  // set last element to 1, instead of norm 1
      Point2 kp_i_undistRect_gtsam =
          KundistRect.uncalibrate(Point2(versor_i(0), versor_i(1)));
      EXPECT_TRUE(assert_equal(Point2(kp_i_undistRect.x, kp_i_undistRect.y),
                               kp_i_undistRect_gtsam,
                               1));

      // TEST: distortUnrectify(undistRectified) = original distorted
      // unrectified point (CHECK UNDISTORTION WORKS)
      KeypointsCV dup;
      std::vector<KeypointStatus> statuses;
      StatusKeypointsCV urp;
      urp.push_back(make_pair(KeypointStatus::VALID, kp_i_undistRect));
      tie(dup, statuses) = StereoFrame::distortUnrectifyPoints(
          urp,
          sfnew->getLeftFrame().cam_param_.undistRect_map_x_,
          sfnew->getLeftFrame().cam_param_.undistRect_map_y_);
      EXPECT_TRUE(assert_equal(Point2(kp_i_distUnrect.x, kp_i_distUnrect.y),
                               Point2(dup.at(0).x, dup.at(0).y),
                               1));

      // TEST: projecting 3d point to left camera (undist and rectified) =
      // original undistorted rectified point (CHECK BACKPROJECTION WORKS)
      Point3 point3d = sfnew->keypoints_3d_.at(i);
      PinholeCamera<Cal3_S2> leftCam_undistRect(gtsam::Pose3(), KundistRect);
      Point2 p2_undistRect = leftCam_undistRect.project(point3d);
      EXPECT_TRUE(assert_equal(
          Point2(kp_i_undistRect.x, kp_i_undistRect.y), p2_undistRect, 1));

      // TEST: projecting 3d point to left camera (distorted and unrectified) =
      // original distorted unrectified point (CHECK BACKPROJECTION WORKS)
      Point3 point3d_unrect = actual_camL_R_camLrect.rotate(
          point3d);  // compensate for the rotation induced by rectification
      Point2 p2_distUnrect =
          sfnew->getLeftFrame().cam_param_.distortion_->project(Pose3(),
                                                                point3d_unrect);
      EXPECT_TRUE(assert_equal(
          Point2(kp_i_distUnrect.x, kp_i_distUnrect.y), p2_distUnrect, 1));

      // TEST: projecting 3d point to stereo camera
      // reproject to camera and check that matches corresponding rectified
      // pixels
      Cal3_S2 sfnew_left_undist_rect_cam_mat_2 =
          sfnew->getLeftUndistRectCamMat();
      Cal3_S2Stereo::shared_ptr K(
          new Cal3_S2Stereo(sfnew_left_undist_rect_cam_mat_2.fx(),
                            sfnew_left_undist_rect_cam_mat_2.fy(),
                            sfnew_left_undist_rect_cam_mat_2.skew(),
                            sfnew_left_undist_rect_cam_mat_2.px(),
                            sfnew_left_undist_rect_cam_mat_2.py(),
                            sfnew->getBaseline()));
      // Note: camera pose is the identity (instead of
      // sfnew->getBPoseCamLRect()) since the 3D point is in the left camera
      // frame
      StereoCamera stereoCam = StereoCamera(gtsam::Pose3(), K);
      StereoPoint2 sp2 = stereoCam.project(point3d);
      EXPECT_NEAR(sp2.uL(), sfnew->left_keypoints_rectified_.at(i).x, 1);
      EXPECT_NEAR(sp2.v(), sfnew->left_keypoints_rectified_.at(i).y, 1);
      EXPECT_NEAR(sp2.uR(), sfnew->right_keypoints_rectified_.at(i).x, 1);
      EXPECT_NEAR(sp2.v(),
                  sfnew->right_keypoints_rectified_.at(i).y,
                  3);  // slightly larger errors
    }
  }
}

/* *************************************************************************
TEST_F(StereoFrameFixture, sparseStereoMatching_v2) {
  // this should be enabled if lines after 66 are uncommented
  // create a brand new stereo frame
  initializeDataStereo();

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // check that data is correctly populated:
  EXPECT_NEAR(0.110078, sfnew->baseline(), 1e-5);
  EXPECT_NEAR(100, sfnew->getRightFrame().keypoints_.size(), 1e-5);
  EXPECT_NEAR(0, sfnew->getRightFrame().scores_.size(), 1e-5);  //
  // scores do not get populated
  EXPECT_NEAR(0, sfnew->getRightFrame().landmarks_.size(),
              1e-5);  // landmarks_ do not get populated
  EXPECT_NEAR(0, sfnew->getRightFrame().landmarksAge_.size(),
              1e-5);  // landmarksAges do not get populated
  EXPECT_NEAR(0, sfnew->getRightFrame().versors_.size(),
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
      gtsam::Vector3 versorExpected = sfnew->getLeftFrame().versors_.at(i) /
                               sfnew->getLeftFrame().versors_.at(i)[2];
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
      gtsam::Vector3 versor_i_rect = sfnew->getLeftFrame().versors_.at(i);
      versor_i_rect = versor_i_rect / versor_i_rect(2);  // set last element to
      1, instead of norm 1

      // TEST: check rotation due to rectification (important when projecting
      points later on) gtsam::Rot3 expected_camL_R_camLrect =
      sfnew->getLeftFrame().cam_param_.body_Pose_cam_.rotation().between(sfnew->getBPoseCamLRect().rotation());
      // camL_R_camLrect gtsam::Rot3 actual_camL_R_camLrect =
      UtilsOpenCV::Cvmat2rot(sfnew->getLeftFrame().cam_param_.R_rectify_)
          .inverse();
      EXPECT_TRUE(
          assert_equal(expected_camL_R_camLrect, actual_camL_R_camLrect, 1e-6));

      // TEST: uncalibrateDistUnrect(versor) = original distorted unrectified
      point(CHECK DIST UNRECT CALIBRATION WORKS) KeypointCV kp_i_distUnrect =
          sfnew->getLeftFrame().keypoints_.at(i);
      // after stereo matching, versor will be in rectified frame, so to
      // compare with unrect measurements we have to compensate rectification
      gtsam::Vector3 versor_i_unRect = actual_camL_R_camLrect.matrix() *
versor_i_rect; versor_i_unRect = versor_i_unRect / versor_i_unRect(2); Point2
kp_i_distUnrect_gtsam =
          sfnew->getLeftFrame().cam_param_.calibration_.uncalibrate(
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
          urp, sfnew->getLeftFrame().cam_param_.undistRect_map_x_,
          sfnew->getLeftFrame().cam_param_.undistRect_map_y_);
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
      = sfnew->getLeftFrame().cam_param_.calibration_;
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
/* ************************************************************************* */

TEST_F(StereoFrameFixture, getLandmarkInfo) {
  // Try to retrieve every single landmark and compare against ground truth.
  const auto& left_frame = sfnew->getLeftFrame();
  for (size_t i = 0; i < left_frame.keypoints_.size(); i++) {
    StereoFrame::LandmarkInfo lmInfo = sfnew->getLandmarkInfo(i);
    EXPECT_DOUBLE_EQ(left_frame.keypoints_.at(i).x, lmInfo.keypoint.x);
    EXPECT_DOUBLE_EQ(left_frame.keypoints_.at(i).y, lmInfo.keypoint.y);
    EXPECT_DOUBLE_EQ(10 * i, lmInfo.score);
    EXPECT_DOUBLE_EQ(5 * i, lmInfo.age);
    gtsam::Vector3 actual = lmInfo.keypoint_3d;
    gtsam::Vector3 expected = sfnew->keypoints_3d_.at(i);
    EXPECT_TRUE(assert_equal(expected, actual));
  }
}

/* ************************************************************************* */
// Test undistortion of fisheye / pinhole equidistant model
TEST(testStereoFrame, undistortFisheye) {
  // Parse camera params
  static CameraParams cam_params_left_fisheye;
  cam_params_left_fisheye.parseYAML(stereo_FLAGS_test_data_path +
                                    "/left_sensor_fisheye.yaml");

  // Parse single image
  cv::Mat left_fisheye_image_dist = UtilsOpenCV::ReadAndConvertToGrayScale(
      stereo_FLAGS_test_data_path + "left_fisheye_img_0.png", false);

  // Declare empty variables
  cv::Mat left_fisheye_image_undist, map_x_fisheye_undist, map_y_fisheye_undist;

  // Undistort image using pinhole equidistant (fisheye) model
  if (cam_params_left_fisheye.distortion_model_ == "equidistant") {
    cv::fisheye::initUndistortRectifyMap(
        cam_params_left_fisheye.camera_matrix_,
        cam_params_left_fisheye.distortion_coeff_,
        // not relevant here
        cv::Mat::eye(3, 3, CV_32F),
        // don't to use default identity!
        cam_params_left_fisheye.camera_matrix_,
        cam_params_left_fisheye.image_size_,
        CV_32FC1,
        // output
        map_x_fisheye_undist,
        map_y_fisheye_undist);
    cv::remap(left_fisheye_image_dist,
              left_fisheye_image_undist,
              map_x_fisheye_undist,
              map_y_fisheye_undist,
              cv::INTER_LINEAR);
  } else {
    LOG(ERROR) << "Distortion model is not pinhole equidistant.";
  }

  // Parse reference image
  cv::Mat left_fisheye_image_ref = UtilsOpenCV::ReadAndConvertToGrayScale(
      stereo_FLAGS_test_data_path + "left_ref_img_0.png", false);

  // Test distortion with image comparison
  EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(
      left_fisheye_image_undist, left_fisheye_image_ref, 1e-3));
}

// TODO: Figure out why this compiles on PC, but not on Jenkins
TEST_F(StereoFrameFixture, DISABLED_undistortFisheyeStereoFrame) {
  // Parse camera params for left and right cameras
  static CameraParams cam_params_left_fisheye;
  cam_params_left_fisheye.parseYAML(stereo_FLAGS_test_data_path +
                                    "/left_sensor_fisheye.yaml");
  static CameraParams cam_params_right_fisheye;
  cam_params_right_fisheye.parseYAML(stereo_FLAGS_test_data_path +
                                     "/right_sensor_fisheye.yaml");

  // Get images
  cv::Mat left_fisheye_image_dist = UtilsOpenCV::ReadAndConvertToGrayScale(
      stereo_FLAGS_test_data_path + "left_fisheye_img_0.png", false);
  cv::Mat right_fisheye_image_dist = UtilsOpenCV::ReadAndConvertToGrayScale(
      stereo_FLAGS_test_data_path + "right_fisheye_img_0.png", false);

  sf = std::make_shared<StereoFrame>(0,
                                     0,  // Default, not used here
                                     // Left frame
                                     left_fisheye_image_dist,
                                     cam_params_left_fisheye,
                                     // Right frame
                                     right_fisheye_image_dist,
                                     cam_params_right_fisheye,
                                     // Relative pose
                                     // Default, not used here
                                     StereoMatchingParams());

  // Get rectified images
  CHECK(sf->isRectified());

  // Define rectified images
  cv::Mat left_image_rectified, right_image_rectified;
  sf->left_img_rectified_.copyTo(left_image_rectified);
  sf->right_img_rectified_.copyTo(right_image_rectified);

  // Get camera matrix for new rectified stereo
  P1 = sf->getLeftFrame().cam_param_.P_;
  P2 = sf->getRightFrame().cam_param_.P_;

  // Get rectified left keypoints.
  gtsam::Cal3_S2 left_undistRectCameraMatrix_fisheye =
      UtilsOpenCV::Cvmat2Cal3_S2(P1);
  StatusKeypointsCV left_keypoints_rectified;
  Frame left_frame_fish = sf->getLeftFrame();
  Frame right_frame_fish = sf->getRightFrame();
  left_frame_fish.extractCorners();
  sf->undistortRectifyPoints(left_frame_fish.keypoints_,
                             left_frame_fish.cam_param_,
                             left_undistRectCameraMatrix_fisheye,
                             &left_keypoints_rectified);

  // Get rectified right keypoints
  StatusKeypointsCV right_keypoints_rectified;
  right_keypoints_rectified =
      sf->getRightKeypointsRectified(left_image_rectified,
                                     right_image_rectified,
                                     left_keypoints_rectified,
                                     left_undistRectCameraMatrix_fisheye.fx(),
                                     sf->getBaseline());

  // Check corresponding features are on epipolar lines (visually and store)
  cv::Mat undist_sidebyside = UtilsOpenCV::ConcatenateTwoImages(
      left_image_rectified, right_image_rectified);

  // Parse reference image -> Remove comments
  cv::Mat undist_sidebyside_ref =
      cv::imread(stereo_FLAGS_test_data_path + "sidebyside_ref_img_0.png",
                 cv::IMREAD_ANYCOLOR);

  // Get keypoints depth
  std::vector<double> keypoints_depth =
      sf->getDepthFromRectifiedMatches(left_keypoints_rectified,
                                       right_keypoints_rectified,
                                       left_undistRectCameraMatrix_fisheye.fx(),
                                       sf->getBaseline());

  // Test distortion with image comparison --> uncomment
  EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(
      undist_sidebyside, undist_sidebyside_ref, 1e-1));
}
