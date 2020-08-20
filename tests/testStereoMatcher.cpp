/* -----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testStereoMatcher.cpp
 * @brief  test StereoMatcher
 * @author Marcus Abate
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/VisionFrontEndParams.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoMatcher.h"

DECLARE_string(test_data_path);

using namespace gtsam;
using namespace std;
using namespace VIO;

static const string stereo_FLAGS_test_data_path(FLAGS_test_data_path +
                                                string("/ForStereoFrame/"));
static const string left_image_name = "left_img_0.png";
static const string right_image_name = "right_img_0.png";

class StereoMatcherFixture : public ::testing::Test {
 public:
  StereoMatcherFixture() : cam_params_left(), cam_params_right() {
    cam_params_left.parseYAML(stereo_FLAGS_test_data_path + "/sensorLeft.yaml");
    cam_params_right.parseYAML(stereo_FLAGS_test_data_path +
                               "/sensorRight.yaml");

    // construct stereo camera
    VIO::FrontendParams tp;
    stereo_camera =std::make_shared<VIO::StereoCamera>(
        cam_params_left, cam_params_right);
    stereo_matcher = VIO::make_unique<StereoMatcher>(
        stereo_camera, tp.stereo_matching_params_);

    initializeDataStereo();
  }
 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  void initializeDataStereo() {
    cam_params_left.parseYAML(stereo_FLAGS_test_data_path + "/sensorLeft.yaml");
    cam_params_right.parseYAML(stereo_FLAGS_test_data_path +
                               "/sensorRight.yaml");

    VIO::FrontendParams tp;
    sf = std::make_shared<StereoFrame>(
        0,
        0,
        Frame(0,
              0,
              cam_params_left,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  stereo_FLAGS_test_data_path + left_image_name,
                  tp.stereo_matching_params_.equalize_image_)),
        Frame(0,
              0,
              cam_params_right,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  stereo_FLAGS_test_data_path + right_image_name,
                  tp.stereo_matching_params_.equalize_image_)));

    sfnew = std::make_shared<StereoFrame>(
        0,
        0,
        Frame(0,
              0,
              cam_params_left,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  stereo_FLAGS_test_data_path + left_image_name,
                  tp.stereo_matching_params_.equalize_image_)),
        Frame(0,
              0,
              cam_params_right,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  stereo_FLAGS_test_data_path + right_image_name,
                  tp.stereo_matching_params_.equalize_image_)));

    Frame* left_frame = sfnew->getLeftFrameMutable();
    UtilsOpenCV::ExtractCorners(left_frame->img_, &left_frame->keypoints_);
    left_frame->versors_.reserve(sfnew->getLeftFrame().keypoints_.size());
    int landmark_count_ = 0;
    for (size_t i = 0; i < sfnew->getLeftFrame().keypoints_.size(); i++) {
      sfnew->getLeftFrameMutable()->landmarks_.push_back(landmark_count_);
      sfnew->getLeftFrameMutable()->landmarks_age_.push_back(
          5 * landmark_count_);  // seen in a single (key)frame
      sfnew->getLeftFrameMutable()->scores_.push_back(10 * landmark_count_);
      sfnew->getLeftFrameMutable()->versors_.push_back(
          Frame::calibratePixel(sfnew->getLeftFrame().keypoints_.at(i),
                                sfnew->getLeftFrame().cam_param_));
      ++landmark_count_;
    }

    // do sparse stereo
    stereo_matcher->sparseStereoReconstruction(sfnew.get());
  }

  VIO::CameraParams cam_params_left;
  VIO::CameraParams cam_params_right;
  StereoFrame::Ptr sf, sfnew;
  VIO::StereoCamera::Ptr stereo_camera;
  VIO::StereoMatcher::UniquePtr stereo_matcher;
};

static cv::Mat cvTranslateImageX(cv::Mat img, double dist) {
  cv::Mat result = cv::Mat(img.rows, img.cols, img.type());
  cv::Mat translation_mat = cv::Mat::eye(3, 3, CV_64F);
  translation_mat.at<double>(0, 2) = dist;
  cv::warpPerspective(
      img, result, translation_mat, img.size(), cv::INTER_NEAREST);
  return result;
}

TEST_F(StereoMatcherFixture, denseStereoReconstruction) {
  // TODO(marcus): implement
}

TEST_F(StereoMatcherFixture, sparseStereoReconstruction) {
  // create a brand new stereo frame
  initializeDataStereo();
  CHECK(sfnew);

  gtsam::Cal3DS2 gtsam_calib;
  CameraParams::createGtsamCalibration(
      sfnew->getLeftFrame().cam_param_.distortion_coeff_mat_,
      sfnew->getLeftFrame().cam_param_.intrinsics_,
      &gtsam_calib);

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // check that data is correctly populated:
  EXPECT_NEAR(0.110078, stereo_camera->getBaseline(), 1e-5);
  EXPECT_NEAR(100, sfnew->getRightFrame().keypoints_.size(), 1e-5);
  EXPECT_NEAR(0,
              sfnew->getRightFrame().scores_.size(),
              1e-5);  // scores do not get populated
  EXPECT_NEAR(0,
              sfnew->getRightFrame().landmarks_.size(),
              1e-5);  // landmarks_ do not get populated
  EXPECT_NEAR(0,
              sfnew->getRightFrame().landmarks_age_.size(),
              1e-5);  // landmarksAges do not get populated
  EXPECT_NEAR(0,
              sfnew->getRightFrame().versors_.size(),
              1e-5);  // landmarksAges do not get populated
  EXPECT_NEAR(100, sfnew->get3DKpts().size(), 1e-5);
  EXPECT_NEAR(100, sfnew->getRightKptsRectified().size(), 1e-5);

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // check that 3d point is consistent with the left versor and the depth
  // IMPORTANT: versors and keypoints3D are in different ref frames (former in
  // camL, latter in camLrect) TEST: check rotation due to rectification
  // (important when projecting points later on)
  // sfnew->setIsRectified(true);  // Fake that the camera is rectified.
  gtsam::Rot3 expected_camL_R_camLrect =
      sfnew->getLeftFrame().cam_param_.body_Pose_cam_.rotation().between(
          stereo_camera->getBodyPoseLeftCamRect().rotation());  // camL_R_camLrect
  gtsam::Rot3 actual_camL_R_camLrect =
      UtilsOpenCV::cvMatToGtsamRot3(stereo_camera->getR1()).inverse();
  EXPECT_TRUE(
      assert_equal(expected_camL_R_camLrect, actual_camL_R_camLrect, 1e-4));

  int nrValid = 0;
  for (size_t i = 0; i < sfnew->get3DKpts().size(); i++) {
    if (sfnew->getRightKptsRectified().at(i).first == KeypointStatus::VALID) {
      nrValid += 1;
      double depth = sfnew->get3DKpts().at(i).z();
      gtsam::Vector3 versorActual = sfnew->get3DKpts().at(i) / depth;
      gtsam::Vector3 versorExpected =
          actual_camL_R_camLrect.inverse().matrix() *
          sfnew->getLeftFrame().versors_.at(i);
      versorExpected = versorExpected / versorExpected[2];
      EXPECT_TRUE(assert_equal(versorExpected, versorActual));
    } else {
      // when invalid 3D point is set to zero..
      gtsam::Vector3 pointActual = sfnew->get3DKpts().at(i);
      gtsam::Vector3 pointExpected = gtsam::Vector3::Zero();
      EXPECT_TRUE(assert_equal(pointExpected, pointActual));
      // and the corresponding depth is set to zero too
    }
  }
  // TODO(Toni): sometimes it is 68(lambda), sometimes 92 (Jenkins)?
  EXPECT_EQ(91, nrValid);

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // check that 3D point reprojects correctly to the two cameras:
  for (size_t i = 0; i < sfnew->get3DKpts().size(); i++) {
    if (sfnew->getRightKptsRectified().at(i).first == KeypointStatus::VALID) {
      // TEST: uncalibrateDistUnrect(versor) = original distorted unrectified
      // point (CHECK DIST UNRECT CALIBRATION WORKS)
      KeypointCV kp_i_distUnrect = sfnew->getLeftFrame().keypoints_.at(i);
      gtsam::Vector3 versor_i = sfnew->getLeftFrame().versors_.at(i);
      versor_i =
          versor_i / versor_i(2);  // set last element to 1, instead of norm 1
      Point2 kp_i_distUnrect_gtsam =
          gtsam_calib.uncalibrate(Point2(versor_i(0), versor_i(1)));
      EXPECT_TRUE(assert_equal(Point2(kp_i_distUnrect.x, kp_i_distUnrect.y),
                               kp_i_distUnrect_gtsam,
                               1));

      // TEST: uncalibrateUndistRect(versor) = original distorted unrectified
      // point (CHECK UNDIST RECT CALIBRATION WORKS)
      VIO::Camera left_camera(cam_params_left);
      KeypointCV kp_i_undistRect = sfnew->getLeftKptsRectified().at(i).second;
      Cal3_S2 sfnew_left_undist_rect_cam_mat = left_camera.getCalibration();
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

      // TEST: projecting 3d point to left camera (undist and rectified) =
      // original undistorted rectified point (CHECK BACKPROJECTION WORKS)
      Point3 point3d = sfnew->get3DKpts().at(i);
      PinholeCamera<Cal3_S2> leftCam_undistRect(gtsam::Pose3(), KundistRect);
      Point2 p2_undistRect = leftCam_undistRect.project(point3d);
      EXPECT_TRUE(assert_equal(
          Point2(kp_i_undistRect.x, kp_i_undistRect.y), p2_undistRect, 1));

      // TEST: projecting 3d point to left camera (distorted and unrectified) =
      // original distorted unrectified point (CHECK BACKPROJECTION WORKS)
      Point3 point3d_unrect = actual_camL_R_camLrect.rotate(
          point3d);  // compensate for the rotation induced by rectification
      Cal3DS2 KdistUnrect = gtsam_calib;
      PinholeCamera<Cal3DS2> leftCam_distUnrect(gtsam::Pose3(), KdistUnrect);
      Point2 p2_distUnrect = leftCam_distUnrect.project(point3d_unrect);
      EXPECT_TRUE(assert_equal(
          Point2(kp_i_distUnrect.x, kp_i_distUnrect.y), p2_distUnrect, 1));

      // TEST: projecting 3d point to stereo camera
      // reproject to camera and check that matches corresponding rectified
      // pixels
      Cal3_S2 sfnew_left_undist_rect_cam_mat_2 = 
          left_camera.getCalibration();
      Cal3_S2Stereo::shared_ptr K(
          new Cal3_S2Stereo(sfnew_left_undist_rect_cam_mat_2.fx(),
                            sfnew_left_undist_rect_cam_mat_2.fy(),
                            sfnew_left_undist_rect_cam_mat_2.skew(),
                            sfnew_left_undist_rect_cam_mat_2.px(),
                            sfnew_left_undist_rect_cam_mat_2.py(),
                            stereo_camera->getBaseline()));
      // Note: camera pose is the identity (instead of
      // sfnew->getBPoseCamLRect()) since the 3D point is in the left camera
      // frame
      gtsam::StereoCamera stereoCam = gtsam::StereoCamera(gtsam::Pose3(), K);
      StereoPoint2 sp2 = stereoCam.project(point3d);
      EXPECT_NEAR(sp2.uL(), sfnew->getLeftKptsRectified().at(i).second.x, 1);
      EXPECT_NEAR(sp2.v(), sfnew->getLeftKptsRectified().at(i).second.y, 1);
      EXPECT_NEAR(sp2.uR(), sfnew->getRightKptsRectified().at(i).second.x, 1);
      EXPECT_NEAR(sp2.v(),
                  sfnew->getRightKptsRectified().at(i).second.y,
                  3);  // slightly larger errors
    }
  }
}

TEST_F(StereoMatcherFixture, sparseStereoReconstructionStereoFrame) {

}

TEST_F(StereoMatcherFixture, getRightKeypointsRectified) {
  // Extract keypoints from the left img!
  Frame* left_frame = sf->getLeftFrameMutable();
  UtilsOpenCV::ExtractCorners(left_frame->img_, &left_frame->keypoints_);
  cv::Mat left_img = sf->getLeftFrame().img_;
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
    cv::Mat right_img = cvTranslateImageX(left_img, (int)offset);

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
      StatusKeypointsCV right_keypoints_rectified;
      stereo_matcher->getRightKeypointsRectified(left_img,
                                                 right_img,
                                                 left_keypoints_rectified,
                                                 458.654,
                                                 stereo_camera->getBaseline(),
                                                 &right_keypoints_rectified);

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
            cv::Mat canvas = UtilsOpenCV::DrawCornersMatches(
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
  EXPECT_EQ(849, countValid);
  // 7 is the number of offsets we try
  EXPECT_EQ(900, totalKeypointsTested);
}

TEST_F(StereoMatcherFixture, searchRightKeypointEpipolar) {
  // TODO(marcus): implement
}
