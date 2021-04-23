/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testUndistortRectifier.h
 * @brief  test UndistortRectifier
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <string>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/frontend/UndistorterRectifier.h"
#include "kimera-vio/frontend/VisionImuFrontendParams.h"

DECLARE_string(test_data_path);

static const std::string stereo_FLAGS_test_data_path(
    FLAGS_test_data_path + std::string("/ForStereoFrame/"));
static const std::string left_image_name = "left_img_0.png";

class UndistortRectifierFixture : public ::testing::Test {
 public:
  UndistortRectifierFixture()
    : cam_params_left(),
      cam_params_right(),
      undistorter_rectifier() {
    cam_params_left.parseYAML(stereo_FLAGS_test_data_path + "/sensorLeft.yaml");
    cam_params_right.parseYAML(stereo_FLAGS_test_data_path + "/sensorRight.yaml");

    // Construct UndistortRectifier
    stereo_camera = std::make_shared<VIO::StereoCamera>(
        cam_params_left, cam_params_right);
    undistorter_rectifier = VIO::make_unique<VIO::UndistorterRectifier>(
        stereo_camera->getP1(), 
        cam_params_left,
        stereo_camera->getR1());
  }

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  // Generate keypoints regularly spaced along a grid of size num_rows x num_rows.
  void GeneratePointGrid(const size_t& num_rows,
                         const size_t& num_cols,
                         const size_t& image_height,
                         const size_t& image_width,
                         VIO::KeypointsCV* keypoints) {
    CHECK_NOTNULL(keypoints);
    for (size_t r = 0; r < num_rows; r++) {
      for (size_t c = 0; c < num_cols; c++) {
        int x = image_width / (num_cols - 1) * c;
        int y = image_height / (num_rows - 1) * r;
        keypoints->push_back(cv::Point2f(x, y));
      }
    }
  }

  VIO::CameraParams cam_params_left;
  VIO::CameraParams cam_params_right;

  VIO::StereoCamera::ConstPtr stereo_camera;
  VIO::UndistorterRectifier::UniquePtr undistorter_rectifier;
};


TEST_F(UndistortRectifierFixture, unidstortRectifyImage) {
  CHECK(undistorter_rectifier);
  // TODO(marcus): implement
}

TEST_F(UndistortRectifierFixture, undistortRectifyKeypoints) {
  CHECK(undistorter_rectifier);

  // get image rows and cols
  VIO::FrontendParams tp;
  cv::Mat img = VIO::UtilsOpenCV::ReadAndConvertToGrayScale(
              stereo_FLAGS_test_data_path + left_image_name,
              tp.stereo_matching_params_.equalize_image_);
  const int img_rows = img.rows;
  const int img_cols = img.cols;

  // generate keypoints regularly spaced along a grid of size numRows x numRows
  VIO::KeypointsCV keypoints_unrectified_gt;
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
  VIO::KeypointsCV keypoints_rectified;
  undistorter_rectifier->undistortRectifyKeypoints(
      keypoints_unrectified_gt,
      &keypoints_rectified);

  // Map back!
  VIO::StatusKeypointsCV status_keypoints_rectified;
  undistorter_rectifier->checkUndistortedRectifiedLeftKeypoints(
      keypoints_unrectified_gt,
      keypoints_rectified,
      &status_keypoints_rectified);

  VIO::KeypointsCV keypoints_unrectified_actual;
  undistorter_rectifier->distortUnrectifyKeypoints(
      status_keypoints_rectified,
      &keypoints_unrectified_actual);

  // Comparision
  EXPECT_EQ(status_keypoints_rectified.size(),
           keypoints_unrectified_actual.size());
  EXPECT_EQ(status_keypoints_rectified.size(),
           keypoints_unrectified_gt.size());

  for (int i = 0; i < keypoints_unrectified_actual.size(); i++) {
    if (status_keypoints_rectified[i].first != VIO::KeypointStatus::VALID)
      continue;
    // compare pixel coordinates of valid points
    EXPECT_NEAR(
        keypoints_unrectified_actual[i].x, 
        keypoints_unrectified_gt[i].x, 1);
    EXPECT_NEAR(
        keypoints_unrectified_actual[i].y, 
        keypoints_unrectified_gt[i].y, 1);
  }
}

TEST_F(UndistortRectifierFixture, checkUndistortedRectifiedLeftKeypoints) {
  CHECK(undistorter_rectifier);
  // TODO(marcus): implement
}

TEST_F(UndistortRectifierFixture, distortUnrectifyKeypoints) {
  gtsam::Cal3DS2 gtsam_calib;
  VIO::CameraParams::createGtsamCalibration(
      cam_params_left.distortion_coeff_mat_,
      cam_params_left.intrinsics_,
      &gtsam_calib);

  // Prepare the input data, on a grid!
  VIO::FrontendParams tp;
  cv::Mat img = VIO::UtilsOpenCV::ReadAndConvertToGrayScale(
      stereo_FLAGS_test_data_path + left_image_name,
      tp.stereo_matching_params_.equalize_image_);
  cv::Mat left_image_rectified;
  undistorter_rectifier->undistortRectifyImage(img, &left_image_rectified);

  const int numRows = 8;
  const int numCols = 10;
  VIO::StatusKeypointsCV keypoints_rectified;
  for (int r = 0; r < numRows; r++) {
    for (int c = 0; c < numCols; c++) {
      int y = left_image_rectified.rows / (numRows - 1) * r;
      int x = left_image_rectified.cols / (numCols - 1) * c;
      VIO::KeypointStatus st = VIO::KeypointStatus::VALID;
      if ((r + c) % 2 == 0) {  // set some of them to NO_RIGHT_RECT
        st = VIO::KeypointStatus::NO_RIGHT_RECT;
      }
      keypoints_rectified.push_back(std::make_pair(st, cv::Point2f(x, y)));
    }
  }

  VIO::KeypointsCV keypoints_unrectified;
  undistorter_rectifier->distortUnrectifyKeypoints(keypoints_rectified,
                                                   &keypoints_unrectified);

  // Manually compute the expected distorted/unrectified keypoints!
  EXPECT_EQ(keypoints_rectified.size(), keypoints_unrectified.size());
  for (int i = 0; i < keypoints_unrectified.size(); i++) {  // for each keypoint
    if (keypoints_rectified[i].first !=
        VIO::KeypointStatus::VALID) {  // if status is not valid, keypoint is
                                  // conventionally
                                  // set to (0,0)
      EXPECT_TRUE(keypoints_unrectified[i].x == 0 &&
                  keypoints_unrectified[i].y == 0);
      continue;
    }

    // if it is VALID, compare actual pixel value against manually computed
    // expected value
    cv::Mat pt = (cv::Mat_<double>(3, 1) << keypoints_rectified[i].second.x,
                  keypoints_rectified[i].second.y,
                  1);  // homogeneous pixel
    cv::Mat P1_inv = stereo_camera->getP1()(cv::Rect(0, 0, 3, 3)).inv();
    cv::Mat R = stereo_camera->getR1();
    cv::Mat xn = R.t() * P1_inv * pt;
    VIO::Point2 pt_in = VIO::Point2(
        xn.at<double>(0, 0) / xn.at<double>(2, 0),
        xn.at<double>(1, 0) /
            xn.at<double>(
                2, 0));  // convert back to pixel (non-homogeneous coordinates)
    VIO::Point2 pt_expected = gtsam_calib.uncalibrate(pt_in);

    // actual value
    VIO::Point2 pt_actual =
        VIO::Point2(keypoints_unrectified[i].x, keypoints_unrectified[i].y);

    // Comparison!
    EXPECT_TRUE(gtsam::assert_equal(pt_expected, pt_actual, 1e-3));
  }
}

// Test undistortion of fisheye / pinhole equidistant model
// TODO(marcus): ref image seems incorrect (see imshow code at end)
// too cropped compared to actual output. Have to regenerate.
TEST_F(UndistortRectifierFixture, DISABLED_undistortFisheye) {
  // Parse camera params
  static VIO::CameraParams cam_params_left_fisheye, cam_params_right_fisheye;
  cam_params_left_fisheye.parseYAML(stereo_FLAGS_test_data_path +
                                    "/left_sensor_fisheye.yaml");
  cam_params_right_fisheye.parseYAML(stereo_FLAGS_test_data_path +
                                     "/right_sensor_fisheye.yaml");
  VIO::StereoCamera fisheye_camera(cam_params_left_fisheye,
                                   cam_params_right_fisheye);

  VIO::UndistorterRectifier fisheye_undistorter_left(fisheye_camera.getP1(),
                                                     cam_params_left_fisheye,
                                                     fisheye_camera.getR1());

  // Parse single image
  cv::Mat left_fisheye_image_dist = 
      VIO::UtilsOpenCV::ReadAndConvertToGrayScale(stereo_FLAGS_test_data_path
        + "left_fisheye_img_0.png", false);

  // Declare empty variables
  cv::Mat left_fisheye_image_undist;

  // Undistort image using pinhole equidistant (fisheye) model
  if (cam_params_left_fisheye.distortion_model_ ==
      VIO::DistortionModel::EQUIDISTANT) {
    fisheye_undistorter_left.undistortRectifyImage(left_fisheye_image_dist,
                                                   &left_fisheye_image_undist);
  } else {
    LOG(FATAL) << "Distortion model is not pinhole equidistant.";
  }

  // Parse reference image
  cv::Mat left_fisheye_image_ref = VIO::UtilsOpenCV::ReadAndConvertToGrayScale(
      stereo_FLAGS_test_data_path + "left_ref_img_0.png", false);

  // cv::imshow("left_fisheye_image_ref", left_fisheye_image_ref);
  // cv::imshow("left_fisheye_image_undist", left_fisheye_image_undist);
  // cv::waitKey(0);

  // Test distortion with image comparison
  EXPECT_TRUE(VIO::UtilsOpenCV::compareCvMatsUpToTol(
      left_fisheye_image_undist, left_fisheye_image_ref, 1e-3));
}

// TODO: Figure out why this compiles on PC, but not on Jenkins
// TODO: reenable and fix
TEST_F(UndistortRectifierFixture, DISABLED_undistortFisheyeStereoFrame) {
  // Parse camera params for left and right cameras
  static VIO::CameraParams cam_params_left_fisheye;
  cam_params_left_fisheye.parseYAML(stereo_FLAGS_test_data_path +
                                    "/left_sensor_fisheye.yaml");
  static VIO::CameraParams cam_params_right_fisheye;
  cam_params_right_fisheye.parseYAML(stereo_FLAGS_test_data_path +
                                     "/right_sensor_fisheye.yaml");

  // Get images
  cv::Mat left_fisheye_image_dist = VIO::UtilsOpenCV::ReadAndConvertToGrayScale(
      stereo_FLAGS_test_data_path + "left_fisheye_img_0.png", false);
  cv::Mat right_fisheye_image_dist = VIO::UtilsOpenCV::ReadAndConvertToGrayScale(
      stereo_FLAGS_test_data_path + "right_fisheye_img_0.png", false);

  VIO::StereoFrame::Ptr sf = std::make_shared<VIO::StereoFrame>(
      0,
      0,
      VIO::Frame(0, 0, cam_params_left_fisheye, left_fisheye_image_dist),
      VIO::Frame(0, 0, cam_params_right_fisheye, right_fisheye_image_dist));

  sf->setIsRectified(true);

  // Define rectified images
  const cv::Mat left_image_rectified = sf->getLeftImgRectified();
  const cv::Mat right_image_rectified = sf->getRightImgRectified();

  VIO::StereoCamera::Ptr fisheye_camera = std::make_shared<VIO::StereoCamera>(
      cam_params_left_fisheye, cam_params_right_fisheye);
  VIO::UndistorterRectifier::Ptr undistorter_left =
      std::make_shared<VIO::UndistorterRectifier>(fisheye_camera->getP1(),
                                                  cam_params_left_fisheye,
                                                  fisheye_camera->getR1());
  VIO::StereoMatcher::Ptr fisheye_matcher =
      std::make_shared<VIO::StereoMatcher>(fisheye_camera,
                                           VIO::StereoMatchingParams());

  // Get rectified left keypoints.
  gtsam::Cal3_S2 left_undistRectCameraMatrix_fisheye =
      VIO::UtilsOpenCV::Cvmat2Cal3_S2(fisheye_camera->getP1());
  VIO::KeypointsCV left_keypoints_rectified;
  VIO::Frame left_frame_fish = sf->left_frame_;
  VIO::Frame right_frame_fish = sf->right_frame_;
  VIO::UtilsOpenCV::ExtractCorners(left_frame_fish.img_,
                                    &left_frame_fish.keypoints_);
  undistorter_left->undistortRectifyKeypoints(left_frame_fish.keypoints_,
                                              &left_keypoints_rectified);

  VIO::StatusKeypointsCV status_left_keypoints_rectified;
  undistorter_left->checkUndistortedRectifiedLeftKeypoints(
      left_frame_fish.img_,
      left_keypoints_rectified,
      &status_left_keypoints_rectified);

  // Get rectified right keypoints
  VIO::StatusKeypointsCV right_keypoints_rectified;
  fisheye_matcher->getRightKeypointsRectified(
      left_image_rectified,
      right_image_rectified,
      status_left_keypoints_rectified,
      left_undistRectCameraMatrix_fisheye.fx(),
      fisheye_camera->getBaseline(),
      &right_keypoints_rectified);

  // Check corresponding features are on epipolar lines (visually and store)
  cv::Mat undist_sidebyside = VIO::UtilsOpenCV::concatenateTwoImages(
      left_image_rectified, right_image_rectified);

  // Parse reference image -> Remove comments
  cv::Mat undist_sidebyside_ref =
      cv::imread(stereo_FLAGS_test_data_path + "sidebyside_ref_img_0.png",
                 cv::IMREAD_ANYCOLOR);

  // Test distortion with image comparison --> uncomment
  EXPECT_TRUE(VIO::UtilsOpenCV::compareCvMatsUpToTol(
      undist_sidebyside, undist_sidebyside_ref, 1e-1));
}
