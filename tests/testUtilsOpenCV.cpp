/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testUtilsOpenCV.h
 * @brief  test UtilsOpenCV
 * @author Antoni Rosinol, Luca Carlone
 */

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

#include <cmath>
#include <fstream>
#include <iostream>
#include <utility>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <opencv2/opencv.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/utils/UtilsNumerical.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

DECLARE_string(test_data_path);

using namespace gtsam;
using namespace std;
using namespace VIO;
using namespace cv;

class UtilsOpenCVFixture : public ::testing::Test {
 public:
  UtilsOpenCVFixture()
      : chess_img_path_(std::string(FLAGS_test_data_path) + "/chessboard.png"),
        real_img_path_(std::string(FLAGS_test_data_path) + "/realImage.png"),
        R_gtsam_(),
        T_gtsam_(),
        pose_gtsam_(),
        R_cvmat_(),
        T_cvmat_() {
    // clang-format off
    R_gtsam_ = gtsam::Rot3(
        pose_vector_[0], pose_vector_[1], pose_vector_[2],
        pose_vector_[4], pose_vector_[5], pose_vector_[6],
        pose_vector_[8], pose_vector_[9], pose_vector_[10]);
    T_gtsam_ = gtsam::Point3(pose_vector_[3], pose_vector_[7], pose_vector_[11]);
    pose_gtsam_ = gtsam::Pose3(R_gtsam_, T_gtsam_);
    R_cvmat_ = (cv::Mat_<double>(3, 3)
               << pose_vector_[0], pose_vector_[1], pose_vector_[2],
                  pose_vector_[4], pose_vector_[5], pose_vector_[6],
                  pose_vector_[8], pose_vector_[9], pose_vector_[10]);
    T_cvmat_ = (cv::Mat_<double>(3, 1)
               << pose_vector_[3], pose_vector_[7], pose_vector_[11]);
    // clang-format on
  }

 protected:
  virtual void SetUp() override {}
  virtual void TearDown() override {}

  pair<cv::Mat, std::vector<KeypointCV>> cvCreateChessboard(
      int square_width,
      int num_pattern_row,
      int num_pattern_col) const {
    // Decide the image dimension
    int img_width = square_width * num_pattern_col;
    int img_height = square_width * num_pattern_row;
    // Generate the chessboard image
    cv::Mat chessboard_img = cv::Mat::zeros(img_height, img_width, CV_8U);
    for (int r = 0; r < img_height; r++) {
      for (int c = 0; c < img_width; c++) {
        if (((r / square_width) + (c / square_width)) % 2 == 0) {
          chessboard_img.at<unsigned char>(r, c) = 255;
        }
      }
    }
    // Generate the corners!
    vector<KeypointCV> corners;
    corners.reserve((num_pattern_row - 1) * (num_pattern_col - 1));
    for (int pr = 1; pr < num_pattern_row; pr++) {
      for (int pc = 1; pc < num_pattern_col; pc++) {
        corners.push_back(
            KeypointCV(pc * square_width - 0.5, pr * square_width - 0.5));
      }
    }
    return std::make_pair(chessboard_img, corners);
  }

 protected:
  static constexpr double x = 1.141516;
  static constexpr double tol_ = 1e-7;

  const std::string chess_img_path_;
  const std::string real_img_path_;

  // 3D pose in vector format
  // clang-format off
  const std::vector<double> pose_vector_ =
    {0.0148655429818,  -0.999880929698,  0.00414029679422, -0.0216401454975,
     0.999557249008,   0.0149672133247,  0.025715529948,   -0.064676986768,
     -0.0257744366974, 0.00375618835797, 0.999660727178,   0.00981073058949,
     0.0,              0.0,              0.0,              1.0};
  // clang-format on

  // 3D pose in gtsam format
  gtsam::Rot3 R_gtsam_;
  gtsam::Point3 T_gtsam_;
  gtsam::Pose3 pose_gtsam_;

  // 3D rotation and translation in open cv format
  cv::Mat R_cvmat_;
  cv::Mat T_cvmat_;
};

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, CvMatCmp) {
  Mat chessboardImg;
  vector<KeypointCV> keypoints_expected;
  tie(chessboardImg, keypoints_expected) = cvCreateChessboard(30, 10, 8);
  // image is identical to itself
  EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(chessboardImg, chessboardImg));
  // but becomes different if we perturb an entry
  Mat chessboardImg2;
  chessboardImg.copyTo(chessboardImg2);
  // perturb some entry:
  chessboardImg2.at<unsigned char>(0, 1) =
      chessboardImg2.at<unsigned char>(0, 1) + 1;
  EXPECT_TRUE(
      !UtilsOpenCV::compareCvMatsUpToTol(chessboardImg, chessboardImg2));
}

/* ************************************************************************** */
TEST(testUtilsOpenCV, CvPointCmp) {
  Point2f p1(2.0, 1.5);
  // point is identical to itself
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(p1, p1));
  // but becomes different if we perturb an entry
  Point2f p2(2.0 + 1e-5, 1.5);
  EXPECT_TRUE(!UtilsOpenCV::CvPointCmp(p1, p2, 1e-7));
  // however we cannot detect differences smaller than our tolerance:
  Point2f p3(2.0 + 1e-8, 1.5);
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(p1, p3, 1e-7));
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, poseVectorToGtsamPose3) {
  // Expected pose
  gtsam::Pose3 pose_expected = pose_gtsam_;
  // Actually converted pose
  gtsam::Pose3 pose_actual = UtilsOpenCV::poseVectorToGtsamPose3(pose_vector_);
  // Comparison!
  EXPECT_TRUE(assert_equal(pose_expected, pose_actual));
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, Pose2cvmats) {
  cv::Mat R_actual, T_actual;
  std::tie(R_actual, T_actual) = UtilsOpenCV::Pose2cvmats(pose_gtsam_);

  cv::Mat R_expected = R_cvmat_, T_expected = T_cvmat_;
  // Comparison
  EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(R_expected, R_actual));
  EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(T_expected, T_actual));
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, gtsamPose3ToCvAffine3d) {
  // Expected result
  cv::Mat R_expected, T_expected;
  std::tie(R_expected, T_expected) = UtilsOpenCV::Pose2cvmats(pose_gtsam_);

  // Actual result
  cv::Affine3d affine_actual = UtilsOpenCV::gtsamPose3ToCvAffine3d(pose_gtsam_);

  // Comparison
  for (int r = 0; r < 3; r++) {
    EXPECT_NEAR(T_expected.at<double>(r, 0),
                double(affine_actual.translation()(r)),
                tol_);
    for (int c = 0; c < 3; c++) {
      EXPECT_NEAR(R_expected.at<double>(r, c),
                  double(affine_actual.rotation()(r, c)),
                  tol_);
    }
  }
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, Cvmats2pose) {
  Pose3 pose_expected = pose_gtsam_;
  Pose3 pose_actual = UtilsOpenCV::cvMatsToGtsamPose3(R_cvmat_, T_cvmat_);
  EXPECT_TRUE(assert_equal(pose_expected, pose_actual));
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, CvMatToGtsamRot) {
  Rot3 rot_expected = pose_gtsam_.rotation();
  Rot3 rot_actual = UtilsOpenCV::cvMatToGtsamRot3(R_cvmat_);
  EXPECT_TRUE(assert_equal(rot_expected, rot_actual));
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, Cvmat2rot) {
  // Expected ROT
  Rot3 Rot_expected = R_gtsam_;
  // Actual ROT
  Rot3 Rot_actual = UtilsOpenCV::cvMatToGtsamRot3(R_cvmat_);
  EXPECT_TRUE(assert_equal(Rot_expected, Rot_actual));
}

/* ************************************************************************** */
TEST(testUtilsOpenCV, Cvmat2Cal3S2) {
  // input CV Mat from MH_easy_01/cam0 (modified)
  Mat M = (cv::Mat_<double>(3, 3) << 458.654,
           0.01,
           367.215,
           0,
           457.296,
           248.375,
           0,
           0,
           1);
  // Expected output
  Cal3_S2 cal_expected(458.654, 457.296, 0.01, 367.215, 248.375);
  // Actual output
  Cal3_S2 cal_actual = UtilsOpenCV::Cvmat2Cal3_S2(M);
  EXPECT_TRUE(assert_equal(cal_expected, cal_actual));
}

/* ************************************************************************** */
TEST(testUtilsOpenCV, Cal3S2ToCvmat) {
  // Expected output
  Mat M = (cv::Mat_<double>(3, 3) << 458.654,
           0.01,
           367.215,
           0,
           457.296,
           248.375,
           0,
           0,
           1);
  // actual output
  Cal3_S2 cal_expected(458.654, 457.296, 0.01, 367.215, 248.375);
  Mat Mactual = UtilsOpenCV::Cal3_S2ToCvmat(cal_expected);
  // Actual output
  EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(M, Mactual, 1e-5));
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, Gvtrans2pose) {
  // Expected Pose
  Pose3 pose_expected = pose_gtsam_;
  // create opengv transformation!
  opengv::transformation_t RT;
  RT << pose_vector_[0], pose_vector_[1], pose_vector_[2], pose_vector_[3],
      pose_vector_[4], pose_vector_[5], pose_vector_[6], pose_vector_[7],
      pose_vector_[8], pose_vector_[9], pose_vector_[10], pose_vector_[11];
  // Actual pose
  Pose3 pose_actual = UtilsOpenCV::openGvTfToGtsamPose3(RT);
  EXPECT_TRUE(assert_equal(pose_expected, pose_actual));
}

/* ************************************************************************** */
TEST(testUtilsOpenCV, CropToSizeInside) {
  // Crop px = (700, 300)in the frame of 800w*600h. px is inside image,
  // hence CropToSize should output the same point.
  Size size(800, 600);
  Point2f px_in(700, 300);
  Point2f px_expected = px_in;
  UtilsOpenCV::cropToSize(&px_in, size);
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_in));
}

/* ************************************************************************** */
TEST(testUtilsOpenCV, CropToSizeBoundary) {
  Size size(800, 600);

  // Crop (799, 599) in the frame of 800w*600h. Should output the same point.
  Point2f px_in(799, 599);
  Point2f px_expected(799, 599);
  EXPECT_FALSE(UtilsOpenCV::cropToSize(&px_in, size));
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_in));

  // Crop (800, 600) in the frame of 800w*600h. Should output (799, 599).
  px_in = Point2f(800, 600);
  px_expected = Point2f(799, 599);
  EXPECT_TRUE(UtilsOpenCV::cropToSize(&px_in, size));
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_in));

  // Crop (0, 0) in the frame of 800w*600h. Should output the same point.
  px_in = Point2f(0, 0);
  px_expected = Point2f(0, 0);
  EXPECT_FALSE(UtilsOpenCV::cropToSize(&px_in, size));
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_in));

  // Crop (-1, -1) in the frame of 800w*600h. Should output (0, 0)
  px_in = Point2f(-1, -1);
  px_expected = Point2f(0, 0);
  EXPECT_TRUE(UtilsOpenCV::cropToSize(&px_in, size));
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_in));
}

/* ************************************************************************** */
TEST(testUtilsOpenCV, CropToSize) {
  // 1. Check that point outside boundary is correctly cropped.
  Size size(800, 600);
  // Crop (1000, 700) in the frame of 800w*600h. Should output(800, 600)
  Point2f px_in(1000, 700);
  Point2f px_expected(799, 599);
  EXPECT_TRUE(UtilsOpenCV::cropToSize(&px_in, size));
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_in));

  // Crop (-100, -200) in the frame of 800w*600h. Should output(-100, -200)
  px_in = Point2f(-100, -200);
  px_expected = Point2f(0, 0);
  EXPECT_TRUE(UtilsOpenCV::cropToSize(&px_in, size));
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_in));

  // 2. Check that point inside boundary remains the same.
  // Crop (700.3f, 399.5f) in the frame of 800w*600h.
  // Should output (700, 400);
  px_in = Point2f(700.3f, 399.5f);
  px_expected = Point2f(700.0f, 400.0f);
  EXPECT_FALSE(UtilsOpenCV::roundAndCropToSize(&px_in, size));
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_in));

  // Crop (699.50001f, 300.499f) in the frame of 800w*600h. Should output(700,
  // 300)
  px_in = Point2f(699.50001f, 300.499f);
  px_expected = Point2f(700, 300);
  EXPECT_FALSE(UtilsOpenCV::roundAndCropToSize(&px_in, size));
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_in));
}

/* ************************************************************************** */
TEST(testUtilsOpenCV, RoundAndCropToSizeBoundary) {
  // Crop (799.5, 599.5) in the frame of 800w*600h.
  // Should output (799, 599);
  Size size(800, 600);
  Point2f px_in(799.5f, 599.5f);
  Point2f px_expected(799, 599);
  EXPECT_TRUE(UtilsOpenCV::roundAndCropToSize(&px_in, size));
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_in));

  // Crop (-0.499f, -0.499f) in the frame of 800w*600h. Should output(0, 0)
  px_in = Point2f(-0.499f, -0.499f);
  px_expected = Point2f(0.0f, 0.0f);
  EXPECT_FALSE(UtilsOpenCV::roundAndCropToSize(&px_in, size));
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_in));
}

/* ************************************************************************** */
TEST(testUtilsOpenCV, RoundAndCropToSizeOutside) {
  // Crop (1000.4f, 700.4f) in the frame of 800w*600h.
  // Should output (799, 599);
  Size size(800, 600);
  Point2f px_in(1000.4f, 700.4f);
  Point2f px_expected(799, 599);
  EXPECT_TRUE(UtilsOpenCV::roundAndCropToSize(&px_in, size));
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_in));

  // Crop (-1000f, -800) in the frame of 800w*600h. Should output(0, 0)
  px_in = Point2f(-1000, -800);
  px_expected = Point2f(0, 0);
  EXPECT_TRUE(UtilsOpenCV::roundAndCropToSize(&px_in, size));
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_in));
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, ExtractCornersChessboard) {
  // Generate the chessboard to extract!
  Mat chessboardImg;
  vector<KeypointCV> keypoints_expected;
  tie(chessboardImg, keypoints_expected) = cvCreateChessboard(30, 10, 8);

  // Extract the corners!
  vector<KeypointCV> keypoints_actual;
  UtilsOpenCV::ExtractCorners(chessboardImg, &keypoints_actual);

  EXPECT_LE(keypoints_actual.size(), 100);
  EXPECT_GE(keypoints_actual.size(), keypoints_expected.size());

  // Assert that all obvious corners are extracted!
  for (int ptid = 0; ptid < keypoints_expected.size(); ptid++) {
    // Find the closest one!
    double minDist = 10000;
    for (int i = 0; i < keypoints_actual.size(); i++) {
      int dist = abs(keypoints_actual[ptid].x - keypoints_actual[i].x) +
                 abs(keypoints_actual[ptid].y - keypoints_actual[i].y);
      if (dist < minDist) {
        minDist = dist;
      }
    }
    // Assert that minDist < 1e-2;
    EXPECT_LT(minDist, 1e-2);
  }
  imwrite("chessboard.png", chessboardImg);
}

/* ************************************************************************** */
TEST(testUtilsOpenCV, ExtractCornersWhiteWall) {
  // Given an image of white wall, no corners should be extracted!!
  Mat whitewallImg = Mat::zeros(800, 600, CV_8U);

  vector<KeypointCV> keypoints_actual;
  EXPECT_NO_THROW(UtilsOpenCV::ExtractCorners(whitewallImg, &keypoints_actual));

  // Assert that no corners are extracted!
  EXPECT_EQ(keypoints_actual.size(), 0);
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, RoundUnit3) {
  {
    Unit3 a(0, 1, 0.9);
    Unit3 a_actual = UtilsOpenCV::RoundUnit3(a);
    Unit3 a_expected(0, 1, 0);
    EXPECT_TRUE(assert_equal(a_expected, a_actual, tol_));
  }
  {
    Unit3 a(0, -1, 0.9);
    Unit3 a_actual = UtilsOpenCV::RoundUnit3(a);
    Unit3 a_expected(0, -1, 0);
    EXPECT_TRUE(assert_equal(a_expected, a_actual, tol_));
  }
  {
    Unit3 a(0, -1, -0.9);
    Unit3 a_actual = UtilsOpenCV::RoundUnit3(a);
    Unit3 a_expected(0, -1, 0);
    EXPECT_TRUE(assert_equal(a_expected, a_actual, tol_));
  }
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, computeRTErrorsidentical) {
  double rot_error_expected = 0, tran_error_expected = 0;

  double rot_error_actual, tran_error_actual;
  tie(rot_error_actual, tran_error_actual) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(pose_gtsam_,
                                                       pose_gtsam_);

  EXPECT_NEAR(rot_error_expected, rot_error_actual, tol_);
  EXPECT_NEAR(tran_error_expected, tran_error_actual, tol_);
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, computeRTErrors) {
  gtsam::Vector3 rot_vec(0.1, 0.2, 0.3);
  gtsam::Rot3 R_delta = gtsam::Rot3::Rodrigues(rot_vec);
  gtsam::Rot3 R_new = R_gtsam_.compose(R_delta);
  gtsam::Point3 T_delta(0.1, -0.2, -0.4);
  gtsam::Point3 T_new = T_gtsam_ + T_delta;

  double rot_error_expected = 0.37416573867;
  double tran_error_expected = 0.45825756949;

  double rot_error_actual, tran_error_actual;
  gtsam::Pose3 pose_new(R_new, T_new);
  std::tie(rot_error_actual, tran_error_actual) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(pose_gtsam_, pose_new);

  EXPECT_NEAR(rot_error_expected, rot_error_actual, tol_);
  EXPECT_NEAR(tran_error_expected, tran_error_actual, tol_);
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, computeRTErrors_upToScale) {
  Vector3 rotVec(0.1, 0.2, 0.3);
  Rot3 R_delta = Rot3::Rodrigues(rotVec);
  Rot3 R_new = R_gtsam_.compose(R_delta);
  bool upToScale = true;

  {
    // translation of new pose is the same of the expected, but with unit norm
    double normT = pose_gtsam_.translation().norm();
    Point3 T_new = Point3(pose_gtsam_.translation() / normT);

    double rot_error_expected = 0.37416573867;
    double tran_error_expected = 0.0;

    double rot_error_actual, tran_error_actual;
    Pose3 pose_new(R_new, T_new);
    tie(rot_error_actual, tran_error_actual) =
        UtilsOpenCV::ComputeRotationAndTranslationErrors(
            pose_gtsam_, pose_new, upToScale);

    EXPECT_NEAR(rot_error_expected, rot_error_actual, tol_);
    EXPECT_NEAR(tran_error_expected, tran_error_actual, tol_);
  }

  {
    // translation of new pose is the same of the expected, but with some other
    // norm
    double normT = pose_gtsam_.translation().norm();
    Point3 T_new = 10 * Point3(pose_gtsam_.translation() / normT);

    double rot_error_expected = 0.37416573867;
    double tran_error_expected = 0.0;

    double rot_error_actual, tran_error_actual;
    Pose3 pose_new(R_new, T_new);
    tie(rot_error_actual, tran_error_actual) =
        UtilsOpenCV::ComputeRotationAndTranslationErrors(
            pose_gtsam_, pose_new, upToScale);

    EXPECT_NEAR(rot_error_expected, rot_error_actual, tol_);
    EXPECT_NEAR(tran_error_expected, tran_error_actual, tol_);
  }
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, ReadAndConvertToGrayScale) {
  // original image is already gray, hence it remains the same
  {
    cv::Mat chessboardImg;
    std::vector<KeypointCV> keypoints_expected;
    std::tie(chessboardImg, keypoints_expected) = cvCreateChessboard(30, 10, 8);
    cv::Mat chessboardImgGray =
        UtilsOpenCV::ReadAndConvertToGrayScale("chessboard.png");
    EXPECT_EQ(chessboardImgGray.channels(), 1);
    EXPECT_TRUE(
        UtilsOpenCV::compareCvMatsUpToTol(chessboardImg, chessboardImgGray));
  }
  // original image is in color and it is converted to gray
  {
    cv::Mat img_gray = UtilsOpenCV::ReadAndConvertToGrayScale(
        FLAGS_test_data_path + "/lena.png");
    cv::imwrite("lenaGrayScale.png", img_gray);
    EXPECT_EQ(img_gray.channels(), 1);
    // we read gray image we just wrote and make sure it is ok
    cv::Mat img_gray_written = imread("lenaGrayScale.png", IMREAD_ANYCOLOR);
    EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(img_gray, img_gray_written));
  }
  // original image is already gray, hence it remains the same
  {
    cv::Mat img =
        cv::imread(FLAGS_test_data_path + "/testImage.png", IMREAD_ANYCOLOR);
    cv::Mat img_gray = UtilsOpenCV::ReadAndConvertToGrayScale(
        FLAGS_test_data_path + "/testImage.png");
    EXPECT_EQ(img_gray.channels(), 1);
    EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(img, img_gray));
  }
}

/* ************************************************************************** */
TEST(testUtilsOpenCV, covariancebvx2xvb) {
  srand(1000000);

  // Create random covariance
  gtsam::Matrix cov_xx = gtsam::Matrix::Random(6, 6);
  gtsam::Matrix cov_xv = gtsam::Matrix::Random(6, 3);
  gtsam::Matrix cov_xb = gtsam::Matrix::Random(6, 6);
  gtsam::Matrix cov_vv = gtsam::Matrix::Random(3, 3);
  gtsam::Matrix cov_vb = gtsam::Matrix::Random(3, 6);
  gtsam::Matrix cov_bb = gtsam::Matrix::Random(6, 6);

  gtsam::Matrix cov_vx = cov_xv.transpose();
  gtsam::Matrix cov_bx = cov_xb.transpose();
  gtsam::Matrix cov_bv = cov_vb.transpose();

  gtsam::Matrix expected_cov_xvb = gtsam::Matrix(15, 15);
  expected_cov_xvb.block<6, 15>(0, 0) << cov_xx, cov_xv, cov_xb;
  expected_cov_xvb.block<3, 15>(6, 0) << cov_vx, cov_vv, cov_vb;
  expected_cov_xvb.block<6, 15>(9, 0) << cov_bx, cov_bv, cov_bb;

  gtsam::Matrix cov_bvx = gtsam::Matrix(15, 15);
  cov_bvx.block<6, 15>(0, 0) << cov_bb, cov_bv, cov_bx;
  cov_bvx.block<3, 15>(6, 0) << cov_vb, cov_vv, cov_vx;
  cov_bvx.block<6, 15>(9, 0) << cov_xb, cov_xv, cov_xx;

  gtsam::Matrix cov_actual_xvb = UtilsOpenCV::Covariance_bvx2xvb(cov_bvx);
  EXPECT_TRUE(assert_equal(expected_cov_xvb, cov_actual_xvb));
}

/* ************************************************************************** */
TEST_F(UtilsOpenCVFixture, ImageLaplacian) {
  cv::Mat chessboardImg;
  std::vector<KeypointCV> notUsed;
  std::tie(chessboardImg, notUsed) = cvCreateChessboard(30, 10, 8);
  cv::Mat actual = UtilsOpenCV::ImageLaplacian(chessboardImg);
  // cv::imshow("actual",actual);
  // cv::waitKey(100);
}
