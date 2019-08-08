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
 * @author Luca Carlone
 */

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

#include <cmath>
#include <fstream>
#include <iostream>
#include <utility>

#include "UtilsOpenCV.h"

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <opencv2/opencv.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

DECLARE_string(test_data_path);

using namespace gtsam;
using namespace std;
using namespace VIO;
using namespace cv;

/* ************************************************************************* */
// Data
static const double x = 1.141516;
static const double tol = 1e-7;

// 3D pose in array format
static const double poseArray[] = {0.0148655429818,
                                   -0.999880929698,
                                   0.00414029679422,
                                   -0.0216401454975,
                                   0.999557249008,
                                   0.0149672133247,
                                   0.025715529948,
                                   -0.064676986768,
                                   -0.0257744366974,
                                   0.00375618835797,
                                   0.999660727178,
                                   0.00981073058949,
                                   0.0,
                                   0.0,
                                   0.0,
                                   1.0};
// 3D pose in array format vector format
static const vector<double> poseVec(poseArray, poseArray + 16);

// 3D pose in gtsam format
static const Rot3 R_gtsam(poseArray[0], poseArray[1], poseArray[2],
                          poseArray[4], poseArray[5], poseArray[6],
                          poseArray[8], poseArray[9], poseArray[10]);
static const Point3 T_gtsam(poseArray[3], poseArray[7], poseArray[11]);
static const Pose3 pose_gtsam(R_gtsam, T_gtsam);

// 3D rotation and translation in open cv format
static const Mat R_cvmat =
    (cv::Mat_<double>(3, 3) << poseArray[0], poseArray[1], poseArray[2],
     poseArray[4], poseArray[5], poseArray[6], poseArray[8], poseArray[9],
     poseArray[10]);
static const Mat T_cvmat =
    (cv::Mat_<double>(3, 1) << poseArray[3], poseArray[7], poseArray[11]);

/* ------------------------------------------------------------------------ */
// helper function
static pair<cv::Mat, vector<cv::Point2f>> cvCreateChessboard(
    int squareWidth, int numPatternRow, int numPatternCol) {
  // Decide the image dimension
  int imgWidth = squareWidth * numPatternCol;
  int imgHeight = squareWidth * numPatternRow;

  // Generate the chessboard image
  Mat chessboardImage = Mat::zeros(imgHeight, imgWidth, CV_8U);
  for (int r = 0; r < imgHeight; r++) {
    for (int c = 0; c < imgWidth; c++) {
      if (((r / squareWidth) + (c / squareWidth)) % 2 == 0) {
        chessboardImage.at<unsigned char>(r, c) = 255;
      }
    }
  }
  // Generate the corners!
  vector<cv::Point2f> corners;
  corners.reserve((numPatternRow - 1) * (numPatternCol - 1));
  for (int pr = 1; pr < numPatternRow; pr++) {
    for (int pc = 1; pc < numPatternCol; pc++) {
      corners.push_back(
          cv::Point2f(pc * squareWidth - 0.5, pr * squareWidth - 0.5));
    }
  }
  return make_pair(chessboardImage, corners);
}

/* ************************************************************************* */
TEST(testUtils, OpenFile) {
  ofstream outputFile;
  UtilsOpenCV::OpenFile("tmp.txt", &outputFile);
  EXPECT_TRUE(outputFile.is_open());
  outputFile.close();
  EXPECT_TRUE(!outputFile.is_open());
}

/* ************************************************************************* */
TEST(testUtils, CvMatCmp) {
  Mat chessboardImg;
  vector<cv::Point2f> keypoints_expected;
  tie(chessboardImg, keypoints_expected) = cvCreateChessboard(30, 10, 8);
  // image is identical to itself
  EXPECT_TRUE(UtilsOpenCV::CvMatCmp(chessboardImg, chessboardImg));
  // but becomes different if we perturb an entry
  Mat chessboardImg2;
  chessboardImg.copyTo(chessboardImg2);
  // perturb some entry:
  chessboardImg2.at<unsigned char>(0, 1) =
      chessboardImg2.at<unsigned char>(0, 1) + 1;
  EXPECT_TRUE(!UtilsOpenCV::CvMatCmp(chessboardImg, chessboardImg2));
}

/* ************************************************************************* */
TEST(testUtils, CvPointCmp) {
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

/* ************************************************************************* */
TEST(testUtils, Vec2pose) {
  // Expected pose
  Pose3 pose_expected = pose_gtsam;
  // Actually converted pose
  Pose3 pose_actual = UtilsOpenCV::Vec2pose(poseVec, 4, 4);
  // Comparison!
  EXPECT_TRUE(assert_equal(pose_expected, pose_actual));
}

/* ************************************************************************* */
TEST(testUtils, Pose2cvmats) {
  Mat R_actual, T_actual;
  tie(R_actual, T_actual) = UtilsOpenCV::Pose2cvmats(pose_gtsam);

  Mat R_expected = R_cvmat, T_expected = T_cvmat;
  // Comparison
  EXPECT_TRUE(UtilsOpenCV::CvMatCmp(R_expected, R_actual));
  EXPECT_TRUE(UtilsOpenCV::CvMatCmp(T_expected, T_actual));
}

/* ************************************************************************* */
TEST(testUtils, Pose2Affine3d) {
  // expected result
  Mat R_expected, T_expected;
  tie(R_expected, T_expected) = UtilsOpenCV::Pose2cvmats(pose_gtsam);

  // actual result
  cv::Affine3f affineActual = UtilsOpenCV::Pose2Affine3f(pose_gtsam);

  // Comparison
  for (int r = 0; r < 3; r++) {
    EXPECT_NEAR(T_expected.at<double>(r, 0),
                double(affineActual.translation()(r)), tol);
    for (int c = 0; c < 3; c++) {
      EXPECT_NEAR(R_expected.at<double>(r, c),
                  double(affineActual.rotation()(r, c)), tol);
    }
  }
}

/* ************************************************************************* */
TEST(testUtils, Cvmats2pose) {
  Pose3 pose_expected = pose_gtsam;
  Pose3 pose_actual = UtilsOpenCV::Cvmats2pose(R_cvmat, T_cvmat);
  EXPECT_TRUE(assert_equal(pose_expected, pose_actual));
}

/* ************************************************************************* */
TEST(testUtils, Cvmat2rot) {
  // Expected ROT
  Rot3 Rot_expected = R_gtsam;
  // Actual ROT
  Rot3 Rot_actual = UtilsOpenCV::Cvmat2rot(R_cvmat);
  EXPECT_TRUE(assert_equal(Rot_expected, Rot_actual));
}

/* ************************************************************************* */
TEST(testUtils, Cvmat2Cal3_S2) {
  // input CV Mat from MH_easy_01/cam0 (modified)
  Mat M = (cv::Mat_<double>(3, 3) << 458.654, 0.01, 367.215, 0, 457.296,
           248.375, 0, 0, 1);
  // Expected output
  Cal3_S2 cal_expected(458.654, 457.296, 0.01, 367.215, 248.375);
  // Actual output
  Cal3_S2 cal_actual = UtilsOpenCV::Cvmat2Cal3_S2(M);
  EXPECT_TRUE(assert_equal(cal_expected, cal_actual));
}

/* ************************************************************************* */
TEST(testUtils, Cal3_S2ToCvmat) {
  // Expected output
  Mat M = (cv::Mat_<double>(3, 3) << 458.654, 0.01, 367.215, 0, 457.296,
           248.375, 0, 0, 1);
  // actual output
  Cal3_S2 cal_expected(458.654, 457.296, 0.01, 367.215, 248.375);
  Mat Mactual = UtilsOpenCV::Cal3_S2ToCvmat(cal_expected);
  // Actual output
  EXPECT_TRUE(UtilsOpenCV::CvMatCmp(M, Mactual, 1e-5));
}

/* ************************************************************************* */
TEST(testUtils, Gvtrans2pose) {
  // Expected Pose
  Pose3 pose_expected = pose_gtsam;
  // create opengv transformation!
  opengv::transformation_t RT;
  RT << poseArray[0], poseArray[1], poseArray[2], poseArray[3], poseArray[4],
      poseArray[5], poseArray[6], poseArray[7], poseArray[8], poseArray[9],
      poseArray[10], poseArray[11];
  // Actual pose
  Pose3 pose_actual = UtilsOpenCV::Gvtrans2pose(RT);
  EXPECT_TRUE(assert_equal(pose_expected, pose_actual));
}

/* ************************************************************************* */
TEST(testUtils, CropToSizeInside) {
  // Crop px = (700, 300)in the frame of 800w*600h. px is inside image,
  // hence CropToSize should output the same point.
  Size size(800, 600);
  Point2f px_in(700, 300);
  Point2f px_expected = px_in;
  Point2f px_actual = UtilsOpenCV::CropToSize(px_in, size);
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_actual));
}

/* ************************************************************************* */
TEST(testUtils, CropToSizeBoundary) {
  Size size(800, 600);

  // Crop (799, 599) in the frame of 800w*600h. Should output the same point.
  Point2f px_in(799, 599);
  Point2f px_expected(799, 599);
  Point2f px_actual = UtilsOpenCV::CropToSize(px_in, size);
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_actual));

  // Crop (800, 600) in the frame of 800w*600h. Should output (799, 599).
  px_in = Point2f(800, 600);
  px_expected = Point2f(799, 599);
  px_actual = UtilsOpenCV::CropToSize(px_in, size);
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_actual));

  // Crop (0, 0) in the frame of 800w*600h. Should output the same point.
  px_in = Point2f(0, 0);
  px_expected = Point2f(0, 0);
  px_actual = UtilsOpenCV::CropToSize(px_in, size);
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_actual));

  // Crop (-1, -1) in the frame of 800w*600h. Should output (0, 0)
  px_in = Point2f(-1, -1);
  px_expected = Point2f(0, 0);
  px_actual = UtilsOpenCV::CropToSize(px_in, size);
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_actual));
}

/* ************************************************************************* */
TEST(testUtils, CropToSize) {
  {  // check that point outside boundary is correctly cropped
    Size size(800, 600);
    // Crop (1000, 700) in the frame of 800w*600h. Should output(800, 600)
    Point2f px_in(1000, 700);
    Point2f px_expected(799, 599);
    Point2f px_actual = UtilsOpenCV::CropToSize(px_in, size);
    EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_actual));

    // Crop (-100, -200) in the frame of 800w*600h. Should output(-100, -200)
    px_in = Point2f(-100, -200);
    px_expected = Point2f(0, 0);
    px_actual = UtilsOpenCV::CropToSize(px_in, size);
    EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_actual));
  }
  {  // check that point inside boundary remains the same
    // Crop (700.3f, 399.5f) in the frame of 800w*600h.
    // Should output (700, 400);
    Size size(800, 600);
    Point2f px_in(700.3f, 399.5f);
    Point2f px_expected(700.0f, 400.0f);
    Point2f px_actual = UtilsOpenCV::RoundAndCropToSize(px_in, size);
    EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_actual));

    // Crop (699.50001f, 300.499f) in the frame of 800w*600h. Should output(700,
    // 300)
    px_in = Point2f(699.50001f, 300.499f);
    px_expected = Point2f(700, 300);
    px_actual = UtilsOpenCV::RoundAndCropToSize(px_in, size);
    EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_actual));
  }
}

/* ************************************************************************* */
TEST(testUtils, RoundAndCropToSizeBoundary) {
  // Crop (799.5, 599.5) in the frame of 800w*600h.
  // Should output (799, 599);
  Size size(800, 600);
  Point2f px_in(799.5f, 599.5f);
  Point2f px_expected(799, 599);
  Point2f px_actual = UtilsOpenCV::RoundAndCropToSize(px_in, size);
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_actual));

  // Crop (-0.499f, -0.499f) in the frame of 800w*600h. Should output(0, 0)
  px_in = Point2f(-0.499f, -0.499f);
  px_expected = Point2f(0, 0);
  px_actual = UtilsOpenCV::RoundAndCropToSize(px_in, size);
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_actual));
}

/* ************************************************************************* */
TEST(testUtils, RoundAndCropToSizeOutside) {
  // Crop (1000.4f, 700.4f) in the frame of 800w*600h.
  // Should output (799, 599);
  Size size(800, 600);
  Point2f px_in(1000.4f, 700.4f);
  Point2f px_expected(799, 599);
  Point2f px_actual = UtilsOpenCV::RoundAndCropToSize(px_in, size);
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_actual));

  // Crop (-1000f, -800) in the frame of 800w*600h. Should output(0, 0)
  px_in = Point2f(-1000, -800);
  px_expected = Point2f(0, 0);
  px_actual = UtilsOpenCV::RoundAndCropToSize(px_in, size);
  EXPECT_TRUE(UtilsOpenCV::CvPointCmp(px_expected, px_actual));
}

/* ************************************************************************* */
TEST(testUtils, ExtractCornersChessboard) {
  // Generate the chessboard to extract!
  Mat chessboardImg;
  vector<cv::Point2f> keypoints_expected;
  tie(chessboardImg, keypoints_expected) = cvCreateChessboard(30, 10, 8);

  // Extract the corners!
  vector<cv::Point2f> keypoints_actual;
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

/* ************************************************************************* */
TEST(testUtils, ExtractCornersWhiteWall) {
  // Given an image of white wall, no corners should be extracted!!
  Mat whitewallImg = Mat::zeros(800, 600, CV_8U);

  vector<cv::Point2f> keypoints_actual;
  EXPECT_NO_THROW(UtilsOpenCV::ExtractCorners(whitewallImg, &keypoints_actual));

  // Assert that no corners are extracted!
  EXPECT_EQ(keypoints_actual.size(), 0);
}

/* ************************************************************************* */
TEST(testUtils, RoundToDigit_2digits) {
  double x_expected = 1.14;  // rounded to the 2nd decimal digit
  double x_actual = UtilsOpenCV::RoundToDigit(x);
  EXPECT_NEAR(x_expected, x_actual, tol);
}

/* ************************************************************************* */
TEST(testUtils, RoundUnit3) {
  {
    Unit3 a(0, 1, 0.9);
    Unit3 a_actual = UtilsOpenCV::RoundUnit3(a);
    Unit3 a_expected(0, 1, 0);
    EXPECT_TRUE(assert_equal(a_expected, a_actual, tol));
  }
  {
    Unit3 a(0, -1, 0.9);
    Unit3 a_actual = UtilsOpenCV::RoundUnit3(a);
    Unit3 a_expected(0, -1, 0);
    EXPECT_TRUE(assert_equal(a_expected, a_actual, tol));
  }
  {
    Unit3 a(0, -1, -0.9);
    Unit3 a_actual = UtilsOpenCV::RoundUnit3(a);
    Unit3 a_expected(0, -1, 0);
    EXPECT_TRUE(assert_equal(a_expected, a_actual, tol));
  }
}

/* ************************************************************************* */
TEST(testUtils, RoundToDigit_3digits) {
  double x_expected = 1.142;  // rounded to the 3rd decimal digit
  double x_actual = UtilsOpenCV::RoundToDigit(x, 3);
  EXPECT_NEAR(x_expected, x_actual, tol);
}

/* ************************************************************************* */
TEST(testUtils, RoundToDigit_neg2digits) {
  double x_expected = -1.14;  // rounded to the 2nd decimal digit
  double x_actual = UtilsOpenCV::RoundToDigit(-x, 2);
  EXPECT_NEAR(x_expected, x_actual, tol);
}

/* ************************************************************************* */
TEST(testUtils, RoundToDigit_neg3digits) {
  double x_expected = -1.142;  // rounded to the 3rd decimal digit!
  double x_actual = UtilsOpenCV::RoundToDigit(-x, 3);
  EXPECT_NEAR(x_expected, x_actual, tol);
}

/* ************************************************************************* */
TEST(testUtils, To_string_with_precision_pos4digits) {
  string str_expected("1.142");
  string str_actual = UtilsOpenCV::To_string_with_precision(x, 4);
  EXPECT_EQ(str_expected.compare(str_actual), 0);
}

/* ************************************************************************* */
TEST(testUtils, To_string_with_precision_neg3digits) {
  string str_expected("-1.14");
  string str_actual = UtilsOpenCV::To_string_with_precision(-x, 3);
  EXPECT_EQ(str_expected.compare(str_actual), 0);
}

/* ************************************************************************* */
TEST(testUtils, NsecToSec) {
  int64_t timestamp = 12345678;
  double sec_expected = 0.012345678;
  double sec_actual = UtilsOpenCV::NsecToSec(timestamp);
  EXPECT_NEAR(sec_expected, sec_actual, tol);
}

/* ************************************************************************* */
TEST(testUtils, GetTimeInSeconds) {
  // I have no idea how to write test for it...
  SUCCEED();
}

/* ************************************************************************* */
TEST(testUtils, computeRTErrors_identical) {
  double rot_error_expected = 0, tran_error_expected = 0;

  double rot_error_actual, tran_error_actual;
  tie(rot_error_actual, tran_error_actual) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(pose_gtsam, pose_gtsam);

  EXPECT_NEAR(rot_error_expected, rot_error_actual, tol);
  EXPECT_NEAR(tran_error_expected, tran_error_actual, tol);
}

/* ************************************************************************* */
TEST(testUtils, computeRTErrors) {
  Vector3 rotVec(0.1, 0.2, 0.3);
  Rot3 R_delta = Rot3::Rodrigues(rotVec);
  Rot3 R_new = R_gtsam.compose(R_delta);
  Point3 T_delta(0.1, -0.2, -0.4);
  Point3 T_new = T_gtsam + T_delta;

  double rot_error_expected = 0.37416573867;
  double tran_error_expected = 0.45825756949;

  double rot_error_actual, tran_error_actual;
  Pose3 pose_new(R_new, T_new);
  tie(rot_error_actual, tran_error_actual) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(pose_gtsam, pose_new);

  EXPECT_NEAR(rot_error_expected, rot_error_actual, tol);
  EXPECT_NEAR(tran_error_expected, tran_error_actual, tol);
}

/* ************************************************************************* */
TEST(testUtils, computeRTErrors_upToScale) {
  Vector3 rotVec(0.1, 0.2, 0.3);
  Rot3 R_delta = Rot3::Rodrigues(rotVec);
  Rot3 R_new = R_gtsam.compose(R_delta);
  bool upToScale = true;

  {
    // translation of new pose is the same of the expected, but with unit norm
    double normT = pose_gtsam.translation().vector().norm();
    Point3 T_new = Point3(pose_gtsam.translation().vector() / normT);

    double rot_error_expected = 0.37416573867;
    double tran_error_expected = 0.0;

    double rot_error_actual, tran_error_actual;
    Pose3 pose_new(R_new, T_new);
    tie(rot_error_actual, tran_error_actual) =
        UtilsOpenCV::ComputeRotationAndTranslationErrors(pose_gtsam, pose_new,
                                                         upToScale);

    EXPECT_NEAR(rot_error_expected, rot_error_actual, tol);
    EXPECT_NEAR(tran_error_expected, tran_error_actual, tol);
  }

  {
    // translation of new pose is the same of the expected, but with some other
    // norm
    double normT = pose_gtsam.translation().vector().norm();
    Point3 T_new = 10 * Point3(pose_gtsam.translation().vector() / normT);

    double rot_error_expected = 0.37416573867;
    double tran_error_expected = 0.0;

    double rot_error_actual, tran_error_actual;
    Pose3 pose_new(R_new, T_new);
    tie(rot_error_actual, tran_error_actual) =
        UtilsOpenCV::ComputeRotationAndTranslationErrors(pose_gtsam, pose_new,
                                                         upToScale);

    EXPECT_NEAR(rot_error_expected, rot_error_actual, tol);
    EXPECT_NEAR(tran_error_expected, tran_error_actual, tol);
  }
}

/* ************************************************************************* */
TEST(testUtils, ReadAndConvertToGrayScale) {
  // original image is already gray, hence it remains the same
  {
    Mat chessboardImg;
    vector<cv::Point2f> keypoints_expected;
    tie(chessboardImg, keypoints_expected) = cvCreateChessboard(30, 10, 8);
    Mat chessboardImgGray =
        UtilsOpenCV::ReadAndConvertToGrayScale("chessboard.png");
    EXPECT_EQ(chessboardImgGray.channels(), 1);
    EXPECT_TRUE(UtilsOpenCV::CvMatCmp(chessboardImg, chessboardImgGray));
  }
  // original image is in color and it is converted to gray
  {
    Mat imageGray = UtilsOpenCV::ReadAndConvertToGrayScale(
        string(FLAGS_test_data_path) + "lena.png");
    imwrite("lenaGrayScale.png", imageGray);
    EXPECT_EQ(imageGray.channels(), 1);
    // we read gray image we just wrote and make sure it is ok
    Mat imgGrayWritten = imread("lenaGrayScale.png", IMREAD_ANYCOLOR);
    EXPECT_TRUE(UtilsOpenCV::CvMatCmp(imageGray, imgGrayWritten));
  }
  // original image is already gray, hence it remains the same
  {
    Mat img =
        imread(string(FLAGS_test_data_path) + "testImage.png", IMREAD_ANYCOLOR);
    Mat imageGray = UtilsOpenCV::ReadAndConvertToGrayScale(
        string(FLAGS_test_data_path) + "testImage.png");
    EXPECT_EQ(imageGray.channels(), 1);
    EXPECT_TRUE(UtilsOpenCV::CvMatCmp(img, imageGray));
  }
}
// helper function
Matrix createRandomMatrix(const int nrRows, const int nrCols) {
  Matrix mat = Matrix::Zero(nrRows, nrCols);
  for (size_t i = 0; i < nrRows; i++) {
    for (size_t j = 0; j < nrCols; j++) {
      mat(i, j) = rand() % 10 + 1;  // random number between 1 and 10
    }
  }
  return mat;
}
/* ************************************************************************* */
TEST(testUtils, covariance_bvx2xvb) {
  srand(1000000);

  // create random covariance
  Matrix cov_xx = createRandomMatrix(6, 6);
  Matrix cov_xv = createRandomMatrix(6, 3);
  Matrix cov_xb = createRandomMatrix(6, 6);
  Matrix cov_vv = createRandomMatrix(3, 3);
  Matrix cov_vb = createRandomMatrix(3, 6);
  Matrix cov_bb = createRandomMatrix(6, 6);

  Matrix cov_vx = cov_xv.transpose();
  Matrix cov_bx = cov_xb.transpose();
  Matrix cov_bv = cov_vb.transpose();

  Matrix expected_cov_xvb = Matrix(15, 15);
  expected_cov_xvb.block<6, 15>(0, 0) << cov_xx, cov_xv, cov_xb;
  expected_cov_xvb.block<3, 15>(6, 0) << cov_vx, cov_vv, cov_vb;
  expected_cov_xvb.block<6, 15>(9, 0) << cov_bx, cov_bv, cov_bb;

  Matrix cov_bvx = Matrix(15, 15);
  cov_bvx.block<6, 15>(0, 0) << cov_bb, cov_bv, cov_bx;
  cov_bvx.block<3, 15>(6, 0) << cov_vb, cov_vv, cov_vx;
  cov_bvx.block<6, 15>(9, 0) << cov_xb, cov_xv, cov_xx;

  Matrix cov_actual_xvb = UtilsOpenCV::Covariance_bvx2xvb(cov_bvx);
  EXPECT_TRUE(assert_equal(expected_cov_xvb, cov_actual_xvb));
}

static const string chessboardImgName =
    string(FLAGS_test_data_path) + "/chessboard.png";
static const string realImgName =
    string(FLAGS_test_data_path) + "/realImage.png";

TEST(UtilsOpenCV, DISABLED_ExtractCornersChessboard) {
  Mat img = UtilsOpenCV::ReadAndConvertToGrayScale(chessboardImgName);
  vector<cv::Point2f> actualCorners, actualCorners2;
  vector<double> actualScores;
  std::pair<std::vector<cv::Point2f>, std::vector<double>> corners_with_scores;
  UtilsOpenCV::MyGoodFeaturesToTrackSubPix(img, 100, 0.01, 10, Mat(), 3, false,
                                           0.04, &corners_with_scores);

  UtilsOpenCV::ExtractCorners(img, &actualCorners2);

  int numCorners_expected = 7 * 9;
  EXPECT_NEAR(numCorners_expected, actualCorners.size(), 1e-3);
  EXPECT_NEAR(numCorners_expected, actualCorners2.size(), 1e-3);
  EXPECT_NEAR(actualCorners.size(), actualScores.size(), 1e-3);

  for (size_t i = 0; i < actualCorners2.size(); ++i) {
    EXPECT_NEAR(actualCorners.at(i).x, actualCorners2.at(i).x, 1e-3);
    EXPECT_NEAR(actualCorners.at(i).y, actualCorners2.at(i).y, 1e-3);
    EXPECT_NEAR(0.333333, actualScores.at(i), 1e-3);
  }
}

TEST(UtilsOpenCV, ExtractCornersImage) {
  Mat img = UtilsOpenCV::ReadAndConvertToGrayScale(realImgName);

  std::pair<std::vector<cv::Point2f>, std::vector<double>> corners_with_scores;
  UtilsOpenCV::MyGoodFeaturesToTrackSubPix(img, 100, 0.01, 10, Mat(), 3, false,
                                           0.04, &corners_with_scores);

  vector<cv::Point2f> actualCorners2;
  UtilsOpenCV::ExtractCorners(img, &actualCorners2);

  EXPECT_NEAR(corners_with_scores.first.size(), actualCorners2.size(), 1e-3);
  EXPECT_NEAR(corners_with_scores.first.size(),
              corners_with_scores.second.size(), 1e-3);

  for (size_t i = 0; i < actualCorners2.size(); ++i) {
    EXPECT_NEAR(corners_with_scores.first.at(i).x, actualCorners2.at(i).x,
                1e-3);
    EXPECT_NEAR(corners_with_scores.first.at(i).y, actualCorners2.at(i).y,
                1e-3);
    if (i < actualCorners2.size() - 1) {
      EXPECT_LE(
          corners_with_scores.second.at(i + 1),
          corners_with_scores.second.at(i));  // check that they are sorted
    }
  }
  // check that smallest (last) score is greater than 0.01 (quality level) times
  // the largest (first)
  EXPECT_GE(corners_with_scores.second.at(actualCorners2.size() - 1),
            0.01 * corners_with_scores.second.at(0));
}

/* ************************************************************************* */
TEST(UtilsOpenCV, VectorUnique) {
  vector<int> vactual{1, 2, 3, 1, 2, 3, 3, 4, 5, 4, 5, 6, 7};
  vector<int> vexpected{1, 2, 3, 4, 5, 6, 7};

  UtilsOpenCV::VectorUnique<int>(vactual);
  UtilsOpenCV::PrintVector<int>(vactual, "vactual");
  UtilsOpenCV::PrintVector<int>(vexpected, "vexpected");

  EXPECT_EQ(vexpected.size(), vactual.size());
  for (size_t i = 0; i < vexpected.size(); i++) {
    EXPECT_EQ(vexpected[i], vactual[i]);
  }
}
/* ************************************************************************* */
TEST(UtilsOpenCV, ImageLaplacian) {
  Mat chessboardImg;
  vector<cv::Point2f> notUsed;
  tie(chessboardImg, notUsed) = cvCreateChessboard(30, 10, 8);
  Mat actual = UtilsOpenCV::ImageLaplacian(chessboardImg);
  // cv::imshow("actual",actual);
  // cv::waitKey(100);
}
