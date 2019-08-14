/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testFrame.h
 * @brief  test Frame
 * @author Luca Carlone
 */

#include "Frame.h"
DECLARE_string(test_data_path);

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

using namespace gtsam;
using namespace std;
using namespace VIO;
using namespace cv;

static const string chessboardImgName =
    string(FLAGS_test_data_path) + "/chessboard.png";
static const string whitewallImgName =
    string(FLAGS_test_data_path) + "/whitewall.png";
static const string sensorPath = string(FLAGS_test_data_path) + "/sensor.yaml";
static const int imgWidth = 752;
static const int imgHeight = 480;

/* ************************************************************************* */
TEST(testFrame, constructor) {
  // Construct a frame from image name.
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName = string(FLAGS_test_data_path) + "/chessboard.png";
  Frame f(id, tmp, CameraParams(),
          UtilsOpenCV::ReadAndConvertToGrayScale(imgName));
  ASSERT_EQ(f.id_, id);
  ASSERT_EQ(f.timestamp_, tmp);
  // check image:
  Mat img = imread(imgName, IMREAD_ANYCOLOR);
  ASSERT_TRUE(UtilsOpenCV::CvMatCmp(f.img_, img));
  ASSERT_TRUE(!f.isKeyframe_);  // false by default
  ASSERT_TRUE(CameraParams().equals(f.cam_param_));
}

/* ************************************************************************* */
TEST(testFrame, ExtractCornersChessboard) {
  Frame f(0, 0, CameraParams(),
          UtilsOpenCV::ReadAndConvertToGrayScale(chessboardImgName));
  f.extractCorners();
  int numCorners_expected = 7 * 9;
  int numCorners_actual = f.keypoints_.size();
  // Assert that there are right number of corners!
  ASSERT_EQ(numCorners_actual, numCorners_expected);
}

/* ************************************************************************* */
TEST(testFrame, ExtractCornersWhiteBoard) {
  Frame f(0, 0, CameraParams(),
          UtilsOpenCV::ReadAndConvertToGrayScale(whitewallImgName));
  f.extractCorners();
  int numCorners_expected = 0;
  int numCorners_actual = f.keypoints_.size();
  // Assert that there are no corners!
  ASSERT_EQ(numCorners_actual, numCorners_expected);
}

/* ------------------------------------------------------------------------- */
TEST(testFrame, getNrValidKeypoints) {
  Frame f(0, 0, CameraParams(),
          UtilsOpenCV::ReadAndConvertToGrayScale(chessboardImgName));
  const int nrValidExpected = 200;
  const int outlier_rate = 5;  // Insert one outlier every 5 valid landmark ids.
  for (int i = 0; i < nrValidExpected; i++) {
    if (i % outlier_rate == 0) {
      f.landmarks_.push_back(-1);
    }
    f.landmarks_.push_back(
        i);  // always push a valid and sometimes also an outlier
  }
  int nrValidActual = f.getNrValidKeypoints();
  ASSERT_EQ(nrValidActual, nrValidExpected);
}

/* ************************************************************************* */
TEST(testFrame, CalibratePixel) {
  // Perform a scan on the grid to verify the correctness of pixel calibration!
  const int numTestRows = 8;
  const int numTestCols = 8;

  // Get the camera parameters
  CameraParams camParams;
  camParams.parseYAML(sensorPath);

  // Generate the pixels
  KeypointsCV testPointsCV;
  testPointsCV.reserve(numTestRows * numTestCols);

  for (int r = 0; r < numTestRows; r++) {
    for (int c = 0; c < numTestCols; c++) {
      testPointsCV.push_back(KeypointCV(c * imgWidth / (numTestCols - 1),
                                        r * imgHeight / (numTestRows - 1)));
    }
  }
  // Calibrate, and uncalibrate the point, verify that we get the same point
  for (KeypointsCV::iterator iter = testPointsCV.begin();
       iter != testPointsCV.end(); iter++) {
    Vector3 versor = Frame::CalibratePixel(*iter, camParams);
    ASSERT_DOUBLE_EQ(versor.norm(), 1);

    // distort the pixel again
    versor = versor / versor(2);
    Point2 uncalibrated_px_actual =
        camParams.calibration_.uncalibrate(Point2(versor(0), versor(1)));
    Point2 uncalibrated_px_expected = Point2(iter->x, iter->y);
    Point2 px_mismatch = uncalibrated_px_actual - uncalibrated_px_expected;
    ASSERT_TRUE(px_mismatch.vector().norm() < 0.5);
  }
}

/* ************************************************************************* */
// TODO: Create test for Calibrate Pixel with pinhole equidistant model
TEST(testFrame, DISABLED_CalibratePixel) {
  // Perform a scan on the grid to verify the correctness of pixel calibration!
  const int numTestRows = 8;
  const int numTestCols = 8;

  // Get the camera parameters
  CameraParams camParams;
  camParams.parseYAML(sensorPath);

  // Generate the pixels
  KeypointsCV testPointsCV;
  testPointsCV.reserve(numTestRows * numTestCols);

  for (int r = 0; r < numTestRows; r++) {
    for (int c = 0; c < numTestCols; c++) {
      testPointsCV.push_back(KeypointCV(c * imgWidth / (numTestCols - 1),
          r * imgHeight / (numTestRows - 1)));
    }
  }
  // Calibrate, and uncalibrate the point, verify that we get the same point
  for (KeypointsCV::iterator iter = testPointsCV.begin(); iter !=
testPointsCV.end(); iter++) { Vector3 versor = Frame::CalibratePixel(*iter,
camParams); ASSERT_DOUBLE_EQ(versor.norm(), 1);

    // distort the pixel again
    versor = versor / versor(2);
    Point2 uncalibrated_px_actual =
camParams.calibration_.uncalibrate(Point2(versor(0), versor(1))); Point2
uncalibrated_px_expected = Point2(iter->x, iter->y); Point2 px_mismatch =
uncalibrated_px_actual - uncalibrated_px_expected;
    ASSERT_TRUE(px_mismatch.vector().norm() < 0.5);
  }
}

// TEST(testFrame, CalibratePixelEquidistant) {}

/* ************************************************************************* */
TEST(testFrame, findLmkIdFromPixel) {
  Frame f(0, 0, CameraParams(),
          UtilsOpenCV::ReadAndConvertToGrayScale(chessboardImgName));
  f.extractCorners();
  for (int i = 0; i < f.keypoints_.size(); i++) {
    f.landmarks_.push_back(
        i + 5);  // always push a valid and sometimes also an outlier
  }
  // check that if you query ith f.keypoints_ you get i+5
  for (int i = 0; i < f.keypoints_.size(); i++) {
    ASSERT_EQ(f.findLmkIdFromPixel(f.keypoints_[i]), i + 5);
  }
}
/* ************************************************************************* */
TEST(testFrame, createMesh2D) {
  // Construct a frame from image name.
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName = string(FLAGS_test_data_path) + "/chessboard_small.png";
  Frame f(id, tmp, CameraParams(),
          UtilsOpenCV::ReadAndConvertToGrayScale(imgName));
  f.extractCorners();
  for (int i = 0; i < f.keypoints_.size();
       i++) {  // populate landmark structure with fake data
    f.landmarks_.push_back(i);
  }

  // Compute mesh.
  const std::vector<Vec6f>& triangulation2D = f.createMesh2D();

  // Expected triangulation
  //  3 -- 2
  //  | /  |
  //  1 -- 0

  // triangle 1:
  Vec6f triangle1 = triangulation2D[0];
  double triangle1_pt1_x = double(triangle1[0]);
  ASSERT_DOUBLE_EQ(f.keypoints_[2].x, triangle1_pt1_x);
  double triangle1_pt1_y = double(triangle1[1]);
  ASSERT_DOUBLE_EQ(f.keypoints_[2].y, triangle1_pt1_y);
  double triangle1_pt2_x = double(triangle1[2]);
  ASSERT_DOUBLE_EQ(f.keypoints_[1].x, triangle1_pt2_x);
  double triangle1_pt2_y = double(triangle1[3]);
  ASSERT_DOUBLE_EQ(f.keypoints_[1].y, triangle1_pt2_y);
  double triangle1_pt3_x = double(triangle1[4]);
  ASSERT_DOUBLE_EQ(f.keypoints_[3].x, triangle1_pt3_x);
  double triangle1_pt3_y = double(triangle1[5]);
  ASSERT_DOUBLE_EQ(f.keypoints_[3].y, triangle1_pt3_y);

  // triangle 2:
  Vec6f triangle2 = triangulation2D[1];
  double triangle2_pt1_x = double(triangle2[0]);
  ASSERT_DOUBLE_EQ(f.keypoints_[1].x, triangle2_pt1_x);
  double triangle2_pt1_y = double(triangle2[1]);
  ASSERT_DOUBLE_EQ(f.keypoints_[1].y, triangle2_pt1_y);
  double triangle2_pt2_x = double(triangle2[2]);
  ASSERT_DOUBLE_EQ(f.keypoints_[2].x, triangle2_pt2_x);
  double triangle2_pt2_y = double(triangle2[3]);
  ASSERT_DOUBLE_EQ(f.keypoints_[2].y, triangle2_pt2_y);
  double triangle2_pt3_x = double(triangle2[4]);
  ASSERT_DOUBLE_EQ(f.keypoints_[0].x, triangle2_pt3_x);
  double triangle2_pt3_y = double(triangle2[5]);
  ASSERT_DOUBLE_EQ(f.keypoints_[0].y, triangle2_pt3_y);
}

/* ************************************************************************* */
TEST(testFrame, createMesh2D_noKeypoints) {
  // Construct a frame from image name.
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName = string(FLAGS_test_data_path) + "/chessboard_small.png";
  Frame f(id, tmp, CameraParams(),
          UtilsOpenCV::ReadAndConvertToGrayScale(imgName));

  // Compute mesh without points.
  const vector<Vec6f>& triangulation2D = f.createMesh2D();

  ASSERT_EQ(triangulation2D.size(), 0);
}
