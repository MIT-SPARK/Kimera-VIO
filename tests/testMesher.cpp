/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testStereoVisionFrontEnd.cpp
 * @brief  test StereoVisionFrontEnd
 * @author Luca Carlone
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <random>
#include <algorithm>

#include <CppUnitLite/TestHarness.h>
#include "Mesher.h"
#include "test_config.h"

using namespace std;
using namespace VIO;
using namespace cv;

static const double tol = 1e-1;

/* ************************************************************************* */
TEST(testMesher, createMesh2D) {

  // Construct a frame from image name.
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName = string(DATASET_PATH) + "/chessboard_small.png";
  Frame f(id, tmp, imgName, CameraParams());
  f.extractCorners();
  for (int i = 0; i < f.keypoints_.size(); i++) { // populate landmark structure with fake data
    f.landmarks_.push_back(i);
  }

  // compute mesh
  vector<Vec6f> triangulation2D = Mesher::CreateMesh2D(f);

  // Expected triangulation
  //  3 -- 2
  //  | /  |
  //  1 -- 0

  // triangle 1:
  Vec6f triangle1 = triangulation2D[0];
  double triangle1_pt1_x = double (triangle1[0]);
  EXPECT_DOUBLES_EQUAL(f.keypoints_[2].x, triangle1_pt1_x, tol);
  double triangle1_pt1_y = double (triangle1[1]);
  EXPECT_DOUBLES_EQUAL(f.keypoints_[2].y, triangle1_pt1_y, tol);
  double triangle1_pt2_x = double (triangle1[2]);
  EXPECT_DOUBLES_EQUAL(f.keypoints_[1].x, triangle1_pt2_x, tol);
  double triangle1_pt2_y = double (triangle1[3]);
  EXPECT_DOUBLES_EQUAL(f.keypoints_[1].y, triangle1_pt2_y, tol);
  double triangle1_pt3_x = double (triangle1[4]);
  EXPECT_DOUBLES_EQUAL(f.keypoints_[3].x, triangle1_pt3_x, tol);
  double triangle1_pt3_y = double (triangle1[5]);
  EXPECT_DOUBLES_EQUAL(f.keypoints_[3].y, triangle1_pt3_y, tol);

  // triangle 2:
  Vec6f triangle2 = triangulation2D[1];
  double triangle2_pt1_x = double (triangle2[0]);
  EXPECT_DOUBLES_EQUAL(f.keypoints_[1].x, triangle2_pt1_x, tol);
  double triangle2_pt1_y = double (triangle2[1]);
  EXPECT_DOUBLES_EQUAL(f.keypoints_[1].y, triangle2_pt1_y, tol);
  double triangle2_pt2_x = double (triangle2[2]);
  EXPECT_DOUBLES_EQUAL(f.keypoints_[2].x, triangle2_pt2_x, tol);
  double triangle2_pt2_y = double (triangle2[3]);
  EXPECT_DOUBLES_EQUAL(f.keypoints_[2].y, triangle2_pt2_y, tol);
  double triangle2_pt3_x = double (triangle2[4]);
  EXPECT_DOUBLES_EQUAL(f.keypoints_[0].x, triangle2_pt3_x, tol);
  double triangle2_pt3_y = double (triangle2[5]);
  EXPECT_DOUBLES_EQUAL(f.keypoints_[0].y, triangle2_pt3_y, tol);
}

/* ************************************************************************* */
TEST(testMesher, createMesh2D_noKeypoints) {

  // Construct a frame from image name.
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName = string(DATASET_PATH) + "/chessboard_small.png";
  Frame f(id, tmp, imgName, CameraParams());

  // compute mesh without points
  vector<Vec6f> triangulation2D = Mesher::CreateMesh2D(f);

  EXPECT(triangulation2D.size() == 0);
}

/* ************************************************************************* */
TEST(testMesher, visualizeMesh2D) {

  // Construct a frame from image name.
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName = string(DATASET_PATH) + "/chessboard_small.png";
  Frame f(id, tmp, imgName, CameraParams());
  f.extractCorners();
  for (int i = 0; i < f.keypoints_.size(); i++) { // populate landmark structure with fake data
    f.landmarks_.push_back(i);
  }

  // compute mesh
  vector<Vec6f> triangulation2D = Mesher::CreateMesh2D(f);
  Mesher::VisualizeMesh2D(f, triangulation2D,1000);
}

/* ************************************************************************* */
TEST(testMesher, mapPoints3dUpdate) {

}


/* ************************************************************************* */
int main() {
  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
