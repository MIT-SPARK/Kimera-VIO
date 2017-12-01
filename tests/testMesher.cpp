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

  // compute mesh
  vector<Vec6f> triangulation2D = Mesher::CreateMesh2D(f);
  Mesher::VisualizeMesh2D(f, triangulation2D);

  cout <<  f.keypoints_[0].x << endl;
  cout <<  f.keypoints_[0].y << endl;
  cout <<  f.keypoints_[1].x << endl;
  cout <<  f.keypoints_[1].y << endl;
  cout <<  f.keypoints_[2].x << endl;
  cout <<  f.keypoints_[2].y << endl;
  cout <<  f.keypoints_[3].x << endl;
  cout <<  f.keypoints_[3].y << endl;
  cout <<  f.keypoints_.size() << endl;

  // triangle 1:
  double triangle1_pt1_x = double (triangulation2D[0][0]);
  EXPECT_DOUBLES_EQUAL(1308, triangle1_pt1_x, tol);
  double triangle1_pt1_y = double (triangulation2D[0][1]);
  EXPECT_DOUBLES_EQUAL(0.0, triangle1_pt1_y, tol);
  double triangle1_pt2_x = double (triangulation2D[0][2]);
  EXPECT_DOUBLES_EQUAL(0.0, triangle1_pt2_x, tol);
  double triangle1_pt2_y = double (triangulation2D[0][3]);
  EXPECT_DOUBLES_EQUAL(1308, triangle1_pt2_y, tol);
  double triangle1_pt3_x = double (triangulation2D[0][4]);
  EXPECT_DOUBLES_EQUAL( 291.49, triangle1_pt3_x, tol);
  double triangle1_pt3_y = double (triangulation2D[0][5]);
  EXPECT_DOUBLES_EQUAL(293.49, triangle1_pt3_y, tol);

}


/* ************************************************************************* */
TEST(testMesher, visualizeMesh2D) {
  // Mesher class
  //
  // vector<Vec6f triangulation2D = mesher.createMesh2D(Frame& currentFrame)
  //
  // mesher.visualizeMesh2D(Frame& currentFrame, vector<Vec6f> triangulation2D);


}


/* ************************************************************************* */
int main() {
  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
