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

using namespace gtsam;
using namespace std;
using namespace VIO;
using namespace cv;

static const string chessboardImgName = string(DATASET_PATH) + "/chessboard.png";
static const string whitewallImgName = string(DATASET_PATH) + "/whitewall.png";
static const string sensorPath = string(DATASET_PATH) + "/sensor.yaml";
static const int imgWidth = 752;
static const int imgHeight = 480;
static const double tol = 1e-7;

/* ************************************************************************* */
TEST(testMesher, createMesh2D) {

  // Construct a frame from image name.
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName = string(DATASET_PATH) + "/chessboard.png";
  Frame f(id, tmp, imgName, CameraParams());

  vector<Vec6f> triangulation2D = Mesher::CreateMesh2D(f);
  Mesher::VisualizeMesh2D(f, triangulation2D);

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
