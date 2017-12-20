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

#include "../src/Visualizer3D.h"
#include "test_config.h"

using namespace std;
using namespace VIO;
using namespace cv;

static const double tol = 1e-1;

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
  Visualizer::VisualizeMesh2D(f, triangulation2D,1000);
}

/* ************************************************************************* */
int main() {
  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
