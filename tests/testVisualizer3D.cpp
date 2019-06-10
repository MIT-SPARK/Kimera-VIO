/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testVisualizer3D.cpp
 * @brief  test Visualizer3D
 * @author Antoni Rosinol
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <random>
#include <algorithm>

#include "Visualizer3D.h"
#include "test_config.h"

// Add last, since it redefines CHECK, which is first defined by glog.
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace VIO;
using namespace cv;

static const double tol = 1e-1;

/* ************************************************************************* */
TEST(testFrame, visualizeMesh2D) {
  // Construct a frame from image name.
  FrameId id = 0;
  Timestamp tmp = 123;
  const string imgName = string(DATASET_PATH) + "/chessboard_small.png";

  Frame f(id, tmp,
          CameraParams(),
          UtilsOpenCV::ReadAndConvertToGrayScale(imgName));
  f.extractCorners();
  for (int i = 0; i < f.keypoints_.size(); i++) { // populate landmark structure with fake data
    f.landmarks_.push_back(i);
  }

  // Compute mesh.
  const std::vector<cv::Vec6f>& mesh_2d = f.createMesh2D();

  // Visualize mesh.
  Visualizer3D visualizer (VisualizationType::NONE, 0);
  visualizer.visualizeMesh2D(mesh_2d, f.img_);
}

/* ************************************************************************* */
int main() {
  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
