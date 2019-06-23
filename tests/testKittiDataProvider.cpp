/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testKittiDataProvider.cpp
 * @brief  test Kitti data parser
 * @author Yun Chang
 */

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>

#include "Visualizer3D.h"
#include "test_config.h"

// Add last, since it redefines CHECK, which is first defined by glog.
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace VIO;
using namespace cv;

static const double tol = 1e-1;

/* ************************************************************************* */
// TODO
TEST(testFrame, KittiDataProvider) {
  // TODO: test kitti data provider Check image lists and also imu data parsing
  // Construct a frame from image name.
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
