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

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/visualizer/Visualizer3D.h"

DECLARE_string(test_data_path);

using namespace std;
using namespace VIO;
using namespace cv;

// TODO(Yun)
TEST(testFrame, KittiDataProvider) {
  // TODO: test kitti data provider Check image lists and also imu data parsing
  // Construct a frame from image name.
}
