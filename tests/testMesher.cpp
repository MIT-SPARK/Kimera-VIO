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

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "mesh/Mesher.h"
DECLARE_string(test_data_path);

using namespace std;
using namespace VIO;
using namespace cv;

static const double tol = 1e-1;

TEST(testMesher, getRatioBetweenLargestAnSmallestSide) {
  Mesher mesher;
  mesher.map_points_3d_.push_back(cv::Point3f(0.5377, 0.3188, 3.5784));   // pt0
  mesher.map_points_3d_.push_back(cv::Point3f(1.8339, -1.3077, 2.7694));  // pt1
  mesher.map_points_3d_.push_back(
      cv::Point3f(-2.2588, -0.4336, -1.3499));                           // pt2
  mesher.map_points_3d_.push_back(cv::Point3f(0.8622, 0.3426, 3.0349));  // pt3

  int rowId_pt1 = 0, rowId_pt2 = 2, rowId_pt3 = 3;
  double d12_out, d23_out, d31_out;
  double actual_ratio = mesher.getRatioBetweenSmallestAndLargestSide(
      rowId_pt1, rowId_pt2, rowId_pt3);

  // from MATLAB
  //   A =[   0.5377    0.3188    3.5784
  //      1.8339   -1.3077    2.7694
  //     -2.2588   -0.4336   -1.3499
  //      0.8622    0.3426    3.0349];
  //  d02 = norm(A(1,:)-A(3,:));
  //  d23 = norm(A(3,:)-A(4,:));
  //  d30 = norm(A(4,:)-A(1,:));
  //  min([d02 d23 d30]) / max([d02 d23 d30])

  double expected_ratio = 0.1108 * 0.1108;  // from matlab
  EXPECT_DOUBLES_EQUAL(expected_ratio, actual_ratio, 1e-4);
}

/* ************************************************************************* */
TEST(testMesher, getRatioBetweenLargestAnSmallestSide2) {
  Mesher mesher;
  mesher.map_points_3d_.push_back(cv::Point3f(0.5377, 0.3188, 3.5784));   // pt0
  mesher.map_points_3d_.push_back(cv::Point3f(1.8339, -1.3077, 2.7694));  // pt1
  mesher.map_points_3d_.push_back(
      cv::Point3f(-2.2588, -0.4336, -1.3499));                           // pt2
  mesher.map_points_3d_.push_back(cv::Point3f(0.8622, 0.3426, 3.0349));  // pt3

  int rowId_pt1 = 0, rowId_pt2 = 2, rowId_pt3 = 3;
  double d12_out_actual, d23_out_actual, d31_out_actual;
  double actual_ratio = mesher.getRatioBetweenSmallestAndLargestSide(
      rowId_pt1, rowId_pt2, rowId_pt3, d12_out_actual, d23_out_actual,
      d31_out_actual);

  // from MATLAB
  //   A =[   0.5377    0.3188    3.5784
  //      1.8339   -1.3077    2.7694
  //     -2.2588   -0.4336   -1.3499
  //      0.8622    0.3426    3.0349];
  //  d02 = norm(A(1,:)-A(3,:));
  //  d23 = norm(A(3,:)-A(4,:));
  //  d30 = norm(A(4,:)-A(1,:));
  //  min([d02 d23 d30]) / max([d02 d23 d30])

  double expected_ratio = 0.1108 * 0.1108;  // from matlab
  EXPECT_DOUBLES_EQUAL(expected_ratio, actual_ratio, 1e-4);

  double d12_out_expected = 5.7162 * 5.7162;
  double d23_out_expected = 5.4378 * 5.4378;
  double d31_out_expected = 0.6334 * 0.6334;

  EXPECT_DOUBLES_EQUAL(d12_out_expected, d12_out_actual, 1e-3);
  EXPECT_DOUBLES_EQUAL(d23_out_expected, d23_out_actual, 1e-3);
  EXPECT_DOUBLES_EQUAL(d31_out_expected, d31_out_actual, 1e-3);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
