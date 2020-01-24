/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testCameraParams.h
 * @brief  test CameraParams
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include <cmath>
#include <fstream>
#include <iostream>
#include <utility>
#include <vector>

#include <gtsam/geometry/Cal3_S2.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

DECLARE_string(test_data_path);

using namespace std;
using namespace gtsam;
using namespace VIO;
using namespace cv;

/* ************************************************************************** */
static void test_calibration(const double* intrinsics_expected,
                             const double* distortion_expected,
                             const CameraParams& cam_params) {
  bool rc =
      cam_params.distortion_->test(intrinsics_expected, distortion_expected);
  EXPECT_EQ(rc, true);
}

TEST(testCameraParams, parseYAML) {
  CameraParams cam_params;
  cam_params.parseYAML(FLAGS_test_data_path + "/sensor.yaml");

  // Frame rate.
  const double frame_rate_expected = 1.0 / 20.0;
  EXPECT_DOUBLE_EQ(frame_rate_expected, cam_params.frame_rate_);

  // Image size.
  const Size size_expected(752, 480);
  EXPECT_EQ(size_expected.width, cam_params.image_size_.width);
  EXPECT_EQ(size_expected.height, cam_params.image_size_.height);

  // Intrinsics.
  const std::vector<double> intrinsics_expected = {
      458.654, 457.296, 367.215, 248.375};
  for (int c = 0u; c < 4u; c++) {
    EXPECT_DOUBLE_EQ(intrinsics_expected[c], cam_params.intrinsics_[c]);
  }
  EXPECT_DOUBLE_EQ(intrinsics_expected[0],
                   cam_params.camera_matrix_.at<double>(0, 0));
  EXPECT_DOUBLE_EQ(intrinsics_expected[1],
                   cam_params.camera_matrix_.at<double>(1, 1));
  EXPECT_DOUBLE_EQ(intrinsics_expected[2],
                   cam_params.camera_matrix_.at<double>(0, 2));
  EXPECT_DOUBLE_EQ(intrinsics_expected[3],
                   cam_params.camera_matrix_.at<double>(1, 2));
  EXPECT_EQ(cam_params.intrinsics_.size(), 4u);

  // Sensor extrinsics wrt. the body-frame.
  Rot3 R_expected(0.0148655429818,  -0.999880929698,  0.00414029679422,
                  0.999557249008,   0.0149672133247,  0.025715529948,
                  -0.0257744366974, 0.00375618835797, 0.999660727178);
  Point3 T_expected(-0.0216401454975, -0.064676986768, 0.00981073058949);
  Pose3 pose_expected(R_expected, T_expected);
  EXPECT_TRUE(assert_equal(pose_expected, cam_params.body_Pose_cam_));

  // Distortion coefficients.
  const std::vector<double> distortion_expected = {
      -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  for (int c = 0u; c < 4u; c++) {
    EXPECT_DOUBLE_EQ(distortion_expected[c],
                     cam_params.distortion_coeff_.at<double>(c));
  }
  EXPECT_EQ(cam_params.distortion_coeff_.rows, 1u);
  EXPECT_EQ(cam_params.distortion_coeff_.cols, 4u);

  test_calibration(
      &intrinsics_expected[0], &distortion_expected[0], cam_params);
}

/* ************************************************************************** */
TEST(testCameraParams, equals) {
  CameraParams camParams;
  camParams.parseYAML(FLAGS_test_data_path + "/sensor.yaml");
  // camParams must be equal to itself
  EXPECT_TRUE(camParams.equals(camParams));
  // and might be different if we perturb something
  CameraParams camParams2 = camParams;
  camParams2.intrinsics_[2] = camParams.intrinsics_[2] + 1e-6;
  EXPECT_TRUE(!camParams.equals(camParams2, 1e-7));
  // however we cannot detect differences smaller than our tolerance:
  camParams2.intrinsics_[2] = camParams.intrinsics_[2] + 1e-8;
  EXPECT_TRUE(camParams.equals(camParams2, 1e-7));
}

/* ************************************************************************** */
TEST(testCameraParams, Cal3_S2ToCvmat) {
  Cal3_S2 K(500, 500, 0.0, 640 / 2, 480 / 2);
}

/* ************************************************************************** */
TEST(testCameraParams, DISABLED_parseKITTICalib) {
  CameraParams camParams;
  // camParams.parseKITTICalib(
  //    FLAGS_test_data_path + "/ForKittiData/calib_cam_to_cam.txt",
  //    cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), "00");

  // Frame rate
  const double frame_rate_expected = 1.0 / 10.0;
  EXPECT_DOUBLE_EQ(frame_rate_expected, camParams.frame_rate_);

  // image size
  const Size size_expected(1392, 512);
  EXPECT_EQ(size_expected.width, camParams.image_size_.width);
  EXPECT_EQ(size_expected.height, camParams.image_size_.height);

  // intrinsics
  const double intrinsics_expected[] = {984.2439, 980.8141, 690.0, 233.1966};
  for (int c = 0; c < 4; c++) {
    EXPECT_DOUBLE_EQ(intrinsics_expected[c], camParams.intrinsics_[c]);
  }

  // Sensor extrinsics wrt. the body-frame
  Rot3 R_expected(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
  Point3 T_expected(2.573699e-16, -1.059758e-16, 1.614870e-16);
  Pose3 pose_expected(R_expected, T_expected);
  EXPECT_TRUE(assert_equal(pose_expected, camParams.body_Pose_cam_));

  // distortion coefficients
  const double distortion_expected[] = {
      -3.728755e-01, 2.037299e-01, 2.219027e-03, 1.383707e-03, -7.233722e-02};
  for (int c = 0; c < 5; c++) {
    EXPECT_DOUBLE_EQ(distortion_expected[c],
                     camParams.distortion_coeff_.at<double>(c));
  }
  EXPECT_DOUBLE_EQ(distortion_expected[2],
                   camParams.distortion_coeff_.at<double>(2));
  test_calibration(intrinsics_expected, distortion_expected, camParams);
}
