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
#include <utility>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include "kimera-vio/frontend/CameraParams.h"

DECLARE_string(test_data_path);

namespace VIO {

TEST(testCameraParams, parseYAML) {
  CameraParams cam_params;
  cam_params.parseYAML(FLAGS_test_data_path + "/sensor.yaml");

  // Frame rate.
  const double frame_rate_expected = 1.0 / 20.0;
  EXPECT_DOUBLE_EQ(frame_rate_expected, cam_params.frame_rate_);

  // Image size.
  const cv::Size size_expected(752, 480);
  EXPECT_EQ(size_expected.width, cam_params.image_size_.width);
  EXPECT_EQ(size_expected.height, cam_params.image_size_.height);

  // Intrinsics.
  const std::vector<double> intrinsics_expected = {
      458.654, 457.296, 367.215, 248.375};
  for (int c = 0u; c < 4u; c++) {
    EXPECT_DOUBLE_EQ(intrinsics_expected[c], cam_params.intrinsics_[c]);
  }
  EXPECT_DOUBLE_EQ(intrinsics_expected[0], cam_params.K_.at<double>(0, 0));
  EXPECT_DOUBLE_EQ(intrinsics_expected[1], cam_params.K_.at<double>(1, 1));
  EXPECT_DOUBLE_EQ(intrinsics_expected[2], cam_params.K_.at<double>(0, 2));
  EXPECT_DOUBLE_EQ(intrinsics_expected[3], cam_params.K_.at<double>(1, 2));
  EXPECT_EQ(cam_params.intrinsics_.size(), 4u);
  gtsam::Cal3DS2 gtsam_calib;
  CameraParams::createGtsamCalibration(cam_params.distortion_coeff_mat_,
                                       cam_params.intrinsics_,
                                       &gtsam_calib);
  EXPECT_DOUBLE_EQ(intrinsics_expected[0], gtsam_calib.fx());
  EXPECT_DOUBLE_EQ(intrinsics_expected[1], gtsam_calib.fy());
  EXPECT_DOUBLE_EQ(0u, gtsam_calib.skew());
  EXPECT_DOUBLE_EQ(intrinsics_expected[2], gtsam_calib.px());
  EXPECT_DOUBLE_EQ(intrinsics_expected[3], gtsam_calib.py());

  // Sensor extrinsics wrt. the body-frame.
  gtsam::Rot3 R_expected(0.0148655429818,
                         -0.999880929698,
                         0.00414029679422,
                         0.999557249008,
                         0.0149672133247,
                         0.025715529948,
                         -0.0257744366974,
                         0.00375618835797,
                         0.999660727178);
  gtsam::Point3 T_expected(-0.0216401454975, -0.064676986768, 0.00981073058949);
  gtsam::Pose3 pose_expected(R_expected, T_expected);
  EXPECT_TRUE(assert_equal(pose_expected, cam_params.body_Pose_cam_));

  // Distortion coefficients.
  const std::vector<double> distortion_expected = {
      -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  for (int c = 0u; c < 4u; c++) {
    EXPECT_DOUBLE_EQ(distortion_expected[c],
                     cam_params.distortion_coeff_mat_.at<double>(c));
  }
  EXPECT_EQ(cam_params.distortion_coeff_mat_.rows, 1u);
  EXPECT_EQ(cam_params.distortion_coeff_mat_.cols, 4u);
  EXPECT_DOUBLE_EQ(distortion_expected[0], gtsam_calib.k1());
  EXPECT_DOUBLE_EQ(distortion_expected[1], gtsam_calib.k2());
  EXPECT_DOUBLE_EQ(distortion_expected[2], gtsam_calib.p1());
  EXPECT_DOUBLE_EQ(distortion_expected[3], gtsam_calib.p2());
}

TEST(testCameraParams, convertDistortionVectorToMatrix) {
  std::vector<double> distortion_coeffs;

  // 4 distortion params
  distortion_coeffs = {1.0, -2.0, 1.3, 10};
  cv::Mat distortion_coeffs_mat;
  CameraParams::convertDistortionVectorToMatrix(distortion_coeffs,
                                                &distortion_coeffs_mat);
  EXPECT_EQ(distortion_coeffs_mat.cols, distortion_coeffs.size());
  EXPECT_EQ(distortion_coeffs_mat.rows, 1u);
  for (size_t i = 0u; i < distortion_coeffs.size(); i++) {
    EXPECT_EQ(distortion_coeffs_mat.at<double>(0, i), distortion_coeffs.at(i));
  }

  // 5 distortion params
  distortion_coeffs = {1, 1.2f, 3u, 4l, 5.34};  //! randomize types as well
  CameraParams::convertDistortionVectorToMatrix(distortion_coeffs,
                                                &distortion_coeffs_mat);
  EXPECT_EQ(distortion_coeffs_mat.cols, distortion_coeffs.size());
  EXPECT_EQ(distortion_coeffs_mat.rows, 1u);
  for (size_t i = 0u; i < distortion_coeffs.size(); i++) {
    EXPECT_EQ(distortion_coeffs_mat.at<double>(0u, i), distortion_coeffs.at(i));
  }

  // n distortion params
  distortion_coeffs = {1.0, 1.2, 3.2, 4.3, 5.34, 10203, 1818.9, 1.9};
  CameraParams::convertDistortionVectorToMatrix(distortion_coeffs,
                                                &distortion_coeffs_mat);
  EXPECT_EQ(distortion_coeffs_mat.cols, distortion_coeffs.size());
  EXPECT_EQ(distortion_coeffs_mat.rows, 1u);
  for (size_t i = 0u; i < distortion_coeffs.size(); i++) {
    EXPECT_EQ(distortion_coeffs_mat.at<double>(0u, i), distortion_coeffs.at(i));
  }
}

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

}  // namespace VIO
