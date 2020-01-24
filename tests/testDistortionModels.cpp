/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testDistortionModels.h
 * @brief  tests the distortion models
 * @author Bernd Pfrommer
 */
#include "kimera-vio/distortion_models/DistortionModel.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <utility>
#include <vector>

DECLARE_string(test_data_path);

/* ************************************************************************** */

// turn intrinsics into K matrix
static cv::Mat make_K_matrix(const std::vector<double> intr) {
  double v[9] = {intr[0], 0, intr[2], 0, intr[1], intr[3], 0, intr[1], 1};
  const cv::Mat K = cv::Mat(3, 3, CV_64F, &v[0]);
  return (K.clone());
}

// lift 2d points to 3d
static std::vector<cv::Point3f> make_homogeneous(
    const std::vector<cv::Point2f>& v) {
  std::vector<cv::Point3f> h;
  for (const auto& p : v) {
    h.push_back(cv::Point3f(p.x, p.y, 1.0));
  }
  return (h);
}

// generate test data common for all tests
static std::vector<cv::Point2f> make_test_data(
    cv::Mat* K,
    std::vector<double>* intrinsics,
    std::vector<double>* dist_coeff) {
  // camera resolution
  const int res[2] = {640, 400};
  // distortion coefficients
  const std::vector<double> dc = {0.03, -0.02, 0.01, -0.005};
  *dist_coeff = dc;
  // camera intrinsics
  const double u_off = -10.0;
  const double v_off = 5.0;
  // note: shift the image center slightly
  const std::vector<double> intr = {
      640.0, 635.0, res[0] / 2 + u_off, res[1] / 2 + v_off};
  *intrinsics = intr;
  // convert intrinsics to K matrix
  *K = make_K_matrix(intr);

  // test points: distorted (u,v) on-sensor points
  // note: due to the center shift, the last one is actually
  // where the optical axis meets the sensor
  const std::vector<cv::Point2f> pts = {
      cv::Point2f(0, 0),                    // top left corners
      cv::Point2f(res[0] - 1, res[1] - 1),  // bottom right corner
      cv::Point2f(res[0] / 2, res[1] / 2),  // center
      cv::Point2f(res[0] / 2 + u_off, res[1] / 2 + v_off)};  // shifted center
  return pts;
}

// helper function to uncalibrate set of points
static std::vector<cv::Point2f> uncalibrate(
    const VIO::DistortionModelPtr& dm,
    const std::vector<cv::Point2f>& undist) {
  std::vector<cv::Point2f> uncalib;
  for (const auto& p : undist) {
    const gtsam::Point2 uc = dm->uncalibrate(gtsam::Point2(p.x, p.y));
    uncalib.push_back(cv::Point2f(uc(0), uc(1)));
  }
  return (uncalib);
}

// helper function to project set of points
static std::vector<cv::Point2f> project(const VIO::DistortionModelPtr& dm,
                                        const std::vector<cv::Point3f>& p3d) {
  std::vector<cv::Point2f> proj;
  for (const auto& p : p3d) {
    const gtsam::Point2 pp =
        dm->project(gtsam::Pose3(), gtsam::Point3(p.x, p.y, p.z));
    proj.push_back(cv::Point2f(pp(0), pp(1)));
  }
  return (proj);
}

// common code for radtan uncalibration testing
static void test_radtan_uncalibrate(const std::vector<double>& extra_coeff) {
  cv::Mat K;
  std::vector<double> dist_coeff, intrinsics;
  // make test data common for all models
  const std::vector<cv::Point2f> dist =
      make_test_data(&K, &intrinsics, &dist_coeff);
  // now add extra coefficients to back
  dist_coeff.insert(dist_coeff.end(), extra_coeff.begin(), extra_coeff.end());

  VIO::DistortionModelPtr dm =
      VIO::DistortionModel::make("radtan", intrinsics, dist_coeff);

  // first undistort the points using opencv
  std::vector<cv::Point2f> undist;
  cv::undistortPoints(dist, undist, K, dist_coeff);

  // now redistort the points using opencv.
  // This is necessary because the undistortion in opencv
  // is not super precise, and when many large k coefficients
  // are used, opencv does not exactly recover the original values
  // during a round trip.
  std::vector<cv::Point3f> undist_h = make_homogeneous(undist);
  cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  std::vector<cv::Point2f> redist;  // redistorted
  // use project function to distort points again
  cv::projectPoints(undist_h, rvec, tvec, K, dist_coeff, redist);

  std::vector<cv::Point2f> uncalib = uncalibrate(dm, undist);
  // Check against the redistorted points,
  // not the original ones!
  for (int i = 0; i < undist.size(); i++) {
    EXPECT_NEAR(uncalib[i].x, redist[i].x, 1e-6);
    EXPECT_NEAR(uncalib[i].y, redist[i].y, 1e-6);
  }
}

// common code for radtan projection testing
static void test_radtan_project(const std::vector<double>& extra_coeff) {
  cv::Mat K;
  std::vector<double> dist_coeff, intrinsics;
  const std::vector<cv::Point2f> dist =
      make_test_data(&K, &intrinsics, &dist_coeff);
  // now add extra coefficients to back
  dist_coeff.insert(dist_coeff.end(), extra_coeff.begin(), extra_coeff.end());

  VIO::DistortionModelPtr dm =
      VIO::DistortionModel::make("radtan", intrinsics, dist_coeff);

  std::vector<cv::Point2f> undist;
  cv::undistortPoints(dist, undist, K, dist_coeff);
  std::vector<cv::Point3f> undist_h = make_homogeneous(undist);
  cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  std::vector<cv::Point2f> proj;  // opencv projected points
  cv::projectPoints(undist_h, rvec, tvec, K, dist_coeff, proj);

  std::vector<cv::Point2f> proj_dm = project(dm, undist_h);
  // check against opencv projection
  for (int i = 0; i < proj.size(); i++) {
    EXPECT_NEAR(proj_dm[i].x, proj[i].x, 1e-5);
    EXPECT_NEAR(proj_dm[i].y, proj[i].y, 1e-5);
  }
}

TEST(testDistortionModels, uncalibrateEquidistant) {
  cv::Mat K;
  std::vector<double> dist_coeff, intrinsics;

  const std::vector<cv::Point2f> dist =
      make_test_data(&K, &intrinsics, &dist_coeff);

  VIO::DistortionModelPtr dm =
      VIO::DistortionModel::make("equidistant", intrinsics, dist_coeff);

  std::vector<cv::Point2f> undist;
  cv::fisheye::undistortPoints(dist, undist, K, dist_coeff);
  std::vector<cv::Point2f> uncalib = uncalibrate(dm, undist);
  // check against original, distorted points
  for (int i = 0; i < dist.size(); i++) {
    EXPECT_NEAR(uncalib[i].x, dist[i].x, 1e-5);
    EXPECT_NEAR(uncalib[i].y, dist[i].y, 1e-5);
  }
}

TEST(testDistortionModels, uncalibrateRadTan4) {
  const std::vector<double> extra_coeff;
  test_radtan_uncalibrate(extra_coeff);
}

TEST(testDistortionModels, uncalibrateRadTan8) {
  // to find trouble, pick large k3..k6
  const std::vector<double> extra_coeff = {0.5, -0.3, 0.2, -0.1};
  test_radtan_uncalibrate(extra_coeff);
}

TEST(testDistortionModels, projectEquidistant) {
  cv::Mat K;
  std::vector<double> dist_coeff, intrinsics;

  const std::vector<cv::Point2f> dist =
      make_test_data(&K, &intrinsics, &dist_coeff);

  VIO::DistortionModelPtr dm =
      VIO::DistortionModel::make("equidistant", intrinsics, dist_coeff);

  std::vector<cv::Point2f> undist;
  cv::fisheye::undistortPoints(dist, undist, K, dist_coeff);
  std::vector<cv::Point3f> undist_h = make_homogeneous(undist);
  cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  std::vector<cv::Point2f> proj;  // opencv projected points
  cv::fisheye::projectPoints(undist_h, proj, rvec, tvec, K, dist_coeff);

  std::vector<cv::Point2f> proj_dm = project(dm, undist_h);
  // check against opencv projection
  for (int i = 0; i < proj.size(); i++) {
    EXPECT_NEAR(proj_dm[i].x, proj[i].x, 1e-5);
    EXPECT_NEAR(proj_dm[i].y, proj[i].y, 1e-5);
  }
}

TEST(testDistortionModels, projectRadTan4) {
  const std::vector<double> extra_coeff;
  test_radtan_project(extra_coeff);
}

TEST(testDistortionModels, projectRadTan8) {
  // to find trouble, pick large k3..k6
  const std::vector<double> extra_coeff = {0.5, -0.3, 0.2, -0.1};
  test_radtan_project(extra_coeff);
}
