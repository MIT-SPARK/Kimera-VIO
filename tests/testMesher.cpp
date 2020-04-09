/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testMesher.cpp
 * @brief  test Mesher implementation
 * @author Antoni Rosinol
 */

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>

#include <gtest/gtest.h>
#include <gflags/gflags.h>

#include "kimera-vio/mesh/Mesher.h"

DECLARE_string(test_data_path);

namespace VIO {

class MesherFixture : public ::testing::Test {
 public:
  MesherFixture()
      : img_(),
        img_name_(),
        frame_(nullptr),
        camera_params(),
        mesher_params_(),
        mesher_(nullptr) {
    img_name_ = std::string(FLAGS_test_data_path) + "/chessboard_small.png";
    img_ = UtilsOpenCV::ReadAndConvertToGrayScale(img_name_);
    // Construct a frame from image name, and extract keypoints/landmarks.
    frame_ = constructFrame(true);
    mesher_params_ =
        MesherParams(camera_params.body_Pose_cam_, camera_params.image_size_);
    mesher_ = VIO::make_unique<Mesher>(mesher_params_);
  }

 protected:
  virtual void SetUp() override {}
  virtual void TearDown() override {}

 private:
  std::unique_ptr<Frame> constructFrame(bool extract_corners) {
    // Construct a frame from image name.
    FrameId id = 0;
    Timestamp tmp = 123;

    std::unique_ptr<Frame> frame =
        VIO::make_unique<Frame>(id, tmp, CameraParams(), img_);

    if (extract_corners) {
      UtilsOpenCV::ExtractCorners(frame->img_, &frame->keypoints_);
      // Populate landmark structure with fake data.
      for (int i = 0; i < frame->keypoints_.size(); i++) {
        frame->landmarks_.push_back(i);
      }
    }

    return frame;
  }

 protected:
  static constexpr double tol = 1e-8;

  cv::Mat img_;
  std::string img_name_;
  std::unique_ptr<Frame> frame_;
  CameraParams camera_params;
  MesherParams mesher_params_;
  Mesher::UniquePtr mesher_;
};

/* ************************************************************************* *
TEST_F(MesherFixture, getRatioBetweenLargestAnSmallestSide) {
  mesher_.map_points_3d_.push_back(cv::Point3f(0.5377, 0.3188, 3.5784));   //
pt0 mesher_.map_points_3d_.push_back(cv::Point3f(1.8339, -1.3077, 2.7694));  //
pt1 mesher_.map_points_3d_.push_back( cv::Point3f(-2.2588, -0.4336, -1.3499));
// pt2 mesher_.map_points_3d_.push_back(cv::Point3f(0.8622, 0.3426, 3.0349)); //
pt3

  int rowId_pt1 = 0, rowId_pt2 = 2, rowId_pt3 = 3;
  double d12_out, d23_out, d31_out;
  double actual_ratio = mesher_.getRatioBetweenSmallestAndLargestSide(
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

/* ************************************************************************* *
TEST_F(MesherFixture, getRatioBetweenLargestAnSmallestSide2) {
  mesher_.map_points_3d_.push_back(cv::Point3f(0.5377, 0.3188, 3.5784));   //
pt0 mesher_.map_points_3d_.push_back(cv::Point3f(1.8339, -1.3077, 2.7694));  //
pt1 mesher_.map_points_3d_.push_back( cv::Point3f(-2.2588, -0.4336, -1.3499));
// pt2 mesher_.map_points_3d_.push_back(cv::Point3f(0.8622, 0.3426, 3.0349)); //
pt3

  int rowId_pt1 = 0, rowId_pt2 = 2, rowId_pt3 = 3;
  double d12_out_actual, d23_out_actual, d31_out_actual;
  double actual_ratio = mesher_.getRatioBetweenSmallestAndLargestSide(
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
TEST_F(MesherFixture, createMesh2D) {
  // Compute mesh with all points.
  std::vector<size_t> selected_indices(frame_->keypoints_.size());
  std::iota(std::begin(selected_indices), std::end(selected_indices), 0);
  // Compute mesh.
  const std::vector<cv::Vec6f>& triangulation2D =
      Mesher::createMesh2D(*frame_, selected_indices);

  // Expected triangulation
  //  3 -- 2
  //  | /  |
  //  1 -- 0

  // triangle 1:
  cv::Vec6f triangle1 = triangulation2D[0];
  double triangle1_pt1_x = double(triangle1[0]);
  ASSERT_DOUBLE_EQ(frame_->keypoints_[2].x, triangle1_pt1_x);
  double triangle1_pt1_y = double(triangle1[1]);
  ASSERT_DOUBLE_EQ(frame_->keypoints_[2].y, triangle1_pt1_y);
  double triangle1_pt2_x = double(triangle1[2]);
  ASSERT_DOUBLE_EQ(frame_->keypoints_[1].x, triangle1_pt2_x);
  double triangle1_pt2_y = double(triangle1[3]);
  ASSERT_DOUBLE_EQ(frame_->keypoints_[1].y, triangle1_pt2_y);
  double triangle1_pt3_x = double(triangle1[4]);
  ASSERT_DOUBLE_EQ(frame_->keypoints_[3].x, triangle1_pt3_x);
  double triangle1_pt3_y = double(triangle1[5]);
  ASSERT_DOUBLE_EQ(frame_->keypoints_[3].y, triangle1_pt3_y);

  // triangle 2:
  cv::Vec6f triangle2 = triangulation2D[1];
  double triangle2_pt1_x = double(triangle2[0]);
  ASSERT_DOUBLE_EQ(frame_->keypoints_[1].x, triangle2_pt1_x);
  double triangle2_pt1_y = double(triangle2[1]);
  ASSERT_DOUBLE_EQ(frame_->keypoints_[1].y, triangle2_pt1_y);
  double triangle2_pt2_x = double(triangle2[2]);
  ASSERT_DOUBLE_EQ(frame_->keypoints_[2].x, triangle2_pt2_x);
  double triangle2_pt2_y = double(triangle2[3]);
  ASSERT_DOUBLE_EQ(frame_->keypoints_[2].y, triangle2_pt2_y);
  double triangle2_pt3_x = double(triangle2[4]);
  ASSERT_DOUBLE_EQ(frame_->keypoints_[0].x, triangle2_pt3_x);
  double triangle2_pt3_y = double(triangle2[5]);
  ASSERT_DOUBLE_EQ(frame_->keypoints_[0].y, triangle2_pt3_y);
}

/* ************************************************************************* */
TEST_F(MesherFixture, createMesh2dNoKeypoints) {
  // Compute mesh without points.
  const std::vector<cv::Vec6f>& triangulation2D =
      Mesher::createMesh2D(*frame_, std::vector<size_t>());
  ASSERT_EQ(triangulation2D.size(), 0);
}

}  // namespace VIO
