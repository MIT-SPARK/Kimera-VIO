/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testUndistortRectifier.h
 * @brief  test UndistortRectifier
 * @author Antoni Rosinol
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

DECLARE_string(test_data_path);

class UndistortRectifierFixture : public ::testing::Test {
 public:
  UndistortRectifierFixture() : cam_params_left(), cam_params_right() {
    cam_params_left.parseYAML(stereo_FLAGS_test_data_path + "/sensorLeft.yaml");
    cam_params_right.parseYAML(stereo_FLAGS_test_data_path + "/sensorRight.yaml");

    // Construct UndistortRectifier
  }

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  // Generate keypoints regularly spaced along a grid of size num_rows x num_rows.
  void GeneratePointGrid(const size_t& num_rows,
                         const size_t& num_cols,
                         const size_t& image_height,
                         const size_t& image_width,
                         KeypointsCV* keypoints) {
    CHECK_NOTNULL(keypoints);
    for (size_t r = 0; r < num_rows; r++) {
      for (size_t c = 0; c < num_cols; c++) {
        int x = image_width / (num_cols - 1) * c;
        int y = image_height / (num_rows - 1) * r;
        keypoints->push_back(cv::Point2f(x, y));
      }
    }
  }

  CameraParams cam_params_left;
  CameraParams cam_params_right;
};
