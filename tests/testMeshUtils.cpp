/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testMeshUtils.cpp
 * @brief  test MeshUtils implementation
 * @author Antoni Rosinol
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kimera-vio/mesh/MeshUtils.h"

DECLARE_string(test_data_path);
DECLARE_bool(display);

namespace VIO {

class MeshUtilsFixture : public ::testing::Test {
 public:
  MeshUtilsFixture() {
    expected_bary_img_ =
        loadImage(FLAGS_test_data_path + '/' + folder_name_, bary_img_name_);
    expected_color_img_ =
        loadImage(FLAGS_test_data_path + '/' + folder_name_, color_img_name_);
  }

 protected:
  virtual void SetUp() override {}
  virtual void TearDown() override {}

  inline float deg2rad(const float& deg) { return deg * M_PI / 180; }
  inline float clamp(const float& lo, const float& hi, const float& v) {
    return std::max(lo, std::min(hi, v));
  }

  /**
   * @brief drawFrameBuffer For visualization of the actual generated images
   * @param framebuffer
   * @param height of the framebuffer
   * @param width of the framebuffer
   * @return Returns drawn image
   */
  cv::Mat drawFrameBuffer(Vec3f* framebuffer, uint32_t height, uint32_t width) {
    CHECK_NOTNULL(framebuffer);
    cv::Mat img = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    for (uint32_t i = 0u; i < height * width; ++i) {
      char r = (char)(255 * clamp(0.0, 1.0, framebuffer[i][0]));
      char g = (char)(255 * clamp(0.0, 1.0, framebuffer[i][1]));
      char b = (char)(255 * clamp(0.0, 1.0, framebuffer[i][2]));
      cv::Vec3b& color = img.at<cv::Vec3b>(i / width, i % width);
      color[0] = r;
      color[1] = g;
      color[2] = b;
    }
    return img;
  }

  /**
   * @brief compareCvImg
   * @param a, b matrices to compare
   * @return  True if they are the same, false otw.
   */
  bool compareCvImg(const cv::Mat& a, const cv::Mat& b) const {
    return cv::countNonZero(a != b) == 0;
  }

  inline cv::Mat loadImage(const std::string& parent_dir,
                           const std::string& file_name) const {
    return cv::imread(parent_dir + '/' + file_name, cv::IMREAD_COLOR);
  }

 protected:
  static constexpr double tol = 1e-8;
  cv::Mat expected_bary_img_;
  cv::Mat expected_color_img_;

 private:
  std::string folder_name_ = "ForMeshUtils";
  std::string bary_img_name_ = "ray_triangle_barycentric.jpg";
  std::string color_img_name_ = "ray_triangle_color.jpg";
};

/**
 * @brief TEST_F This is simply testing that the algorithm is not being modified
 * Its actual validity ought to be tested using for example the bary centric
 * coordinates and comparing where the rayIntersect says vs what plane to line
 * intersection says.
 */
TEST_F(MeshUtilsFixture, testRayIntersectAlgorithm) {
  Vec3f v0(-1, -1, -5);
  Vec3f v1(1, -1, -5);
  Vec3f v2(0, 1, -5);

  const uint32_t width = 640;
  const uint32_t height = 480;

  Vec3f colors[3] = {{0.6, 0.4, 0.1}, {0.1, 0.5, 0.3}, {0.1, 0.3, 0.7}};

  Vec3f* color_framebuffer = new Vec3f[width * height];
  Vec3f* bary_framebuffer = new Vec3f[width * height];

  Vec3f* pix = color_framebuffer;
  Vec3f* bary_pix = bary_framebuffer;

  float fov = 51.52;
  float scale = tan(deg2rad(fov * 0.5));
  float imageAspectRatio = width / (float)height;

  Vec3f orig(0);
  for (uint32_t j = 0; j < height; ++j) {
    for (uint32_t i = 0; i < width; ++i) {
      // compute primary ray
      float x = (2 * (i + 0.5) / (float)width - 1) * imageAspectRatio * scale;
      float y = (1 - 2 * (j + 0.5) / (float)height) * scale;
      Vec3f dir(x, y, -1);
      dir.normalize();
      float t, u, v;
      if (rayTriangleIntersect(orig, dir, v0, v1, v2, t, u, v)) {
        // Interpolate colors using the barycentric coordinates
        *pix = u * colors[0] + v * colors[1] + (1 - u - v) * colors[2];

        // Visualize the row barycentric coordinates
        *bary_pix = Vec3f(u, v, 1 - u - v);
      }
      pix++;
      bary_pix++;
    }
  }

  // Check generated images (and optionally visualize them).
  cv::Mat actual_color_img;
  actual_color_img = drawFrameBuffer(color_framebuffer, height, width);
  compareCvImg(expected_color_img_, actual_color_img);

  if (FLAGS_display) {
    cv::imshow("Expected color img", expected_color_img_);
    cv::imshow("Actual color img", actual_color_img);
  }

  cv::Mat actual_bary_img;
  actual_bary_img = drawFrameBuffer(bary_framebuffer, height, width);
  compareCvImg(expected_bary_img_, actual_bary_img);

  if (FLAGS_display) {
    cv::imshow("Expected bary img", expected_bary_img_);
    cv::imshow("Actual bary img", actual_bary_img);
    cv::waitKey(0);
  }

  delete[] color_framebuffer;
  delete[] bary_framebuffer;
}

}  // namespace VIO
