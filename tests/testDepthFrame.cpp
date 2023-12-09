/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testDepthFrame.cpp
 * @author Nathan Hughes
 */

#include <gtest/gtest.h>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/DepthFrame.h"

DECLARE_string(test_data_path);

namespace VIO {

struct TestDepthFrame : public ::testing::Test {
  TestDepthFrame()
      : ::testing::Test(), rgbd_data_path(FLAGS_test_data_path + "/ForRgbd/") {}

  virtual ~TestDepthFrame() = default;

  virtual void SetUp() {}

  virtual void TearDown() {}

  cv::Mat loadDepthImage(const std::string& name) {
    std::string image_path = rgbd_data_path + name;
    return cv::imread(image_path, cv::IMREAD_UNCHANGED);
  }

  std::string rgbd_data_path;
};

TEST_F(TestDepthFrame, ConstructorInvariants) {
  const auto depth_img = loadDepthImage("depth_img_0.tiff");
  EXPECT_TRUE(!depth_img.empty());

  FrameId expected_id = 5;
  Timestamp expected_stamp = 10;

  DepthFrame frame(expected_id, expected_stamp, depth_img);
  EXPECT_EQ(frame.timestamp_, expected_stamp);
  EXPECT_EQ(frame.id_, expected_id);
  EXPECT_TRUE(!frame.depth_img_.empty());
  EXPECT_TRUE(!frame.is_registered_);
  EXPECT_TRUE(frame.registered_img_.empty());
}

TEST_F(TestDepthFrame, DetectionMask) {
  const auto depth_img = loadDepthImage("depth_img_0.tiff");
  EXPECT_TRUE(!depth_img.empty());

  DepthFrame frame(5, 10, depth_img);
  {  // first case: everything allowed
    CameraParams params;
    params.depth.min_depth_ = 0.0;
    params.depth.max_depth_ = std::numeric_limits<float>::infinity();
    const auto mask = frame.getDetectionMask(params);
    EXPECT_EQ(mask.type(), CV_8UC1);
    const auto total_value = cv::sum(mask)[0];
    const auto avg_value = total_value / mask.total();
    EXPECT_NEAR(avg_value, 255.0, 1.0e-9);
  }

  {  // second case: nothing allowed
    CameraParams params;
    params.depth.min_depth_ = std::numeric_limits<float>::infinity();
    params.depth.max_depth_ = std::numeric_limits<float>::infinity();
    const auto mask = frame.getDetectionMask(params);
    EXPECT_EQ(mask.type(), CV_8UC1);
    const auto total_value = cv::sum(mask)[0];
    const auto avg_value = total_value / mask.total();
    EXPECT_NEAR(avg_value, 0.0, 1.0e-9);
  }

  {  // third case: some depth allowed
    CameraParams params;
    params.depth.min_depth_ = 0.2;
    params.depth.max_depth_ = 3.0;
    const auto mask = frame.getDetectionMask(params);
    EXPECT_EQ(mask.type(), CV_8UC1);
    const auto total_value = cv::sum(mask)[0];
    const auto avg_value = total_value / mask.total();
    EXPECT_GT(avg_value, 0.0);
    EXPECT_LT(avg_value, 255.0);
  }
}

TEST_F(TestDepthFrame, DepthRegistration) {
  const auto depth_img = loadDepthImage("depth_img_0.tiff");
  EXPECT_TRUE(!depth_img.empty());

  DepthFrame frame(5, 10, depth_img);
  EXPECT_FALSE(frame.is_registered_);
  EXPECT_TRUE(frame.registered_img_.empty());

  CameraParams params;
  params.parseYAML(rgbd_data_path + "sensorLeft.yaml");

  frame.registerDepth(params);
  EXPECT_TRUE(frame.is_registered_);
  EXPECT_FALSE(frame.registered_img_.empty());
}

struct TestDepthFrameParam
    : public TestDepthFrame,
      public testing::WithParamInterface<std::function<cv::Mat()>> {
  virtual ~TestDepthFrameParam() = default;
};

cv::Mat makeTestDepthFloat() {
  cv::Mat depth_img(2, 2, CV_32FC1);
  depth_img.at<float>(0, 0) = 1.0f;
  depth_img.at<float>(0, 1) = 2.0f;
  depth_img.at<float>(1, 0) = 3.0f;
  depth_img.at<float>(1, 1) = 4.0f;
  return depth_img;
}

cv::Mat makeTestDepthUINT16() {
  cv::Mat depth_img(2, 2, CV_16UC1);
  depth_img.at<uint16_t>(0, 0) = 1;
  depth_img.at<uint16_t>(0, 1) = 2;
  depth_img.at<uint16_t>(1, 0) = 3;
  depth_img.at<uint16_t>(1, 1) = 4;
  return depth_img;
}

TEST_P(TestDepthFrameParam, GetDepthAtPoint) {
  const auto depth_img = (GetParam())();

  const DepthFrame frame(5, 10, depth_img);

  CameraParams params;
  params.depth.depth_to_meters_ = 1.0;
  params.depth.min_depth_ = 0.1;

  {  // first case: invalid pixels (x)
    KeypointCV point(-1, 0);
    EXPECT_TRUE(std::isnan(frame.getDepthAtPoint(params, point)));
    point.x = 2.0;
    EXPECT_TRUE(std::isnan(frame.getDepthAtPoint(params, point)));
    point.x = 0.0;
    point.y = -1.0;
    EXPECT_TRUE(std::isnan(frame.getDepthAtPoint(params, point)));
    point.y = 2.0;
    EXPECT_TRUE(std::isnan(frame.getDepthAtPoint(params, point)));
  }

  {  // second case: valid point, unit depth
    KeypointCV point(0, 0);
    EXPECT_NEAR(frame.getDepthAtPoint(params, point), 1.0f, 1.0e-9f);
  }

  {  // third case: valid point, non-unit depth
    KeypointCV point(0, 0);
    params.depth.depth_to_meters_ = 5.0;
    EXPECT_NEAR(frame.getDepthAtPoint(params, point), 5.0f, 1.0e-9f);
  }

  {  // fourth case: valid point, non-unit depth, less than min_depth
    KeypointCV point(0, 0);
    params.depth.depth_to_meters_ = 0.01;
    EXPECT_TRUE(std::isnan(frame.getDepthAtPoint(params, point)));
  }
}

INSTANTIATE_TEST_SUITE_P(GetDepthParameterized,
                         TestDepthFrameParam,
                         testing::Values(makeTestDepthFloat,
                                         makeTestDepthUINT16));

}  // namespace VIO
