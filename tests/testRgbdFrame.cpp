/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testRgbdFrame.cpp
 * @author Nathan Hughes
 */

#include <gtest/gtest.h>

#include "kimera-vio/frontend/RgbdCamera.h"
#include "kimera-vio/frontend/RgbdFrame.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

DECLARE_string(test_data_path);

namespace VIO {

struct TestRgbdFrame : public ::testing::Test {
  TestRgbdFrame()
      : ::testing::Test(), rgbd_data_path(FLAGS_test_data_path + "/ForRgbd/") {}

  virtual ~TestRgbdFrame() = default;

  virtual void SetUp() {}

  virtual void TearDown() {}

  cv::Mat loadDepthImage(const std::string& name) {
    std::string image_path = rgbd_data_path + name;
    return cv::imread(image_path, cv::IMREAD_UNCHANGED);
  }

  cv::Mat loadMonoImage(const std::string& name) {
    std::string image_path = rgbd_data_path + name;
    return UtilsOpenCV::ReadAndConvertToGrayScale(image_path);
  }

  std::string rgbd_data_path;
};

TEST_F(TestRgbdFrame, ConstructorInvariants) {
  const auto depth_img = loadDepthImage("depth_img_0.tiff");
  const auto mono_img = loadMonoImage("left_img_0.png");
  EXPECT_TRUE(!depth_img.empty());
  EXPECT_TRUE(!mono_img.empty());

  CameraParams params;

  FrameId expected_id = 5;
  Timestamp expected_stamp = 10;

  DepthFrame dframe(expected_id, expected_stamp, depth_img);
  Frame lframe(expected_id, expected_stamp, params, mono_img);
  RgbdFrame frame(expected_id, expected_stamp, lframe, dframe);

  EXPECT_EQ(frame.timestamp_, expected_stamp);
  EXPECT_EQ(frame.id_, expected_id);
}

TEST_F(TestRgbdFrame, GetStereoFrame) {
  const auto depth_img = loadDepthImage("depth_img_0.tiff");
  const auto mono_img = loadMonoImage("left_img_0.png");

  DepthFrame dframe(5, 10, depth_img);

  CameraParams params;
  Frame lframe(5, 10, params, mono_img);

  RgbdFrame frame(5, 10, lframe, dframe);

  const auto stereo_frame = frame.getStereoFrame();
  ASSERT_TRUE(stereo_frame != nullptr);
  EXPECT_EQ(stereo_frame->id_, frame.id_);
  EXPECT_EQ(stereo_frame->timestamp_, frame.timestamp_);
  EXPECT_TRUE(stereo_frame->right_frame_.img_.empty());
  EXPECT_FALSE(stereo_frame->left_frame_.img_.empty());
}

TEST_F(TestRgbdFrame, FillStereoFrame) {
  CameraParams params;
  params.parseYAML(rgbd_data_path + "sensorLeft.yaml");
  params.depth.depth_to_meters_ = 1.0;
  params.depth.min_depth_ = 1.5;
  params.depth.is_registered_ = true;
  params.depth.virtual_baseline_ = 1.0e-1f;

  RgbdCamera camera(params);

  // set depth to be scaled by fx to ensure disparity remains inside image
  const float fx = camera.getCamParams().intrinsics_.at(0);
  cv::Mat depth_img(2, 2, CV_32FC1);
  depth_img.at<float>(0, 0) = 1.0f;
  depth_img.at<float>(0, 1) = fx * 2.0f;
  depth_img.at<float>(1, 0) = 3.0f;
  depth_img.at<float>(1, 1) = fx * 4.0f;
  cv::Mat mono_img(2, 2, CV_8UC1);

  DepthFrame dframe(5, 10, depth_img);
  Frame lframe(5, 10, params, mono_img);

  RgbdFrame rgbd_frame(5, 10, lframe, dframe);

  {  // case 1: no features
    auto frame = rgbd_frame.getStereoFrame();
    ASSERT_TRUE(frame != nullptr);
    rgbd_frame.fillStereoFrame(camera, *frame);
    EXPECT_TRUE(frame->right_keypoints_rectified_.empty());
    EXPECT_TRUE(frame->keypoints_depth_.empty());
    EXPECT_TRUE(frame->keypoints_3d_.empty());
    EXPECT_TRUE(frame->right_frame_.keypoints_.empty());
  }

  std::cout << "starting second case" << std::endl;
  {  // case 2: some features
    auto frame = rgbd_frame.getStereoFrame();
    ASSERT_TRUE(frame != nullptr);

    // versors are invalid, but set to 1 to avoid divide by zero
    frame->left_frame_.keypoints_.push_back(KeypointCV(0.0, 0.0));
    frame->left_frame_.versors_.push_back(gtsam::Vector3::Ones());
    frame->left_frame_.keypoints_.push_back(KeypointCV(1.0, 0.0));
    frame->left_frame_.versors_.push_back(gtsam::Vector3::Ones());
    frame->left_frame_.keypoints_.push_back(KeypointCV(0.0, 1.0));
    frame->left_frame_.versors_.push_back(gtsam::Vector3::Ones());
    frame->left_frame_.keypoints_.push_back(KeypointCV(1.0, 1.0));
    frame->left_frame_.versors_.push_back(gtsam::Vector3::Ones());

    frame->left_keypoints_rectified_.push_back(
        {KeypointStatus::VALID, frame->left_frame_.keypoints_[0]});
    frame->left_keypoints_rectified_.push_back(
        {KeypointStatus::VALID, frame->left_frame_.keypoints_[1]});
    frame->left_keypoints_rectified_.push_back(
        {KeypointStatus::FAILED_ARUN, frame->left_frame_.keypoints_[2]});
    frame->left_keypoints_rectified_.push_back(
        {KeypointStatus::VALID, frame->left_frame_.keypoints_[3]});

    rgbd_frame.fillStereoFrame(camera, *frame);
    ASSERT_EQ(frame->right_keypoints_rectified_.size(), 4u);
    ASSERT_EQ(frame->keypoints_depth_.size(), 4u);
    ASSERT_EQ(frame->keypoints_3d_.size(), 4u);
    ASSERT_EQ(frame->right_frame_.keypoints_.size(), 4u);

    std::vector<float> expected_depths{0.0f, fx * 2.0f, 0.0, fx * 4.0f};
    for (size_t i = 0; i < expected_depths.size(); ++i) {
      EXPECT_NEAR(frame->keypoints_depth_[i], expected_depths[i], 1.0e-9);
    }

    EXPECT_EQ(frame->right_keypoints_rectified_[0].first,
              KeypointStatus::NO_DEPTH);
    EXPECT_EQ(frame->right_keypoints_rectified_[1].first,
              KeypointStatus::VALID);
    EXPECT_EQ(frame->right_keypoints_rectified_[2].first,
              KeypointStatus::FAILED_ARUN);
    EXPECT_EQ(frame->right_keypoints_rectified_[3].first,
              KeypointStatus::VALID);
    // 1 - disparity (where disparity is 0.1 * 1/2 or 1/4)
    EXPECT_NEAR(frame->right_keypoints_rectified_[1].second.x, 0.95, 1.0e-6);
    EXPECT_NEAR(frame->right_keypoints_rectified_[3].second.x, 0.975, 1.0e-6);
  }
}

}  // namespace VIO
