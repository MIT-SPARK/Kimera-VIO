/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testCamera.cpp
 * @brief  test Camera
 * @author Antoni Rosinol
 */

#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"

DECLARE_string(test_data_path);

namespace VIO {

class CameraFixture : public ::testing::Test {
 public:
  CameraFixture()
      : vio_params_(FLAGS_test_data_path + "/EurocParams"),
        mono_camera_(nullptr),
        left_frame_queue_("left_frame_queue"),
        right_frame_queue_("right_frame_queue") {
        // window_() {
    // Parse data
    parseEuroc();
    // Create Mono Camera
    mono_camera_ = VIO::make_unique<Camera>(vio_params_.camera_params_.at(0));
    CHECK(mono_camera_);
  }
  ~CameraFixture() override = default;

 protected:
  void SetUp() override {}
  void TearDown() override {}

  void parseEuroc() {
    // Create euroc data parser
    // Only parse one mono frame... 0 - 1
    euroc_data_provider_ = VIO::make_unique<EurocDataProvider>(
        FLAGS_test_data_path + "/MicroEurocDataset/", 10, 11, vio_params_);

    // Register Callbacks
    euroc_data_provider_->registerLeftFrameCallback(std::bind(
        &CameraFixture::fillLeftFrameQueue, this, std::placeholders::_1));
    euroc_data_provider_->registerRightFrameCallback(std::bind(
        &CameraFixture::fillRightFrameQueue, this, std::placeholders::_1));

    // Parse Euroc dataset.
    // Since we run in sequential mode, we need to spin it till it finishes.
    while (euroc_data_provider_->spin()) {
    };  // Fill queues.
  }

  //! Callbacks to fill queues: they should be all lighting fast.
  void fillLeftFrameQueue(Frame::UniquePtr left_frame) {
    CHECK(left_frame);
    left_frame_queue_.push(std::move(left_frame));
  }

  //! Callbacks to fill queues: they should be all lighting fast.
  void fillRightFrameQueue(Frame::UniquePtr left_frame) {
    CHECK(left_frame);
    right_frame_queue_.push(std::move(left_frame));
  }

  /**
   * @brief compareKeypoints compares two sets of keypoints
   */
  void compareKeypoints(const KeypointsCV& kpts_1,
                        const KeypointsCV& kpts_2,
                        const float& tol) {
    ASSERT_EQ(kpts_1.size(), kpts_2.size());
    for (size_t i = 0u; i < kpts_1.size(); i++) {
      const auto& kpt_1 = kpts_1[i];
      const auto& kpt_2 = kpts_2[i];
      EXPECT_NEAR(kpt_1.x, kpt_2.x, tol);
      EXPECT_NEAR(kpt_1.y, kpt_2.y, tol);
    }
  }

  /**
   * @brief compareLandmarks compares two sets of landmarks
   */
  void compareLandmarks(const LandmarksCV& lmks_1,
                        const LandmarksCV& lmks_2,
                        const float& tol) {
    ASSERT_EQ(lmks_1.size(), lmks_2.size());
    for (size_t i = 0u; i < lmks_1.size(); i++) {
      const auto& lmk_1 = lmks_1[i];
      const auto& lmk_2 = lmks_2[i];
      EXPECT_NEAR(lmk_1.x, lmk_2.x, tol);
      EXPECT_NEAR(lmk_1.y, lmk_2.y, tol);
      EXPECT_NEAR(lmk_1.z, lmk_2.z, tol);
    }
  }

  /** Visualization **/
  // void drawPixelOnImg(const cv::Point2f& pixel,
  //                     cv::Mat& img,
  //                     const cv::viz::Color& color = cv::viz::Color::red(),
  //                     const size_t& pixel_size = 5u,
  //                     const uint8_t& alpha = 255u) {
  //   // Draw the pixel on the image
  //   cv::Scalar color_with_alpha =
  //       cv::Scalar(color[0], color[1], color[2], alpha);
  //   cv::circle(img, pixel, pixel_size, color_with_alpha, -1);
  // }

  // void drawPixelsOnImg(const std::vector<cv::Point2f>& pixels,
  //                      cv::Mat& img,
  //                      const cv::viz::Color& color = cv::viz::Color::red(),
  //                      const size_t& pixel_size = 5u,
  //                      const uint8_t& alpha = 255u) {
  //   // Draw the pixel on the image
  //   for (const auto& pixel : pixels) {
  //     drawPixelOnImg(pixel, img, color, pixel_size, alpha);
  //   }
  // }

  // void spinDisplay() {
  //   // Display 3D window
  //   static constexpr bool kDisplay = false;
  //   if (kDisplay) {
  //     window_.spin();
  //   }
  // }

 protected:
  // Default Parms
  //! Params
  VioParams vio_params_;
  Camera::UniquePtr mono_camera_;

  EurocDataProvider::UniquePtr euroc_data_provider_;
  ThreadsafeQueue<Frame::UniquePtr> left_frame_queue_;
  ThreadsafeQueue<Frame::UniquePtr> right_frame_queue_;

//  private:
//   cv::viz::Viz3d window_;
};

TEST_F(CameraFixture, project) {
  LandmarksCV lmks;
  lmks.push_back(LandmarkCV(0.0, 0.0, 1.0));
  lmks.push_back(LandmarkCV(0.0, 0.0, 2.0));
  lmks.push_back(LandmarkCV(0.0, 1.0, 2.0));
  lmks.push_back(LandmarkCV(0.0, 10.0, 20.0));
  lmks.push_back(LandmarkCV(1.0, 0.0, 2.0));

  CameraParams& camera_params = vio_params_.camera_params_.at(0);
  // Make it easy first, use identity pose and simple intrinsics
  camera_params.body_Pose_cam_ = gtsam::Pose3();
  CameraParams::Intrinsics& intrinsics = camera_params.intrinsics_;
  intrinsics.at(0) = 1.0;  // fx
  intrinsics.at(1) = 1.0;  // fy
  intrinsics.at(2) = 3.0;  // u0
  intrinsics.at(3) = 2.0;  // v0
  KeypointsCV expected_kpts;
  expected_kpts.push_back(KeypointCV(intrinsics.at(2), intrinsics.at(3)));
  expected_kpts.push_back(KeypointCV(intrinsics.at(2), intrinsics.at(3)));
  expected_kpts.push_back(KeypointCV(3.0, 1.0 / 2.0 + 2.0));
  expected_kpts.push_back(KeypointCV(3.0, 1.0 / 2.0 + 2.0));
  expected_kpts.push_back(KeypointCV(1.0 / 2.0 + 3.0, 2.0));

  mono_camera_ = VIO::make_unique<Camera>(camera_params);

  KeypointsCV actual_kpts;
  EXPECT_NO_THROW(mono_camera_->project(lmks, &actual_kpts));
  compareKeypoints(expected_kpts, actual_kpts, 0.0001f);
}

TEST_F(CameraFixture, projectCheirality) {
  // landmark behind camera
  CameraParams& camera_params = vio_params_.camera_params_.at(0);
  // Make it easy first, use identity pose and simple intrinsics
  camera_params.body_Pose_cam_ = gtsam::Pose3();
  mono_camera_ = VIO::make_unique<Camera>(camera_params);

  LandmarkCV lmk_behind_cam = LandmarkCV(0.0, 0.0, -2.0);

  KeypointCV kpt;
  EXPECT_THROW(mono_camera_->project(lmk_behind_cam, &kpt),
               gtsam::CheiralityException);
}

TEST_F(CameraFixture, backProjectSingleSimple) {
  // Easy test first, back-project keypoint at the center of the image with
  // a given depth.
  CameraParams& camera_params = vio_params_.camera_params_.at(0);
  camera_params.body_Pose_cam_ = gtsam::Pose3();
  mono_camera_ = VIO::make_unique<Camera>(camera_params);

  KeypointCV kpt(camera_params.intrinsics_.at(2),
                 camera_params.intrinsics_.at(3));
  LandmarkCV actual_lmk;
  double depth = 2.0;
  mono_camera_->backProject(kpt, depth, &actual_lmk);

  LandmarkCV expected_lmk(0.0, 0.0, depth);
  EXPECT_NEAR(expected_lmk.x, actual_lmk.x, 0.0001);
  EXPECT_NEAR(expected_lmk.y, actual_lmk.y, 0.0001);
  EXPECT_NEAR(expected_lmk.z, actual_lmk.z, 0.0001);
}

// Removed because projectOmni is not yet implemented.
// TEST_F(CameraFixture, projectSingleOmni) {
//   CameraParams camera_params;
//   camera_params.parseYAML(FLAGS_test_data_path +
//                           "/ForOmniCamera/OmniCamParams.yaml");
//   mono_camera_ = VIO::make_unique<Camera>(camera_params);

//   LandmarkCV point_3d(0.2,0.3,3.0);
//   KeypointCV actual_kpt;
//   mono_camera_->project(point_3d, &actual_kpt);

//   KeypointCV expected_kpt();
//   EXPECT_NEAR(expected_kpt.x, actual_kpt.x, 0.0001);
//   EXPECT_NEAR(expected_kpt.y, actual_kpt.y, 0.0001);
// }

TEST_F(CameraFixture, backProjectSingleOmni) {
  CameraParams camera_params;
  camera_params.parseYAML(FLAGS_test_data_path +
                          "/ForOmniCamera/OmniCamParams.yaml");
  mono_camera_ = VIO::make_unique<Camera>(camera_params);

  KeypointCV kpt(490.397735595703, 423.252136230469);  // From MATLAB
  LandmarkCV actual_lmk;
  double depth = 1.0;
  mono_camera_->backProject(kpt, depth, &actual_lmk);

  LandmarkCV expected_lmk(
      -0.682794074354265, -0.423626331144069, 1);
  EXPECT_NEAR(expected_lmk.x, actual_lmk.x, 0.0001);
  EXPECT_NEAR(expected_lmk.y, actual_lmk.y, 0.0001);
  EXPECT_NEAR(expected_lmk.z, actual_lmk.z, 0.0001);
}

TEST_F(CameraFixture, undistortKeypointsOmni) {
  CameraParams camera_params;
  camera_params.parseYAML(FLAGS_test_data_path +
                          "/ForOmniCamera/OmniCamParams.yaml");
  mono_camera_ = VIO::make_unique<Camera>(camera_params);

  KeypointsCV distorted_kpts;
  distorted_kpts.push_back(
      KeypointCV(490.3977355957031, 423.2521362304688));  // From MATLAB

  StatusKeypointsCV actual_kpts;
  mono_camera_->undistortKeypoints(distorted_kpts, &actual_kpts);
  KeypointCV actual_kpt = actual_kpts.at(0).second;

  KeypointCV expected_kpt(488.4044442343012, 432.7802516419484);  // From MATLAB
  EXPECT_NEAR(expected_kpt.x, actual_kpt.x, 0.0001);
  EXPECT_NEAR(expected_kpt.y, actual_kpt.y, 0.0001);
}

TEST_F(CameraFixture, backProjectMultipleSimple) {
  // Easy test first, back-project keypoints at the center of the image with
  // different depths.
  CameraParams& camera_params = vio_params_.camera_params_.at(0);
  camera_params.body_Pose_cam_ = gtsam::Pose3();
  mono_camera_ = VIO::make_unique<Camera>(camera_params);

  KeypointCV kpt(camera_params.intrinsics_.at(2),
                 camera_params.intrinsics_.at(3));
  // Create 3 keypoints centered at image with different depths
  KeypointsCV kpts(3, kpt);
  std::vector<double> depths = {2.0, 3.0, 4.5};
  LandmarksCV actual_lmks;
  mono_camera_->backProject(kpts, depths, &actual_lmks);

  LandmarksCV expected_lmks;
  for (const auto& depth : depths) {
    expected_lmks.push_back(LandmarkCV(0.0, 0.0, depth));
  }

  compareLandmarks(actual_lmks, expected_lmks, 0.0001);
}

TEST_F(CameraFixture, backProjectSingleTopLeft) {
  // Back-project keypoint at the center of the image with a given depth.
  CameraParams& camera_params = vio_params_.camera_params_.at(0);
  double fx = 3.0 / 2.0;
  double fy = 1.0 / 2.0;
  double px = 190.0;
  double py = 240.0;
  camera_params.intrinsics_ = {fx, fy, px, py};
  camera_params.body_Pose_cam_ = gtsam::Pose3();
  mono_camera_ = VIO::make_unique<Camera>(camera_params);

  LandmarkCV actual_lmk;
  double depth = 2.0;
  KeypointCV kpt(0.0, 0.0);  // Top-left corner
  mono_camera_->backProject(kpt, depth, &actual_lmk);

  LandmarkCV expected_lmk(depth / fx * (-px), depth / fy * (-py), depth);
  EXPECT_NEAR(expected_lmk.x, actual_lmk.x, 0.0001);
  EXPECT_NEAR(expected_lmk.y, actual_lmk.y, 0.0001);
  EXPECT_NEAR(expected_lmk.z, actual_lmk.z, 0.0001);
}

TEST_F(CameraFixture, backProjectSingleRandom) {
  // Back-project keypoint at the center of the image with a given depth.
  CameraParams& camera_params = vio_params_.camera_params_.at(0);
  double fx = 30.9 / 2.2;
  double fy = 12.0 / 23.0;
  double px = 390.8;
  double py = 142.2;
  camera_params.intrinsics_ = {fx, fy, px, py};
  camera_params.body_Pose_cam_ = gtsam::Pose3();
  mono_camera_ = VIO::make_unique<Camera>(camera_params);

  LandmarkCV actual_lmk;
  double depth = 1.3;
  KeypointCV kpt(123.2f, 450.9f);  // Top-left corner
  mono_camera_->backProject(kpt, depth, &actual_lmk);

  LandmarkCV expected_lmk(
      depth / fx * (kpt.x - px), depth / fy * (kpt.y - py), depth);
  EXPECT_NEAR(expected_lmk.x, actual_lmk.x, 0.0001);
  EXPECT_NEAR(expected_lmk.y, actual_lmk.y, 0.0001);
  EXPECT_NEAR(expected_lmk.z, actual_lmk.z, 0.0001);
}

TEST_F(CameraFixture, backProjectMultipleComplex) {
  // Back-project keypoints at the center of the image with
  // different depths.
  CameraParams& camera_params = vio_params_.camera_params_.at(0);
  double fx = 30.9 / 2.2;
  double fy = 12.0 / 23.0;
  double px = 390.8;
  double py = 142.2;
  camera_params.intrinsics_ = {fx, fy, px, py};
  camera_params.body_Pose_cam_ = gtsam::Pose3();
  mono_camera_ = VIO::make_unique<Camera>(camera_params);

  KeypointsCV kpts;
  std::vector<double> depths;
  static constexpr size_t kKeypoints = 10u;
  //! This is just to avoid 0 depth
  static constexpr double kEpsilonDepth = 0.01;
  // Make a star of pixels so every axis is covered, with random depths
  for (size_t i = 0u; i < kKeypoints; i++) {
    // diagonal top-left bottom-right
    kpts.push_back(
        KeypointCV(i * (camera_params.image_size_.width - 1) / kKeypoints,
                   i * (camera_params.image_size_.height - 1) / kKeypoints));
    depths.push_back(i * 2.3 + kEpsilonDepth);
    // diagonal bottom-left top-right
    kpts.push_back(
        KeypointCV(i * (camera_params.image_size_.width - 1) / kKeypoints,
                   (kKeypoints - i) * (camera_params.image_size_.height - 1) /
                       kKeypoints));
    depths.push_back(i * 1.3 + kEpsilonDepth);
    // horizontal
    kpts.push_back(
        KeypointCV(i * (camera_params.image_size_.width - 1) / kKeypoints, py));
    depths.push_back(i * 0.3 + kEpsilonDepth);
    // vertical
    kpts.push_back(KeypointCV(
        px, i * (camera_params.image_size_.height - 1) / kKeypoints));
    depths.push_back(i * 10.3 + kEpsilonDepth);
  }
  CHECK_EQ(depths.size(), kpts.size());
  // Create 3 keypoints centered at image with different depths
  LandmarksCV actual_lmks;
  mono_camera_->backProject(kpts, depths, &actual_lmks);

  LandmarksCV expected_lmks;
  for (size_t i = 0u; i < kpts.size(); i++) {
    expected_lmks.push_back(LandmarkCV(depths[i] / fx * (kpts[i].x - px),
                                       depths[i] / fy * (kpts[i].y - py),
                                       depths[i]));
  }
  ASSERT_EQ(actual_lmks.size(), expected_lmks.size());

  compareLandmarks(actual_lmks, expected_lmks, 0.0001);
}

}  // namespace VIO
