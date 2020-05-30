/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testStereoCamera.cpp
 * @brief  test StereoCamera
 * @author Antoni Rosinol
 */

#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/mesh/MeshUtils.h"  // a bit weird... it's for isValidPoint.
#include "kimera-vio/pipeline/Pipeline-definitions.h"

DECLARE_string(test_data_path);
DECLARE_bool(display);

namespace VIO {

class StereoCameraFixture : public ::testing::Test {
 public:
  StereoCameraFixture()
      : vio_params_(FLAGS_test_data_path + "/EurocParams"),
        stereo_camera_(nullptr),
        left_frame_queue_("left_frame_queue"),
        right_frame_queue_("right_frame_queue"),
        window_() {
    // Parse data
    parseEuroc();
    // Create Stereo Camera
    stereo_camera_ = VIO::make_unique<StereoCamera>(
        vio_params_.camera_params_.at(0),
        vio_params_.camera_params_.at(1),
        vio_params_.frontend_params_.stereo_matching_params_);
    CHECK_NOTNULL(stereo_camera_);
  }
  ~StereoCameraFixture() override = default;

 protected:
  void SetUp() override {}
  void TearDown() override {}

  void parseEuroc() {
    // Create euroc data parser
    // Only parse one stereo frame... 0 - 1
    euroc_data_provider_ = VIO::make_unique<EurocDataProvider>(
        FLAGS_test_data_path + "/V1_01_easy/", 10, 11, vio_params_);

    // Register Callbacks
    euroc_data_provider_->registerLeftFrameCallback(std::bind(
        &StereoCameraFixture::fillLeftFrameQueue, this, std::placeholders::_1));
    euroc_data_provider_->registerRightFrameCallback(
        std::bind(&StereoCameraFixture::fillRightFrameQueue,
                  this,
                  std::placeholders::_1));

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

  /** Visualization **/
  void drawPixelOnImg(const cv::Point2f& pixel,
                      cv::Mat& img,
                      const cv::viz::Color& color = cv::viz::Color::red(),
                      const size_t& pixel_size = 5u,
                      const uint8_t& alpha = 255u) {
    // Draw the pixel on the image
    cv::Scalar color_with_alpha =
        cv::Scalar(color[0], color[1], color[2], alpha);
    cv::circle(img, pixel, pixel_size, color_with_alpha, -1);
  }

  void drawPixelsOnImg(const std::vector<cv::Point2f>& pixels,
                       cv::Mat& img,
                       const cv::viz::Color& color = cv::viz::Color::red(),
                       const size_t& pixel_size = 5u,
                       const uint8_t& alpha = 255u) {
    // Draw the pixel on the image
    for (const auto& pixel : pixels) {
      drawPixelOnImg(pixel, img, color, pixel_size, alpha);
    }
  }

  void spinDisplay() {
    // Display 3D window
    static constexpr bool kDisplay = false;
    if (kDisplay) {
      window_.spin();
    }
  }

 protected:
  // Default Parms
  //! Params
  VioParams vio_params_;
  StereoCamera::UniquePtr stereo_camera_;

  EurocDataProvider::UniquePtr euroc_data_provider_;
  ThreadsafeQueue<Frame::UniquePtr> left_frame_queue_;
  ThreadsafeQueue<Frame::UniquePtr> right_frame_queue_;

 private:
  cv::viz::Viz3d window_;
};

/**
 * @brief TEST_F backProjectDisparityTo3D basically check that when a disparity
 * map is computed, and its 3D projection calculated, the 3D points still
 * project into the same pixels that generated them (implicitly what this
 * really tests is that the project function works, rather than the
 * backProjectDisparityTo3D...)
 */
TEST_F(StereoCameraFixture, backProjectDisparityTo3D) {
  CHECK_NOTNULL(stereo_camera_);

  Frame::UniquePtr left_frame = nullptr;
  Frame::UniquePtr right_frame = nullptr;
  left_frame_queue_.pop(left_frame);
  right_frame_queue_.pop(right_frame);
  CHECK(left_frame);
  CHECK(right_frame);
  StereoFrame stereo_frame(
      left_frame->id_,
      left_frame->timestamp_,
      *left_frame,
      *right_frame,
      vio_params_.frontend_params_.stereo_matching_params_);

  // Compute depth map just to see.
  cv::Mat disp_img =
      cv::Mat(left_frame->img_.rows, left_frame->img_.cols, CV_32F);
  CHECK(stereo_frame.isRectified());
  stereo_camera_->undistortRectifyStereoFrame(&stereo_frame);
  stereo_camera_->stereoDisparityReconstruction(
      stereo_frame.getLeftImgRectified(),
      stereo_frame.getRightImgRectified(),
      &disp_img);
  cv::Mat disp_viz_img;
  UtilsOpenCV::getDisparityVis(disp_img, disp_viz_img, 1.0);
  cv::imshow("Left Image", stereo_frame.getLeftImgRectified());
  cv::imshow("Right Image", stereo_frame.getRightImgRectified());
  cv::imshow("Disparity Image", disp_viz_img);

  // Check
  // https://github.com/opencv/opencv/blob/master/samples/cpp/stereo_match.cpp
  cv::Mat floatDisp;
  disp_img.convertTo(floatDisp, CV_32F, 1.0f / 16.0f);
  // disp_img.convertTo(floatDisp, CV_32F, 1.0f / 16.0f);
  disp_img = floatDisp;

  // I think this is the perfect container for mesh optimization
  // since it encodes in (u, v) => (x, y, z).
  // Maybe ideally it should be (u, v) => 1/z
  cv::Mat_<cv::Point3f> depth_map;
  // Need to move all points according to pose of stereo camera!
  // since backProjectDisparityTo3D gives the depth map in camera coords
  stereo_camera_->backProjectDisparityTo3D(disp_img, &depth_map);
  CHECK_EQ(depth_map.type(), CV_32FC3);
  CHECK_EQ(depth_map.rows, left_frame->img_.rows);
  CHECK_EQ(depth_map.cols, left_frame->img_.cols);
  // Would that work? interpret xyz as rgb?
  cv::imshow("Depth Image", depth_map);
  if (FLAGS_display) {
    cv::waitKey(1);
  }

  // CHECK that the projection of the depth_map falls inside the expected
  // pixels!
  // La pregunta es, em de fer la projeccio utilitzant la rectified o la no
  // rectified
  // i els pixels seran en camera coords ints o que?
  static constexpr float kMaxZ = 5.0;  // 5 meters
  KeypointsCV expected_left_kpts;
  KeypointsCV expected_right_kpts;
  KeypointsCV actual_left_kpts;
  KeypointsCV actual_right_kpts;
  for (int32_t u = 0; u < depth_map.rows; ++u) {
    for (int32_t v = 0; v < depth_map.cols; ++v) {
      const cv::Point3f& xyz = depth_map.at<cv::Point3f>(u, v);
      if (isValidPoint(xyz) && xyz.z <= kMaxZ) {
        // Convert xyz to global coords (as they are given in camera coords).
        const gtsam::Pose3& left_cam_rect_pose =
            stereo_camera_->getLeftCamRectPose();
        // transform from left cam frame of reference to body because the
        // stereo camera projection function expects landmarks in the body
        // frame of reference!
        const gtsam::Point3& pt_body = left_cam_rect_pose.transformFrom(
            gtsam::Point3(xyz.x, xyz.y, xyz.z));
        LandmarkCV lmk_cv;
        lmk_cv.x = pt_body.x();
        lmk_cv.y = pt_body.y();
        lmk_cv.z = pt_body.z();
        // Ok, now for real, project to camera and get pixel coordinates.
        KeypointCV kp_left;
        KeypointCV kp_right;
        stereo_camera_->project(lmk_cv, &kp_left, &kp_right);
        // We would expect the 3D position of the pixel in u,v coords to
        // project at or at least near (u, v).
        actual_left_kpts.push_back(kp_left);
        expected_left_kpts.push_back(KeypointCV(u, v));
        actual_right_kpts.push_back(kp_right);
        expected_right_kpts.push_back(
            KeypointCV(u, v - disp_img.at<float>(u, v)));
      }
    }
  }
  compareKeypoints(expected_left_kpts, actual_left_kpts, 0.0001f);
  compareKeypoints(expected_right_kpts, actual_right_kpts, 0.0001f);
}

}  // namespace VIO
