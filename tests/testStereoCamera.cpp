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
 * @author Marcus Abate
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <string>

#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/mesh/MeshUtils.h"  // a bit weird... it's for isValidPoint.
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"

DECLARE_string(test_data_path);
DECLARE_bool(display);

static const double tol = 1e-7;

namespace VIO {

class StereoCameraFixture : public ::testing::Test {
 public:
  StereoCameraFixture()
      : vio_params_(FLAGS_test_data_path + "/EurocParams"),
        stereo_camera_(nullptr),
        stereo_matcher_(nullptr),
        left_frame_queue_("left_frame_queue"),
        right_frame_queue_("right_frame_queue") {
        // window_() {
    // Parse data
    parseEuroc();
    // Create Stereo Camera
    stereo_camera_ = std::make_shared<StereoCamera>(
        vio_params_.camera_params_.at(0),
        vio_params_.camera_params_.at(1));
    CHECK(stereo_camera_);
    stereo_matcher_ = VIO::make_unique<StereoMatcher>(
        stereo_camera_,
        vio_params_.frontend_params_.stereo_matching_params_);
    CHECK(stereo_matcher_);
  }
  ~StereoCameraFixture() override = default;

 protected:
  void SetUp() override {}
  void TearDown() override {}

  void parseEuroc() {
    // Create euroc data parser
    // Only parse one stereo frame... 0 - 1
    euroc_data_provider_ = VIO::make_unique<EurocDataProvider>(
        FLAGS_test_data_path + "/MicroEurocDataset/", 10, 11, vio_params_);

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

  /**
   * @brief compareLandmarks compares two sets of 3D landmarks
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
  StereoCamera::ConstPtr stereo_camera_;
  StereoMatcher::UniquePtr stereo_matcher_;

  EurocDataProvider::UniquePtr euroc_data_provider_;
  ThreadsafeQueue<Frame::UniquePtr> left_frame_queue_;
  ThreadsafeQueue<Frame::UniquePtr> right_frame_queue_;

//  private:
//   cv::viz::Viz3d window_;
};

// Check that original left camera is distorted + unrectified
TEST_F(StereoCameraFixture, getOriginalLeftCamera) {
  CHECK(stereo_camera_);
  // Left camera from stereo camera is identical to a camera
  // constructed from params:
  VIO::Camera raw_left_camera(vio_params_.camera_params_.at(0));
  EXPECT_TRUE(raw_left_camera.getCalibration().equals(
      stereo_camera_->getOriginalLeftCamera()->getCalibration()));

  // Left camera is different from stereo camera calibration (undistorted
  // rectified):
  const VIO::Camera::ConstPtr left_camera =
      stereo_camera_->getOriginalLeftCamera();
  EXPECT_FALSE(left_camera->getCalibration().equals(
      stereo_camera_->getStereoCalib()->calibration()));
}

// Check that original right camera is distorted + unrectified
TEST_F(StereoCameraFixture, getOriginalRightCamera) {
  CHECK(stereo_camera_);
  // Right camera from stereo camera is identical to a camera
  // constructed from params:
  VIO::Camera raw_right_camera(vio_params_.camera_params_.at(1));
  EXPECT_TRUE(raw_right_camera.getCalibration().equals(
      stereo_camera_->getOriginalRightCamera()->getCalibration()));

  // Right camera is different from stereo camera calibration (undistorted
  // rectified):
  const VIO::Camera::ConstPtr right_camera =
      stereo_camera_->getOriginalRightCamera();
  EXPECT_FALSE(right_camera->getCalibration().equals(
      stereo_camera_->getStereoCalib()->calibration()));
}

TEST_F(StereoCameraFixture, project) {
  CHECK(stereo_camera_);
  // TODO(marcus): implement for both versions of function.
}

TEST_F(StereoCameraFixture, backProjectDepth) {
  CHECK(stereo_camera_);
  LandmarksCV expected_lmks;
  LandmarksCV actual_lmks;

  KeypointCV kp(stereo_camera_->getStereoCalib()->calibration().px(),
                stereo_camera_->getStereoCalib()->calibration().py());
  double depth = 1.0;
  LandmarkCV expected_lmk(0.0, 0.0, depth);
  gtsam::Point3 lmk_body =
      stereo_camera_->getBodyPoseLeftCamRect().transformFrom(
          gtsam::Point3(expected_lmk.x, expected_lmk.y, expected_lmk.z));
  expected_lmk.x = lmk_body.x();
  expected_lmk.y = lmk_body.y();
  expected_lmk.z = lmk_body.z();

  LandmarkCV actual_lmk;
  stereo_camera_->backProjectDepth(kp, depth, &actual_lmk);
  expected_lmks.push_back(expected_lmk);
  actual_lmks.push_back(actual_lmk);

  // Increase depth by 20
  depth = 20.0;
  expected_lmk = LandmarkCV(0.0, 0.0, depth);
  lmk_body = stereo_camera_->getBodyPoseLeftCamRect().transformFrom(
      gtsam::Point3(expected_lmk.x, expected_lmk.y, expected_lmk.z));
  expected_lmk.x = lmk_body.x();
  expected_lmk.y = lmk_body.y();
  expected_lmk.z = lmk_body.z();
  stereo_camera_->backProjectDepth(kp, depth, &actual_lmk);
  expected_lmks.push_back(expected_lmk);
  actual_lmks.push_back(actual_lmk);

  compareLandmarks(expected_lmks, actual_lmks, 0.00001);
}

TEST_F(StereoCameraFixture, backProjectDisparity) {
  CHECK(stereo_camera_);
  // TODO(marcus): implement
}

TEST_F(StereoCameraFixture, backProject) {
  CHECK(stereo_camera_);
  // TODO(marcus): implement
}

/**
 * @brief TEST_F backProjectDisparityTo3D basically check that when a disparity
 * map is computed, and its 3D projection calculated, the 3D points still
 * project into the same pixels that generated them (implicitly what this
 * really tests is that the project function works, rather than the
 * backProjectDisparityTo3D...)
 */
TEST_F(StereoCameraFixture, backProjectDisparityTo3D) {
  CHECK(stereo_camera_);

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
      *right_frame);

  // Compute depth map just to see.
  cv::Mat disp_img =
      cv::Mat(left_frame->img_.rows, left_frame->img_.cols, CV_32F);
  stereo_camera_->undistortRectifyStereoFrame(&stereo_frame);
  stereo_matcher_->denseStereoReconstruction(
      stereo_frame.getLeftImgRectified(),
      stereo_frame.getRightImgRectified(),
      &disp_img);
  CHECK(stereo_frame.isRectified());
  cv::Mat disp_viz_img;
  UtilsOpenCV::getDisparityVis(disp_img, disp_viz_img, 1.0);
  if (FLAGS_display) {
    cv::imshow("Left Image", stereo_frame.getLeftImgRectified());
    cv::imshow("Right Image", stereo_frame.getRightImgRectified());
    cv::imshow("Disparity Image", disp_viz_img);
  }

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
  if (FLAGS_display) {
    cv::imshow("Depth Image", depth_map);
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
  for (size_t v = 0u; v < depth_map.rows; ++v) {
    for (size_t u = 0u; u < depth_map.cols; ++u) {
      KeypointCV pixel(u, v);
      const cv::Point3f& xyz = depth_map.at<cv::Point3f>(pixel);
      if (isValidPoint(xyz) && xyz.z <= kMaxZ) {
        // Convert xyz to global coords (as they are given in camera coords).
        const gtsam::Pose3& left_cam_rect_pose =
            stereo_camera_->getBodyPoseLeftCamRect();
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
        expected_left_kpts.push_back(pixel);
        actual_right_kpts.push_back(kp_right);
        expected_right_kpts.push_back(
            KeypointCV(u - disp_img.at<float>(pixel), v));
      }
    }
  }
  compareKeypoints(expected_left_kpts, actual_left_kpts, 0.0001f);
  compareKeypoints(expected_right_kpts, actual_right_kpts, 0.0001f);
}

TEST_F(StereoCameraFixture, backProjectDisparityTo3DMaual) {
  CHECK(stereo_camera_);
  // TODO(marcus): implement
}

TEST_F(StereoCameraFixture, unidstortRectifyStereoFrame) {
  CHECK(stereo_camera_);
  // TODO(marcus): implement
}

TEST_F(StereoCameraFixture, undistortRectifyLeftKeypoints) {
  CHECK(stereo_camera_);
  // Rectification is already called in initializeData(), to avoid calling it in
  // every function

  // Verify the correctness of the rectification matrix!
  CameraParams left_camera_params = vio_params_.camera_params_.at(0);
  CameraParams right_camera_params = vio_params_.camera_params_.at(1);

  // THIS TEST IS REDUNDANT AS IT REPLICATES A SANITY CHECK ALREADY PRESENT IN
  // THE CODE Make sure that relative pose between camera AFTER rectification
  // satisfies: 1) no rotation 2) translation only along x axis = baseline
  // Compensate for the fact that opencv works on the inverse of rotations
  gtsam::Rot3 camL_Rot_camLrect =
      UtilsOpenCV::cvMatToGtsamRot3(stereo_camera_->getR1()).inverse();
  gtsam::Pose3 camL_Pose_camLrect =
      gtsam::Pose3(camL_Rot_camLrect, gtsam::Point3::Zero());

  // B_Pose_camLrect
  gtsam::Pose3 B_Pose_camLrect_expect =
      left_camera_params.body_Pose_cam_.compose(camL_Pose_camLrect);
  EXPECT_TRUE(
      stereo_camera_->getBodyPoseLeftCamRect().equals(B_Pose_camLrect_expect));

  // Right camera pose after rectification
  gtsam::Rot3 camR_Rot_camRrect =
      UtilsOpenCV::cvMatToGtsamRot3(stereo_camera_->getR2()).inverse();
  gtsam::Pose3 camR_Pose_camRrect = gtsam::Pose3(camR_Rot_camRrect, gtsam::Point3::Zero());

  // B_Pose_camRrect
  gtsam::Pose3 B_Pose_camRrect =
      right_camera_params.body_Pose_cam_.compose(camR_Pose_camRrect);
  // Relative pose
  gtsam::Pose3 camLrect_Pose_camRrect =
      stereo_camera_->getBodyPoseLeftCamRect().between(B_Pose_camRrect);

  // Verify the quality of the rectification!
  // Baseline
  double baseline_expect = left_camera_params.body_Pose_cam_
                               .between(right_camera_params.body_Pose_cam_)
                               .translation()
                               .norm();
  // Make sure that it is compatible with the baseline used in the test data
  EXPECT_TRUE(baseline_expect >= 0.10 && baseline_expect <= 0.12);
  EXPECT_NEAR(baseline_expect, stereo_camera_->getBaseline(), 1e-5);
  // check condition 1) on Rotation
  double rotation_deviation =
      gtsam::Rot3::Logmap(camLrect_Pose_camRrect.rotation()).norm();
  EXPECT_LT(rotation_deviation, tol);
  // check condition 2) on Translation
  double y_deviation = camLrect_Pose_camRrect.translation().y();
  double z_deviation = camLrect_Pose_camRrect.translation().z();
  EXPECT_LT(y_deviation, tol);
  EXPECT_LT(z_deviation, tol);

  // Check that the intrinsics of P1 and P2 are exactly the same!
  EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(
    stereo_camera_->getP1()(cv::Rect(0, 0, 3, 3)),
    stereo_camera_->getP2()(cv::Rect(0, 0, 3, 3))));

  // Test projection using 3x4 (rectified) camera matrix
  cv::Mat P2_T_expect = stereo_camera_->getP1()(cv::Rect(0, 0, 3, 3)) *
                        (cv::Mat_<double>(3, 1) << -baseline_expect, 0, 0);
  P2_T_expect = P2_T_expect.clone();
  cv::Mat P2_T_actual = stereo_camera_->getP2()(cv::Rect(3, 0, 1, 3)).clone();
  EXPECT_TRUE(UtilsOpenCV::compareCvMatsUpToTol(P2_T_expect, P2_T_actual));
}

TEST_F(StereoCameraFixture, computeRectificationParameters) {
  CHECK(stereo_camera_);
  // TODO(marcus): implement
}

}  // namespace VIO
