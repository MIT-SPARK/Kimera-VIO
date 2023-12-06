/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testOpticalFlowPredictor.cpp
 * @brief  test OpticalFlowPredictor
 * @author Antoni Rosinol
 */

#include <string>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/optical-flow/OpticalFlowPredictor.h"
#include "kimera-vio/frontend/optical-flow/OpticalFlowPredictorFactory.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"

DECLARE_string(test_data_path);
DECLARE_bool(display);

namespace VIO {

class OpticalFlowPredictorFixture : public ::testing::Test {
 public:
  using PinholeCalibration = gtsam::Cal3DS2;
  using PinholeCamera = gtsam::PinholeCamera<gtsam::Cal3DS2>;
  OpticalFlowPredictorFixture()
      : lmks_(),
        simulated_calib_(),
        euroc_calib_(),
        cam_1_kpts_(),
        cam_2_kpts_(),
        cam_1_pose_(),
        cam_2_pose_(),
        cam_1_(),
        cam_2_(),
        camera_params_(VioParams(FLAGS_test_data_path + "/EurocParams")
                           .camera_params_.at(0)),
        optical_flow_predictor_(nullptr),
        window_(nullptr) {
    // Generate 3D points (landmarks) of the scene.
    // We scale it by 2.5 to make some keypoints re-project out of the img
    // for large rotations of cam2 wrt cam1
    buildSceneLandmarks(&lmks_, 1.0);

    // You could use euroc_calib_ as well, but you'll have similar results
    // but you need to account for distortion, since the projectSafe function
    // of GTSAM also uncalibrates the pixels using the distortion params.

    // Simulated calibration without distortion or skewness, and with the
    // same fx and fy focal length.
    const auto& fx = camera_params_.intrinsics_.at(0);
    simulated_calib_ = gtsam::Cal3DS2(fx,
                                      fx,
                                      0.0,
                                      camera_params_.image_size_.width / 2u,
                                      camera_params_.image_size_.height / 2u,
                                      0.0,
                                      0.0);

    // Create Left Camera
    // Move left camera negative x direction so it can see the whole scene.
    cam_1_pose_ =
        gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(0.0, 0.0, -4.0));
    cam_1_ = PinholeCamera(cam_1_pose_, simulated_calib_);

    // Project Scene landmarks to cam1
    projectLandmarks(lmks_, cam_1_, &cam_1_kpts_);
  }
  ~OpticalFlowPredictorFixture() override = default;

 protected:
  void SetUp() override {}
  void TearDown() override {}

  // Helper function to construct optica flow predictors
  OpticalFlowPredictor::UniquePtr buildOpticalFlowPredictor(
      const OpticalFlowPredictorType& optical_flow_predictor_type) {
    return OpticalFlowPredictorFactory::makeOpticalFlowPredictor(
        optical_flow_predictor_type,
        UtilsOpenCV::gtsamMatrix3ToCvMat(simulated_calib_.K()),
        camera_params_.image_size_);
  }

  // Build scene
  void buildSceneLandmarks(Landmarks* lmks, const int& scale = 1u) {
    CHECK_NOTNULL(lmks)->clear();

    // A cube in 3D, axis-aligned with the world frame of reference.
    // Bottom
    lmks->push_back(scale * Landmark(0.0, 0.0, 0.0));
    lmks->push_back(scale * Landmark(0.0, 1.0, 0.0));
    lmks->push_back(scale * Landmark(1.0, 0.0, 0.0));
    lmks->push_back(scale * Landmark(1.0, 1.0, 0.0));

    // Top
    lmks->push_back(scale * Landmark(0.0, 0.0, 1.0));
    lmks->push_back(scale * Landmark(0.0, 1.0, 1.0));
    lmks->push_back(scale * Landmark(1.0, 0.0, 1.0));
    lmks->push_back(scale * Landmark(1.0, 1.0, 1.0));
  }

  // Generate cam_2, which is cam_1 after having moved by the given
  // pose.
  void generateCam2(const gtsam::Pose3& cam_1_P_cam_2) {
    // Move cam 2 by transforming the pose of cam 1.
    cam_2_pose_ = cam_1_pose_.compose(cam_1_P_cam_2);

    // Re-create cam 2.
    cam_2_ = PinholeCamera(cam_2_pose_, simulated_calib_);

    // Re-project landmarks to cam_2_
    projectLandmarks(lmks_, cam_2_, &cam_2_kpts_);
  }

  void projectLandmarks(const Landmarks& lmks,
                        const PinholeCamera& cam,
                        KeypointsCV* kpts) {
    CHECK_NOTNULL(kpts)->clear();
    for (const Landmark& lmk : lmks) {
      const auto& kpt_bool = cam.projectSafe(lmk);
      if (kpt_bool.second) {
        const auto& kpt = kpt_bool.first;
        kpts->push_back(KeypointCV(kpt.x(), kpt.y()));
      } else {
        LOG(WARNING) << "Landmark projection to camera failed!";
      }
    }
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

  void visualizeRay(const cv::Point3f& lmk,
                    const std::string& id,
                    const cv::Point3f& cam_world_origin,
                    const double& text_thickness = 0.2,
                    const cv::viz::Color& color = cv::viz::Color::blue(),
                    const bool& display_text = false) {
    CHECK(window_);
    // Display 3D rays from cam origin to lmks.
    if (display_text) {
      window_->showWidget(
          "Ray Label " + id,
          cv::viz::WText3D(id, lmk, text_thickness, true, color));
    }
    window_->showWidget("Ray " + id,
                        cv::viz::WLine(cam_world_origin, lmk, color));
  }

  void visualizePointCloud(const std::string& id, const cv::Mat& pointcloud) {
    CHECK(window_);
    cv::viz::WCloud cloud(pointcloud, cv::viz::Color::red());
    cloud.setRenderingProperty(cv::viz::POINT_SIZE, 6);
    window_->showWidget(id, cloud);
  }

  void visualizeScene(const std::string& test_name,
                      const KeypointsCV& predicted_kpts) {
    if (FLAGS_display) {
      window_ = std::make_unique<cv::viz::Viz3d>(test_name);
      window_->setBackgroundColor(cv::viz::Color::white());

      cv::Matx33d K = UtilsOpenCV::gtsamMatrix3ToCvMat(simulated_calib_.K());
      cv::Mat cam_1_img = cv::Mat(camera_params_.image_size_,
                                  CV_8UC4,
                                  cv::Scalar(255u, 255u, 255u, 40u));
      cv::Mat cam_2_img = cv::Mat(
          camera_params_.image_size_, CV_8UC4, cv::Scalar(255u, 0u, 0u, 125u));

      // Visualize world coords.
      window_->showWidget("World Coordinates", cv::viz::WCoordinateSystem(0.5));

      // Visualize left/right cameras
      const auto& cam_1_cv_pose =
          UtilsOpenCV::gtsamPose3ToCvAffine3d(cam_1_pose_);
      const auto& cam_2_cv_pose =
          UtilsOpenCV::gtsamPose3ToCvAffine3d(cam_2_pose_);
      // Camera Coordinate axes
      cv::viz::WCameraPosition cpw1(0.2);
      cv::viz::WCameraPosition cpw2(0.4);
      window_->showWidget("Cam 1 Coordinates", cpw1, cam_1_cv_pose);
      window_->showWidget("Cam 2 Coordinates", cpw2, cam_2_cv_pose);

      // Visualize landmarks
      cv::Mat pointcloud = cv::Mat(0, 3, CV_64FC1);
      cv::Point3f cam_1_position =
          UtilsOpenCV::gtsamVector3ToCvPoint3(cam_1_pose_.translation());
      cv::Point3f cam_2_position =
          UtilsOpenCV::gtsamVector3ToCvPoint3(cam_2_pose_.translation());
      for (size_t i = 0u; i < lmks_.size(); i++) {
        cv::Point3f lmk_cv = UtilsOpenCV::gtsamVector3ToCvPoint3(lmks_[i]);
        visualizeRay(lmk_cv, "lmk-cam1" + std::to_string(i), cam_1_position);
        visualizeRay(lmk_cv,
                     "lmk-cam2" + std::to_string(i),
                     cam_2_position,
                     0.2,
                     cv::viz::Color::red());
        pointcloud.push_back(cv::Mat(lmk_cv).reshape(1).t());
      }
      pointcloud = pointcloud.reshape(3, lmks_.size());
      visualizePointCloud("Scene Landmarks", pointcloud);

      // Color image 2 with pixel reprojections (the ground-truth)
      drawPixelsOnImg(
          cam_2_kpts_, cam_2_img, cv::viz::Color::green(), 3u, 125u);

      // Color image 2 with pixel prediction if no prediction is done
      // Expected result if using NoPredictionOpticalFlow
      drawPixelsOnImg(
          cam_1_kpts_, cam_2_img, cv::viz::Color::brown(), 6u, 125u);

      // Show the estimated kpt positions in red and smaller
      drawPixelsOnImg(
          predicted_kpts, cam_2_img, cv::viz::Color::red(), 1u, 125u);

      // Camera frustums
      // cv::viz::WCameraPosition cpw_1_frustum(K, cam_1_img, 2.0);
      cv::viz::WCameraPosition cpw_1_frustum(K, 2.0);  // No img
      cv::viz::WCameraPosition cpw_2_frustum(K, cam_2_img, 2.0);
      window_->showWidget("Cam 1 Frustum", cpw_1_frustum, cam_1_cv_pose);
      window_->showWidget("Cam 2 Frustum", cpw_2_frustum, cam_2_cv_pose);

      // Finally, spin
      spinDisplay();
    } else {
      LOG(WARNING) << "Requested scene visualization but display gflag is "
                   << "set to false.";
    }
  }

  void chessboardImgCreator() {
    int block_size = 75;
    int img_size = block_size * 8;
    cv::Mat chessboard(img_size, img_size, CV_8UC3, cv::Scalar::all(0));
    unsigned char color = 0;

    for (int i = 0; i < img_size; i = i + block_size) {
      color = ~color;
      for (int j = 0; j < img_size; j = j + block_size) {
        cv::Mat ROI = chessboard(cv::Rect(i, j, block_size, block_size));
        ROI.setTo(cv::Scalar::all(color));
        color = ~color;
      }
    }
    cv::imshow("Chess board", chessboard);
  }

  // Display 3D window
  void spinDisplay() {
    CHECK(window_);
    window_->spin();
  }

 protected:
  // Default Parms
  Landmarks lmks_;
  PinholeCalibration simulated_calib_;
  PinholeCalibration euroc_calib_;
  KeypointsCV cam_1_kpts_;
  KeypointsCV cam_2_kpts_;
  gtsam::Pose3 cam_1_pose_;
  gtsam::Pose3 cam_2_pose_;
  PinholeCamera cam_1_;
  PinholeCamera cam_2_;
  CameraParams camera_params_;
  OpticalFlowPredictor::UniquePtr optical_flow_predictor_;

 private:
  std::unique_ptr<cv::viz::Viz3d> window_;
};

// Checks that the math has not been changed by accident.
TEST_F(OpticalFlowPredictorFixture, DefaultNoPredictionOpticalFlowPrediction) {
  optical_flow_predictor_ =
      buildOpticalFlowPredictor(OpticalFlowPredictorType::kNoPrediction);
  ASSERT_TRUE(optical_flow_predictor_);

  // Create Right Camera
  //! Cam2 is at 45 degree rotation wrt z axis wrt Cam1.
  gtsam::Rot3 rot_45_z(0.924, 0.0, 0.0, 0.383);
  gtsam::Pose3 cam_1_P_cam_2(rot_45_z, gtsam::Vector3::Zero());
  generateCam2(cam_1_P_cam_2);

  // Since this is a no prediction optical flow predictor, the kpts should
  // remain at the same location from cam1 to cam2.
  KeypointsCV expected_kpts = cam_1_kpts_;

  // Calculate actual kpts
  KeypointsCV actual_kpts;
  const gtsam::Rot3& inter_frame_rot = cam_1_P_cam_2.rotation();
  optical_flow_predictor_->predictSparseFlow(
      cam_1_kpts_, inter_frame_rot, &actual_kpts);

  // Compare for equality
  compareKeypoints(expected_kpts, actual_kpts, 1e-1);

  visualizeScene("NoPrediction", actual_kpts);
}

// Checks that the prediction forward and then backwards preserves kpts location
TEST_F(OpticalFlowPredictorFixture,
       NoPredictionOpticalFlowPredictionInvariance) {
  optical_flow_predictor_ =
      buildOpticalFlowPredictor(OpticalFlowPredictorType::kNoPrediction);
  ASSERT_TRUE(optical_flow_predictor_);

  // Create Right Camera
  //! Cam2 is at 45 degree rotation wrt z axis wrt Cam1.
  gtsam::Rot3 rot_45_z(0.924, 0.0, 0.0, 0.383);
  gtsam::Pose3 cam_1_P_cam_2(rot_45_z, gtsam::Vector3::Zero());
  generateCam2(cam_1_P_cam_2);

  // Since this is a no prediction optical flow predictor, the kpts should
  // remain at the same location from cam1 to cam2.
  KeypointsCV expected_kpts = cam_1_kpts_;

  // Calculate actual kpts
  KeypointsCV actual_kpts;
  const gtsam::Rot3& inter_frame_rot = cam_1_P_cam_2.rotation();
  optical_flow_predictor_->predictSparseFlow(
      cam_1_kpts_, inter_frame_rot, &actual_kpts);

  optical_flow_predictor_->predictSparseFlow(
      actual_kpts, inter_frame_rot, &actual_kpts);

  // Compare for equality
  compareKeypoints(expected_kpts, actual_kpts, 1e-1);
}

// Checks that the math has not been changed by accident.
TEST_F(OpticalFlowPredictorFixture, DefaultRotationalOpticalFlowPrediction) {
  optical_flow_predictor_ =
      buildOpticalFlowPredictor(OpticalFlowPredictorType::kRotational);
  ASSERT_TRUE(optical_flow_predictor_);

  // Create Right Camera
  //! Cam2 is at 45 degree rotation wrt z axis wrt Cam1.
  gtsam::Rot3 rot_45_z(0.924, 0.0, 0.0, 0.383);
  gtsam::Pose3 cam_1_P_cam_2(rot_45_z, gtsam::Vector3::Zero());
  generateCam2(cam_1_P_cam_2);

  // Since this is a rotational optical flow predictor, the kpts should
  // move given a rotation-only optical flow.
  const gtsam::Matrix3 K = simulated_calib_.K();
  const gtsam::Rot3& R = cam_2_pose_.rotation();
  const gtsam::Matrix3 H = K * R.matrix().inverse() * K.inverse();
  KeypointsCV expected_kpts;
  for (const auto& kpt : cam_1_kpts_) {
    gtsam::Vector3 kp_h(kpt.x, kpt.y, 1.0f);  // Homogeneous keypoint.
    gtsam::Vector3 kp2_h = H * kp_h;
    expected_kpts.push_back(
        KeypointCV(kp2_h.x() / kp2_h.z(), kp2_h.y() / kp2_h.z()));
  }

  // Calculate actual kpts
  KeypointsCV actual_kpts;
  const gtsam::Rot3& inter_frame_rot = cam_1_P_cam_2.rotation();
  optical_flow_predictor_->predictSparseFlow(
      cam_1_kpts_, inter_frame_rot, &actual_kpts);

  // Compare for equality
  compareKeypoints(expected_kpts, actual_kpts, 1e-1);
}

// Checks that the prediction forward and then backwards preserves kpts location
TEST_F(OpticalFlowPredictorFixture, RotationalOpticalFlowPredictionInvariance) {
  optical_flow_predictor_ =
      buildOpticalFlowPredictor(OpticalFlowPredictorType::kRotational);
  ASSERT_TRUE(optical_flow_predictor_);

  // Predicting forward and then backward should yield same result.
  KeypointsCV expected_kpts = cam_1_kpts_;

  // Create Right Camera
  //! Cam2 is at 45 degree rotation wrt z axis wrt Cam1.
  gtsam::Rot3 rot_45_z(0.924, 0.0, 0.0, 0.383);
  gtsam::Pose3 cam_1_P_cam_2(rot_45_z, gtsam::Vector3::Zero());
  generateCam2(cam_1_P_cam_2);

  // Calculate actual kpts
  KeypointsCV actual_kpts;
  const gtsam::Rot3& inter_frame_rot = cam_2_pose_.rotation();
  optical_flow_predictor_->predictSparseFlow(
      cam_1_kpts_, inter_frame_rot, &actual_kpts);

  optical_flow_predictor_->predictSparseFlow(
      actual_kpts, inter_frame_rot.inverse(), &actual_kpts);

  // Expect invariance.
  compareKeypoints(expected_kpts, actual_kpts, 1e-1);
}

// Checks that the prediction coincides with the actual projection of the
// 3D landmarks in the camera using synthetic data.
TEST_F(OpticalFlowPredictorFixture,
       RotationalOpticalFlowPredictionRotationOnly) {
  optical_flow_predictor_ =
      buildOpticalFlowPredictor(OpticalFlowPredictorType::kRotational);
  ASSERT_TRUE(optical_flow_predictor_);

  // Create Right Camera with small rotation with respect to cam1
  //! Cam2 is at 20 degree rotation wrt z axis wrt Cam1.
  gtsam::Rot3 rot_20_z(0.985, 0.0, 0.0, 0.174);
  gtsam::Pose3 cam_1_P_cam_2(rot_20_z, gtsam::Vector3::Zero());
  generateCam2(cam_1_P_cam_2);

  // Call predictor to get next_kps
  KeypointsCV actual_kpts;
  const gtsam::Rot3& inter_frame_rot = cam_1_P_cam_2.rotation();
  optical_flow_predictor_->predictSparseFlow(
      cam_1_kpts_, inter_frame_rot, &actual_kpts);

  // Expect equal wrt the projection of X landmarks on cam 2
  compareKeypoints(cam_2_kpts_, actual_kpts, 1e-1);

  visualizeScene("RotationOnly", actual_kpts);
}

// Checks that small rotations are handled as if it there was no prediction.
// EDIT: actually, we also expect to predict even when small rotations, bcs
// at the border of the image, even small rotations lead to large optical flow.
TEST_F(OpticalFlowPredictorFixture,
       RotationalOpticalFlowPredictionSmallRotationOnly) {
  optical_flow_predictor_ =
      buildOpticalFlowPredictor(OpticalFlowPredictorType::kRotational);
  ASSERT_TRUE(optical_flow_predictor_);

  // Create Right Camera with small rotation with respect to cam1
  //! Cam2 is at 5 degree rotation wrt z axis wrt Cam1.
  gtsam::Rot3 rot_5_z(0.99999999, 0.0, 0.0, 0.0);
  gtsam::Pose3 cam_1_P_cam_2(rot_5_z, gtsam::Vector3::Zero());
  generateCam2(cam_1_P_cam_2);

  // Call predictor to get next_kps
  KeypointsCV actual_kpts;
  const gtsam::Rot3& inter_frame_rot = cam_1_P_cam_2.rotation();
  optical_flow_predictor_->predictSparseFlow(
      cam_1_kpts_, inter_frame_rot, &actual_kpts);

  // Expect equal wrt the projection of X landmarks on cam 1 instead of 2!
  // because we have a small rotation.
  compareKeypoints(cam_1_kpts_, actual_kpts, 1e-1);

  // Create Right Camera with small rotation with respect to cam1
  //! Cam2 is at 5 degree rotation wrt z axis wrt Cam1.
  //! Ensure double cover of quaternion is respected.
  gtsam::Rot3 rot_5_minus_z(-0.999, 0.0, 0.0, -0.001);
  cam_1_P_cam_2 = gtsam::Pose3(rot_5_minus_z, gtsam::Vector3::Zero());
  generateCam2(cam_1_P_cam_2);

  // Call predictor to get next_kps
  optical_flow_predictor_->predictSparseFlow(
      cam_1_kpts_, inter_frame_rot, &actual_kpts);

  // Expect equal wrt the projection of X landmarks on cam 1 instead of 2!
  // because we have a small rotation.
  compareKeypoints(cam_1_kpts_, actual_kpts, 1e-1);

  visualizeScene("SmallRotationOnly", actual_kpts);
}

// Checks that the prediction forward does not go outside the image!
// landmarks are not visible to cam2 but still in front of cam2.
TEST_F(OpticalFlowPredictorFixture, RotationalOpticalFlowPredictionOutOfImage) {
  optical_flow_predictor_ =
      buildOpticalFlowPredictor(OpticalFlowPredictorType::kRotational);
  ASSERT_TRUE(optical_flow_predictor_);

  // Create Right Camera
  //! Cam2 is at 45 degree rotation wrt x axis wrt Cam1: so that landmarks
  //! remain in front of camera, but outside of image of cam2.
  gtsam::Rot3 rot_45_x(0.924, 0.383, 0.0, 0.0);
  gtsam::Pose3 cam_1_P_cam_2(rot_45_x, gtsam::Vector3::Zero());
  generateCam2(cam_1_P_cam_2);

  // Note: cam_2_kpts_ are generated in generateCam2!
  // Predicting forward should just copy previous keypoint positions if lmks
  // fall outside the image.
  KeypointsCV expected_kpts = cam_1_kpts_;

  // Calculate actual kpts
  KeypointsCV actual_kpts;
  const gtsam::Rot3& inter_frame_rot = cam_2_pose_.rotation();
  optical_flow_predictor_->predictSparseFlow(
      cam_1_kpts_, inter_frame_rot, &actual_kpts);

  // Expect not out of image kpts.
  compareKeypoints(expected_kpts, actual_kpts, 1e-1);

  visualizeScene("OutOfImage", actual_kpts);
}

// Checks that the prediction forward does not get wrong prediction when
// landmarks are behind cam2, but still project inside the image.
TEST_F(OpticalFlowPredictorFixture, RotationalOpticalFlowPredictionBehindCam) {
  optical_flow_predictor_ =
      buildOpticalFlowPredictor(OpticalFlowPredictorType::kRotational);
  ASSERT_TRUE(optical_flow_predictor_);

  // Create Right Camera
  //! Cam2 is at 180 degree rotation wrt x axis wrt Cam1: so that landmarks
  //! are in the back of the camera, but still "project" inside of image of cam2
  gtsam::Rot3 rot_180_y(0.0, 0.0, 1.0, 0.0);
  gtsam::Pose3 cam_1_P_cam_2(rot_180_y, gtsam::Vector3::Zero());
  generateCam2(cam_1_P_cam_2);

  // Note: cam_2_kpts_ are generated in generateCam2!
  // Predicting forward should just copy previous keypoint positions if lmks
  // fall outside the image.
  KeypointsCV expected_kpts = cam_1_kpts_;

  // Calculate actual kpts
  KeypointsCV actual_kpts;
  const gtsam::Rot3& inter_frame_rot = cam_2_pose_.rotation();
  optical_flow_predictor_->predictSparseFlow(
      cam_1_kpts_, inter_frame_rot, &actual_kpts);

  // Expect not out of image kpts.
  compareKeypoints(expected_kpts, actual_kpts, 1e-1);

  visualizeScene("BehindCamera", actual_kpts);
}

TEST_F(OpticalFlowPredictorFixture,
       RotationalOpticalFlowPredictionRotationAndTranslation) {
  optical_flow_predictor_ =
      buildOpticalFlowPredictor(OpticalFlowPredictorType::kRotational);
  ASSERT_TRUE(optical_flow_predictor_);

  // Create Right Camera with small rotation with respect to cam1
  //! Cam2 is at 20 degree rotation wrt z axis wrt Cam1.
  gtsam::Rot3 rot_20_z(0.985, 0.0, 0.0, 0.174);
  gtsam::Vector3 t(0.3, 0.0, 0.0);
  gtsam::Pose3 cam_1_P_cam_2(rot_20_z, t);
  generateCam2(cam_1_P_cam_2);

  // Note: cam_2_kpts_ are generated in generateCam2!
  // Predicting forward should predict the projection on cam_2 - errors
  // due to translation!
  // KeypointsCV expected_kpts = cam_2_kpts_;

  // Call predictor to get next_kps
  KeypointsCV actual_kpts;
  const gtsam::Rot3& inter_frame_rot = cam_1_P_cam_2.rotation();
  optical_flow_predictor_->predictSparseFlow(
      cam_1_kpts_, inter_frame_rot, &actual_kpts);

  // Expect equal wrt the projection of X landmarks on cam 2
  // compareKeypoints(expected_kpts, actual_kpts, 1e-1);

  // This is copy-pasted from the actual calculations, so this test is a bit
  // irrelevant beyond having an insight on how much pixel error you can
  // expect...
  ASSERT_EQ(8, actual_kpts.size());
  EXPECT_NEAR(376.00003051757812, actual_kpts[0].x, 1e-1);
  EXPECT_NEAR(239.99998474121094, actual_kpts[0].y, 1e-1);
  EXPECT_NEAR(415.302001953125, actual_kpts[1].x, 1e-1);
  EXPECT_NEAR(347.7138671875, actual_kpts[1].y, 1e-1);
  EXPECT_NEAR(483.71389770507812, actual_kpts[2].x, 1e-1);
  EXPECT_NEAR(200.69801330566406, actual_kpts[2].y, 1e-1);
  EXPECT_NEAR(523.015869140625, actual_kpts[3].x, 1e-1);
  EXPECT_NEAR(308.41189575195312, actual_kpts[3].y, 1e-1);
  EXPECT_NEAR(376.00003051757812, actual_kpts[4].x, 1e-1);
  EXPECT_NEAR(239.99998474121094, actual_kpts[4].y, 1e-1);
  EXPECT_NEAR(407.44161987304688, actual_kpts[5].x, 1e-1);
  EXPECT_NEAR(326.17108154296875, actual_kpts[5].y, 1e-1);
  EXPECT_NEAR(462.17111206054688, actual_kpts[6].x, 1e-1);
  EXPECT_NEAR(208.55841064453125, actual_kpts[6].y, 1e-1);
  EXPECT_NEAR(493.61270141601562, actual_kpts[7].x, 1e-1);
  EXPECT_NEAR(294.7294921875, actual_kpts[7].y, 1e-1);

  visualizeScene("RotationAndTranslation", actual_kpts);
}

}  // namespace VIO
