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
#include <gtest/gtest.h>

DECLARE_string(test_data_path);

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
        window_() {
    // Generate 3D points (landmarks) of the scene.
    // We scale it by 2.5 to make some keypoints re-project out of the img
    // for large rotations of cam2 wrt cam1
    buildSceneLandmarks(&lmks_, 1.0);

    // Create cameras calibration from Euroc dataset.
    // Euroc dataset with distortion and skewness.
    euroc_calib_ = camera_params_.calibration_;

    // You could use euroc_calib_ as well, but you'll have similar results
    // but you need to account for distortion, since the projectSafe function
    // of GTSAM also uncalibrates the pixels using the distortion params.

    // Simulated calibration without distortion or skewness, and with the
    // same fx and fy focal length.
    simulated_calib_ = gtsam::Cal3DS2(euroc_calib_.fx(),
                                      euroc_calib_.fx(),
                                      0.0,
                                      camera_params_.image_size_.width / 2u,
                                      camera_params_.image_size_.height / 2u,
                                      0.0,
                                      0.0);

    // Create Left Camera
    // Move left camera negative x direction so it can see the whole scene.
    cam_1_pose_ = gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(0.0, 0.0, -4.0));
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
    // Display 3D rays from cam origin to lmks.
    if (display_text) {
      window_.showWidget(
          "Ray Label " + id,
          cv::viz::WText3D(id, lmk, text_thickness, true, color));
    }
    window_.showWidget("Ray " + id,
                       cv::viz::WLine(cam_world_origin, lmk, color));
  }

  void visualizePointCloud(const std::string& id, const cv::Mat& pointcloud) {
    cv::viz::WCloud cloud(pointcloud, cv::viz::Color::red());
    cloud.setRenderingProperty(cv::viz::POINT_SIZE, 6);
    window_.showWidget(id, cloud);
  }

  void visualizeScene(const std::string& test_name,
                      const KeypointsCV& predicted_kpts) {
    window_ = cv::viz::Viz3d(test_name);
    window_.setBackgroundColor(cv::viz::Color::white());

    cv::Matx33d K = UtilsOpenCV::gtsamMatrix3ToCvMat(simulated_calib_.K());
    cv::Mat cam_1_img = cv::Mat(
        camera_params_.image_size_, CV_8UC4, cv::Scalar(255u, 255u, 255u, 40u));
    cv::Mat cam_2_img = cv::Mat(
        camera_params_.image_size_, CV_8UC4, cv::Scalar(255u, 0u, 0u, 125u));

    // Visualize world coords.
    window_.showWidget("World Coordinates", cv::viz::WCoordinateSystem(0.5));

    // Visualize left/right cameras
    const auto& cam_1_cv_pose =
        UtilsOpenCV::gtsamPose3ToCvAffine3d(cam_1_pose_);
    const auto& cam_2_cv_pose =
        UtilsOpenCV::gtsamPose3ToCvAffine3d(cam_2_pose_);
    // Camera Coordinate axes
    cv::viz::WCameraPosition cpw1(0.2);
    cv::viz::WCameraPosition cpw2(0.4);
    window_.showWidget("Cam 1 Coordinates", cpw1, cam_1_cv_pose);
    window_.showWidget("Cam 2 Coordinates", cpw2, cam_2_cv_pose);

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
    drawPixelsOnImg(cam_2_kpts_, cam_2_img, cv::viz::Color::green(), 3u, 125u);

    // Color image 2 with pixel prediction if no prediction is done
    // Expected result if using NoPredictionOpticalFlow
    drawPixelsOnImg(cam_1_kpts_, cam_2_img, cv::viz::Color::brown(), 6u, 125u);

    // Show the estimated kpt positions in red and smaller
    drawPixelsOnImg(predicted_kpts, cam_2_img, cv::viz::Color::red(), 1u, 125u);

    // Camera frustums
    // cv::viz::WCameraPosition cpw_1_frustum(K, cam_1_img, 2.0);
    cv::viz::WCameraPosition cpw_1_frustum(K, 2.0);  // No img
    cv::viz::WCameraPosition cpw_2_frustum(K, cam_2_img, 2.0);
    window_.showWidget("Cam 1 Frustum", cpw_1_frustum, cam_1_cv_pose);
    window_.showWidget("Cam 2 Frustum", cpw_2_frustum, cam_2_cv_pose);
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

  void spinDisplay() {
    // Display 3D window
    static constexpr bool kDisplay = false;
    if (kDisplay) {
      window_.spin();
    }
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
  cv::viz::Viz3d window_;
};

// Checks that the math has not been changed by accident.
TEST_F(CameraFixture, BaselineCalculation) {
}

}  // namespace VIO
