/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testStereoVisionImuFrontend.cpp
 * @brief  test StereoVisionImuFrontend
 * @author Antoni Rosinol
 * @author Luca Carlone
 * @author Marcus Abate
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/StereoVisionImuFrontend.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

DECLARE_string(test_data_path);
DECLARE_bool(display);

namespace VIO {

class StereoVisionImuFrontendFixture : public ::testing::Test {
 public:
  StereoVisionImuFrontendFixture()
      : stereo_FLAGS_test_data_path(FLAGS_test_data_path +
                                    std::string("/ForStereoFrame/")) {
    initializeData();
  }

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  // Helper function
  void initializeData() {
    cam_params_left_.parseYAML(stereo_FLAGS_test_data_path +
                               "/sensorLeft.yaml");
    cam_params_right_.parseYAML(stereo_FLAGS_test_data_path +
                                "/sensorRight.yaml");

    std::string img_name_ref_left =
        stereo_FLAGS_test_data_path + "left_img_0.png";
    std::string img_name_ref_right =
        stereo_FLAGS_test_data_path + "right_img_0.png";

    std::string img_name_cur_left =
        stereo_FLAGS_test_data_path + "left_img_1.png";
    std::string img_name_cur_right =
        stereo_FLAGS_test_data_path + "right_img_1.png";

    // Data for testing "geometricOutlierRejection2d2d"
    ref_frame = std::make_shared<Frame>(
        id_ref,
        timestamp_ref,
        cam_params_left_,
        UtilsOpenCV::ReadAndConvertToGrayScale(img_name_ref_left));
    cur_frame = std::make_shared<Frame>(
        id_cur,
        timestamp_cur,
        cam_params_left_,
        UtilsOpenCV::ReadAndConvertToGrayScale(img_name_cur_left));

    FrontendParams tp;

    ref_stereo_frame = std::make_shared<StereoFrame>(
        id_ref,
        timestamp_ref,
        Frame(
            id_ref,
            timestamp_ref,
            cam_params_left_,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_ref_left, tp.stereo_matching_params_.equalize_image_)),
        Frame(id_ref,
              timestamp_ref,
              cam_params_right_,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  img_name_ref_right,
                  tp.stereo_matching_params_.equalize_image_)));

    cur_stereo_frame = std::make_shared<StereoFrame>(
        id_cur,
        timestamp_cur,
        Frame(
            id_cur,
            timestamp_cur,
            cam_params_left_,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_cur_left, tp.stereo_matching_params_.equalize_image_)),
        Frame(id_cur,
              timestamp_cur,
              cam_params_right_,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  img_name_cur_right,
                  tp.stereo_matching_params_.equalize_image_)));

    // Imu Params
    imu_params_.acc_random_walk_ = 1;
    imu_params_.gyro_random_walk_ = 1;
    imu_params_.acc_noise_density_ = 1;
    imu_params_.gyro_noise_density_ = 1;
    imu_params_.imu_integration_sigma_ = 1;

    // Set randomness!
    srand(0);
  }

  void clearFrame(Frame* f) {
    f->keypoints_.clear();
    f->landmarks_.clear();
    f->landmarks_age_.clear();
    f->versors_.clear();
  }

  void clearStereoFrame(std::shared_ptr<StereoFrame>& sf) {
    clearFrame(&sf->left_frame_);
    clearFrame(&sf->right_frame_);
    sf->keypoints_3d_.clear();
    sf->right_keypoints_rectified_.clear();
  }

  void fillStereoFrame(std::shared_ptr<StereoFrame>& sf) {
    // Fill the fields in a StereoFrame to pass the sanity check
    // StereoFrame::checkStereoFrame
    const int num_keypoints = sf->left_frame_.landmarks_.size();

    // left.y == right.y
    // (right_keypoints_status[i] == valid && right_frame_.keypoints_[i] != 0)
    // OR (right_keypoints_status[i] != valid)

    // right_keypoints_status_.size
    if (sf->right_keypoints_rectified_.size() != num_keypoints) {
      sf->right_keypoints_rectified_.resize(num_keypoints);
      for (int i = 0; i < num_keypoints; i++) {
        sf->right_keypoints_rectified_.at(i).first = KeypointStatus::VALID;
        sf->right_keypoints_rectified_.at(i).second = KeypointCV();
      }
    }

    // left_frame_.keypoints_.size
    if (sf->left_frame_.keypoints_.size() != num_keypoints) {
      sf->left_frame_.keypoints_ = KeypointsCV(num_keypoints);
      for (int i = 0; i < num_keypoints; i++) {
        sf->left_frame_.keypoints_[i] = KeypointCV(i, i);
      }
    }

    // right_frame_.keypoints_.size
    if (sf->right_frame_.keypoints_.size() != num_keypoints) {
      sf->right_frame_.keypoints_ = KeypointsCV(num_keypoints);
      for (int i = 0; i < num_keypoints; i++) {
        if (sf->right_keypoints_rectified_[i].first == KeypointStatus::VALID) {
          sf->right_frame_.keypoints_[i] = KeypointCV(i + 20, i + (i % 3 - 1));
        } else {
          sf->right_frame_.keypoints_[i] = KeypointCV(0, 0);
        }
      }
    }

    // keypoints_3d_.size
    if (sf->keypoints_3d_.size() != num_keypoints) {
      sf->keypoints_3d_.resize(num_keypoints);
      for (int i = 0; i < num_keypoints; i++) {
        if (sf->right_keypoints_rectified_[i].first == KeypointStatus::VALID) {
          sf->keypoints_3d_.at(i)(2) = 1;
        } else {
          sf->keypoints_3d_.at(i)(2) = 0;
        }
      }
    }

    // left_keypoints_rectified.size
    if (sf->left_keypoints_rectified_.size() != num_keypoints) {
      sf->left_keypoints_rectified_.resize(num_keypoints);
    }
  }

  std::vector<gtsam::Point2> loadCorners(const std::string& filepath) {
    std::vector<gtsam::Point2> corners;
    std::ifstream fin(filepath);
    int num_corners;
    fin >> num_corners;
    corners.reserve(num_corners);
    for (int i = 0; i < num_corners; i++) {
      double x, y;
      fin >> x >> y;
      // Convert points coordinates from MATLAB convention to c++ convention:
      corners.push_back(gtsam::Point2(x - 1, y - 1));
    }
    return corners;
  }

  std::vector<double> loadDepth(const std::string& filepath) {
    std::vector<double> depth;
    std::ifstream fin(filepath);
    int num_corners;
    fin >> num_corners;
    depth.reserve(num_corners);
    for (int i = 0; i < num_corners; i++) {
      double d;
      fin >> d;
      // Convert points coordinates from MATLAB convention to c++ convention:
      depth.push_back(d);
    }
    return depth;
  }

  int findPointInVector(const gtsam::Point2& pt_query,
                        const std::vector<gtsam::Point2>& pt_set,
                        double tolerance = 3.0) {
    for (size_t i = 0u; i < pt_set.size(); i++) {
      // check inf norm to make sure all elements are within tolerance
      // LOG(INFO) << "checking correspondences" << std::endl;
      // LOG(INFO) << pt_set[i] << std::endl;
      // LOG(INFO) << pt_query << std::endl;
      if ((pt_set[i] - pt_query).lpNorm<Eigen::Infinity>() < tolerance) {
        return i;
      }
    }
    return -1;
  }

  std::vector<gtsam::Point2> convertCornersAcrossCameras(
      const std::vector<gtsam::Point2>& corners_in,
      const gtsam::Cal3DS2& calib_in,
      const gtsam::Cal3_S2& calib_out) {
    CHECK_DOUBLE_EQ(calib_in.skew(), 0.0);
    CHECK_DOUBLE_EQ(calib_out.skew(), 0.0);

    std::vector<gtsam::Point2> corners_out;
    corners_out.reserve(corners_in.size());
    for (int i = 0; i < corners_in.size(); i++) {
      double xn = (corners_in[i].x() - calib_in.px()) / calib_in.fx();
      double yn = (corners_in[i].y() - calib_in.py()) / calib_in.fy();
      double xo = xn * calib_out.fx() + calib_out.px();
      double yo = yn * calib_out.fy() + calib_out.py();
      corners_out.push_back(gtsam::Point2(xo, yo));
    }
    return corners_out;
  }

  // Data
  std::shared_ptr<Frame> ref_frame, cur_frame;
  std::shared_ptr<StereoFrame> ref_stereo_frame, cur_stereo_frame;
  const FrameId id_ref = 0u;
  const FrameId id_cur = 1u;
  const Timestamp timestamp_ref = 1000u;
  const Timestamp timestamp_cur = 2000u;
  CameraParams cam_params_left_, cam_params_right_;

  ImuParams imu_params_;
  std::string stereo_FLAGS_test_data_path;
};

void get_undistorted_points(gtsam::Point2 distorted,
                            const CameraParams cam_params,
                            StereoCamera::ConstPtr stereo_camera,
                            std::vector<cv::Point2d>& undistorted) {
  double distorted_x = distorted.x();
  double distorted_y = distorted.y();
  std::vector<cv::Point2d> distorted_cv;
  distorted_cv.push_back(cv::Point2d(distorted_x, distorted_y));
  cv::Mat intrinsics = cv::Mat::zeros(3, 3, CV_64F);
  cv::Mat dist_coeffs = cv::Mat::zeros(1, 4, CV_64F);
  intrinsics.at<double>(2, 2) = 1;
  intrinsics.at<double>(0, 0) = cam_params.intrinsics_[0];  // fx
  intrinsics.at<double>(1, 1) = cam_params.intrinsics_[1];  // fy
  intrinsics.at<double>(0, 2) = cam_params.intrinsics_[2];  // cx
  intrinsics.at<double>(1, 2) = cam_params.intrinsics_[3];  // cy

  cv::undistortPoints(distorted_cv,
                      undistorted,
                      intrinsics,
                      cam_params.distortion_coeff_mat_,
                      stereo_camera->getR1(),
                      stereo_camera->getP1());
}

/* This test is flawed in that it is using private members of the Frontend...
 * There is three ways to go around this:
 * 1) Proper way: test getRelativePoseBodyStereo by just using the public
 * interface of stereo vision Frontend
 * 2) Proper but cumbersome way: understand the Frontend in depth and
 * create new smaller classes which have public interfaces that can be tested
 * and are used by the Frontend. In this case, getRelativePoseBody seems to be
 * an extremely low-level function so there is a lot of refactoring to do.
 * 3) Dirty way: make this test a 'friend class' and use the private members...
TEST_F(StereoVisionImuFrontendFixture, getRelativePoseBodyMono) {
  // How to test this in the simplest possible way?
  // Compute everything twice?
  ref_stereo_frame->setIsKeyframe(true);
  ref_stereo_frame->setIsRectified(true);
  cur_stereo_frame->setIsKeyframe(true);
  cur_stereo_frame->setIsRectified(true);
  // Avoid the Most Vexing Parse compilation error with bracket initialization.
  StereoVisionImuFrontend st(imu_params_, ImuBias{});
  st.stereoFrame_lkf_ = make_shared<StereoFrame>(*ref_stereo_frame);
  st.trackerStatusSummary_.lkf_T_k_mono_ = Pose3(
      Rot3::Expmap(Vector3(0.1, -0.1, 0.2)), Vector3(0.1, 0.1, 0.1));
  Pose3 body_pose_cam = ref_stereo_frame->getBPoseCamLRect();
  // Expected answer
  Pose3 pose_expected = body_pose_cam *
      st.trackerStatusSummary_.lkf_T_k_mono_ * body_pose_cam.inverse();
  // Actual answer
  Pose3 pose_actual = st.getRelativePoseBodyMono();
  // Comparison
  EXPECT(gtsam::assert_equal(pose_actual, pose_expected));
}

/* ************************************************************************* */
/* This test is flawed in that it is using private members of the Frontend...
 * There is three ways to go around this:
 * 1) Proper way: test getRelativePoseBodyStereo by just using the public
 * interface of stereo vision Frontend
 * 2) Proper but cumbersome way: understand the Frontend in depth and
 * create new smaller classes which have public interfaces that can be tested
 * and are used by the Frontend. In this case, getRelativePoseBody seems to be
 * an extremely low-level function so there is a lot of refactoring to do.
 * 3) Dirty way: make this test a 'friend class' and use the private members...
TEST_F(StereoVisionImuFrontendFixture, getRelativePoseBodyStereo) {
  // How to test this in the simplest possible way?
  // Compute everything twice?
  ref_stereo_frame->setIsKeyframe(true);
  ref_stereo_frame->setIsRectified(true);
  StereoVisionImuFrontend st(imu_params_, ImuBias{});
  st.stereoFrame_lkf_ = make_shared<StereoFrame>(*ref_stereo_frame);
  st.trackerStatusSummary_.lkf_T_k_stereo_ = Pose3(
      Rot3::Expmap(Vector3(0.1, -0.1, 0.2)), Vector3(0.1, 0.1, 0.1));
  Pose3 body_pose_cam = ref_stereo_frame->getBPoseCamLRect();
  // Expected answer
  Pose3 pose_expected = body_pose_cam *
      st.trackerStatusSummary_.lkf_T_k_stereo_ * body_pose_cam.inverse();
  // Actual answer
  Pose3 pose_actual = st.getRelativePoseBodyStereo();
  // Comparison
  EXPECT(gtsam::assert_equal(pose_actual, pose_expected));
}

/* ************************************************************************* */
TEST_F(StereoVisionImuFrontendFixture, getSmartStereoMeasurements) {
  clearStereoFrame(ref_stereo_frame);
  ref_stereo_frame->setIsKeyframe(true);

  auto stereo_camera =
      std::make_shared<StereoCamera>(cam_params_left_, cam_params_right_);

  StereoVisionImuFrontend st(
      FrontendParams(), imu_params_, ImuBias(), stereo_camera);

  // Landmarks_, left_keypoints_rectified_, right_keypoints_rectified_,
  // rightKeypoints_status

  // Parameters for the synthesis!
  const int num_valid = 12;
  const int num_landmark_invalid = 12;
  const int num_right_missing = 12;

  // Synthesize the input data!

  // valid!
  for (int i = 0; i < num_valid; i++) {
    double uL = rand() % 800;
    double uR = uL + (rand() % 80 - 40);
    double v = rand() % 600;
    ref_stereo_frame->left_frame_.landmarks_.push_back(i);
    ref_stereo_frame->left_frame_.scores_.push_back(1.0);
    ref_stereo_frame->left_keypoints_rectified_.push_back(
        StatusKeypointCV(KeypointStatus::VALID, cv::Point2f(uL, v)));
    ref_stereo_frame->right_keypoints_rectified_.push_back(
        StatusKeypointCV(KeypointStatus::VALID, cv::Point2f(uL, v)));
  }

  // right keypoints invalid!
  for (int i = 0; i < num_right_missing; i++) {
    double uL = rand() % 800;
    double uR = uL + (rand() % 80 - 40);
    double v = rand() % 600;
    ref_stereo_frame->left_frame_.landmarks_.push_back(i + num_valid);
    ref_stereo_frame->left_frame_.scores_.push_back(1.0);
    ref_stereo_frame->left_keypoints_rectified_.push_back(
        StatusKeypointCV(KeypointStatus::NO_RIGHT_RECT, cv::Point2f(uL, v)));
    ref_stereo_frame->right_keypoints_rectified_.push_back(
        StatusKeypointCV(KeypointStatus::NO_RIGHT_RECT, cv::Point2f(uL, v)));
  }

  // landmark missing!
  for (int i = 0; i < num_landmark_invalid; i++) {
    double uL = rand() % 800;
    double uR = uL + (rand() % 80 - 40);
    double v = rand() % 600;
    ref_stereo_frame->left_frame_.landmarks_.push_back(-1);
    ref_stereo_frame->left_frame_.scores_.push_back(1.0);
    ref_stereo_frame->left_keypoints_rectified_.push_back(
        StatusKeypointCV(KeypointStatus::VALID, cv::Point2f(uL, v)));
    ref_stereo_frame->right_keypoints_rectified_.push_back(
        StatusKeypointCV(KeypointStatus::VALID, cv::Point2f(uL, v)));
  }

  ref_stereo_frame->setIsRectified(true);
  fillStereoFrame(ref_stereo_frame);
  // Call the function!
  StereoMeasurements ssm;
  st.getSmartStereoMeasurements(ref_stereo_frame, &ssm);

  // Verify the correctness of the results!
  EXPECT_EQ(ssm.size(), num_valid + num_right_missing);
  std::set<int> landmark_set;

  for (auto s : ssm) {
    // No order requirement for the entries in ssm.
    // To avoid searching for the location for a landmark, the data is
    // synthesized following a simple convention:
    //         landmark_[i] = i; for landmark_[i] != -1;
    int landmark_id = s.first;
    EXPECT_EQ(ref_stereo_frame->left_keypoints_rectified_[landmark_id].second.x,
              s.second.uL());
    EXPECT_EQ(ref_stereo_frame->left_keypoints_rectified_[landmark_id].second.y,
              s.second.v());
    if (ref_stereo_frame->right_keypoints_rectified_[landmark_id].first ==
        KeypointStatus::VALID) {
      EXPECT_EQ(
          ref_stereo_frame->right_keypoints_rectified_[landmark_id].second.x,
          s.second.uR());
    } else {
      EXPECT_TRUE(std::isnan(s.second.uR()));
    }

    // Verify that there is no replicated entries in landmark_set.
    EXPECT_EQ(landmark_set.find(landmark_id), landmark_set.end());
    landmark_set.insert(landmark_id);
  }
}

TEST_F(StereoVisionImuFrontendFixture, processFirstFrame) {
  // Things to test:
  // 1. Feature detection (from tracker)
  // 2. results from sparseStereoMatching

  // Load a synthetic stereo pair with known ground-truth.
  CameraParams cam_params_left, cam_params_right;
  std::string synthetic_stereo_path(FLAGS_test_data_path + "/ForStereoTracker");
  cam_params_left.parseYAML(synthetic_stereo_path + "/camLeft.yaml");
  cam_params_right.parseYAML(synthetic_stereo_path + "/camRight.yaml");

  std::string img_name_left = synthetic_stereo_path + "/img_distort_left.png";
  std::string img_name_right = synthetic_stereo_path + "/img_distort_right.png";

  Pose3 camL_Pose_camR =
      cam_params_left.body_Pose_cam_.between(cam_params_right.body_Pose_cam_);

  // Assert certain properties of the synthetic data
  // Same intrinsics
  for (int i = 0; i < 4; i++) {
    EXPECT_EQ(cam_params_left.intrinsics_[i], cam_params_right.intrinsics_[i]);
  }

  // Already rectified
  Vector3 baseline = camL_Pose_camR.translation();
  EXPECT_GT(baseline(0), 0);
  EXPECT_NEAR(0, baseline(1), 1e-4);
  EXPECT_NEAR(0, baseline(2), 1e-4);

  FrontendParams p;  // default
  p.feature_detector_params_.min_distance_btw_tracked_and_detected_features_ =
      0.05;
  p.feature_detector_params_.quality_level_ = 0.1;
  p.stereo_matching_params_.nominal_baseline_ = baseline(0);
  p.stereo_matching_params_.max_point_dist_ = 500;
  p.stereo_matching_params_.templ_cols_ = 9;
  // TODO(nathan) this is required for depth to be correct; not sure if it's
  // loss of precision from the rhs integer feature positions or something else
  p.stereo_matching_params_.subpixel_refinement_ = true;

  StereoFrame first_stereo_frame(
      0,
      0,
      Frame(0,
            0,
            cam_params_left,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_left, p.stereo_matching_params_.equalize_image_)),
      Frame(0,
            0,
            cam_params_right,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_right, p.stereo_matching_params_.equalize_image_)));

  // Load the expected corners
  std::vector<gtsam::Point2> left_distort_corners =
      loadCorners(synthetic_stereo_path + "/corners_normal_left.txt");

  std::vector<gtsam::Point2> right_distort_corners =
      loadCorners(synthetic_stereo_path + "/corners_normal_right.txt");

  // Call StereoVisionImuFrontend::Process first frame!
  Camera::ConstPtr left_camera = std::make_shared<Camera>(cam_params_left);
  Camera::ConstPtr right_camera = std::make_shared<Camera>(cam_params_right);
  StereoCamera::ConstPtr stereo_camera =
      std::make_shared<StereoCamera>(cam_params_left, cam_params_right);

  StereoVisionImuFrontend st(p, imu_params_, ImuBias(), stereo_camera);

  EXPECT_FALSE(st.isInitialized());
  ImuStampS fake_imu_stamps;
  fake_imu_stamps.setZero(1, 3);
  ImuAccGyrS fake_imu_acc_gyr;
  fake_imu_acc_gyr.setRandom(6, 3);
  auto input = std::make_unique<StereoImuSyncPacket>(
      first_stereo_frame, fake_imu_stamps, fake_imu_acc_gyr);
  auto output_base = st.spinOnce(std::move(input));
  auto output = castUnique<StereoFrontendOutput>(std::move(output_base));
  EXPECT_TRUE(st.isInitialized());
  ASSERT_TRUE(output);
  const StereoFrame& sf = output->stereo_frame_lkf_;

  // Check the following results:
  // 1. Feature Detection
  // 2. SparseStereoMatching

  // Check feature detection results!
  // landmarks_, landmarksAge_, keypoints_, versors_
  const auto& left_frame = sf.left_frame_;
  const size_t& num_corners = left_frame.landmarks_.size();
  EXPECT_EQ(num_corners, left_frame.landmarks_age_.size());
  EXPECT_EQ(num_corners, left_frame.keypoints_.size());
  EXPECT_EQ(num_corners, left_frame.versors_.size());
  for (const auto& lmk_age : left_frame.landmarks_age_) {
    EXPECT_EQ(lmk_age, 1u);
  }
  for (const auto& lmk : left_frame.landmarks_) {
    EXPECT_GE(lmk, 0);
  }

  // Fill corner id map, useful for the tests later
  std::vector<int> corner_id_map_frame2gt;
  corner_id_map_frame2gt.reserve(num_corners);
  for (size_t i = 0u; i < num_corners; i++) {
    const KeypointCV& kp = left_frame.keypoints_[i];
    gtsam::Point2 gtsam_kp(kp.x, kp.y);
    int idx = findPointInVector(gtsam_kp, left_distort_corners);
    EXPECT_NE(idx, -1);
    corner_id_map_frame2gt.push_back(idx);
  }

  // Check for coherent versors
  for (size_t i = 0u; i < num_corners; i++) {
    Vector3 v_expect = UndistorterRectifier::GetBearingVector(
        left_frame.keypoints_[i], left_frame.cam_param_);
    Vector3 v_actual = left_frame.versors_[i];
    EXPECT_LT((v_actual - v_expect).norm(), 0.1);
  }

  // Test the results of sparse stereo matching!
  // Check:
  // bool isRectified_; // make sure to do that on each captured image
  // bool isKeyframe_;
  // KeypointsCV left_keypoints_rectified_;
  // KeypointsCV right_keypoints_rectified_;
  // vector<Kstatus> right_keypoints_status_;
  // vector<Vector3> keypoints_3d_;

  // The test data is simple enough so that all left corners have unique and
  // valid corresponding corner!
  EXPECT_TRUE(sf.isKeyframe());
  EXPECT_TRUE(sf.isRectified());

  // left_keypoints_rectified!
  const CameraParams& left_cam_params = sf.left_frame_.cam_param_;

  for (size_t i = 0u; i < num_corners; i++) {
    int idx_gt = corner_id_map_frame2gt[i];
    std::vector<cv::Point2d> undistorted;
    EXPECT_TRUE(
        gtsam::assert_equal(left_distort_corners[idx_gt],
                            gtsam::Point2(sf.left_frame_.keypoints_[i].x,
                                          sf.left_frame_.keypoints_[i].y),
                            2));

    get_undistorted_points(left_distort_corners[idx_gt],
                           left_cam_params,
                           stereo_camera,
                           undistorted);
    gtsam::Point2 pt_expect;
    pt_expect.x() = undistorted[0].x;
    pt_expect.y() = undistorted[0].y;

    gtsam::Point2 pt_actual(sf.left_keypoints_rectified_[i].second.x,
                            sf.left_keypoints_rectified_[i].second.y);

    EXPECT_TRUE(gtsam::assert_equal(pt_expect, pt_actual, 2));
  }

  // right_keypoints_rectified
  const CameraParams& right_cam_params = sf.right_frame_.cam_param_;

  for (size_t i = 0u; i < num_corners; i++) {
    int idx_gt = corner_id_map_frame2gt[i];
    std::vector<cv::Point2d> undistorted;
    EXPECT_TRUE(
        gtsam::assert_equal(right_distort_corners[idx_gt],
                            gtsam::Point2(sf.right_frame_.keypoints_[i].x,
                                          sf.right_frame_.keypoints_[i].y),
                            2));

    get_undistorted_points(right_distort_corners[idx_gt],
                           right_cam_params,
                           stereo_camera,
                           undistorted);
    gtsam::Point2 pt_expect;
    pt_expect.x() = undistorted[0].x;
    pt_expect.y() = undistorted[0].y;

    gtsam::Point2 pt_actual(sf.right_keypoints_rectified_[i].second.x,
                            sf.right_keypoints_rectified_[i].second.y);

    EXPECT_TRUE(gtsam::assert_equal(pt_expect, pt_actual, 2));
  }

  // right_keypoints_status_
  for (const auto& status : sf.left_keypoints_rectified_) {
    EXPECT_EQ(status.first, KeypointStatus::VALID);
  }

  // right_keypoints_status_
  for (const auto& status : sf.right_keypoints_rectified_) {
    EXPECT_EQ(status.first, KeypointStatus::VALID);
  }

  // keypoints depth
  std::vector<double> depth_gt =
      loadDepth(synthetic_stereo_path + "/depth_left.txt");

  for (size_t i = 0u; i < num_corners; i++) {
    int idx_gt = corner_id_map_frame2gt[i];
    double depth_expect = depth_gt[idx_gt];
    double depth_actual = sf.keypoints_3d_[i](2);

    // tollerance value is <1% of depth_expect
    // depth_expect is very large in our test data
    EXPECT_NEAR(depth_expect, depth_actual, 4);
  }

  // keypoints 3d
  for (size_t i = 0u; i < num_corners; i++) {
    int idx_gt = corner_id_map_frame2gt[i];
    gtsam::Vector3 v_actual = sf.keypoints_3d_[i];
    Vector3 versor = left_frame.versors_[i];
    Vector3 v_expected = versor * depth_gt[idx_gt] / versor(2);

    EXPECT_LT((v_expected - v_actual).norm(), 5);
  }
}

TEST_F(StereoVisionImuFrontendFixture, testDisparityCheck) {
  // Things to test:
  // check keyframe if current frame has low disparity but the last keyframe did
  // not check not keyframe if current frame has low disparity and last keyframe
  // had low disparity check keyframe if disparity is high and min time between
  // keyframes is reached check keyframe if max disparity thresehold reached
  // (and min time between keyframes has not passed) check if disparity is low
  // but max time between keyframes is reached

  CameraParams cam_params_left, cam_params_right;
  std::string synthetic_stereo_path(FLAGS_test_data_path + "/ForStereoTracker");
  cam_params_left.parseYAML(synthetic_stereo_path + "/camLeftEuroc.yaml");
  cam_params_right.parseYAML(synthetic_stereo_path + "/camRightEuroc.yaml");

  Pose3 camL_Pose_camR =
      cam_params_left.body_Pose_cam_.between(cam_params_right.body_Pose_cam_);

  // Already rectified
  Vector3 baseline = camL_Pose_camR.translation();

  FrontendParams p;  // default
  p.feature_detector_params_.min_distance_btw_tracked_and_detected_features_ =
      0.05;
  p.feature_detector_params_.quality_level_ = 0.1;
  p.stereo_matching_params_.nominal_baseline_ = baseline(0);
  p.stereo_matching_params_.max_point_dist_ = 500;
  p.max_disparity_since_lkf_ =
      30;  // not necessarily a value that would be put into practice, but used
           // to make testing easier

  ///// make initial stereo frame /////
  std::string img_name_left = synthetic_stereo_path + "/left_frame0000.jpg";
  std::string img_name_right = synthetic_stereo_path + "/right_frame0000.jpg";

  StereoFrame zeroth_stereo_frame(
      0,
      0,
      Frame(0,
            0,
            cam_params_left,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_left, p.stereo_matching_params_.equalize_image_)),
      Frame(0,
            0,
            cam_params_right,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_right, p.stereo_matching_params_.equalize_image_)));

  auto left_camera = std::make_shared<Camera>(cam_params_left);
  auto right_camera = std::make_shared<Camera>(cam_params_right);
  auto stereo_camera =
      std::make_shared<StereoCamera>(cam_params_left, cam_params_right);

  StereoVisionImuFrontend st(p, imu_params_, ImuBias(), stereo_camera);

  EXPECT_FALSE(st.isInitialized());
  ImuStampS zeroth_imu_stamps(1, 3);
  zeroth_imu_stamps << 0, 0, 0;
  ImuAccGyrS fake_imu_acc_gyr;
  fake_imu_acc_gyr.setRandom(6, 3);
  auto input0 = std::make_unique<StereoImuSyncPacket>(
      zeroth_stereo_frame, zeroth_imu_stamps, fake_imu_acc_gyr);
  auto output_base0 = st.spinOnce(std::move(input0));
  auto output0 = castUnique<StereoFrontendOutput>(std::move(output_base0));
  const auto& sf0 = output0->stereo_frame_lkf_;
  EXPECT_TRUE(sf0.isKeyframe());

  ///// make next stereo frame - disparity is low and is keyframe /////
  img_name_left = synthetic_stereo_path + "/left_frame0000.jpg";
  img_name_right = synthetic_stereo_path + "/right_frame0000.jpg";

  StereoFrame first_stereo_frame(
      1,
      3.1e+6,
      Frame(1,
            3.1e+6,
            cam_params_left,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_left, p.stereo_matching_params_.equalize_image_)),
      Frame(1,
            3.1e+6,
            cam_params_right,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_right, p.stereo_matching_params_.equalize_image_)));
  ImuAccGyrS fake_imu_acc_gyr1;
  fake_imu_acc_gyr1.setRandom(6, 2);
  ImuStampS first_imu_stamps(1, 2);
  first_imu_stamps << 3e+6, 3.1e+6;
  auto input1 = std::make_unique<StereoImuSyncPacket>(
      first_stereo_frame, first_imu_stamps, fake_imu_acc_gyr1);
  auto output_base1 = st.spinOnce(std::move(input1));
  auto output1 = castUnique<StereoFrontendOutput>(std::move(output_base1));
  EXPECT_TRUE(st.isInitialized());
  ASSERT_TRUE(output1);
  const auto& sf1 = output1->stereo_frame_lkf_;

  EXPECT_TRUE(sf1.isKeyframe());

  ///// make next stereo frame - disparity is high and is keyframe /////
  img_name_left = synthetic_stereo_path + "/left_frame0001.jpg";
  img_name_right = synthetic_stereo_path + "/right_frame0001.jpg";
  StereoFrame second_stereo_frame(
      2,
      6.1e+6,
      Frame(2,
            6.1e+6,
            cam_params_left,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_left, p.stereo_matching_params_.equalize_image_)),
      Frame(2,
            6.1e+6,
            cam_params_right,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_right, p.stereo_matching_params_.equalize_image_)));
  ImuAccGyrS fake_imu_acc_gyr2;
  fake_imu_acc_gyr2.setRandom(6, 2);
  ImuStampS second_imu_stamps(1, 2);
  second_imu_stamps << 6e+6, 6.1e+6;
  auto input2 = std::make_unique<StereoImuSyncPacket>(
      second_stereo_frame, second_imu_stamps, fake_imu_acc_gyr2);
  auto output_base2 = st.spinOnce(std::move(input2));
  auto output2 = castUnique<StereoFrontendOutput>(std::move(output_base2));
  EXPECT_TRUE(st.isInitialized());
  ASSERT_TRUE(output2);
  const auto& sf2 = output2->stereo_frame_lkf_;

  EXPECT_TRUE(sf2.isKeyframe());

  ///// make next stereo frame - disparity is very high, above max disparity
  /// threshold
  //                             while min time has not passed between keyframes
  //                             - is keyframe
  img_name_left = synthetic_stereo_path + "/left_frame0008.jpg";
  img_name_right = synthetic_stereo_path + "/right_frame0008.jpg";
  StereoFrame third_stereo_frame(
      3,
      6.11e+6,
      Frame(3,
            6.11e+6,
            cam_params_left,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_left, p.stereo_matching_params_.equalize_image_)),
      Frame(3,
            6.11e+6,
            cam_params_right,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_right, p.stereo_matching_params_.equalize_image_)));
  ImuAccGyrS fake_imu_acc_gyr3;
  fake_imu_acc_gyr3.setRandom(6, 2);
  ImuStampS third_imu_stamps(1, 2);
  third_imu_stamps << 6.105e+6, 6.11e+6;
  auto input3 = std::make_unique<StereoImuSyncPacket>(
      third_stereo_frame, third_imu_stamps, fake_imu_acc_gyr3);
  auto output_base3 = st.spinOnce(std::move(input3));
  auto output3 = castUnique<StereoFrontendOutput>(std::move(output_base3));
  EXPECT_TRUE(st.isInitialized());
  ASSERT_TRUE(output3);
  const auto& sf3 = output3->stereo_frame_lkf_;

  EXPECT_TRUE(sf3.isKeyframe());

  ///// make next stereo frame - disparity is low but is keyframe since prior
  /// keyframe didn't have low disparity /////
  img_name_left = synthetic_stereo_path + "/left_frame0008.jpg";
  img_name_right = synthetic_stereo_path + "/right_frame0008.jpg";
  StereoFrame fourth_stereo_frame(
      4,
      9.11e+6,
      Frame(4,
            9.11e+6,
            cam_params_left,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_left, p.stereo_matching_params_.equalize_image_)),
      Frame(4,
            9.11e+6,
            cam_params_right,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_right, p.stereo_matching_params_.equalize_image_)));
  ImuAccGyrS fake_imu_acc_gyr4;
  fake_imu_acc_gyr4.setRandom(6, 2);
  ImuStampS fourth_imu_stamps(1, 2);
  fourth_imu_stamps << 9.105e+6, 9.11e+6;
  auto input4 = std::make_unique<StereoImuSyncPacket>(
      fourth_stereo_frame, fourth_imu_stamps, fake_imu_acc_gyr4);
  auto output_base4 = st.spinOnce(std::move(input4));
  auto output4 = castUnique<StereoFrontendOutput>(std::move(output_base4));
  EXPECT_TRUE(st.isInitialized());
  ASSERT_TRUE(output4);
  const auto& sf4 = output4->stereo_frame_lkf_;

  EXPECT_TRUE(sf4.isKeyframe());

  ///// make next stereo frame - disparity is low and is not keyframe /////
  img_name_left = synthetic_stereo_path + "/left_frame0008.jpg";
  img_name_right = synthetic_stereo_path + "/right_frame0008.jpg";
  StereoFrame fifth_stereo_frame(
      5,
      12.11e+6,
      Frame(5,
            12.11e+6,
            cam_params_left,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_left, p.stereo_matching_params_.equalize_image_)),
      Frame(5,
            12.11e+6,
            cam_params_right,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_right, p.stereo_matching_params_.equalize_image_)));
  ImuAccGyrS fake_imu_acc_gyr5;
  fake_imu_acc_gyr5.setRandom(6, 2);
  ImuStampS fifth_imu_stamps(1, 2);
  fifth_imu_stamps << 12.105e+6, 12.11e+6;
  auto input5 = std::make_unique<StereoImuSyncPacket>(
      fifth_stereo_frame, fifth_imu_stamps, fake_imu_acc_gyr5);
  auto output_base5 = st.spinOnce(std::move(input5));
  auto output5 = castUnique<StereoFrontendOutput>(std::move(output_base5));
  EXPECT_TRUE(st.isInitialized());
  ASSERT_TRUE(output5);
  const auto& sf5 = output5->stereo_frame_lkf_;

  EXPECT_FALSE(sf5.isKeyframe());

  ///// make next stereo frame - disparity is low but is a keyframe since max
  /// time since lk has passed /////
  img_name_left = synthetic_stereo_path + "/left_frame0008.jpg";
  img_name_right = synthetic_stereo_path + "/right_frame0008.jpg";
  StereoFrame sixth_stereo_frame(
      6,
      52.11e+9,
      Frame(6,
            52.11e+9,
            cam_params_left,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_left, p.stereo_matching_params_.equalize_image_)),
      Frame(6,
            52.11e+9,
            cam_params_right,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_right, p.stereo_matching_params_.equalize_image_)));
  ImuAccGyrS fake_imu_acc_gyr6;
  fake_imu_acc_gyr6.setRandom(6, 2);
  ImuStampS sixth_imu_stamps(1, 2);
  sixth_imu_stamps << 52.105e+9, 52.11e+9;
  auto input6 = std::make_unique<StereoImuSyncPacket>(
      sixth_stereo_frame, sixth_imu_stamps, fake_imu_acc_gyr6);
  auto output_base6 = st.spinOnce(std::move(input6));
  auto output6 = castUnique<StereoFrontendOutput>(std::move(output_base6));
  EXPECT_TRUE(st.isInitialized());
  ASSERT_TRUE(output6);
  const auto& sf6 = output6->stereo_frame_lkf_;

  EXPECT_TRUE(sf6.isKeyframe());
}

TEST_F(StereoVisionImuFrontendFixture, testLostFeatureTrack) {
  // Things to test:
  // receives a frame that is blinded (feature tracking is lost but some texture
  // is still in the image) pipeline should not make the blinded frame a
  // keyframe and should not throw an error

  CameraParams cam_params_left, cam_params_right;
  std::string synthetic_stereo_path(FLAGS_test_data_path + "/ForStereoTracker");
  cam_params_left.parseYAML(synthetic_stereo_path + "/camLeftEuroc.yaml");
  cam_params_right.parseYAML(synthetic_stereo_path + "/camRightEuroc.yaml");

  Pose3 camL_Pose_camR =
      cam_params_left.body_Pose_cam_.between(cam_params_right.body_Pose_cam_);

  // Already rectified
  Vector3 baseline = camL_Pose_camR.translation();

  FrontendParams p;  // default
  p.feature_detector_params_.min_distance_btw_tracked_and_detected_features_ =
      0.05;
  p.feature_detector_params_.quality_level_ = 0.1;
  p.stereo_matching_params_.nominal_baseline_ = baseline(0);
  p.stereo_matching_params_.max_point_dist_ = 500;
  p.max_disparity_since_lkf_ =
      30;  // not necessarily a value that would be put into practice, but used
           // to make testing easier

  // make two simple images
  cv::Mat simple_left = cv::Mat::zeros(cam_params_left.image_size_, CV_8UC1);
  cv::Mat simple_right = cv::Mat::zeros(cam_params_right.image_size_, CV_8UC1);
  simple_left.colRange(4, 6).rowRange(2, 4) = 255.0;
  simple_right.colRange(2, 4).rowRange(2, 4) = 255.0;

  // make two blinded images
  cv::Mat blinded_left = cv::Mat::zeros(cam_params_left.image_size_, CV_8UC1);
  cv::Mat blinded_right = cv::Mat::zeros(cam_params_right.image_size_, CV_8UC1);
  // create minimal texture, allows some keypoints to be generated but they
  // won't correspond to anything in the prior frame
  blinded_left.colRange(300, 310).rowRange(300, 320) = 255.0;
  blinded_right.colRange(290, 300).rowRange(300, 320) = 255.0;

  StereoFrame zeroth_stereo_frame(0,
                                  0,
                                  Frame(0, 0, cam_params_left, simple_left),
                                  Frame(0, 0, cam_params_right, simple_right));

  Camera::ConstPtr left_camera =
      std::make_shared<Camera>(cam_params_left);
  Camera::ConstPtr right_camera =
      std::make_shared<Camera>(cam_params_right);
  StereoCamera::ConstPtr stereo_camera =
      std::make_shared<StereoCamera>(cam_params_left, cam_params_right);

  StereoVisionImuFrontend st(p, imu_params_, ImuBias(), stereo_camera);

  EXPECT_FALSE(st.isInitialized());
  ImuStampS zeroth_imu_stamps(1, 3);
  zeroth_imu_stamps << 0, 0, 0;
  ImuAccGyrS fake_imu_acc_gyr;
  fake_imu_acc_gyr.setRandom(6, 3);
  auto input0 = std::make_unique<StereoImuSyncPacket>(
      zeroth_stereo_frame, zeroth_imu_stamps, fake_imu_acc_gyr);
  auto output_base0 = st.spinOnce(std::move(input0));
  auto output0 = castUnique<StereoFrontendOutput>(std::move(output_base0));
  const StereoFrame& sf0 = output0->stereo_frame_lkf_;
  EXPECT_TRUE(sf0.isKeyframe());

  StereoFrame first_stereo_frame(
      1,
      3.1e+6,
      Frame(1, 3.1e+6, cam_params_left, blinded_left),
      Frame(1, 3.1e+6, cam_params_right, blinded_right));
  ImuAccGyrS fake_imu_acc_gyr1;
  fake_imu_acc_gyr1.setRandom(6, 2);
  ImuStampS first_imu_stamps(1, 2);
  first_imu_stamps << 3e+6, 3.1e+6;
  auto input1 = std::make_unique<StereoImuSyncPacket>(
      first_stereo_frame, first_imu_stamps, fake_imu_acc_gyr1);
  auto output_base1 = st.spinOnce(std::move(input1));
  auto output1 = castUnique<StereoFrontendOutput>(std::move(output_base1));
  EXPECT_TRUE(st.isInitialized());
  ASSERT_TRUE(output1);
  const auto& sf1 = output1->stereo_frame_lkf_;

  EXPECT_FALSE(sf1.isKeyframe());

  StereoFrame second_stereo_frame(
      2,
      6.1e+6,
      Frame(2, 6.1e+6, cam_params_left, blinded_left),
      Frame(2, 6.1e+6, cam_params_right, blinded_right));
  ImuAccGyrS fake_imu_acc_gyr2;
  fake_imu_acc_gyr2.setRandom(6, 2);
  ImuStampS second_imu_stamps(1, 2);
  second_imu_stamps << 6e+6, 6.1e+6;
  auto input2 = std::make_unique<StereoImuSyncPacket>(
      second_stereo_frame, second_imu_stamps, fake_imu_acc_gyr2);
  auto output_base2 = st.spinOnce(std::move(input2));
  auto output2 = castUnique<StereoFrontendOutput>(std::move(output_base2));
  EXPECT_TRUE(st.isInitialized());
  ASSERT_TRUE(output2);
  const auto& sf2 = output2->stereo_frame_lkf_;

  // should be able to track, optical flow works since points bear
  // correspondence to prior image
  EXPECT_TRUE(sf2.isKeyframe());
}

}  // namespace VIO
