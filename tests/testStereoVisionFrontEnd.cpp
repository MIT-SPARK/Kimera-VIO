/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testStereoVisionFrontEnd.cpp
 * @brief  test StereoVisionFrontEnd
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd.h"
#include "kimera-vio/frontend/Tracker.h"

DECLARE_string(test_data_path);

using namespace gtsam;
using namespace VIO;
using namespace cv;

class StereoVisionFrontEndFixture : public ::testing::Test {
 public:
  StereoVisionFrontEndFixture()
      : stereo_FLAGS_test_data_path(FLAGS_test_data_path +
                                    std::string("/ForStereoFrame/")) {
    initializeData();
  }

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  // Helper function
  void initializeData() {
    CameraParams cam_params_left, cam_params_right;
    cam_params_left.parseYAML(stereo_FLAGS_test_data_path + "/sensorLeft.yaml");
    cam_params_right.parseYAML(stereo_FLAGS_test_data_path +
                               "/sensorRight.yaml");

    std::string img_name_ref_left =
        stereo_FLAGS_test_data_path + "left_img_0.png";
    std::string img_name_ref_right =
        stereo_FLAGS_test_data_path + "right_img_0.png";

    std::string img_name_cur_left =
        stereo_FLAGS_test_data_path + "left_img_1.png";
    std::string img_name_cur_right =
        stereo_FLAGS_test_data_path + "right_img_1.png";

    // Data for testing "geometricOutlierRejectionMono"
    ref_frame = std::make_shared<Frame>(
        id_ref, timestamp_ref, cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(img_name_ref_left));
    cur_frame = std::make_shared<Frame>(
        id_cur, timestamp_cur, cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(img_name_cur_left));

    VioFrontEndParams tp;

    ref_stereo_frame = std::make_shared<StereoFrame>(
        id_ref,
        timestamp_ref,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_ref_left, tp.stereo_matching_params_.equalize_image_),
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_ref_right, tp.stereo_matching_params_.equalize_image_),
        cam_params_right,
        tp.stereo_matching_params_);

    cur_stereo_frame = std::make_shared<StereoFrame>(
        id_cur,
        timestamp_cur,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_cur_left, tp.stereo_matching_params_.equalize_image_),
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_cur_right, tp.stereo_matching_params_.equalize_image_),
        cam_params_right,
        tp.stereo_matching_params_);

    // Imu Params
    imu_params_.acc_walk_ = 1;
    imu_params_.gyro_walk_ = 1;
    imu_params_.acc_noise_ = 1;
    imu_params_.gyro_noise_ = 1;
    imu_params_.imu_integration_sigma_ = 1;

    // Set randomness!
    srand(0);
  }

  void clearFrame(Frame* f) {
    f->keypoints_.clear();
    f->landmarks_.clear();
    f->landmarksAge_.clear();
    f->versors_.clear();
  }

  void clearStereoFrame(std::shared_ptr<StereoFrame>& sf) {
    clearFrame(sf->getLeftFrameMutable());
    clearFrame(sf->getRightFrameMutable());
    sf->keypoints_3d_.clear();
    sf->keypoints_depth_.clear();
    sf->right_keypoints_status_.clear();
  }

  void fillStereoFrame(std::shared_ptr<StereoFrame>& sf) {
    // Fill the fields in a StereoFrame to pass the sanity check
    // StereoFrame::checkStereoFrame
    const int num_keypoints = sf->getLeftFrame().landmarks_.size();

    // left.y == right.y
    // keypoints_3d[i](2) == keypoints_depth_[i]
    // (right_keypoints_status[i] == valid && right_frame_.keypoints_[i] != 0)
    // OR (right_keypoints_status[i] != valid && keypoints_depth_[i] == 0)

    // right_keypoints_status_.size
    if (sf->right_keypoints_status_.size() != num_keypoints) {
      sf->right_keypoints_status_ = std::vector<KeypointStatus>(num_keypoints);
      for (int i = 0; i < num_keypoints; i++) {
        sf->right_keypoints_status_[i] = KeypointStatus::VALID;
      }
    }

    // left_frame_.keypoints_.size
    if (sf->getLeftFrame().keypoints_.size() != num_keypoints) {
      sf->getLeftFrameMutable()->keypoints_ = KeypointsCV(num_keypoints);
      for (int i = 0; i < num_keypoints; i++) {
        sf->getLeftFrameMutable()->keypoints_[i] = KeypointCV(i, i);
      }
    }

    // right_frame_.keypoints_.size
    if (sf->getRightFrame().keypoints_.size() != num_keypoints) {
      sf->getRightFrameMutable()->keypoints_ = KeypointsCV(num_keypoints);
      for (int i = 0; i < num_keypoints; i++) {
        if (sf->right_keypoints_status_[i] == KeypointStatus::VALID) {
          sf->getRightFrameMutable()->keypoints_[i] =
              KeypointCV(i + 20, i + (i % 3 - 1));
        } else {
          sf->getRightFrameMutable()->keypoints_[i] = KeypointCV(0, 0);
        }
      }
    }

    // keypoints_depth_.size
    if (sf->keypoints_depth_.size() != num_keypoints) {
      sf->keypoints_depth_ = std::vector<double>(num_keypoints);
      for (int i = 0; i < num_keypoints; i++) {
        if (sf->right_keypoints_status_[i] == KeypointStatus::VALID) {
          sf->keypoints_depth_[i] = 1;
        } else {
          sf->keypoints_depth_[i] = 0;
        }
      }
    }

    // keypoints_3d_.size
    if (sf->keypoints_3d_.size() != num_keypoints) {
      sf->keypoints_3d_ = std::vector<Vector3>(num_keypoints);
      for (int i = 0; i < num_keypoints; i++) {
        sf->keypoints_3d_[i](2) = sf->keypoints_depth_[i];
      }
    }

    // left_keypoints_rectified.size
    if (sf->left_keypoints_rectified_.size() != num_keypoints) {
      sf->left_keypoints_rectified_ = KeypointsCV(num_keypoints);
    }

    // right_keypoints_rectified.size
    if (sf->right_keypoints_rectified_.size() != num_keypoints) {
      sf->right_keypoints_rectified_ = KeypointsCV(num_keypoints);
    }
  }

  std::vector<Point2> loadCorners(const std::string& filepath) {
    std::vector<Point2> corners;
    std::ifstream fin(filepath);
    int num_corners;
    fin >> num_corners;
    corners.reserve(num_corners);
    for (int i = 0; i < num_corners; i++) {
      double x, y;
      fin >> x >> y;
      // Convert points coordinates from MATLAB convention to c++ convention:
      corners.push_back(Point2(x - 1, y - 1));
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

  int findCorners(const Point2 pt_query, const std::vector<Point2>& pt_set) {
    for (int i = 0; i < pt_set.size(); i++) {
      if (pt_set[i].equals(pt_query, 3)) return i;
    }
    return -1;
  }

  std::vector<Point2> convertCornersAcrossCameras(
      const std::vector<Point2>& corners_in,
      const DistortionModel& calib_in,
      const Cal3_S2& calib_out) {
    CHECK_DOUBLE_EQ(calib_in.skew(), 0.0);
    CHECK_DOUBLE_EQ(calib_out.skew(), 0.0);

    std::vector<Point2> corners_out;
    corners_out.reserve(corners_in.size());
    for (int i = 0; i < corners_in.size(); i++) {
      double xn = (corners_in[i].x() - calib_in.px()) / calib_in.fx();
      double yn = (corners_in[i].y() - calib_in.py()) / calib_in.fy();
      double xo = xn * calib_out.fx() + calib_out.px();
      double yo = yn * calib_out.fy() + calib_out.py();
      corners_out.push_back(Point2(xo, yo));
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

  ImuParams imu_params_;
  std::string stereo_FLAGS_test_data_path;
};

/* This test is flawed in that it is using private members of the frontend...
 * There is three ways to go around this:
 * 1) Proper way: test getRelativePoseBodyStereo by just using the public
 * interface of stereo vision frontend
 * 2) Proper but cumbersome way: understand the frontend in depth and
 * create new smaller classes which have public interfaces that can be tested
 * and are used by the frontend. In this case, getRelativePoseBody seems to be
 * an extremely low-level function so there is a lot of refactoring to do.
 * 3) Dirty way: make this test a 'friend class' and use the private members...
TEST_F(StereoVisionFrontEndFixture, getRelativePoseBodyMono) {
  // How to test this in the simplest possible way?
  // Compute everything twice?
  ref_stereo_frame->setIsKeyframe(true);
  ref_stereo_frame->setIsRectified(true);
  cur_stereo_frame->setIsKeyframe(true);
  cur_stereo_frame->setIsRectified(true);
  // Avoid the Most Vexing Parse compilation error with bracket initialization.
  StereoVisionFrontEnd st(imu_params_, ImuBias{});
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
  EXPECT(assert_equal(pose_actual, pose_expected));
}

/* ************************************************************************* */
/* This test is flawed in that it is using private members of the frontend...
 * There is three ways to go around this:
 * 1) Proper way: test getRelativePoseBodyStereo by just using the public
 * interface of stereo vision frontend
 * 2) Proper but cumbersome way: understand the frontend in depth and
 * create new smaller classes which have public interfaces that can be tested
 * and are used by the frontend. In this case, getRelativePoseBody seems to be
 * an extremely low-level function so there is a lot of refactoring to do.
 * 3) Dirty way: make this test a 'friend class' and use the private members...
TEST_F(StereoVisionFrontEndFixture, getRelativePoseBodyStereo) {
  // How to test this in the simplest possible way?
  // Compute everything twice?
  ref_stereo_frame->setIsKeyframe(true);
  ref_stereo_frame->setIsRectified(true);
  StereoVisionFrontEnd st(imu_params_, ImuBias{});
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
  EXPECT(assert_equal(pose_actual, pose_expected));
}

/* ************************************************************************* */
TEST_F(StereoVisionFrontEndFixture, getSmartStereoMeasurements) {
  clearStereoFrame(ref_stereo_frame);
  ref_stereo_frame->setIsKeyframe(true);
  ref_stereo_frame->setIsRectified(true);

  StereoVisionFrontEnd st(imu_params_, ImuBias{});

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
    ref_stereo_frame->getLeftFrameMutable()->landmarks_.push_back(i);
    ref_stereo_frame->getLeftFrameMutable()->scores_.push_back(1.0);
    ref_stereo_frame->left_keypoints_rectified_.push_back(cv::Point2f(uL, v));
    ref_stereo_frame->right_keypoints_rectified_.push_back(cv::Point2f(uL, v));
    ref_stereo_frame->right_keypoints_status_.push_back(KeypointStatus::VALID);
  }

  // right keypoints invalid!
  for (int i = 0; i < num_right_missing; i++) {
    double uL = rand() % 800;
    double uR = uL + (rand() % 80 - 40);
    double v = rand() % 600;
    ref_stereo_frame->getLeftFrameMutable()->landmarks_.push_back(i +
                                                                  num_valid);
    ref_stereo_frame->getLeftFrameMutable()->scores_.push_back(1.0);
    ref_stereo_frame->left_keypoints_rectified_.push_back(cv::Point2f(uL, v));
    ref_stereo_frame->right_keypoints_rectified_.push_back(cv::Point2f(uL, v));
    ref_stereo_frame->right_keypoints_status_.push_back(
        KeypointStatus::NO_RIGHT_RECT);
  }

  // landmark missing!
  for (int i = 0; i < num_landmark_invalid; i++) {
    double uL = rand() % 800;
    double uR = uL + (rand() % 80 - 40);
    double v = rand() % 600;
    ref_stereo_frame->getLeftFrameMutable()->landmarks_.push_back(-1);
    ref_stereo_frame->getLeftFrameMutable()->scores_.push_back(1.0);
    ref_stereo_frame->left_keypoints_rectified_.push_back(cv::Point2f(uL, v));
    ref_stereo_frame->right_keypoints_rectified_.push_back(cv::Point2f(uL, v));
    ref_stereo_frame->right_keypoints_status_.push_back(KeypointStatus::VALID);
  }

  fillStereoFrame(ref_stereo_frame);
  // Call the function!
  SmartStereoMeasurements ssm;
  auto ssm_ptr = st.getSmartStereoMeasurements(*ref_stereo_frame);
  CHECK(ssm_ptr);
  ssm = *ssm_ptr;

  // Verify the correctness of the results!
  EXPECT_EQ(ssm.size(), num_valid + num_right_missing);
  std::set<int> landmark_set;

  for (auto s : ssm) {
    // No order requirement for the entries in ssm.
    // To avoid searching for the location for a landmark, the data is
    // synthesized following a simple convention:
    //         landmark_[i] = i; for landmark_[i] != -1;
    int landmark_id = s.first;
    EXPECT_EQ(ref_stereo_frame->left_keypoints_rectified_[landmark_id].x,
              s.second.uL());
    EXPECT_EQ(ref_stereo_frame->left_keypoints_rectified_[landmark_id].y,
              s.second.v());
    if (ref_stereo_frame->right_keypoints_status_[landmark_id] ==
        KeypointStatus::VALID) {
      EXPECT_EQ(ref_stereo_frame->right_keypoints_rectified_[landmark_id].x,
                s.second.uR());
    } else {
      EXPECT_TRUE(std::isnan(s.second.uR()));
    }

    // Verify that there is no replicated entries in landmark_set.
    EXPECT_EQ(landmark_set.find(landmark_id), landmark_set.end());
    landmark_set.insert(landmark_id);
  }
}

// TODO(Toni): completely change this test...
TEST_F(StereoVisionFrontEndFixture, DISABLED_processFirstFrame) {
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

  // cout << "camL_Pose_camR = " << camL_Pose_camR << endl;
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

  VioFrontEndParams p = VioFrontEndParams();  // default
  p.min_distance_ = 0.05;
  p.quality_level_ = 0.1;
  p.stereo_matching_params_.nominal_baseline_ = baseline(0);

  StereoFrame first_stereo_frame(
      0,
      0,
      UtilsOpenCV::ReadAndConvertToGrayScale(
          img_name_left, p.stereo_matching_params_.equalize_image_),
      cam_params_left,
      UtilsOpenCV::ReadAndConvertToGrayScale(
          img_name_right, p.stereo_matching_params_.equalize_image_),
      cam_params_right,
      p.stereo_matching_params_);

  first_stereo_frame.getLeftFrameMutable()->cam_param_.body_Pose_cam_ = Pose3(
      first_stereo_frame.getLeftFrame().cam_param_.body_Pose_cam_.rotation(),
      first_stereo_frame.getLeftFrame()
          .cam_param_.body_Pose_cam_.translation());

  first_stereo_frame.getRightFrameMutable()->cam_param_.body_Pose_cam_ = Pose3(
      first_stereo_frame.getRightFrame().cam_param_.body_Pose_cam_.rotation(),
      first_stereo_frame.getRightFrame()
          .cam_param_.body_Pose_cam_.translation());

  // Load the expected corners
  std::vector<Point2> left_distort_corners =
      loadCorners(synthetic_stereo_path + "/corners_normal_left.txt");

  // Call StereoVisionFrontEnd::Process first frame!
  StereoVisionFrontEnd st(imu_params_, ImuBias(), p);
  const StereoFrame &sf = st.processFirstStereoFrame(first_stereo_frame);

  // Check the following results:
  // 1. Feature Detection
  // 2. SparseStereoMatching

  // Check feature detection results!
  // landmarks_, landmarksAge_, keypoints_, versors_
  const Frame &left_frame = sf.getLeftFrame();
  const int num_corners = left_frame.landmarks_.size();
  EXPECT_EQ(num_corners, left_frame.landmarksAge_.size());
  EXPECT_EQ(num_corners, left_frame.keypoints_.size());
  EXPECT_EQ(num_corners, left_frame.versors_.size());
  for (auto lm : left_frame.landmarksAge_) {
    EXPECT_EQ(lm, 1);
  }
  for (auto lid : left_frame.landmarks_) {
    EXPECT_GE(lid, 0);
  }

  std::vector<int> corner_id_map_frame2gt;  // useful for the tests later
  corner_id_map_frame2gt.reserve(num_corners);
  for (int i = 0; i < num_corners; i++) {
    KeypointCV pt_cv = left_frame.keypoints_[i];
    Point2 pt(pt_cv.x, pt_cv.y);
    int idx = findCorners(pt, left_distort_corners);
    EXPECT_GE(idx, 0);
    corner_id_map_frame2gt.push_back(idx);
  }
  for (int i = 0; i < num_corners; i++) {
    Vector3 v_expect =
        Frame::calibratePixel(left_frame.keypoints_[i], left_frame.cam_param_);
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
  // vector<double> keypoints_depth_;
  // vector<Vector3> keypoints_3d_;

  // The test data is simple enough so that all left corners have unique and
  // valid corresponding corner!
  EXPECT_TRUE(sf.isKeyframe());
  EXPECT_TRUE(sf.isRectified());

  // left_keypoints_rectified!
  std::vector<Point2> left_undistort_corners =
      loadCorners(synthetic_stereo_path + "/corners_undistort_left.txt");
  std::vector<Point2> left_rect_corners =
      convertCornersAcrossCameras(left_undistort_corners,
                                  *(sf.getLeftFrame().cam_param_.distortion_),
                                  sf.getLeftUndistRectCamMat());
  for (int i = 0; i < num_corners; i++) {
    int idx_gt = corner_id_map_frame2gt[i];
    Point2 pt_expect = left_rect_corners[idx_gt];
    Point2 pt_actual = Point2(sf.left_keypoints_rectified_[i].x,
                              sf.left_keypoints_rectified_[i].y);
    EXPECT_TRUE(assert_equal(pt_expect, pt_actual, 2));
  }

  // right_keypoints_rectified
  std::vector<Point2> right_undistort_corners =
      loadCorners(synthetic_stereo_path + "/corners_undistort_right.txt");
  std::vector<Point2> right_rect_corners =
      convertCornersAcrossCameras(right_undistort_corners,
                                  *(sf.getRightFrame().cam_param_.distortion_),
                                  sf.getRightUndistRectCamMat());
  for (int i = 0; i < num_corners; i++) {
    int idx_gt = corner_id_map_frame2gt[i];
    Point2 pt_expect = right_rect_corners[idx_gt];
    Point2 pt_actual = Point2(sf.right_keypoints_rectified_[i].x,
                              sf.right_keypoints_rectified_[i].y);
    EXPECT_TRUE(assert_equal(pt_expect, pt_actual, 2));
  }

  // right_keypoints_status_
  for (auto status : sf.right_keypoints_status_) {
    EXPECT_EQ(status, KeypointStatus::VALID);
  }

  // keypoints depth
  std::vector<double> depth_gt =
      loadDepth(synthetic_stereo_path + "/depth_left.txt");

  for (int i = 0; i < num_corners; i++) {
    int idx_gt = corner_id_map_frame2gt[i];
    double depth_expect = depth_gt[idx_gt];
    double depth_actual = sf.keypoints_depth_[i];
    EXPECT_NEAR(depth_expect, depth_actual, 1e-2);
  }

  // keypoints 3d
  for (int i = 0; i < num_corners; i++) {
    int idx_gt = corner_id_map_frame2gt[i];
    double depth_expect = depth_gt[idx_gt];
    double depth_actual = sf.keypoints_depth_[i];
    Vector3 v_expected =
        Frame::calibratePixel(KeypointCV(left_distort_corners[idx_gt].x(),
                                         left_distort_corners[idx_gt].y()),
                              left_frame.cam_param_);
    v_expected = v_expected * (depth_gt[idx_gt]);
    Vector3 v_actual = sf.keypoints_3d_[i];
    EXPECT_LT((v_expected - v_actual).norm(), 0.1);
  }
}
