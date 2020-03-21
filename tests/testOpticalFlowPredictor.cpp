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

#include "kimera-vio/frontend/OpticalFlowPredictor.h"
#include "kimera-vio/frontend/OpticalFlowPredictorFactory.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"

DECLARE_string(test_data_path);

namespace VIO {

class OpticalFlowPredictorFixture : public ::testing::Test {
 public:
  OpticalFlowPredictorFixture()
      : vio_params_(FLAGS_test_data_path + "/EurocParams"),
        calib_(1.0, 1.0, 0.0, 1.0, 1.0),
        lmks_(),
        optical_flow_predictor_(nullptr) {
    CHECK_GT(camera_params_.size(), 1u);
    optical_flow_predictor_ =
        OpticalFlowPredictorFactory::makeOpticalFlowPredictor(
            vio_params_.frontend_params_.optical_flow_predictor_type_,
            vio_params_.camera_params_.at(0).camera_matrix_);
    buildSceneLandmarks(&lmks_);
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
        vio_params_.camera_params_.at(0).camera_matrix_);
  }

  // Build scene
  void buildSceneLandmarks(Landmarks* lmks) {
    CHECK_NOTNULL(lmks)->resize();
    // A cube in 3D, axis-aligned with the world frame of reference.

    // Bottom
    lmks->push_back(Landmark(0.0, 0.0, 0.0));
    lmks->push_back(Landmark(0.0, 1.0, 0.0));
    lmks->push_back(Landmark(1.0, 0.0, 0.0));
    lmks->push_back(Landmark(1.0, 1.0, 0.0));

    // Top
    lmks->push_back(Landmark(0.0, 0.0, 1.0));
    lmks->push_back(Landmark(0.0, 1.0, 1.0));
    lmks->push_back(Landmark(1.0, 0.0, 1.0));
    lmks->push_back(Landmark(1.0, 1.0, 1.0));
  }

  // Default Parms
  Landmarks lmks_;
  gtsam::Cal3_S2 calib_;
  gtsam::PinholeCamera<gtsam::Cal3_S2> cam_1_;
  gtsam::PinholeCamera<gtsam::Cal3_S2> cam_2_;
  VioParams vio_params_;
  OpticalFlowPredictor::UniquePtr optical_flow_predictor_;
};

TEST_F(OpticalFlowPredictorFixture, StaticOpticalFlowPrediction) {
  optical_flow_predictor_ =
      buildOpticalFlowPredictor(OpticalFlowPredictorType::kNoPrediction);

  Point3 point3d = sfnew->keypoints_3d_.at(i);
  gtsam::PinholeCamera<Cal3_S2> leftCam_undistRect(gtsam::Pose3(), KundistRect);
  Point2 p2_undistRect = leftCam_undistRect.project(point3d);
  KeypointsCV prev_kps;
  prev_kps.push_back(KeypointCV(1.0, 3.2));
  gtsam::Rot3 inter_frame_rot =
      gtsam::Rot3::AxisAngle(gtsam::Unit3(1.0, 1.0, 1.0), 2.0);
  optical_flow_predictor_->predictFlow(prev_kps, inter_frame_rot, next_kps);
}

TEST_F(OpticalFlowPredictorFixture, RotationalOpticalFlowPrediction) {
  optical_flow_predictor_ =
      buildOpticalFlowPredictor(OpticalFlowPredictorType::kRotational);
  KeypointsCV prev_kps;
  // u, v
  prev_kps << 1, 2;
  //
  gtsam::Rot3 inter_frame_pose;
  optical_flow_predictor_->predictFlow(prev_kps, inter_frame_pose, next_kps);
  // Check next_kps = H * prev_kps
  EXPECT_EQ(actual_next_kps, expected_next_kps);
}

TEST_F(OpticalFlowPredictorFixture, SimulatedRotationOnly) {
  optical_flow_predictor_ =
      buildOpticalFlowPredictor(OpticalFlowPredictorType::kRotational);
  // Create two cameras at same location with diff rotations + X landmarks

  // Project X landmarks in both cameras

  // Set prev_kps to projection on camera 1

  // Call predictor to get next_kps

  // Expect equal wrt the projection of X landmarks on cam 2

  // And the other way cam 2 to cam 1
}

TEST_F(OpticalFlowPredictorFixture, EurocRotationOnly) {
  // Calibration params from Euroc

  // Create two cameras + X landmarks

  // Project X landmarks in both cameras

  // Set prev_kps to projection on camera 1

  // Call predictor to get next_kps

  // Expect equal wrt the projection of X landmarks on cam 2

  // And the other way cam 2 to cam 1
}

TEST_F(OpticalFlowPredictorFixture, SimulatedRotationTranslation) {
  // Create two cameras at diff rot + trans (just a bit) + X landmarks

  // Project X landmarks in both cameras

  // Set prev_kps to projection on camera 1

  // Call predictor to get next_kps

  // Expect equal wrt the projection of X landmarks on cam 2
  // Add extra tolerance bcs of translation

  // And the other way cam 2 to cam 1
}

TEST_F(OpticalFlowPredictorFixture, EurocRotationTranslation) {
  // Create two cameras at diff rot + trans (just a bit) + X landmarks

  // Project X landmarks in both cameras

  // Set prev_kps to projection on camera 1

  // Call predictor to get next_kps

  // Expect equal wrt the projection of X landmarks on cam 2
  // Add extra tolerance bcs of translation

  // And the other way cam 2 to cam 1
}

}  // namespace VIO
