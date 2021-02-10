/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testLoopClosureDetector.cpp
 * @brief  test LoopClosureDetector
 * @author Marcus Abate, Luca Carlone
 */

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/loopclosure/LoopClosureDetector.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

DECLARE_string(test_data_path);
DECLARE_string(vocabulary_path);

namespace VIO {

class LCDFixture : public ::testing::Test {
 protected:
  // Tolerances
  const double tol = 1e-7;
  const double rot_tol_mono = 0.05;  // radians
  const double tran_tol_mono =
      0.1;  // meters (error rescaled using ground-truth)
  const double rot_tol_stereo = 0.3;   // radians
  const double tran_tol_stereo = 0.5;  // meters

 public:
  LCDFixture()
      : lcd_test_data_path_(FLAGS_test_data_path +
                            std::string("/ForLoopClosureDetector")),
        frontend_params_(),
        stereo_camera_(nullptr),
        stereo_matcher_(nullptr),
        lcd_detector_(nullptr),
        match1_T_query1_(),
        match2_T_query2_(),
        match1_stereo_frame_(nullptr),
        match2_stereo_frame_(nullptr),
        query1_stereo_frame_(nullptr),
        query2_stereo_frame_(nullptr),
        id_match1_(0),
        id_query1_(1),
        id_match2_(2),
        id_query2_(3),
        timestamp_match1_(1000),
        timestamp_query1_(2000),
        timestamp_match2_(3000),
        timestamp_query2_(4000) {
    // First set value of vocabulary path for LoopClosureDetector
    FLAGS_vocabulary_path = "../vocabulary/ORBvoc.yml";
    frontend_params_.parseYAML(lcd_test_data_path_ + "/FrontendParams.yaml");

    // Initialize CameraParams for both frames
    cam_params_left_.parseYAML(lcd_test_data_path_ + "/sensorLeft.yaml");
    cam_params_right_.parseYAML(lcd_test_data_path_ + "/sensorRight.yaml");

    // Initialize stereo camera and matcher
    stereo_camera_ =
        std::make_shared<StereoCamera>(cam_params_left_, cam_params_right_);
    CHECK(stereo_camera_);
    stereo_matcher_ = VIO::make_unique<StereoMatcher>(
        stereo_camera_, frontend_params_.stereo_matching_params_);
    CHECK(stereo_matcher_);

    LoopClosureDetectorParams params;
    params.parseYAML(lcd_test_data_path_ + "/testLCDParameters.yaml");

    FLAGS_vocabulary_path =
        FLAGS_test_data_path +
        std::string("/ForLoopClosureDetector/small_voc.yml.gz");

    lcd_detector_ = VIO::make_unique<LoopClosureDetector>(
        params, 
        stereo_camera_, 
        frontend_params_.stereo_matching_params_, 
        false);

    // Euroc V1_01_easy ts: 1403715386762142976
    world_T_match1_ = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.338337, 0.608466, -0.535476, 0.478082)),
        gtsam::Point3(1.573832, 2.023348, 1.738755));

    // Euroc V1_01_easy ts: 1403715288312143104
    world_T_query1_ = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.478634, 0.415595, -0.700197, 0.328505)),
        gtsam::Point3(1.872115, 1.786064, 1.586159));

    // Euroc V1_01_easy ts: 1403715400762142976
    world_T_match2_ = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.3394, -0.672895, -0.492724, -0.435018)),
        gtsam::Point3(-0.662997, -0.495046, 1.347300));

    // Euroc V1_01_easy ts: 1403715400262142976
    world_T_query2_ = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.39266, -0.590667, -0.58023, -0.400326)),
        gtsam::Point3(-0.345638, -0.501712, 1.320441));

    match1_T_query1_ = world_T_match1_.between(world_T_query1_);
    match2_T_query2_ = world_T_match2_.between(world_T_query2_);

    initializeData();
  }

 protected:
  void initializeData() {
    std::string img_name_match1_left = lcd_test_data_path_ + "/left_img_0.png";
    std::string img_name_match1_right = lcd_test_data_path_ + "/right_img_0.png";

    std::string img_name_query1_left = lcd_test_data_path_ + "/left_img_1.png";
    std::string img_name_query1_right =
        lcd_test_data_path_ + "/right_img_1.png";

    std::string img_name_match2_left = lcd_test_data_path_ + "/left_img_2.png";
    std::string img_name_match2_right =
        lcd_test_data_path_ + "/right_img_2.png";

    std::string img_name_query2_left = lcd_test_data_path_ + "/left_img_3.png";
    std::string img_name_query2_right =
        lcd_test_data_path_ + "/right_img_3.png";

    // Initialize StereoFrame objects for reference and current frames
    FrontendParams tp;
    FeatureDetectorParams fdp;
    FeatureDetector feature_detector(fdp);

    match1_stereo_frame_ = VIO::make_unique<StereoFrame>(
        id_match1_,
        timestamp_match1_,
        Frame(id_match1_,
              timestamp_match1_,
              cam_params_left_,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  img_name_match1_left, 
                  tp.stereo_matching_params_.equalize_image_)),
        Frame(id_match1_,
              timestamp_match1_,
              cam_params_right_,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  img_name_match1_right, 
                  tp.stereo_matching_params_.equalize_image_)));

    feature_detector.featureDetection(
        &match1_stereo_frame_->left_frame_);
    CHECK(match1_stereo_frame_);
    match1_stereo_frame_->setIsKeyframe(true);
    stereo_matcher_->sparseStereoReconstruction(match1_stereo_frame_.get());

    query1_stereo_frame_ = VIO::make_unique<StereoFrame>(
        id_query1_,
        timestamp_query1_,
        Frame(id_query1_,
              timestamp_query1_,
              cam_params_left_,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  img_name_query1_left,
                  tp.stereo_matching_params_.equalize_image_)),
        Frame(id_query1_,
              timestamp_query1_,
              cam_params_right_,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  img_name_query1_right,
                  tp.stereo_matching_params_.equalize_image_)));

    feature_detector.featureDetection(
        &query1_stereo_frame_->left_frame_);
    CHECK(query1_stereo_frame_);
    query1_stereo_frame_->setIsKeyframe(true);
    stereo_matcher_->sparseStereoReconstruction(query1_stereo_frame_.get());

    match2_stereo_frame_ = VIO::make_unique<StereoFrame>(
        id_match2_,
        timestamp_match2_,
        Frame(id_match2_,
              timestamp_match2_,
              cam_params_left_,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_match2_left,
                tp.stereo_matching_params_.equalize_image_)),
        Frame(id_match2_,
              timestamp_match2_,
              cam_params_right_,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  img_name_match2_right,
                  tp.stereo_matching_params_.equalize_image_)));

    feature_detector.featureDetection(
        &match2_stereo_frame_->left_frame_);
    CHECK(match2_stereo_frame_);
    match2_stereo_frame_->setIsKeyframe(true);
    stereo_matcher_->sparseStereoReconstruction(match2_stereo_frame_.get());

    query2_stereo_frame_ = VIO::make_unique<StereoFrame>(
        id_query2_,
        timestamp_query2_,
        Frame(id_query2_,
              timestamp_query2_,
              cam_params_left_,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  img_name_query2_left,
                  tp.stereo_matching_params_.equalize_image_)),
        Frame(id_query2_,
              timestamp_query2_,
              cam_params_right_,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  img_name_query2_right,
                  tp.stereo_matching_params_.equalize_image_)));

    feature_detector.featureDetection(
        &query2_stereo_frame_->left_frame_);
    CHECK(query2_stereo_frame_);
    query2_stereo_frame_->setIsKeyframe(true);
    stereo_matcher_->sparseStereoReconstruction(query2_stereo_frame_.get());
  }

  // Standard gtest methods, unnecessary for now
  virtual void SetUp() {}
  virtual void TearDown() {}

 protected:
  // Data-related members
  std::string lcd_test_data_path_;
  FrontendParams frontend_params_;
  CameraParams cam_params_left_, cam_params_right_;

  // Stereo members
  StereoCamera::ConstPtr stereo_camera_;
  StereoMatcher::UniquePtr stereo_matcher_;

  // LCD members
  LoopClosureDetector::UniquePtr lcd_detector_;

  // Stored frame members
  gtsam::Pose3 world_T_match1_, world_T_match2_, world_T_query1_,
      world_T_query2_;
  gtsam::Pose3 match1_T_query1_, match2_T_query2_;
  std::unique_ptr<StereoFrame> match1_stereo_frame_, query1_stereo_frame_;
  std::unique_ptr<StereoFrame> match2_stereo_frame_, query2_stereo_frame_;
  const FrameId id_match1_;
  const FrameId id_query1_;
  const FrameId id_match2_;
  const FrameId id_query2_;
  const VIO::Timestamp timestamp_match1_;
  const VIO::Timestamp timestamp_query1_;
  const VIO::Timestamp timestamp_match2_;
  const VIO::Timestamp timestamp_query2_;
};  // class LCDFixture

TEST_F(LCDFixture, defaultConstructor) {
  /* Test default constructor to ensure that vocabulary is loaded correctly. */
  CHECK(lcd_detector_);
  EXPECT_GT(lcd_detector_->getBoWDatabase()->getVocabulary()->size(), 0);
}

TEST_F(LCDFixture, rewriteStereoFrameFeatures) {
  /* Test the replacement of StereoFrame keypoints, versors, etc with ORB */
  float keypoint_diameter = 2;
  unsigned int nfeatures = 500;

  CHECK(match1_stereo_frame_);
  StereoFrame stereo_frame = *match1_stereo_frame_;

  std::vector<cv::KeyPoint> keypoints;
  keypoints.reserve(nfeatures);

  for (unsigned int i = 0; i < nfeatures; i++) {
    keypoints.push_back(cv::KeyPoint(5, i, keypoint_diameter));
  }

  lcd_detector_->rewriteStereoFrameFeatures(keypoints, &stereo_frame);

  // TODO(marcus): Don't need mutable frames!

  const Frame& left_frame = stereo_frame.left_frame_;
  const Frame& right_frame = stereo_frame.right_frame_;

  EXPECT_EQ(left_frame.keypoints_.size(), nfeatures);
  EXPECT_EQ(right_frame.keypoints_.size(), nfeatures);
  EXPECT_EQ(left_frame.versors_.size(), nfeatures);
  EXPECT_EQ(left_frame.scores_.size(), nfeatures);

  for (unsigned int i = 0; i < left_frame.keypoints_.size(); i++) {
    EXPECT_EQ(left_frame.keypoints_[i], keypoints[i].pt);
    EXPECT_EQ(left_frame.versors_[i],
              UndistorterRectifier::UndistortKeypointAndGetVersor(keypoints[i].pt, left_frame.cam_param_));
  }

  EXPECT_EQ(stereo_frame.keypoints_3d_.size(), nfeatures);
  EXPECT_EQ(stereo_frame.left_keypoints_rectified_.size(), nfeatures);
  EXPECT_EQ(stereo_frame.right_keypoints_rectified_.size(), nfeatures);
}

TEST_F(LCDFixture, processAndAddFrame) {
  /* Test adding frame to database without BoW Loop CLosure Detection */
  CHECK(lcd_detector_);
  EXPECT_EQ(lcd_detector_->getFrameDatabasePtr()->size(), 0);

  FrameId id_0 = lcd_detector_->processAndAddFrame(*match1_stereo_frame_);

  EXPECT_EQ(id_0, 0);
  EXPECT_EQ(lcd_detector_->getFrameDatabasePtr()->size(), 1);
  EXPECT_EQ(lcd_detector_->getFrameDatabasePtr()->at(0).timestamp_,
            timestamp_match1_);
  EXPECT_EQ(lcd_detector_->getFrameDatabasePtr()->at(0).id_kf_, id_match1_);
  EXPECT_EQ(lcd_detector_->getFrameDatabasePtr()->at(0).keypoints_.size(),
            lcd_detector_->getLCDParams().nfeatures_);

  FrameId id_1 = lcd_detector_->processAndAddFrame(*query1_stereo_frame_);

  EXPECT_EQ(id_1, 1);
  EXPECT_EQ(lcd_detector_->getFrameDatabasePtr()->size(), 2);
  EXPECT_EQ(lcd_detector_->getFrameDatabasePtr()->at(1).timestamp_,
            timestamp_query1_);
  EXPECT_EQ(lcd_detector_->getFrameDatabasePtr()->at(1).id_kf_, id_query1_);
  EXPECT_EQ(lcd_detector_->getFrameDatabasePtr()->at(1).keypoints_.size(),
            lcd_detector_->getLCDParams().nfeatures_);
}

TEST_F(LCDFixture, geometricVerificationCheck) {
  /* Test geometric verification using RANSAC Nister 5pt method */
  CHECK(lcd_detector_);
  CHECK(match1_stereo_frame_);
  CHECK(query1_stereo_frame_);
  lcd_detector_->processAndAddFrame(*match1_stereo_frame_);
  lcd_detector_->processAndAddFrame(*query1_stereo_frame_);

  // Find correspondences between keypoints.
  std::vector<FrameId> i_query, i_match;
  lcd_detector_->computeMatchedIndices(1, 0, &i_query, &i_match, true);

  gtsam::Pose3 camMatch1_T_camQuery1_mono;
  lcd_detector_->geometricVerificationCheck(
      1, 0, &camMatch1_T_camQuery1_mono, &i_query, &i_match);

  // Pose transforms points from query1 to match1 frame.
  gtsam::Pose3 match1_T_query1_ground_truth = match1_T_query1_;

  gtsam::Pose3 bodyMatch1_T_bodyQuery1;
  lcd_detector_->transformCameraPoseToBodyPose(camMatch1_T_camQuery1_mono,
                                               &bodyMatch1_T_bodyQuery1);
  // match1_T_query1_ground_truth.print("Ground truth. \n");
  // bodyMatch1_T_bodyQuery1.print("Estimate. \n");
  // bodyMatch1_T_bodyQuery1.inverse().print("Inverse of estimate. \n");

  std::pair<double, double> error =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(
          match1_T_query1_ground_truth, bodyMatch1_T_bodyQuery1, true);

  EXPECT_LT(error.first, rot_tol_mono);
  EXPECT_LT(error.second, tran_tol_mono);
}

TEST_F(LCDFixture, recoverPoseArun) {
  /* Test proper scaled pose recovery between two identical images */
  CHECK(lcd_detector_);
  lcd_detector_->getLCDParamsMutable()->pose_recovery_option_ =
      PoseRecoveryOption::RANSAC_ARUN;
  gtsam::Pose3 empty_pose;
  std::pair<double, double> error;

  /* Test proper scaled pose recovery between ref and cur images */
  CHECK(match1_stereo_frame_);
  CHECK(query1_stereo_frame_);
  lcd_detector_->processAndAddFrame(*match1_stereo_frame_);
  lcd_detector_->processAndAddFrame(*query1_stereo_frame_);

  // Find correspondences between keypoints.
  std::vector<FrameId> i_match1, i_query1;
  lcd_detector_->computeMatchedIndices(1, 0, &i_query1, &i_match1, true);

  gtsam::Pose3 bodyMatch1_T_bodyQuery1_stereo;
  lcd_detector_->recoverPose(
      1, 0, empty_pose, &bodyMatch1_T_bodyQuery1_stereo, &i_query1, &i_match1);
  error = UtilsOpenCV::ComputeRotationAndTranslationErrors(
      match1_T_query1_, bodyMatch1_T_bodyQuery1_stereo, true);

  EXPECT_LT(error.first, rot_tol_stereo);
  EXPECT_LT(error.second, tran_tol_stereo);

  /* Test pose recovery on other two images */
  CHECK(match2_stereo_frame_);
  CHECK(query2_stereo_frame_);
  lcd_detector_->processAndAddFrame(*match2_stereo_frame_);
  lcd_detector_->processAndAddFrame(*query2_stereo_frame_);

  // Find correspondences between keypoints.
  std::vector<FrameId> i_query2, i_match2;
  lcd_detector_->computeMatchedIndices(3, 2, &i_query2, &i_match2, true);

  gtsam::Pose3 bodyMatch2_T_bodyQuery2_stereo;
  lcd_detector_->recoverPose(
      3, 2, empty_pose, &bodyMatch2_T_bodyQuery2_stereo, &i_query2, &i_match2);

  error = UtilsOpenCV::ComputeRotationAndTranslationErrors(
      match2_T_query2_, bodyMatch2_T_bodyQuery2_stereo, false);

  EXPECT_LT(error.first, rot_tol_stereo * 1.5);
  EXPECT_LT(error.second, tran_tol_stereo * 1.5);
}

TEST_F(LCDFixture, recoverPoseGivenRot) {
  CHECK(lcd_detector_);
  lcd_detector_->getLCDParamsMutable()->pose_recovery_option_ =
      PoseRecoveryOption::GIVEN_ROT;

  gtsam::Pose3 bodyMatch1_T_bodyQuery1_gt, camMatch1_T_camQuery1_gt;
  std::pair<double, double> error;

  /* Test pose recovery given ground truth rotation and unit translation */
  CHECK(match1_stereo_frame_);
  CHECK(query1_stereo_frame_);
  lcd_detector_->processAndAddFrame(*match1_stereo_frame_);
  lcd_detector_->processAndAddFrame(*query1_stereo_frame_);

  bodyMatch1_T_bodyQuery1_gt = match1_T_query1_;
  lcd_detector_->transformBodyPoseToCameraPose(bodyMatch1_T_bodyQuery1_gt,
                                               &camMatch1_T_camQuery1_gt);
  // Fake mono ransac with translation up to scale
  gtsam::Pose3 camMatch1_T_camQuery1_mono_gt =
      gtsam::Pose3(camMatch1_T_camQuery1_gt.rotation(),
                   camMatch1_T_camQuery1_gt.translation() /
                       camMatch1_T_camQuery1_gt.translation().norm());
  // Find correspondences between keypoints.
  std::vector<FrameId> i_query1, i_match1;
  lcd_detector_->computeMatchedIndices(1, 0, &i_query1, &i_match1, true);

  gtsam::Pose3 bodyMatch1_T_bodyQuery1_stereo;
  lcd_detector_->recoverPose(1,
                             0,
                             camMatch1_T_camQuery1_mono_gt,
                             &bodyMatch1_T_bodyQuery1_stereo,
                             &i_query1,
                             &i_match1);

  error = UtilsOpenCV::ComputeRotationAndTranslationErrors(
      match1_T_query1_, bodyMatch1_T_bodyQuery1_stereo, false);
  EXPECT_LT(error.first, rot_tol_stereo);
  EXPECT_LT(error.second, tran_tol_stereo);

  /* Test pose recovery on other two images */
  gtsam::Pose3 bodyMatch2_T_bodyQuery2_gt, camMatch2_T_camQuery2_gt;

  CHECK(match2_stereo_frame_);
  CHECK(query2_stereo_frame_);
  FrameId frm_2 = lcd_detector_->processAndAddFrame(*match2_stereo_frame_);
  FrameId frm_3 = lcd_detector_->processAndAddFrame(*query2_stereo_frame_);

  bodyMatch2_T_bodyQuery2_gt = match2_T_query2_;
  lcd_detector_->transformBodyPoseToCameraPose(bodyMatch2_T_bodyQuery2_gt,
                                               &camMatch2_T_camQuery2_gt);

  // Fake mono ransac with translation up to scale
  gtsam::Pose3 camMatch2_T_camQuery2_mono_gt =
      gtsam::Pose3(camMatch2_T_camQuery2_gt.rotation(),
                   camMatch2_T_camQuery2_gt.translation() /
                       camMatch2_T_camQuery2_gt.translation().norm());

  // Find correspondences between keypoints.
  std::vector<FrameId> i_query2, i_match2;
  lcd_detector_->computeMatchedIndices(3, 2, &i_query2, &i_match2, true);

  gtsam::Pose3 bodyMatch2_T_bodyQuery2_stereo;
  lcd_detector_->recoverPose(3,
                             2,
                             camMatch2_T_camQuery2_gt,
                             &bodyMatch2_T_bodyQuery2_stereo,
                             &i_query2,
                             &i_match2);

  error = UtilsOpenCV::ComputeRotationAndTranslationErrors(
      match2_T_query2_, bodyMatch2_T_bodyQuery2_stereo, false);
  EXPECT_LT(error.first, rot_tol_stereo);
  EXPECT_LT(error.second, tran_tol_stereo);
}

TEST_F(LCDFixture, detectLoop) {
  std::pair<double, double> error;

  CHECK(lcd_detector_);
  lcd_detector_->getLCDParamsMutable()->pose_recovery_option_ =
      PoseRecoveryOption::GIVEN_ROT;

  /* Test the detectLoop method against two images without closure */
  LoopResult loop_result_0, loop_result_1, loop_result_2, loop_result_3;

  CHECK(match1_stereo_frame_);
  CHECK(match2_stereo_frame_);
  CHECK(query1_stereo_frame_);
  lcd_detector_->detectLoop(*match2_stereo_frame_, &loop_result_0);
  EXPECT_EQ(loop_result_0.isLoop(), false);

  /* Test the detectLoop method against two images that are identical */
  lcd_detector_->detectLoop(*match2_stereo_frame_, &loop_result_2);
  EXPECT_EQ(loop_result_2.isLoop(), true);
  EXPECT_EQ(loop_result_2.query_id_, 1);
  EXPECT_EQ(loop_result_2.match_id_, 0);
  EXPECT_TRUE(loop_result_2.relative_pose_.equals(
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0)), tol));

  lcd_detector_->detectLoop(*match1_stereo_frame_, &loop_result_1);
  EXPECT_EQ(loop_result_1.isLoop(), false);

  /* Test the detectLoop method against two unidentical, similar images */
  lcd_detector_->detectLoop(*query1_stereo_frame_, &loop_result_3);
  EXPECT_EQ(loop_result_3.isLoop(), true);
  EXPECT_EQ(loop_result_3.match_id_, 2);
  EXPECT_EQ(loop_result_3.query_id_, 3);

  error = UtilsOpenCV::ComputeRotationAndTranslationErrors(
      match1_T_query1_, loop_result_3.relative_pose_, false);

  EXPECT_LT(error.first, rot_tol_stereo);
  EXPECT_LT(error.second, tran_tol_stereo);
}

TEST_F(LCDFixture, addOdometryFactorAndOptimize) {
  /* Test the addition of odometry factors to the PGO */
  CHECK(lcd_detector_);
  lcd_detector_->initializePGO(OdometryFactor(
      0, gtsam::Pose3::identity(), gtsam::noiseModel::Isotropic::Variance(6, 0.1)));

  OdometryFactor odom_factor(
      1, world_T_match1_, gtsam::noiseModel::Isotropic::Variance(6, 0.1));
  lcd_detector_->addOdometryFactorAndOptimize(odom_factor);

  gtsam::Values pgo_trajectory = lcd_detector_->getPGOTrajectory();
  gtsam::NonlinearFactorGraph pgo_nfg = lcd_detector_->getPGOnfg();

  EXPECT_EQ(pgo_trajectory.size(), 2);
  EXPECT_EQ(pgo_nfg.size(), 2);
}

TEST_F(LCDFixture, addLoopClosureFactorAndOptimize) {
  /* Test the addition of odometry and loop closure factors to the PGO */
  CHECK(lcd_detector_);
  OdometryFactor odom_factor_1(
      0, world_T_match1_, gtsam::noiseModel::Isotropic::Variance(6, 0.1));
  lcd_detector_->initializePGO(odom_factor_1);

  OdometryFactor odom_factor_2(
      1, world_T_query1_, gtsam::noiseModel::Isotropic::Variance(6, 0.1));
  lcd_detector_->addOdometryFactorAndOptimize(odom_factor_2);

  LoopClosureFactor lc_factor_1_2(
      0, 1, match1_T_query1_, gtsam::noiseModel::Isotropic::Variance(6, 0.1));
  lcd_detector_->addLoopClosureFactorAndOptimize(lc_factor_1_2);

  gtsam::Values pgo_trajectory = lcd_detector_->getPGOTrajectory();
  gtsam::NonlinearFactorGraph pgo_nfg = lcd_detector_->getPGOnfg();

  EXPECT_EQ(pgo_trajectory.size(), 2);
  EXPECT_EQ(pgo_nfg.size(), 3);
}

TEST_F(LCDFixture, spinOnce) {
  /* Test the full pipeline with one loop closure and full PGO optimization */
  CHECK(lcd_detector_);
  CHECK(match1_stereo_frame_);
  StereoFrontendOutput::Ptr stereo_frontend_output =
      std::make_shared<StereoFrontendOutput>(match1_stereo_frame_->isKeyframe(),
                                             StatusStereoMeasurementsPtr(),
                                             TrackingStatus(),
                                             gtsam::Pose3::identity(),
                                             gtsam::Pose3::identity(),
                                             gtsam::Pose3::identity(),
                                             *match1_stereo_frame_,
                                             ImuFrontend::PimPtr(),
                                             ImuAccGyrS(),
                                             cv::Mat(),
                                             DebugTrackerInfo());
  FrontendOutputPacketBase::Ptr frontend_output_match1 =
      VIO::safeCast<StereoFrontendOutput, FrontendOutputPacketBase>(
          stereo_frontend_output);
  LcdOutput::Ptr output_0 =
      lcd_detector_->spinOnce(LcdInput(timestamp_match1_,
                                       frontend_output_match1,
                                       FrameId(0),
                                       gtsam::Pose3()));

  CHECK(match2_stereo_frame_);
  stereo_frontend_output =
      std::make_shared<StereoFrontendOutput>(match2_stereo_frame_->isKeyframe(),
                                             StatusStereoMeasurementsPtr(),
                                             TrackingStatus(),
                                             gtsam::Pose3::identity(),
                                             gtsam::Pose3::identity(),
                                             gtsam::Pose3::identity(),
                                             *match2_stereo_frame_,
                                             ImuFrontend::PimPtr(),
                                             ImuAccGyrS(),
                                             cv::Mat(),
                                             DebugTrackerInfo());
  FrontendOutputPacketBase::Ptr frontend_output_match2 =
      VIO::safeCast<StereoFrontendOutput, FrontendOutputPacketBase>(
          stereo_frontend_output);
  LcdOutput::Ptr output_1 =
      lcd_detector_->spinOnce(LcdInput(timestamp_match2_,
                                       frontend_output_match2,
                                       FrameId(1),
                                       gtsam::Pose3()));

  CHECK(query1_stereo_frame_);
  stereo_frontend_output =
      std::make_shared<StereoFrontendOutput>(query1_stereo_frame_->isKeyframe(),
                                             StatusStereoMeasurementsPtr(),
                                             TrackingStatus(),
                                             gtsam::Pose3::identity(),
                                             gtsam::Pose3::identity(),
                                             gtsam::Pose3::identity(),
                                             *query1_stereo_frame_,
                                             ImuFrontend::PimPtr(),
                                             ImuAccGyrS(),
                                             cv::Mat(),
                                             DebugTrackerInfo());
  FrontendOutputPacketBase::Ptr frontend_output_query1 =
      VIO::safeCast<StereoFrontendOutput, FrontendOutputPacketBase>(
          stereo_frontend_output);
  LcdOutput::Ptr output_2 =
      lcd_detector_->spinOnce(LcdInput(timestamp_query1_,
                                       frontend_output_query1,
                                       FrameId(2),
                                       gtsam::Pose3()));

  EXPECT_EQ(output_0->is_loop_closure_, false);
  EXPECT_EQ(output_0->timestamp_, timestamp_match1_);
  EXPECT_EQ(output_0->timestamp_query_, 0);
  EXPECT_EQ(output_0->timestamp_match_, 0);
  EXPECT_EQ(output_0->id_match_, 0);
  EXPECT_EQ(output_0->id_recent_, 0);
  EXPECT_EQ(output_0->states_.size(), 1);
  EXPECT_EQ(output_0->nfg_.size(), 1);

  EXPECT_EQ(output_1->is_loop_closure_, false);
  EXPECT_EQ(output_1->timestamp_, timestamp_match2_);
  EXPECT_EQ(output_1->timestamp_query_, 0);
  EXPECT_EQ(output_1->timestamp_match_, 0);
  EXPECT_EQ(output_1->id_match_, 0);
  EXPECT_EQ(output_1->id_recent_, 0);
  EXPECT_EQ(output_1->states_.size(), 2);
  EXPECT_EQ(output_1->nfg_.size(), 2);

  EXPECT_EQ(output_2->is_loop_closure_, true);
  EXPECT_EQ(output_2->timestamp_, timestamp_query1_);
  EXPECT_EQ(output_2->timestamp_query_, timestamp_query1_);
  EXPECT_EQ(output_2->timestamp_match_, timestamp_match1_);
  EXPECT_EQ(output_2->id_match_, 0);
  EXPECT_EQ(output_2->id_recent_, 2);
  EXPECT_EQ(output_2->states_.size(), 3);
}

}  // namespace VIO
