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

#include <DBoW2/DBoW2.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/frontend/VisionImuFrontend.h"
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
  const double loose_tol = 1e-5;
  const double rot_tol_mono = 0.055;  // radians
  const double tran_tol_mono =
      0.11;  // meters (error rescaled using ground-truth)
  const double rot_tol_stereo = 0.3;  // radians
  const double tran_tol_stereo =
      0.6;  // meters TODO see why 0.5 will not pass TEST_F(LCDFixture,
            // recoverPoseBodyPnpMono)

 public:
  LCDFixture()
      : lcd_test_data_path_(FLAGS_test_data_path +
                            std::string("/ForLoopClosureDetector")),
        frontend_params_(),
        stereo_camera_(nullptr),
        stereo_matcher_(nullptr),
        lcd_detector_(nullptr),
        bodyMatch1_T_bodyQuery1_gt_(),
        bodyMatch2_T_bodyQuery2_gt_(),
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
        timestamp_query2_(4000),
        is_backend_queue_filled_(false) {
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
    stereo_matcher_ = std::make_unique<StereoMatcher>(
        stereo_camera_, frontend_params_.stereo_matching_params_);
    CHECK(stereo_matcher_);

    lcd_params_.parseYAML(lcd_test_data_path_ + "/testLCDParameters.yaml");

    FLAGS_vocabulary_path =
        FLAGS_test_data_path +
        std::string("/ForLoopClosureDetector/small_voc.yml.gz");

    lcd_detector_ = std::make_unique<LoopClosureDetector>(
        lcd_params_,
        stereo_camera_->getLeftCamParams(),
        stereo_camera_->getBodyPoseLeftCamRect(),
        stereo_camera_,
        frontend_params_.stereo_matching_params_,
        std::nullopt,
        false);

    lcd_detector_->registerIsBackendQueueFilledCallback(
        std::bind(&LCDFixture::lcdInputQueueCb, this));

    // Euroc V1_01_easy ts: 1403715386762142976
    world_T_bodyMatch1_ = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.338337, 0.608466, -0.535476, 0.478082)),
        gtsam::Point3(1.573832, 2.023348, 1.738755));

    // Euroc V1_01_easy ts: 1403715288312143104
    world_T_bodyQuery1_ = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.478634, 0.415595, -0.700197, 0.328505)),
        gtsam::Point3(1.872115, 1.786064, 1.586159));

    // Euroc V1_01_easy ts: 1403715400762142976
    world_T_bodyMatch2_ = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.3394, -0.672895, -0.492724, -0.435018)),
        gtsam::Point3(-0.662997, -0.495046, 1.347300));

    // Euroc V1_01_easy ts: 1403715400262142976
    world_T_bodyQuery2_ = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.39266, -0.590667, -0.58023, -0.400326)),
        gtsam::Point3(-0.345638, -0.501712, 1.320441));

    bodyMatch1_T_bodyQuery1_gt_ =
        world_T_bodyMatch1_.between(world_T_bodyQuery1_);
    bodyMatch2_T_bodyQuery2_gt_ =
        world_T_bodyMatch2_.between(world_T_bodyQuery2_);

    // Initialize StereoFrame objects for reference and current frames
    feature_detector_ =
        std::make_unique<FeatureDetector>(feature_detector_params_);

    initializeData();
  }

 protected:
  StereoFrame::UniquePtr generateStereoFrameWithStereoRecons(
      const FrameId& frame_id,
      const Timestamp& timestamp,
      const std::string& left_img_name,
      const std::string& right_img_name) const {
    CHECK(feature_detector_);
    CHECK(stereo_camera_);
    CHECK(stereo_matcher_);

    StereoFrame::UniquePtr stereo_frame = std::make_unique<StereoFrame>(
        frame_id,
        timestamp,
        Frame(frame_id,
              timestamp,
              cam_params_left_,
              UtilsOpenCV::ReadAndConvertToGrayScale(left_img_name)),
        Frame(frame_id,
              timestamp,
              cam_params_right_,
              UtilsOpenCV::ReadAndConvertToGrayScale(right_img_name)));

    feature_detector_->featureDetection(&stereo_frame->left_frame_);
    stereo_frame->setIsKeyframe(true);
    stereo_matcher_->sparseStereoReconstruction(stereo_frame.get());
    stereo_frame->checkStereoFrame();

    // Also copy the rectified keypoints to the frame level for mono
    stereo_frame->left_frame_.keypoints_undistorted_ =
        stereo_frame->left_keypoints_rectified_;
    stereo_frame->right_frame_.keypoints_undistorted_ =
        stereo_frame->right_keypoints_rectified_;

    return stereo_frame;
  }

  void initializeData() {
    std::string img_name_match1_left = lcd_test_data_path_ + "/left_img_0.png";
    std::string img_name_match1_right =
        lcd_test_data_path_ + "/right_img_0.png";

    std::string img_name_query1_left = lcd_test_data_path_ + "/left_img_1.png";
    std::string img_name_query1_right =
        lcd_test_data_path_ + "/right_img_1.png";

    std::string img_name_match2_left = lcd_test_data_path_ + "/left_img_2.png";
    std::string img_name_match2_right =
        lcd_test_data_path_ + "/right_img_2.png";

    std::string img_name_query2_left = lcd_test_data_path_ + "/left_img_3.png";
    std::string img_name_query2_right =
        lcd_test_data_path_ + "/right_img_3.png";

    match1_stereo_frame_ =
        generateStereoFrameWithStereoRecons(id_match1_,
                                            timestamp_match1_,
                                            img_name_match1_left,
                                            img_name_match1_right);
    query1_stereo_frame_ =
        generateStereoFrameWithStereoRecons(id_query1_,
                                            timestamp_query1_,
                                            img_name_query1_left,
                                            img_name_query1_right);
    match2_stereo_frame_ =
        generateStereoFrameWithStereoRecons(id_match2_,
                                            timestamp_match2_,
                                            img_name_match2_left,
                                            img_name_match2_right);
    query2_stereo_frame_ =
        generateStereoFrameWithStereoRecons(id_query2_,
                                            timestamp_query2_,
                                            img_name_query2_left,
                                            img_name_query2_right);

    // Store points with IDs in the world frame for each stereo frame
    const gtsam::Pose3& body_T_cam = stereo_camera_->getBodyPoseLeftCamRect();

    const Landmarks& match1Cam_lmks3d = match1_stereo_frame_->keypoints_3d_;
    CHECK_EQ(match1Cam_lmks3d.size(),
             match1_stereo_frame_->left_frame_.landmarks_.size());
    for (size_t i = 0; i < match1Cam_lmks3d.size(); i++) {
      // world_T_bodyMatch1_ is the VIO estimate of body wrt world
      Landmark W_lmk = world_T_bodyMatch1_ * body_T_cam * match1Cam_lmks3d[i];
      W_match1_lmks3d_.insert(std::make_pair(
          match1_stereo_frame_->left_frame_.landmarks_[i], W_lmk));
    }

    const Landmarks& query1Cam_lmks3d = query1_stereo_frame_->keypoints_3d_;
    CHECK_EQ(query1Cam_lmks3d.size(),
             query1_stereo_frame_->left_frame_.landmarks_.size());
    for (size_t i = 0; i < query1Cam_lmks3d.size(); i++) {
      // world_T_bodyMatch1_ is the VIO estimate of body wrt world
      Landmark W_lmk = world_T_bodyQuery1_ * body_T_cam * query1Cam_lmks3d[i];
      W_query1_lmks3d_.insert(std::make_pair(
          query1_stereo_frame_->left_frame_.landmarks_[i], W_lmk));
    }

    const Landmarks& match2Cam_lmks3d = match2_stereo_frame_->keypoints_3d_;
    CHECK_EQ(match2Cam_lmks3d.size(),
             match2_stereo_frame_->left_frame_.landmarks_.size());
    for (size_t i = 0; i < match2Cam_lmks3d.size(); i++) {
      // world_T_bodyMatch1_ is the VIO estimate of body wrt world
      Landmark W_lmk = world_T_bodyMatch2_ * body_T_cam * match2Cam_lmks3d[i];
      W_match2_lmks3d_.insert(std::make_pair(
          match2_stereo_frame_->left_frame_.landmarks_[i], W_lmk));
    }

    const Landmarks& query2Cam_lmks3d = query2_stereo_frame_->keypoints_3d_;
    CHECK_EQ(query2Cam_lmks3d.size(),
             query2_stereo_frame_->left_frame_.landmarks_.size());
    for (size_t i = 0; i < query2Cam_lmks3d.size(); i++) {
      // world_T_bodyMatch1_ is the VIO estimate of body wrt world
      Landmark W_lmk = world_T_bodyQuery2_ * body_T_cam * query2Cam_lmks3d[i];
      W_query2_lmks3d_.insert(std::make_pair(
          query2_stereo_frame_->left_frame_.landmarks_[i], W_lmk));
    }
  }

 public:
  // Just to prevent a CHECK from failing.
  bool lcdInputQueueCb() { return is_backend_queue_filled_; }

  // Standard gtest methods, unnecessary for now
  virtual void SetUp() {}
  virtual void TearDown() {}

 protected:
  // Data-related members
  std::string lcd_test_data_path_;
  FrontendParams frontend_params_;
  FeatureDetectorParams feature_detector_params_;
  CameraParams cam_params_left_, cam_params_right_;
  LoopClosureDetectorParams lcd_params_;

  FeatureDetector::UniquePtr feature_detector_;

  // Stereo members
  StereoCamera::ConstPtr stereo_camera_;
  StereoMatcher::UniquePtr stereo_matcher_;

  // LCD members
  LoopClosureDetector::UniquePtr lcd_detector_;

  // Stored frame members
  gtsam::Pose3 world_T_bodyMatch1_, world_T_bodyMatch2_, world_T_bodyQuery1_,
      world_T_bodyQuery2_, world_T_match3_, world_T_query3_;
  gtsam::Pose3 bodyMatch1_T_bodyQuery1_gt_, bodyMatch2_T_bodyQuery2_gt_,
      match3_T_query3_;
  PointsWithIdMap W_match1_lmks3d_, W_query1_lmks3d_;
  PointsWithIdMap W_match2_lmks3d_, W_query2_lmks3d_;
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

  bool is_backend_queue_filled_;
};  // class LCDFixture

TEST_F(LCDFixture, defaultConstructor) {
  /* Test default constructor to ensure that vocabulary is loaded correctly. */
  CHECK(lcd_detector_);
  EXPECT_GT(lcd_detector_->getBoWDatabase()->getVocabulary()->size(), 0);
}

TEST_F(LCDFixture, monoConstructor) {
  /* Test default constructor when in mono mode. */
  LoopClosureDetector::UniquePtr lcd = std::make_unique<LoopClosureDetector>(
      lcd_params_,
      stereo_camera_->getLeftCamParams(),
      stereo_camera_->getBodyPoseLeftCamRect(),
      std::nullopt,
      std::nullopt,
      std::nullopt,
      false);
  EXPECT_GT(lcd_detector_->getBoWDatabase()->getVocabulary()->size(), 0);
}

TEST_F(LCDFixture, computeDescriptorMatches) {
  /* Test descriptor matching for two LCDFrames */
  CHECK(match1_stereo_frame_);
  CHECK(query1_stereo_frame_);
  const Frame& match1_frame = match1_stereo_frame_->left_frame_;
  const Frame& query1_frame = query1_stereo_frame_->left_frame_;

  cv::Ptr<cv::ORB> orb_feature_detector_ = cv::ORB::create(500);
  OrbDescriptor match1_descriptors, query1_descriptors;
  std::vector<cv::KeyPoint> match1_keypoints, query1_keypoints;

  orb_feature_detector_->detectAndCompute(
      match1_frame.img_, cv::Mat(), match1_keypoints, match1_descriptors);
  orb_feature_detector_->detectAndCompute(
      query1_frame.img_, cv::Mat(), query1_keypoints, query1_descriptors);

  // Check that descriptor matching between two identical descriptor sets works:
  KeypointMatches matches;
  lcd_detector_->computeDescriptorMatches(
      match1_descriptors, match1_descriptors, &matches, false);
  size_t num_kpts;
  num_kpts = match1_keypoints.size();
  EXPECT_EQ(matches.size(), num_kpts);
  for (const auto& match : matches) {
    EXPECT_EQ(match.first, match.second);
  }

  // Check descriptor matching between different descriptor sets:
  matches = KeypointMatches();
  lcd_detector_->computeDescriptorMatches(
      match1_descriptors, query1_descriptors, &matches, false);
  EXPECT_LE(matches.size(),
            num_kpts);  // not all pairs will be matched for real data
}

TEST_F(LCDFixture, rewriteStereoFrameFeatures) {
  /* Test the replacement of StereoFrame keypoints, versors, etc with ORB */
  unsigned int nfeatures = 200;

  CHECK(match1_stereo_frame_);
  StereoFrame stereo_frame = *match1_stereo_frame_;

  std::vector<cv::KeyPoint> keypoints;
  keypoints.reserve(nfeatures);

  for (unsigned int i = 0; i < nfeatures; i++) {
    keypoints.push_back(cv::KeyPoint(cv::Point2f(300, (i + 1) * 2), 2.0));
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
              UndistorterRectifier::GetBearingVector(keypoints[i].pt,
                                                     left_frame.cam_param_));
  }

  EXPECT_EQ(stereo_frame.keypoints_3d_.size(), nfeatures);
  EXPECT_EQ(stereo_frame.left_keypoints_rectified_.size(), nfeatures);
  EXPECT_EQ(stereo_frame.right_keypoints_rectified_.size(), nfeatures);
}

TEST_F(LCDFixture, processAndAddMonoFrame) {
  /* Test adding frame to database without BoW Loop CLosure Detection */
  lcd_params_.pose_recovery_type_ = PoseRecoveryType::kPnP;
  lcd_detector_ = std::make_unique<LoopClosureDetector>(
      lcd_params_,
      stereo_camera_->getLeftCamParams(),
      stereo_camera_->getBodyPoseLeftCamRect(),
      stereo_camera_,
      frontend_params_.stereo_matching_params_,
      std::nullopt,
      false);

  const auto& cache = lcd_detector_->getFrameCache();
  CHECK(lcd_detector_);
  EXPECT_EQ(cache.size(), 0u);

  FrameId id_0 = lcd_detector_->processAndAddMonoFrame(
      match1_stereo_frame_->left_frame_, W_match1_lmks3d_, world_T_bodyMatch1_);

  ASSERT_EQ(cache.size(), 1u);
  EXPECT_EQ(id_0, 0);
  EXPECT_EQ(cache.getFrame(0)->timestamp_, timestamp_match1_);
  EXPECT_EQ(cache.getFrame(0)->id_kf_, id_match1_);
  size_t nr_kpts = cache.getFrame(0)->keypoints_.size();
  EXPECT_EQ(cache.getFrame(0)->keypoints_3d_.size(), nr_kpts);

  FrameId id_1 = lcd_detector_->processAndAddMonoFrame(
      query1_stereo_frame_->left_frame_, W_query1_lmks3d_, world_T_bodyQuery1_);

  EXPECT_EQ(query1_stereo_frame_->left_frame_.keypoints_.size(),
            W_query1_lmks3d_.size());

  ASSERT_EQ(cache.size(), 2u);
  EXPECT_EQ(id_1, 1);
  EXPECT_EQ(cache.getFrame(1)->timestamp_, timestamp_query1_);
  EXPECT_EQ(cache.getFrame(1)->id_kf_, id_query1_);
  nr_kpts = cache.getFrame(1)->keypoints_.size();
  EXPECT_EQ(cache.getFrame(1)->keypoints_3d_.size(), nr_kpts);
}

TEST_F(LCDFixture, processAndAddStereoFrame) {
  /* Test adding frame to database without BoW Loop CLosure Detection */
  CHECK(lcd_detector_);
  const auto& cache = lcd_detector_->getFrameCache();
  EXPECT_EQ(cache.size(), 0);

  FrameId id_0 = lcd_detector_->processAndAddStereoFrame(*match1_stereo_frame_);
  EXPECT_EQ(id_0, 0);
  ASSERT_EQ(cache.size(), 1u);

  auto stereo_lcd_frame =
      std::dynamic_pointer_cast<StereoLCDFrame>(cache.getFrame(0));
  ASSERT_TRUE(stereo_lcd_frame != nullptr);
  EXPECT_EQ(stereo_lcd_frame->timestamp_, timestamp_match1_);
  EXPECT_EQ(stereo_lcd_frame->id_kf_, id_match1_);
  EXPECT_EQ(stereo_lcd_frame->keypoints_.size(), lcd_params_.nfeatures_);
  EXPECT_EQ(stereo_lcd_frame->keypoints_3d_.size(), lcd_params_.nfeatures_);
  EXPECT_EQ(stereo_lcd_frame->descriptors_vec_.size(), lcd_params_.nfeatures_);
  EXPECT_EQ(stereo_lcd_frame->descriptors_mat_.size().height,
            lcd_params_.nfeatures_);
  EXPECT_EQ(stereo_lcd_frame->bearing_vectors_.size(), lcd_params_.nfeatures_);
  EXPECT_EQ(stereo_lcd_frame->left_keypoints_rectified_.size(),
            lcd_params_.nfeatures_);
  EXPECT_EQ(stereo_lcd_frame->right_keypoints_rectified_.size(),
            lcd_params_.nfeatures_);

  FrameId id_1 = lcd_detector_->processAndAddStereoFrame(*query1_stereo_frame_);
  EXPECT_EQ(id_1, 1);
  ASSERT_EQ(cache.size(), 2u);

  stereo_lcd_frame =
      std::dynamic_pointer_cast<StereoLCDFrame>(cache.getFrame(1));
  ASSERT_TRUE(stereo_lcd_frame != nullptr);
  EXPECT_EQ(stereo_lcd_frame->timestamp_, timestamp_query1_);
  EXPECT_EQ(stereo_lcd_frame->id_kf_, id_query1_);
  EXPECT_EQ(stereo_lcd_frame->keypoints_.size(), lcd_params_.nfeatures_);
  EXPECT_EQ(stereo_lcd_frame->keypoints_3d_.size(), lcd_params_.nfeatures_);
  EXPECT_EQ(stereo_lcd_frame->descriptors_vec_.size(), lcd_params_.nfeatures_);
  EXPECT_EQ(stereo_lcd_frame->descriptors_mat_.size().height,
            lcd_params_.nfeatures_);
  EXPECT_EQ(stereo_lcd_frame->bearing_vectors_.size(), lcd_params_.nfeatures_);
  EXPECT_EQ(stereo_lcd_frame->left_keypoints_rectified_.size(),
            lcd_params_.nfeatures_);
  EXPECT_EQ(stereo_lcd_frame->right_keypoints_rectified_.size(),
            lcd_params_.nfeatures_);
}

TEST_F(LCDFixture, geometricVerificationCam2d2d) {
  lcd_params_.tracker_params_.pose_2d2d_algorithm_ = Pose2d2dAlgorithm::NISTER;
  lcd_detector_ = std::make_unique<LoopClosureDetector>(
      lcd_params_,
      stereo_camera_->getLeftCamParams(),
      stereo_camera_->getBodyPoseLeftCamRect(),
      stereo_camera_,
      frontend_params_.stereo_matching_params_,
      std::nullopt,
      false);

  /* Test geometric verification using RANSAC Nister 5pt method */
  CHECK(lcd_detector_);
  const auto& cache = lcd_detector_->getFrameCache();

  CHECK(match1_stereo_frame_);
  CHECK(query1_stereo_frame_);
  lcd_detector_->processAndAddStereoFrame(*match1_stereo_frame_);
  lcd_detector_->processAndAddStereoFrame(*query1_stereo_frame_);

  ASSERT_EQ(cache.size(), 2u);
  auto lcd_frame_0 = cache.getFrame(0);
  ASSERT_TRUE(lcd_frame_0);
  auto lcd_frame_1 = cache.getFrame(1);
  ASSERT_TRUE(lcd_frame_1);

  // Find correspondences between keypoints.
  KeypointMatches matches;
  lcd_detector_->computeDescriptorMatches(lcd_frame_0->descriptors_mat_,
                                          lcd_frame_1->descriptors_mat_,
                                          &matches,
                                          true);

  gtsam::Pose3 camMatch_T_camQuery;
  std::vector<int> inliers;
  lcd_detector_->geometricVerificationCam2d2d(
      *lcd_frame_0, *lcd_frame_1, matches, &camMatch_T_camQuery, &inliers);

  // Pose transforms points from query1 to match1 frame.
  gtsam::Pose3 gt = bodyMatch1_T_bodyQuery1_gt_;
  gtsam::Pose3 bodyMatch_T_bodyQuery;
  lcd_detector_->transformCameraPoseToBodyPose(camMatch_T_camQuery,
                                               &bodyMatch_T_bodyQuery);
  // gt.print("Ground truth. \n");
  // bodyMatch_T_bodyQuery.print("Estimate. \n");
  // bodyMatch_T_bodyQuery.inverse().print("Inverse of estimate. \n");

  std::pair<double, double> error =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(
          gt, bodyMatch_T_bodyQuery, true);

  EXPECT_LT(error.first, rot_tol_mono);
  EXPECT_LT(error.second, tran_tol_mono);
}

TEST_F(LCDFixture, recoverPoseBodyArun) {
  CHECK(lcd_detector_);
  lcd_params_.tracker_params_.ransac_use_1point_stereo_ = false;
  lcd_detector_ = std::make_unique<LoopClosureDetector>(
      lcd_params_,
      stereo_camera_->getLeftCamParams(),
      stereo_camera_->getBodyPoseLeftCamRect(),
      stereo_camera_,
      frontend_params_.stereo_matching_params_,
      std::nullopt,
      false);
  gtsam::Pose3 empty_pose = gtsam::Pose3();
  std::pair<double, double> error;

  const auto& cache = lcd_detector_->getFrameCache();

  /* Test proper scaled pose recovery between ref and cur images */
  CHECK(match1_stereo_frame_);
  CHECK(query1_stereo_frame_);
  lcd_detector_->processAndAddStereoFrame(*match1_stereo_frame_);
  lcd_detector_->processAndAddStereoFrame(*query1_stereo_frame_);

  ASSERT_EQ(cache.size(), 2u);
  auto lcd_frame_0 = cache.getFrame(0);
  ASSERT_TRUE(lcd_frame_0);
  auto lcd_frame_1 = cache.getFrame(1);
  ASSERT_TRUE(lcd_frame_1);

  // Find correspondences between keypoints.
  KeypointMatches matches1;
  lcd_detector_->computeDescriptorMatches(lcd_frame_0->descriptors_mat_,
                                          lcd_frame_1->descriptors_mat_,
                                          &matches1,
                                          true);

  gtsam::Pose3 bodyMatch1_T_bodyQuery1_stereo;
  std::vector<int> inliers_1;
  lcd_detector_->recoverPoseBody(*lcd_frame_0,
                                 *lcd_frame_1,
                                 empty_pose,
                                 matches1,
                                 &bodyMatch1_T_bodyQuery1_stereo,
                                 &inliers_1);
  error = UtilsOpenCV::ComputeRotationAndTranslationErrors(
      bodyMatch1_T_bodyQuery1_gt_, bodyMatch1_T_bodyQuery1_stereo, false);

  EXPECT_LT(error.first, rot_tol_stereo);
  EXPECT_LT(error.second, tran_tol_stereo);

  /* Test pose recovery on other two images */
  CHECK(match2_stereo_frame_);
  CHECK(query2_stereo_frame_);
  lcd_detector_->processAndAddStereoFrame(*match2_stereo_frame_);
  lcd_detector_->processAndAddStereoFrame(*query2_stereo_frame_);

  ASSERT_EQ(cache.size(), 4u);
  auto lcd_frame_2 = cache.getFrame(2);
  ASSERT_TRUE(lcd_frame_2);
  auto lcd_frame_3 = cache.getFrame(3);
  ASSERT_TRUE(lcd_frame_3);

  // Find correspondences between keypoints.
  KeypointMatches matches2;
  lcd_detector_->computeDescriptorMatches(lcd_frame_2->descriptors_mat_,
                                          lcd_frame_3->descriptors_mat_,
                                          &matches2,
                                          true);

  gtsam::Pose3 bodyMatch2_T_bodyQuery2_stereo;
  std::vector<int> inliers_2;
  lcd_detector_->recoverPoseBody(*lcd_frame_2,
                                 *lcd_frame_3,
                                 empty_pose,
                                 matches2,
                                 &bodyMatch2_T_bodyQuery2_stereo,
                                 &inliers_2);

  error = UtilsOpenCV::ComputeRotationAndTranslationErrors(
      bodyMatch2_T_bodyQuery2_gt_, bodyMatch2_T_bodyQuery2_stereo, false);

  // TODO(marcus): add comment everywhere where this is tuned explaining why
  // tol_loose for use in places like this
  EXPECT_LT(error.first, rot_tol_stereo * 1.5);
  EXPECT_LT(error.second, tran_tol_stereo * 1.5);
}

TEST_F(LCDFixture, recoverPoseBodyGivenRot) {
  CHECK(lcd_detector_);
  lcd_params_.tracker_params_.ransac_use_1point_stereo_ = true;
  lcd_detector_ = std::make_unique<LoopClosureDetector>(
      lcd_params_,
      stereo_camera_->getLeftCamParams(),
      stereo_camera_->getBodyPoseLeftCamRect(),
      stereo_camera_,
      frontend_params_.stereo_matching_params_,
      std::nullopt,
      false);

  const auto& cache = lcd_detector_->getFrameCache();

  gtsam::Pose3 bodyMatch1_T_bodyQuery1_gt, camMatch1_T_camQuery1_gt;
  std::pair<double, double> error;

  /* Test pose recovery given ground truth rotation and unit translation */
  CHECK(match1_stereo_frame_);
  CHECK(query1_stereo_frame_);
  lcd_detector_->processAndAddStereoFrame(*match1_stereo_frame_);
  lcd_detector_->processAndAddStereoFrame(*query1_stereo_frame_);

  ASSERT_EQ(cache.size(), 2u);
  auto lcd_frame_0 = cache.getFrame(0);
  ASSERT_TRUE(lcd_frame_0);
  auto lcd_frame_1 = cache.getFrame(1);
  ASSERT_TRUE(lcd_frame_1);

  bodyMatch1_T_bodyQuery1_gt = bodyMatch1_T_bodyQuery1_gt_;
  lcd_detector_->transformBodyPoseToCameraPose(bodyMatch1_T_bodyQuery1_gt,
                                               &camMatch1_T_camQuery1_gt);
  // Fake mono ransac with translation up to scale
  gtsam::Pose3 camMatch1_T_camQuery1_mono_gt =
      gtsam::Pose3(camMatch1_T_camQuery1_gt.rotation(),
                   camMatch1_T_camQuery1_gt.translation() /
                       camMatch1_T_camQuery1_gt.translation().norm());
  // Find correspondences between keypoints.
  KeypointMatches matches1;
  lcd_detector_->computeDescriptorMatches(lcd_frame_0->descriptors_mat_,
                                          lcd_frame_1->descriptors_mat_,
                                          &matches1,
                                          true);

  gtsam::Pose3 bodyMatch1_T_bodyQuery1_stereo;
  std::vector<int> inliers_1;
  lcd_detector_->recoverPoseBody(*lcd_frame_1,
                                 *lcd_frame_0,
                                 camMatch1_T_camQuery1_mono_gt,
                                 matches1,
                                 &bodyMatch1_T_bodyQuery1_stereo,
                                 &inliers_1);

  error = UtilsOpenCV::ComputeRotationAndTranslationErrors(
      bodyMatch1_T_bodyQuery1_gt_, bodyMatch1_T_bodyQuery1_stereo, false);
  EXPECT_LT(error.first, rot_tol_stereo);
  EXPECT_LT(error.second, tran_tol_stereo);

  /* Test pose recovery on other two images */
  gtsam::Pose3 bodyMatch2_T_bodyQuery2_gt, camMatch2_T_camQuery2_gt;

  CHECK(match2_stereo_frame_);
  CHECK(query2_stereo_frame_);
  FrameId frm_2 =
      lcd_detector_->processAndAddStereoFrame(*match2_stereo_frame_);
  FrameId frm_3 =
      lcd_detector_->processAndAddStereoFrame(*query2_stereo_frame_);

  ASSERT_EQ(cache.size(), 4u);
  auto lcd_frame_2 = cache.getFrame(2);
  ASSERT_TRUE(lcd_frame_2);
  auto lcd_frame_3 = cache.getFrame(3);
  ASSERT_TRUE(lcd_frame_3);

  bodyMatch2_T_bodyQuery2_gt = bodyMatch2_T_bodyQuery2_gt_;
  lcd_detector_->transformBodyPoseToCameraPose(bodyMatch2_T_bodyQuery2_gt,
                                               &camMatch2_T_camQuery2_gt);

  // Fake mono ransac with translation up to scale
  gtsam::Pose3 camMatch2_T_camQuery2_mono_gt =
      gtsam::Pose3(camMatch2_T_camQuery2_gt.rotation(),
                   camMatch2_T_camQuery2_gt.translation() /
                       camMatch2_T_camQuery2_gt.translation().norm());

  // Find correspondences between keypoints.
  KeypointMatches matches2;
  lcd_detector_->computeDescriptorMatches(lcd_frame_2->descriptors_mat_,
                                          lcd_frame_3->descriptors_mat_,
                                          &matches2,
                                          true);

  gtsam::Pose3 bodyMatch2_T_bodyQuery2_stereo;
  std::vector<int> inliers_2;
  lcd_detector_->recoverPoseBody(*lcd_frame_3,
                                 *lcd_frame_2,
                                 camMatch2_T_camQuery2_gt,
                                 matches2,
                                 &bodyMatch2_T_bodyQuery2_stereo,
                                 &inliers_2);

  error = UtilsOpenCV::ComputeRotationAndTranslationErrors(
      bodyMatch2_T_bodyQuery2_gt_, bodyMatch2_T_bodyQuery2_stereo, false);
  EXPECT_LT(error.first, rot_tol_stereo);
  EXPECT_LT(error.second, tran_tol_stereo);
}

TEST_F(LCDFixture, recoverPoseBodyPnpMono) {
  /* Test proper scaled pose recovery between ref and cur images */
  CHECK(lcd_detector_);
  lcd_params_.tracker_params_.ransac_use_1point_stereo_ = false;
  lcd_params_.tracker_params_.ransac_randomize_ = false;
  lcd_params_.pose_recovery_type_ = PoseRecoveryType::kPnP;
  lcd_detector_ = std::make_unique<LoopClosureDetector>(
      lcd_params_,
      stereo_camera_->getLeftCamParams(),
      stereo_camera_->getBodyPoseLeftCamRect(),
      std::nullopt,
      std::nullopt,
      std::nullopt,
      false);
  gtsam::Pose3 empty_pose = gtsam::Pose3();
  std::pair<double, double> error;

  const auto& cache = lcd_detector_->getFrameCache();

  // Process mono frames
  // TODO(marcus): remove pose from here entirely, not necessary anymore
  lcd_detector_->processAndAddMonoFrame(
      match1_stereo_frame_->left_frame_, W_match1_lmks3d_, world_T_bodyMatch1_);
  lcd_detector_->processAndAddMonoFrame(
      query1_stereo_frame_->left_frame_, W_query1_lmks3d_, world_T_bodyQuery1_);

  ASSERT_EQ(cache.size(), 2u);
  auto lcd_frame_0 = cache.getFrame(0);
  ASSERT_TRUE(lcd_frame_0);
  auto lcd_frame_1 = cache.getFrame(1);
  ASSERT_TRUE(lcd_frame_1);

  // Find correspondences between keypoints.
  KeypointMatches matches1;
  lcd_detector_->computeDescriptorMatches(lcd_frame_0->descriptors_mat_,
                                          lcd_frame_1->descriptors_mat_,
                                          &matches1,
                                          true);

  // All matches are inliers for this test
  std::vector<int> inliers_1;
  for (size_t i = 0; i < matches1.size(); i++) {
    inliers_1.push_back(i);
  }
  gtsam::Pose3 bodyMatch1_T_bodyQuery1_stereo;
  lcd_detector_->recoverPoseBody(*lcd_frame_0,
                                 *lcd_frame_1,
                                 empty_pose,
                                 matches1,
                                 &bodyMatch1_T_bodyQuery1_stereo,
                                 &inliers_1);

  error = UtilsOpenCV::ComputeRotationAndTranslationErrors(
      bodyMatch1_T_bodyQuery1_gt_, bodyMatch1_T_bodyQuery1_stereo, true);

  EXPECT_LT(error.first, rot_tol_stereo);
  EXPECT_LT(error.second, tran_tol_stereo);
}

TEST_F(LCDFixture, recoverPoseBody5ptMono) {
  /* Test proper scaled pose recovery between ref and cur images */
  CHECK(lcd_detector_);
  lcd_params_.tracker_params_.ransac_use_1point_stereo_ = false;
  lcd_params_.tracker_params_.ransac_randomize_ = false;
  lcd_params_.pose_recovery_type_ = PoseRecoveryType::k5ptRotOnly;
  lcd_detector_ = std::make_unique<LoopClosureDetector>(
      lcd_params_,
      stereo_camera_->getLeftCamParams(),
      stereo_camera_->getBodyPoseLeftCamRect(),
      std::nullopt,
      std::nullopt,
      std::nullopt,
      false);
  gtsam::Pose3 empty_pose = gtsam::Pose3();
  std::pair<double, double> error;

  const auto& cache = lcd_detector_->getFrameCache();

  // Process mono frames
  // TODO(marcus): remove pose from here entirely, not necessary anymore
  lcd_detector_->processAndAddMonoFrame(
      match1_stereo_frame_->left_frame_, W_match1_lmks3d_, world_T_bodyMatch1_);
  lcd_detector_->processAndAddMonoFrame(
      query1_stereo_frame_->left_frame_, W_query1_lmks3d_, world_T_bodyQuery1_);

  ASSERT_EQ(cache.size(), 2u);
  auto lcd_frame_0 = cache.getFrame(0);
  ASSERT_TRUE(lcd_frame_0);
  auto lcd_frame_1 = cache.getFrame(1);
  ASSERT_TRUE(lcd_frame_1);

  // Find correspondences between keypoints.
  KeypointMatches matches1;
  lcd_detector_->computeDescriptorMatches(lcd_frame_0->descriptors_mat_,
                                          lcd_frame_1->descriptors_mat_,
                                          &matches1,
                                          true);

  // All matches are inliers for this test
  std::vector<int> inliers_1;
  for (size_t i = 0; i < matches1.size(); i++) {
    inliers_1.push_back(i);
  }
  gtsam::Pose3 bodyMatch1_T_bodyQuery1_stereo;
  lcd_detector_->recoverPoseBody(*lcd_frame_0,
                                 *lcd_frame_1,
                                 empty_pose,
                                 matches1,
                                 &bodyMatch1_T_bodyQuery1_stereo,
                                 &inliers_1);

  error = UtilsOpenCV::ComputeRotationAndTranslationErrors(
      bodyMatch1_T_bodyQuery1_gt_, bodyMatch1_T_bodyQuery1_stereo, true);

  EXPECT_LT(error.first, rot_tol_stereo * 2.5);
  EXPECT_LT(error.second, tran_tol_stereo);
}

TEST_F(LCDFixture, recoverPoseBodyPnpStereo) {
  /* Test proper scaled pose recovery between ref and cur images */
  CHECK(lcd_detector_);
  lcd_params_.pose_recovery_type_ = PoseRecoveryType::kPnP;
  lcd_params_.tracker_params_.pnp_algorithm_ = Pose3d2dAlgorithm::EPNP;
  lcd_params_.refine_pose_ = false;
  lcd_detector_ = std::make_unique<LoopClosureDetector>(
      lcd_params_,
      stereo_camera_->getLeftCamParams(),
      stereo_camera_->getBodyPoseLeftCamRect(),
      stereo_camera_,
      frontend_params_.stereo_matching_params_,
      std::nullopt,
      false);
  gtsam::Pose3 empty_pose = gtsam::Pose3();
  std::pair<double, double> error;

  const auto& cache = lcd_detector_->getFrameCache();

  CHECK(match1_stereo_frame_);
  CHECK(query1_stereo_frame_);

  lcd_detector_->processAndAddStereoFrame(*match1_stereo_frame_);
  lcd_detector_->processAndAddStereoFrame(*query1_stereo_frame_);

  ASSERT_EQ(cache.size(), 2u);
  auto lcd_frame_0 = cache.getFrame(0);
  ASSERT_TRUE(lcd_frame_0);
  auto lcd_frame_1 = cache.getFrame(1);
  ASSERT_TRUE(lcd_frame_1);

  // Find correspondences between keypoints.
  KeypointMatches matches1;
  lcd_detector_->computeDescriptorMatches(lcd_frame_0->descriptors_mat_,
                                          lcd_frame_1->descriptors_mat_,
                                          &matches1,
                                          true);

  // All matches are inliers for this test
  std::vector<int> inliers_1;
  for (size_t i = 0; i < matches1.size(); i++) {
    inliers_1.push_back(i);
  }
  gtsam::Pose3 bodyMatch1_T_bodyQuery1_stereo;
  lcd_detector_->recoverPoseBody(*lcd_frame_0,
                                 *lcd_frame_1,
                                 empty_pose,
                                 matches1,
                                 &bodyMatch1_T_bodyQuery1_stereo,
                                 &inliers_1);
  error = UtilsOpenCV::ComputeRotationAndTranslationErrors(
      bodyMatch1_T_bodyQuery1_gt_, bodyMatch1_T_bodyQuery1_stereo, false);

  EXPECT_LT(error.first, rot_tol_stereo);
  EXPECT_LT(error.second, tran_tol_stereo);
}

TEST_F(LCDFixture, detectLoop) {
  lcd_params_.tracker_params_.pose_2d2d_algorithm_ = Pose2d2dAlgorithm::NISTER;
  lcd_detector_ = std::make_unique<LoopClosureDetector>(
      lcd_params_,
      stereo_camera_->getLeftCamParams(),
      stereo_camera_->getBodyPoseLeftCamRect(),
      stereo_camera_,
      frontend_params_.stereo_matching_params_,
      std::nullopt,
      false);

  std::pair<double, double> error;

  CHECK(lcd_detector_);

  CHECK(match1_stereo_frame_);
  CHECK(match2_stereo_frame_);
  CHECK(query1_stereo_frame_);

  lcd_detector_->processAndAddStereoFrame(*match1_stereo_frame_);
  FrameId frame_id_1 =
      lcd_detector_->processAndAddStereoFrame(*match2_stereo_frame_);
  FrameId frame_id_2 =
      lcd_detector_->processAndAddStereoFrame(*query1_stereo_frame_);

  /* Test the detectLoop method against two images without closure */
  LoopResult loop_result_0;
  lcd_detector_->detectLoopById(frame_id_1, &loop_result_0);
  EXPECT_EQ(loop_result_0.isLoop(), false);

  /* Test the detectLoop method against two non-identical, similar images */
  LoopResult loop_result_1;
  lcd_detector_->detectLoopById(frame_id_2, &loop_result_1);
  EXPECT_EQ(loop_result_1.isLoop(), true);
  EXPECT_EQ(loop_result_1.match_id_, 0);
  EXPECT_EQ(loop_result_1.query_id_, 2);

  error = UtilsOpenCV::ComputeRotationAndTranslationErrors(
      bodyMatch1_T_bodyQuery1_gt_, loop_result_1.relative_pose_, false);

  EXPECT_LT(error.first, rot_tol_stereo);
  EXPECT_LT(error.second, tran_tol_stereo);
}

TEST_F(LCDFixture, addOdometryFactorAndOptimize) {
  /* Test the addition of odometry factors to the PGO */
  CHECK(lcd_detector_);
  lcd_detector_->initializePGO(OdometryFactor(
      0, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Variance(6, 0.1)));

  OdometryFactor odom_factor(
      1, world_T_bodyMatch1_, gtsam::noiseModel::Isotropic::Variance(6, 0.1));
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
      0, world_T_bodyMatch1_, gtsam::noiseModel::Isotropic::Variance(6, 0.1));
  lcd_detector_->initializePGO(odom_factor_1);

  OdometryFactor odom_factor_2(
      1, world_T_bodyQuery1_, gtsam::noiseModel::Isotropic::Variance(6, 0.1));
  lcd_detector_->addOdometryFactorAndOptimize(odom_factor_2);

  LoopClosureFactor lc_factor_1_2(
      0,
      1,
      bodyMatch1_T_bodyQuery1_gt_,
      gtsam::noiseModel::Isotropic::Variance(6, 0.1));
  lcd_detector_->addLoopClosureFactorAndOptimize(lc_factor_1_2);

  gtsam::Values pgo_trajectory = lcd_detector_->getPGOTrajectory();
  gtsam::NonlinearFactorGraph pgo_nfg = lcd_detector_->getPGOnfg();

  EXPECT_EQ(pgo_trajectory.size(), 2);
  EXPECT_EQ(pgo_nfg.size(), 3);
}

TEST_F(LCDFixture, addLoopClosureFactorNoOptimize) {
  /* Add a lc but don't optimize because backend queue reports more packets */
  LoopClosureDetectorParams params;
  params.odom_rot_threshold_ = -1;
  params.odom_trans_threshold_ = -1;
  params.pcm_rot_threshold_ = -1;
  params.pcm_trans_threshold_ = -1;
  params.gnc_alpha_ = 0;
  params.max_lc_cached_before_optimize_ = 1000;
  lcd_detector_ = std::make_unique<LoopClosureDetector>(
      params,
      stereo_camera_->getLeftCamParams(),
      stereo_camera_->getBodyPoseLeftCamRect(),
      stereo_camera_,
      frontend_params_.stereo_matching_params_,
      std::nullopt,
      false);
  lcd_detector_->registerIsBackendQueueFilledCallback(
      std::bind(&LCDFixture::lcdInputQueueCb, this));
  CHECK(lcd_detector_);

  lcd_detector_->initializePGO(OdometryFactor(
      0, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Variance(6, 0.1)));

  size_t num_odom = 10;
  for (size_t i = 1; i < num_odom; i++) {
    lcd_detector_->addOdometryFactorAndOptimize(
        OdometryFactor(i,
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2 * i, 0, 0)),
                       gtsam::noiseModel::Isotropic::Variance(6, 0.1)));
  }

  // Check that the trajectory is just odometry factors concatenated together.
  const gtsam::Values pgo_trajectory_odom_only =
      lcd_detector_->getPGOTrajectory();
  const gtsam::NonlinearFactorGraph pgo_nfg_odom_only =
      lcd_detector_->getPGOnfg();
  EXPECT_EQ(pgo_trajectory_odom_only.size(), num_odom);
  EXPECT_EQ(pgo_nfg_odom_only.size(), num_odom);

  for (size_t i = 0; i < num_odom; i++) {
    EXPECT_EQ(pgo_trajectory_odom_only.keys().at(i), i);

    const auto& this_pose = pgo_trajectory_odom_only.at<gtsam::Pose3>(i);
    EXPECT_TRUE(this_pose.rotation().equals(gtsam::Rot3()));
    EXPECT_EQ(this_pose.translation().x(), 2 * i);
    EXPECT_EQ(this_pose.translation().y(), 0);
    EXPECT_EQ(this_pose.translation().z(), 0);
  }

  // Tell PGO not to optimize via the backend queue callback
  is_backend_queue_filled_ = true;
  // Push a bad loop closure that would throw off the trajectory after
  // optimization. One per pair of odometry measurements
  for (size_t i = 1; i < num_odom; i++) {
    lcd_detector_->addLoopClosureFactorAndOptimize(
        LoopClosureFactor(i - 1,
                          i,
                          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(10, 0, 0)),
                          gtsam::noiseModel::Isotropic::Variance(6, 0.1)));
  }

  // Check that trajectory is the same as before.
  gtsam::Values pgo_trajectory_first_lc = lcd_detector_->getPGOTrajectory();
  EXPECT_EQ(pgo_trajectory_first_lc.size(), pgo_trajectory_odom_only.size());
  for (size_t i = 0; i < pgo_trajectory_odom_only.size(); i++) {
    // Because PGO hasn't been optimized, new LC factor hasn't been used to
    // move the trajectory.
    EXPECT_TRUE(pgo_trajectory_first_lc.at<gtsam::Pose3>(i).equals(
        pgo_trajectory_odom_only.at<gtsam::Pose3>(i)));
  }

  // Now tell PGO to perform optimization via backend queue callback.
  is_backend_queue_filled_ = false;
  lcd_detector_->addLoopClosureFactorAndOptimize(
      LoopClosureFactor(0,
                        1,
                        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(10, 0, 0)),
                        gtsam::noiseModel::Isotropic::Variance(6, 0.1)));

  // Check that trajectory is the different from before.
  gtsam::Values pgo_trajectory_second_lc = lcd_detector_->getPGOTrajectory();
  EXPECT_EQ(pgo_trajectory_second_lc.size(), pgo_trajectory_odom_only.size());
  for (size_t i = 0; i < pgo_trajectory_odom_only.size(); i++) {
    EXPECT_FALSE(pgo_trajectory_second_lc.at<gtsam::Pose3>(i).equals(
        pgo_trajectory_odom_only.at<gtsam::Pose3>(i)));
  }
}

TEST_F(LCDFixture, spinOnce) {
  /* Test the full pipeline with one loop closure and full PGO optimization */
  CHECK(lcd_detector_);
  CHECK(match1_stereo_frame_);
  StereoFrontendOutput::Ptr stereo_frontend_output =
      std::make_shared<StereoFrontendOutput>(
          match1_stereo_frame_->isKeyframe(),
          StatusStereoMeasurementsPtr(),
          stereo_camera_->getBodyPoseLeftCamRect(),
          stereo_camera_->getBodyPoseRightCamRect(),
          *match1_stereo_frame_,
          ImuFrontend::PimPtr(),
          ImuAccGyrS(),
          cv::Mat(),
          DebugTrackerInfo());
  LcdOutput::Ptr output_0 =
      lcd_detector_->spinOnce(LcdInput(timestamp_match1_,
                                       stereo_frontend_output,
                                       FrameId(0),
                                       W_match1_lmks3d_,
                                       gtsam::Pose3()));

  CHECK(match2_stereo_frame_);
  stereo_frontend_output = std::make_shared<StereoFrontendOutput>(
      match2_stereo_frame_->isKeyframe(),
      StatusStereoMeasurementsPtr(),
      stereo_camera_->getBodyPoseLeftCamRect(),
      stereo_camera_->getBodyPoseRightCamRect(),
      *match2_stereo_frame_,
      ImuFrontend::PimPtr(),
      ImuAccGyrS(),
      cv::Mat(),
      DebugTrackerInfo());
  LcdOutput::Ptr output_1 =
      lcd_detector_->spinOnce(LcdInput(timestamp_match2_,
                                       stereo_frontend_output,
                                       FrameId(1),
                                       W_match2_lmks3d_,
                                       gtsam::Pose3()));

  CHECK(query1_stereo_frame_);
  stereo_frontend_output = std::make_shared<StereoFrontendOutput>(
      query1_stereo_frame_->isKeyframe(),
      StatusStereoMeasurementsPtr(),
      stereo_camera_->getBodyPoseLeftCamRect(),
      stereo_camera_->getBodyPoseRightCamRect(),
      *query1_stereo_frame_,
      ImuFrontend::PimPtr(),
      ImuAccGyrS(),
      cv::Mat(),
      DebugTrackerInfo());
  LcdOutput::Ptr output_2 =
      lcd_detector_->spinOnce(LcdInput(timestamp_query1_,
                                       stereo_frontend_output,
                                       FrameId(2),
                                       W_query1_lmks3d_,
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

TEST_F(LCDFixture, noRefinePosesInMono) {
  /* Make sure the LCD pipline fails if refine_poses_ is set to true in mono */
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  CHECK(lcd_detector_);
  lcd_params_.refine_pose_ = true;
  lcd_params_.pose_recovery_type_ = PoseRecoveryType::kPnP;
  lcd_detector_ = std::make_unique<LoopClosureDetector>(
      lcd_params_,
      stereo_camera_->getLeftCamParams(),
      stereo_camera_->getBodyPoseLeftCamRect(),
      stereo_camera_,
      frontend_params_.stereo_matching_params_,
      std::nullopt,
      false);

  const auto& cache = lcd_detector_->getFrameCache();

  lcd_detector_->processAndAddMonoFrame(
      match1_stereo_frame_->left_frame_, W_match1_lmks3d_, world_T_bodyMatch1_);
  lcd_detector_->processAndAddMonoFrame(
      query1_stereo_frame_->left_frame_, W_query1_lmks3d_, world_T_bodyQuery1_);

  ASSERT_EQ(cache.size(), 2u);
  auto lcd_frame_0 = cache.getFrame(0);
  ASSERT_TRUE(lcd_frame_0);
  auto lcd_frame_1 = cache.getFrame(1);
  ASSERT_TRUE(lcd_frame_1);

  // Find correspondences between keypoints.
  KeypointMatches matches1;
  lcd_detector_->computeDescriptorMatches(lcd_frame_0->descriptors_mat_,
                                          lcd_frame_1->descriptors_mat_,
                                          &matches1,
                                          true);

  gtsam::Pose3 bodyMatch1_T_bodyQuery1_stereo;
  std::vector<int> inliers_1;
  EXPECT_DEATH(lcd_detector_->recoverPoseBody(*lcd_frame_0,
                                              *lcd_frame_1,
                                              gtsam::Pose3(),
                                              matches1,
                                              &bodyMatch1_T_bodyQuery1_stereo,
                                              &inliers_1),
               "LoopClosureDetector: Stereo required for refinePose");
}

}  // namespace VIO
