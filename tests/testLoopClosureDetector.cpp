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

/* TEST ideas:
     1) test constructor of LoopClosureDetector to see if the proper number of
         visual works are loaded. Or do a test for > 1 basically
     2) likely do not need to test spin or spinOnce
     3) test initLoopDetector() to make sure loop_detector_ is defined
         and the OrbLoopDetector params are setup properly and vocab is setup
     4) test checkLoopClosure for both a correct loop closure and an incorrect
         consider splitting that function up into testable helper functions
     5) test extractOrb to make sure that keypoints and descriptors are of
         size larger than 1. In fact, size should be same as parameter for
         number of descriptors.
     6) test extractOrb to make sure that the descriptors are correct for some
         ground truth on some image.
     7) test extractOrb to make sure that the keypoints are in the right spot
         for some ground truth on some image.
     8) test checkLoopClosure to get a loop closure between two images.
         Make sure that the image id's are correct (load up some extra useless
         images as well) and also check the transform.
     9) test calcScaledRelativePose to make sure it gives the correct output
         for some known input.

     NOTE: you will likely need unit tests for DLoopDetector
     You probably don't need unit tests for DLib and DBoW2?

     NOTE: it seems like testVioBackEnd.cpp has a test on two images, one of
     which is a left and the other is a right cam of the same scene.
     You could build a StereoFrame in which both of these are left images,
     and since you know the transform because you know the baseline that could
     be an easy ground truth check.
     You will have to do a ground truth check with some kind of rotation
     involved as well, since we need to test the rotation matrix.

     NOTE: consider using a fixture so that you can have protected and private
     members for the testing class.
*/

#include "LoopClosureDetector.h"

#include "CameraParams.h"
#include "Frame.h"
#include "StereoFrame.h"
#include "StereoVisionFrontEnd.h"
#include "Tracker.h"
#include "UtilsOpenCV.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

DECLARE_string(test_data_path);

static const double tol = 1e-7;
static const double rot_tol = 0.04;
static const double tran_tol = 0.10;

using namespace VIO;

class LCDFixture :public ::testing::Test {
public:
  LCDFixture()
      : lcd_FLAGS_test_data_path_(FLAGS_test_data_path +
                                  string("/ForLoopClosureDetector")) {
    lcd_detector_ = VIO::make_unique<LoopClosureDetector>(
        LoopClosureDetectorParams(), false, 3);
    lcd_detector_->getLCDParamsMutable()->min_temporal_matches_ = 0;
    lcd_detector_->getLCDParamsMutable()->dist_local_ = 0;
    lcd_detector_->getLCDParamsMutable()->di_levels_ = 0;
    lcd_detector_->getLCDParamsMutable()->alpha_ = 0.0001;
    lcd_detector_->getLCDParamsMutable()->use_nss_ = false;
    lcd_detector_->getLCDParamsMutable()->pose_recovery_option_ =
        Pose3DRecoveryOption::RANSAC_3PT;
    lcd_detector_->getLCDParamsMutable()->ransac_randomize_3d_ = false;
    lcd_detector_->getLCDParamsMutable()->min_Fpoints_ = 5; // to match extra with extra_2

    // TODO: make this parse from the file saved for ground truth
    // gtsam::Pose3 ref_pose = gtsam::Pose3(
    //     gtsam::Rot3(gtsam::Quaternion(0.478634,0.415595,-0.700197,0.328505)),
    //     gtsam::Point3(1.872115,1.786064,1.586159));

    gtsam::Pose3 ref_pose = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.338337,0.608466,-0.535476,0.478082)),
        gtsam::Point3(1.573832,2.023348,1.738755));

    // gtsam::Pose3 cur_pose = gtsam::Pose3(
    //     gtsam::Rot3(gtsam::Quaternion(0.480033,0.405427,-0.707909,0.322588)),
    //     gtsam::Point3(1.811982,1.746032,1.586722));

    gtsam::Pose3 cur_pose = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.478634,0.415595,-0.700197,0.328505)),
        gtsam::Point3(1.872115,1.786064,1.586159));

    gtsam::Pose3 extra_pose = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.339410,-0.672895,-0.492724,-0.435018)),
        gtsam::Point3(-0.662997,-0.495046,1.347300));
    gtsam::Pose3 extra_2_pose = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.392664,-0.590667,-0.580231,-0.400326)),
        gtsam::Point3(-0.345638,-0.501712,1.320441));

    // gtsam::Pose3 extra_pose = gtsam::Pose3(
    //     gtsam::Rot3(gtsam::Quaternion(0.209213,-0.785229,-0.236562,-0.532620)),
    //     gtsam::Point3(1.181681,0.119393,1.128875));
    // gtsam::Pose3 extra_2_pose = gtsam::Pose3(
    //     gtsam::Rot3(gtsam::Quaternion(0.218505,-0.771097,-0.264223,-0.536518)),
    //     gtsam::Point3(1.331751,0.565291,1.249742));

    ref_to_cur_pose_ = ref_pose.between(cur_pose);
    extra_to_extra_2_pose_ = extra_pose.between(extra_2_pose);

    initializeData();
  }

protected:
  void initializeData() {
    // Initialize CameraParams for both frames
    CameraParams cam_params_left, cam_params_right;
    cam_params_left.parseYAML(lcd_FLAGS_test_data_path_+"/sensorLeft.yaml");
    cam_params_right.parseYAML(lcd_FLAGS_test_data_path_+"/sensorRight.yaml");

    // string img_name_ref_left = lcd_FLAGS_test_data_path_+"/left_img_0.png";
    // string img_name_ref_right = lcd_FLAGS_test_data_path_+"/right_img_0.png";
    string img_name_ref_left = lcd_FLAGS_test_data_path_+"/test_left.png";
    string img_name_ref_right = lcd_FLAGS_test_data_path_+"/test_right.png";

    // string img_name_cur_left = lcd_FLAGS_test_data_path_+"/left_img_1.png";
    // string img_name_cur_right = lcd_FLAGS_test_data_path_+"/right_img_1.png";
    string img_name_cur_left = lcd_FLAGS_test_data_path_+"/left_img_0.png";
    string img_name_cur_right = lcd_FLAGS_test_data_path_+"/right_img_0.png";
    // string img_name_cur_left = lcd_FLAGS_test_data_path_+"/test_left.png";
    // string img_name_cur_right = lcd_FLAGS_test_data_path_+"/test_right.png";


    // Get ground truth camera relative poses
    gtsam::Pose3 camL_Pose_camR =
        cam_params_left.body_Pose_cam_.between(cam_params_right.body_Pose_cam_);

    // Initialize StereoFrame objects for reference and current frames
    VioFrontEndParams tp;
    Tracker tracker(tp, 0);

    ref_stereo_frame_ = std::make_shared<StereoFrame>(
        id_ref_, timestamp_ref_,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_ref_left, tp.getStereoMatchingParams().equalize_image_),
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_ref_right, tp.getStereoMatchingParams().equalize_image_),
        cam_params_right, camL_Pose_camR, tp.getStereoMatchingParams());

    tracker.featureDetection(ref_stereo_frame_->getLeftFrameMutable());
    ref_stereo_frame_->setIsKeyframe(true);
    // ref_stereo_frame_->setIsRectified(true);
    ref_stereo_frame_->sparseStereoMatching();

    cur_stereo_frame_ = std::make_shared<StereoFrame>(
        id_cur_, timestamp_cur_,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_cur_left, tp.getStereoMatchingParams().equalize_image_),
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_cur_right, tp.getStereoMatchingParams().equalize_image_),
        cam_params_right, camL_Pose_camR, tp.getStereoMatchingParams());

    tracker.featureDetection(cur_stereo_frame_->getLeftFrameMutable());
    cur_stereo_frame_->setIsKeyframe(true);
    // cur_stereo_frame_->setIsRectified(true);
    cur_stereo_frame_->sparseStereoMatching();

    // Initialize everything for the extra frame
    CameraParams cam_params_extra_left, cam_params_extra_right;
    cam_params_extra_left.parseYAML(
        lcd_FLAGS_test_data_path_+"/sensorLeft.yaml");
    cam_params_extra_right.parseYAML(
        lcd_FLAGS_test_data_path_+"/sensorRight.yaml");

    string img_name_extra_left = lcd_FLAGS_test_data_path_+"/left_img_3.png";
    string img_name_extra_right = lcd_FLAGS_test_data_path_+"/right_img_3.png";

    gtsam::Pose3 camL_Pose_camR_extra =
        cam_params_extra_left.body_Pose_cam_.between(
            cam_params_extra_right.body_Pose_cam_);

    extra_stereo_frame_ = std::make_shared<StereoFrame>(
        id_extra_, timestamp_extra_,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_extra_left, tp.getStereoMatchingParams().equalize_image_),
        cam_params_extra_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_extra_right, tp.getStereoMatchingParams().equalize_image_),
        cam_params_extra_right, camL_Pose_camR_extra,
        tp.getStereoMatchingParams());

    tracker.featureDetection(extra_stereo_frame_->getLeftFrameMutable());
    extra_stereo_frame_->setIsKeyframe(true);
    // extra_stereo_frame_->setIsRectified(true);
    extra_stereo_frame_->sparseStereoMatching();

    // Frame 4 initialization
    CameraParams cam_params_extra_2_left, cam_params_extra_2_right;
    cam_params_extra_2_left.parseYAML(
        lcd_FLAGS_test_data_path_+"/sensorLeft.yaml");
    cam_params_extra_2_right.parseYAML(
        lcd_FLAGS_test_data_path_+"/sensorRight.yaml");

    string img_name_extra_2_left = lcd_FLAGS_test_data_path_+"/left_img_4.png";
    string img_name_extra_2_right = lcd_FLAGS_test_data_path_+"/right_img_4.png";

    gtsam::Pose3 camL_Pose_camR_extra_2_ =
        cam_params_extra_2_left.body_Pose_cam_.between(
            cam_params_extra_2_right.body_Pose_cam_);

    extra_2_stereo_frame_ = std::make_shared<StereoFrame>(
        id_extra_2_, timestamp_extra_2_,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_extra_2_left, tp.getStereoMatchingParams().equalize_image_),
        cam_params_extra_2_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_extra_2_right, tp.getStereoMatchingParams().equalize_image_),
        cam_params_extra_2_right, camL_Pose_camR_extra_2_,
        tp.getStereoMatchingParams());

    tracker.featureDetection(extra_2_stereo_frame_->getLeftFrameMutable());
    extra_2_stereo_frame_->setIsKeyframe(true);
    // extra_2_stereo_frame_->setIsRectified(true);
    extra_2_stereo_frame_->sparseStereoMatching();

    // Set intrinsics for essential matrix calculation:
    lcd_detector_->getLCDParamsMutable()->focal_length_ =
        ref_stereo_frame_->getLeftFrame().cam_param_.intrinsics_[0];
    lcd_detector_->getLCDParamsMutable()->principle_point_ =
        cv::Point2d(
            ref_stereo_frame_->getLeftFrame().cam_param_.intrinsics_[2],
            ref_stereo_frame_->getLeftFrame().cam_param_.intrinsics_[3]);
  }

  // Standard gtest methods, unnecessary for now
  virtual void SetUp() {}
  virtual void TearDown() {}

protected:
  // Data-related members
  std::string lcd_FLAGS_test_data_path_;

  // LCD members
  std::unique_ptr<LoopClosureDetector> lcd_detector_;

  // Stored frame members
  gtsam::Pose3 ref_to_cur_pose_, extra_to_extra_2_pose_;
  std::shared_ptr<StereoFrame> ref_stereo_frame_, cur_stereo_frame_;
  std::shared_ptr<StereoFrame> extra_stereo_frame_, extra_2_stereo_frame_;
  const FrameId id_ref_ = 0, id_cur_ = 1;
  const FrameId id_extra_ = 2, id_extra_2_=3;
  const VIO::Timestamp timestamp_ref_ = 1000, timestamp_cur_ = 2000;
  const VIO::Timestamp timestamp_extra_ = 3000, timestamp_extra_2_ = 4000;


}; // class LCDFixture

TEST_F(LCDFixture, defaultConstructor) {
  /* Test default constructor to ensure that vocabulary is loaded correctly. */
  EXPECT_GT(lcd_detector_->getVocabulary().size(), 0);
}

TEST_F(LCDFixture, geometricVerificationCheck) {
  double relativeRotError, relativeTranError;

  lcd_detector_->getLCDParamsMutable()->geom_check_ = GeomVerifOption::NISTER;
  lcd_detector_->getLCDParamsMutable()->ransac_verification_ = false;

  FrameId frm_0 = lcd_detector_->processAndAddFrame(*ref_stereo_frame_.get());
  FrameId frm_1 = lcd_detector_->processAndAddFrame(*cur_stereo_frame_.get());

  cv::Mat matched_img = lcd_detector_->computeAndDrawMatchesBetweenFrames(
      cur_stereo_frame_->getLeftFrame().img_,
      ref_stereo_frame_->getLeftFrame().img_,
      frm_1, frm_0);

  cv::imshow("Good Matches", matched_img );
  cv::waitKey();

  LoopResult res_0 = lcd_detector_->detectLoop(frm_0);
  LoopResult res_1 = lcd_detector_->detectLoop(frm_1);

  gtsam::Point3 T = ref_to_cur_pose_.translation();
  double magnitude = sqrt(T[0]*T[0] + T[1]*T[1] + T[2]*T[2]);
  gtsam::Point3 unit_T_ref_to_cur = T / magnitude;
  gtsam::Pose3 gnd_truth = gtsam::Pose3(ref_to_cur_pose_.rotation(),
     unit_T_ref_to_cur);

  gtsam::Point3 col1(0.0, T[2], -T[1]);
  gtsam::Point3 col2(-T[2], 0.0, T[0]);
  gtsam::Point3 col3(T[1], -T[0], 0.0);
  gtsam::Rot3 t_gt_mat(col1, col2, col3);
  gtsam::Rot3 E_gt = ref_to_cur_pose_.rotation() * t_gt_mat;
  std::cout <<"E_gt: " << E_gt << std::endl;

  std::cout << "\ngnd_truth pose: " << gnd_truth
            << "\nactual: " << res_1.relative_pose_2d_
            << std::endl;

  std::tie(relativeRotError,relativeTranError) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(gnd_truth,
          res_1.relative_pose_2d_, true);

  EXPECT_LT(relativeRotError, rot_tol);
  EXPECT_LT(relativeTranError, tran_tol);
}

// TODO: this could be private, and might be a part of processAndAddFrame()
TEST_F(LCDFixture, DISABLED_detectLoop) {

}

TEST_F(LCDFixture, DISABLED_mat2pose) {

}

TEST_F(LCDFixture, DISABLED_rewriteStereoFrameFeatures) {

}

TEST_F(LCDFixture, compute3DPose) {
  /* Test proper scaled pose recovery between two identical images */
  double relativeRotError, relativeTranError;
  gtsam::Pose3 empty_pose;

  lcd_detector_->getLCDParamsMutable()->pose_recovery_option_ = Pose3DRecoveryOption::ARUN_3PT;

  lcd_detector_->processAndAddFrame(*ref_stereo_frame_.get());
  lcd_detector_->processAndAddFrame(*ref_stereo_frame_.get());

  gtsam::Pose3 pose_0 = lcd_detector_->compute3DPose(1,0,empty_pose);
  EXPECT_TRUE(pose_0.rotation().equals(gtsam::Rot3::identity(), tol));
  EXPECT_TRUE(pose_0.translation().equals(gtsam::Point3(0,0,0), tol));

  /* Test proper scaled pose recovery between ref and cur images */
  lcd_detector_->processAndAddFrame(*cur_stereo_frame_.get());

  gtsam::Pose3 pose_1 = lcd_detector_->compute3DPose(2,0,empty_pose);

  std::tie(relativeRotError,relativeTranError) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(ref_to_cur_pose_,
          pose_1, true);

  // TODO: unique tol's. Find out what tol we should have.
  EXPECT_LT(relativeRotError, rot_tol);
  EXPECT_LT(relativeTranError, tran_tol);

  /* Test proper scaled pose recovery between extra and extra_2 images */
  lcd_detector_->processAndAddFrame(*extra_stereo_frame_.get());
  lcd_detector_->processAndAddFrame(*extra_2_stereo_frame_.get());

  gtsam::Pose3 pose_2 = lcd_detector_->compute3DPose(4,3,empty_pose);

  std::tie(relativeRotError,relativeTranError) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(extra_to_extra_2_pose_,
          pose_2, true);

  EXPECT_LT(relativeRotError, rot_tol);
  EXPECT_LT(relativeTranError, tran_tol);

  /* Test failed conversion between ref and extra images */
  gtsam::Pose3 pose_3 = lcd_detector_->compute3DPose(4,0,empty_pose);
  EXPECT_TRUE(pose_3.rotation().equals(gtsam::Rot3(), tol));
  EXPECT_TRUE(pose_3.translation().equals(gtsam::Point3(), tol));
}

TEST_F(LCDFixture, compute3DPoseGiven2D) {
  /* Test pose recovery using scaling factor calculation */
  double relativeRotError, relativeTranError;
  lcd_detector_->getLCDParamsMutable()->pose_recovery_option_ =
      Pose3DRecoveryOption::GIVEN_ROT;

  gtsam::Pose3 pose_2d;
  FrameId frm_0 = lcd_detector_->processAndAddFrame(*ref_stereo_frame_.get());
  FrameId frm_1 = lcd_detector_->processAndAddFrame(*cur_stereo_frame_.get());

  LoopResult res_0 = lcd_detector_->detectLoop(frm_0);
  LoopResult res_1 = lcd_detector_->detectLoop(frm_1);

  pose_2d = res_1.relative_pose_2d_;
  gtsam::Pose3 pose_0_1 = lcd_detector_->compute3DPose(1,0,pose_2d);

  std::tie(relativeRotError,relativeTranError) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(ref_to_cur_pose_,
          pose_0_1, true);
  EXPECT_LT(relativeRotError, rot_tol);
  EXPECT_LT(relativeTranError, tran_tol);

  FrameId frm_2 = lcd_detector_->processAndAddFrame(*extra_stereo_frame_.get());
  FrameId frm_3 = lcd_detector_->processAndAddFrame(*extra_2_stereo_frame_.get());

  // TODO: this is a cheating way of doing this, you need to find two frames that identify as a loop closure
  gtsam::Point3 unit_t = extra_to_extra_2_pose_.translation() /
      extra_to_extra_2_pose_.translation().norm();
  pose_2d = gtsam::Pose3(extra_to_extra_2_pose_.rotation(), unit_t);

  gtsam::Pose3 pose_2_3 = lcd_detector_->compute3DPose(3,2,pose_2d);

  std::tie(relativeRotError,relativeTranError) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(extra_to_extra_2_pose_,
          pose_2_3, true);
  EXPECT_LT(relativeRotError, rot_tol);
  EXPECT_LT(relativeTranError, tran_tol);
}

TEST_F(LCDFixture, checkLoopClosure) {
  double relativeRotError, relativeTranError;

  lcd_detector_->getLCDParamsMutable()->pose_recovery_option_ =
      Pose3DRecoveryOption::GIVEN_ROT;
  lcd_detector_->getLCDParamsMutable()->geom_check_ = GeomVerifOption::NISTER;
  lcd_detector_->getLCDParamsMutable()->use_2d_rot_ = false;

  /* Test the checkLoopClosure method against two images without closure */
  LoopClosureDetectorOutputPayload output_payload_0 =
      lcd_detector_->checkLoopClosure(*extra_stereo_frame_.get());
  EXPECT_EQ(output_payload_0.is_loop_, false);

  LoopClosureDetectorOutputPayload output_payload_1 =
      lcd_detector_->checkLoopClosure(*ref_stereo_frame_.get());
  EXPECT_EQ(output_payload_1.is_loop_, false);

  /* Test the checkLoopClosure method against two images that are identical */
  // TODO: why isn't geom_check working for two identical images?
  LoopClosureDetectorOutputPayload output_payload_2 =
      lcd_detector_->checkLoopClosure(*ref_stereo_frame_.get());
  // EXPECT_EQ(output_payload_2.is_loop_, true);
  // EXPECT_EQ(output_payload_2.timestamp_kf_, timestamp_ref_);
  // EXPECT_EQ(output_payload_2.id_recent_, id_ref_);
  // EXPECT_EQ(output_payload_2.id_match_, id_ref_);
  // EXPECT_TRUE(output_payload_2.relative_pose_.equals(gtsam::Pose3(
  //     gtsam::Rot3::identity(), gtsam::Point3(0,0,0)), tol));

  /* Test the checkLoopClosure method against two unidentical, similar images */
  LoopClosureDetectorOutputPayload output_payload_3 =
      lcd_detector_->checkLoopClosure(*cur_stereo_frame_.get());
  EXPECT_EQ(output_payload_3.is_loop_, true);
  EXPECT_EQ(output_payload_3.timestamp_kf_, timestamp_cur_);
  EXPECT_EQ(output_payload_3.id_match_, id_ref_);
  EXPECT_EQ(output_payload_3.id_recent_, id_cur_);
  // EXPECT_TRUE(output_payload_3.relative_pose_.rotation().equals(
  //     ref_to_cur_pose_.rotation(), tol));
  // EXPECT_TRUE(output_payload_3.relative_pose_.translation().equals(
  //     ref_to_cur_pose_.translation(), tol));

  std::tie(relativeRotError,relativeTranError) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(ref_to_cur_pose_,
          output_payload_3.relative_pose_, true);

  // TODO: unique tol's. Find out what tol we should have.
  EXPECT_LT(relativeRotError, rot_tol);
  EXPECT_LT(relativeTranError, tran_tol);

  // std::cout << "\nground truth pose: " << ref_to_cur_pose_ << std::endl;
  // std::cout << "acquired pose : " << output_payload_3.relative_pose_ << std::endl;

  LoopClosureDetectorOutputPayload output_payload_4 =
      lcd_detector_->checkLoopClosure(*extra_2_stereo_frame_.get());
  EXPECT_EQ(output_payload_4.is_loop_, true);
  EXPECT_EQ(output_payload_4.timestamp_kf_, timestamp_extra_2_);
  EXPECT_EQ(output_payload_4.id_match_, id_extra_);
  EXPECT_EQ(output_payload_4.id_recent_, id_extra_2_);

  // std::cout << "\nground truth pose: " << extra_to_extra_2_pose_ << std::endl;
  // std::cout << "acquired pose : " << output_payload_4.relative_pose_ << std::endl;

  std::tie(relativeRotError,relativeTranError) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(extra_to_extra_2_pose_,
          output_payload_4.relative_pose_, true);

  EXPECT_LT(relativeRotError, rot_tol);
  EXPECT_LT(relativeTranError, tran_tol);
}
