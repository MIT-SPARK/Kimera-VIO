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

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

DECLARE_string(test_data_path);

static const double tol = 1e-7;

using namespace VIO;

class LCDFixture :public ::testing::Test {
public:
  LCDFixture()
      : stereo_FLAGS_test_data_path(FLAGS_test_data_path +
                                    string("/ForStereoFrame/"))
        lcd_FLAGS_test_data_path_(FLAGS_test_data_path +
                                  string("/ForLoopClosureDetector")) {
    default_lcd_detector_ = LoopClosureDetector();
    lcd_params_ptr_ = default_lcd_detector_.getLCDParamsMutable();

    // TODO: make this parse from the file saved for ground truth
    gtsam::Pose3 ref_pose = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.478634,0.415595,-0.700197,0.328505)),
        gtsam::Point3(1.872115,1.786064,1.586159));
    gtsam::Pose3 cur_pose = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.480033,0.405427,-0.707909,0.322588)),
        gtsam::Point3(1.811982,1.746032,1.586722));
    gtsam::Pose3 third_pose = gtsam::Pose3(
        gtsam::Rot3(gtsam::Quaternion(0.480371,0.406972,-0.707424,0.321200)),
        gtsam::Point3(1.803667,1.744080,1.594542));

    ref_to_cur_pose_ = ref_pose.between(cur_pose);

    initializeData();
  }

protected:
  void initializeData() {
    // Initialize CameraParams for both frames
    CameraParams cam_params_left, cam_params_right;
    cam_params_left.parseYAML(lcd_FLAGS_test_data_path_+"/sensorLeft.yaml");
    cam_params_right.parseYAML(lcd_FLAGS_test_data_path_+"/sensorRight.yaml");

    string img_name_ref_left = lcd_FLAGS_test_data_path_+"left_img_0.png";
    string img_name_ref_right = lcd_FLAGS_test_data_path_+"right_img_0.png";

    string img_name_cur_left = lcd_FLAGS_test_data_path_+"left_img_1.png";
    string img_name_cur_right = lcd_FLAGS_test_data_path_+"right_img_1.png";

    // Initialize reference and current mono frames
    ref_frame_ = std::make_shared<Frame>(
        id_ref_, timestamp_ref_, cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(img_name_ref_left));
    cur_frame_ = std::make_shared<Frame>(
        id_cur_, timestamp_cur_, cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(img_name_cur_left));

    // Get ground truth camera relative poses
    gtsam::Pose3 camL_Pose_camR =
        cam_params_left.body_Pose_cam_.between(cam_params_right.body_Pose_cam_);

    // Initialize StereoFrame objects for reference and current frames
    VioFrontEndParams tp;

    ref_stereo_frame_ = std::make_shared<StereoFrame>(
        id_ref_, timestamp_ref_,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_ref_left, tp.getStereoMatchingParams().equalize_image_),
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_ref_right, tp.getStereoMatchingParams().equalize_image_),
        cam_params_right, camL_Pose_camR, tp.getStereoMatchingParams());

    cur_stereo_frame_ = std::make_shared<StereoFrame>(
        id_cur_, timestamp_cur_,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_cur_left, tp.getStereoMatchingParams().equalize_image_),
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_cur_right, tp.getStereoMatchingParams().equalize_image_),
        cam_params_right, camL_Pose_camR, tp.getStereoMatchingParams());

    // Initialize everything for the extra frame
    CameraParams cam_params_extra_left, cam_params_extra_right;
    cam_params_extra_left.parseYAML(
        stereo_FLAGS_test_data_path_+"/sensorLeft.yaml");
    cam_params_extra_right.parseYAML(
        stereo_FLAGS_test_data_path_+"/sensorRight.yaml");

    string img_name_rand_left = stereo_FLAGS_test_data_path_+"left_img_0.png";
    string img_name_rand_right = stereo_FLAGS_test_data_path_+"right_img_0.png";

    extra_frame_ = std::Make_shared<Frame>(
        id_extra_, timestamp_extra_, cam_params_extra_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(img_name_rand_left));

    gtsam::Pose3 camL_Pose_camR_extra =
        cam_params_extra_left.body_Pose_cam_.between(
            cam_params_extra_leftright.body_Pose_cam_);

    extra_stereo_frame_ = std::make_shared<StereoFrame>(
        id_extra_, timestamp_extra_,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_rand_left, tp.getStereoMatchingParams().equalize_image_),
        cam_params_extra_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_rand_right, tp.getStereoMatchingParams().equalize_image_),
        cam_params_extra_right, camL_Pose_camR_extra,
        tp.getStereoMatchingParams());
  }

  // Standard gtest methods, unnecessary for now
  virtual void SetUp() {}
  virtual void TearDown() {}

protected:
  // LCD members
  LoopClosureDetector default_lcd_detector_;
  LoopClosureDetectorParams* lcd_params_ptr_;

  // Data-related members
  std::string stereo_FLAGS_test_data_path_;
  std::string lcd_FLAGS_test_data_path_

  gtsam::Pose3 ref_to_cur_pose_;

  std::shared_ptr<Frame> ref_frame_, cur_frame_, extra_frame_;
  std::shared_ptr<StereoFrame> ref_stereo_frame_, cur_stereo_frame_,
      extra_stereo_frame_;

  const FrameId id_ref_ = 0, id_cur_ = 1;
  const int64_t timestamp_ref_ = 1000, timestamp_cur_ = 2000;


}; // class LCDFixture

TEST_F(LCDFixture, defaultConstructor) {
  /* Test default constructor to ensure that vocabulary is loaded correctly. */
  EXPECT_GT(default_lcd_detector_.getVocab().size(), 0); // TODO: decide on a vocab and make this an EXPECT_EQ for that exact number.
}

TEST_F(LCDFixture, initLoopDetector) {
  /* Test parameter finalization and OrbLoopDetector initialization */
  lcd_params_ptr_->focal_length_ = 50;
  lcd_params_ptr_->principle_point_ = cv::Point2d(0,0);
  default_lcd_detector_.initLoopDetector();

  EXPECT_EQ(lcd_params_ptr_->focal_length_, 50);
  EXPECT_EQ(lcd_params_ptr_->principle_point_,
      cv::Point2d(0,0));
  EXPECT_EQ(default_lcd_detector_.getLoopDetector().getVocabulary(),
      default_lcd_detector_.getVocab());
  EXPECT_GT(default_lcd_detector_.getLoopDetector().getVocabulary(), 0);
}

TEST_F(LCDFixture, extractOrb) {
  /* Test whether size of extractOrb() keypoints and descriptors are correct. */
  std::vector<cv::Keypoint> keypoints;
  std::vector<cv::Mat> descriptors;
  default_lcd_detector_.extractOrb(ref_frame_->img_, keypoints, descriptors);

  EXPECT_GT(keypoints.size(), 0);
  EXPECT_GT(descriptors.size(), 0);
  EXPECT_EQ(keypoints.size(), lcd_params_ptr_->nfeatures_); // TODO: is this right?
  EXPECT_EQ(descriptors.size(), lcd_params_ptr_->nfeatures_);

  /* Test the keypoints and descriptors obtained from extractOrb */
}

// TODO: need some kind of ground truth for this test
TEST_F(LCDFixture, rewriteStereoFrameFeatures) {
  /* Test simple ORB feature repopulation */

  /* Test accurate field population after sparseStereoMatching */
}

TEST_F(LCDFixture, calcScaledRelativePose) {
  /* Test for correct scaled relative pose for identical frame poses */
  DLoopDetector::DetectionResult identity_loop_result;
  identity_loop_result.status = DLoopDetector::LOOP_DETECTED;
  identity_loop_result.query = id_ref_;
  identity_loop_result.match = id_ref_;
  identity_loop_result.translation = cv::Mat(1,3,cv::CV_64F, double(0));
  identity_loop_result.rotation = cv::eye(3,3,cv::CV_64F);

  gtsam::Pose3 trivial_relative_pose =
      default_lcd_detector_.calcScaledRelativePose(ref_stereo_frame_,
          identity_loop_result_);

  EXPECT_EQ(trivial_relative_pose.translation(), gtsam::Point3(0,0,0));
  EXPECT_EQ(trivial_relative_pose.rotation(), gtsam::Rot3::identity());

  /* Test for correct scaled relative pose for translation under unit length */
  // idea: create a StereoFrame from left image and a StereoFrame from right
  // image. Then, since you know the baseline you can call that the transform.

  /* Test for correct scaled relative pose for translation over unit length */

  /* Test for correct scaled relative pose for rotation and translation */

}

// NOTE: This probably won't work because temporal matching won't work with
// only two frames! Probably don't need to unit test this anyway if you've
// already tested the other components... (that might not be true...)
TEST_F(LCDFixture, checkLoopClosure) {
  /* Test the checkLoopClosure method against two images without closure */
  LoopClosureDetectorOutputPayload output_payload_0 =
      default_lcd_detector_.checkLoopClosure(extra_stereo_frame_);
      EXPECT_EQ(output_payload_0.is_loop_, false);

  LoopClosureDetectorOutputPayload output_payload_1 =
      default_lcd_detector_.checkLoopClosure(ref_stereo_frame_);
  EXPECT_EQ(output_payload_1.is_loop_, false);

  /* Test the checkLoopClosure method against two images that are identical */

  LoopClosureDetectorOutputPayload output_payload_2 =
      default_lcd_detector_.checkLoopClosure(ref_stereo_frame_);
  EXPECT_EQ(output_payload_2.is_loop_, true);
  EXPECT_EQ(output_payload_2.timestamp_kf_, timestamp_ref_);
  EXPECT_EQ(output_payload_2.id_recent_, id_ref_);
  EXPECT_EQ(output_payload_2.id_match_, id_ref_);
  EXPECT_EQ(output_payload_2.relative_pose_, gtsam::Pose3(
      gtsam::Rot3::identity(), gtsam::Point3(0,0,0)));

  /* Test the checkLoopClosure method against two unidentical, similar images */

  gtsam::Pose3 ground_truth_rel_pose; // get from images somehow
  LoopClosureDetectorOutputPayload output_payload_3 =
      default_lcd_detector_.checkLoopClosure(cur_stereo_frame_);
  EXPECT_EQ(output_payload_3.is_loop_, true);
  EXPECT_EQ(output_payload_3.timestamp_kf_, timestamp_cur_);
  EXPECT_EQ(output_payload_3.id_recent_, id_cur_);
  EXPECT_EQ(output_payload_3.id_match_, id_ref_);
  EXPECT_EQ(output_payload_3.relative_pose_, ref_to_cur_pose_); // TODO: introduce a tolerance?
}
