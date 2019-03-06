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
 * @author Luca Carlone
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <random>
#include <algorithm>

#include "Frame.h"
#include "StereoFrame.h"
#include "Tracker.h"
#include "StereoVisionFrontEnd.h"
#include "CameraParams.h"
#include "test_config.h"

// Add last, since it redefines CHECK, which is first defined by glog.
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;
using namespace VIO;
using namespace cv;

/* ************************************************************************* */
// Data
static const double tol = 1e-7;
static Frame *ref_frame, *cur_frame;
static StereoFrame *ref_stereo_frame, *cur_stereo_frame;
static const FrameId id_ref = 0, id_cur = 1;
static const int64_t timestamp_ref = 1000, timestamp_cur = 2000;
static string stereo_dataset_path(DATASET_PATH +
    string("/ForStereoFrame/"));
static const int repeat_times = 1;
static const VioFrontEndParams trackerParams;

/* ************************************************************************* */
// Helper function
void InitializeData() {
  CameraParams cam_params_left, cam_params_right;
  cam_params_left.parseYAML(stereo_dataset_path + "/sensorLeft.yaml");
  cam_params_right.parseYAML(stereo_dataset_path + "/sensorRight.yaml");

  string img_name_ref_left = stereo_dataset_path + "left_img_0.png";
  string img_name_ref_right = stereo_dataset_path + "right_img_0.png";

  string img_name_cur_left = stereo_dataset_path + "left_img_1.png";
  string img_name_cur_right = stereo_dataset_path + "right_img_1.png";

  // Data for testing "geometricOutlierRejectionMono"
  ref_frame = new Frame(id_ref, timestamp_ref,
                        cam_params_left,
                        UtilsOpenCV::ReadAndConvertToGrayScale(img_name_ref_left));
  cur_frame = new Frame(id_cur, timestamp_cur,
                        cam_params_left,
                        UtilsOpenCV::ReadAndConvertToGrayScale(img_name_cur_left));

  // Data for testing "geometricOutlilerRejectionStereo"
  Pose3 camL_Pose_camR = cam_params_left.body_Pose_cam_.between(cam_params_right.body_Pose_cam_);

  VioFrontEndParams tp;

  ref_stereo_frame = new StereoFrame(
        id_ref, timestamp_ref,
        UtilsOpenCV::ReadAndConvertToGrayScale(
          img_name_ref_left,
          tp.getStereoMatchingParams().equalize_image_),
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
          img_name_ref_right,
          tp.getStereoMatchingParams().equalize_image_),
        cam_params_right,
        camL_Pose_camR, tp.getStereoMatchingParams());

  cur_stereo_frame = new StereoFrame(
        id_cur, timestamp_cur,
        UtilsOpenCV::ReadAndConvertToGrayScale(
          img_name_cur_left,
          tp.getStereoMatchingParams().equalize_image_),
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
          img_name_cur_right,
          tp.getStereoMatchingParams().equalize_image_),
        cam_params_right,
        camL_Pose_camR, tp.getStereoMatchingParams());

  // Set randomness!
  srand(0);
}
/* ------------------------------------------------------------------------- */
void ClearFrame(Frame* f) {
  f->keypoints_.clear();
  f->landmarks_.clear();
  f->landmarksAge_.clear();
  f->versors_.clear();
}
/* ------------------------------------------------------------------------- */
void ClearStereoFrame(StereoFrame* sf) {
  ClearFrame(sf->getLeftFrameMutable());
  ClearFrame(sf->getRightFrameMutable());
  sf->keypoints_3d_.clear();
  sf->keypoints_depth_.clear();
  sf->right_keypoints_status_.clear();
}
/* ------------------------------------------------------------------------- */
void FillStereoFrame(StereoFrame* sf) {
  // Fill the fields in a StereoFrame to pass the sanity check
  // StereoFrame::checkStereoFrame
  const int num_keypoints = sf->getLeftFrame().landmarks_.size();

  // left.y == right.y
  // keypoints_3d[i](2) == keypoints_depth_[i]
  // (right_keypoints_status[i] == valid && right_frame_.keypoints_[i] != 0)
  // OR (right_keypoints_status[i] != valid && keypoints_depth_[i] == 0)

  // right_keypoints_status_.size
  if (sf->right_keypoints_status_.size() != num_keypoints) {
    sf->right_keypoints_status_ = vector<Kstatus>(num_keypoints);
    for (int i = 0; i < num_keypoints; i++) {
      sf->right_keypoints_status_[i] = Kstatus::VALID;
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
      if (sf->right_keypoints_status_[i] == Kstatus::VALID) {
        sf->getRightFrameMutable()->keypoints_[i] = KeypointCV(i + 20, i + (i % 3 - 1));
      } else {
        sf->getRightFrameMutable()->keypoints_[i] = KeypointCV(0, 0);
      }

    }
  }

  // keypoints_depth_.size
  if (sf->keypoints_depth_.size() != num_keypoints) {
    sf->keypoints_depth_ = vector<double>(num_keypoints);
    for (int i = 0; i < num_keypoints; i++) {
      if (sf->right_keypoints_status_[i] == Kstatus::VALID) {
        sf->keypoints_depth_[i] = 1;
      } else {
        sf->keypoints_depth_[i] = 0;
      }
    }
  }

  // keypoints_3d_.size
  if (sf->keypoints_3d_.size() != num_keypoints) {
    sf->keypoints_3d_ = vector<Vector3>(num_keypoints);
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

/* ------------------------------------------------------------------------- */
vector<Point2> LoadCorners(const string filepath) {
  vector<Point2> corners;
  ifstream fin(filepath);
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
/* ------------------------------------------------------------------------- */
vector<double> LoadDepth(const string filepath) {
  vector<double> depth;
  ifstream fin(filepath);
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
/* ------------------------------------------------------------------------- */
int FindCorners(const Point2 pt_query,
    const vector<Point2> &pt_set) {
  for (int i = 0; i < pt_set.size(); i++) {
    if (pt_set[i].equals(pt_query, 3)) return i;
  }
  return -1;
}
/* ------------------------------------------------------------------------- */
vector<Point2> ConvertCornersAcrossCameras(
      const vector<Point2> &corners_in,
      const Cal3DS2 &calib_in, const Cal3_S2 &calib_out) {
  if (abs(calib_in.skew()) > 1e-7 || abs(calib_out.skew()) > 1e-7) {
    throw runtime_error("Skew of the calibration is non-zero!");
  }
  vector<Point2> corners_out;
  corners_out.reserve(corners_in.size());
  for (int i = 0; i < corners_in.size(); i++) {
    double xn = (corners_in[i].x() - calib_in.px() ) / calib_in.fx();
    double yn = (corners_in[i].y() - calib_in.py() ) / calib_in.fy();
    double xo = xn * calib_out.fx() + calib_out.px();
    double yo = yn * calib_out.fy() + calib_out.py();
    corners_out.push_back(Point2(xo, yo));
  }
  return corners_out;
}
/* ************************************************************************* */
TEST(testStereoVisionFrontEnd, getRelativePoseBodyMono) {
  // How to test this in the simplest possible way?
  // Compute everything twice?
  ref_stereo_frame->setIsKeyframe(true);
  cur_stereo_frame->setIsKeyframe(true);

  StereoVisionFrontEnd st;
  st.stereoFrame_lkf_ = make_shared<StereoFrame>(*ref_stereo_frame);
  st.trackerStatusSummary_.lkf_T_k_mono_ = Pose3(
      Rot3::Expmap(Vector3(0.1, -0.1, 0.2)), Vector3(0.1, 0.1, 0.1));
  ref_stereo_frame->setIsRectified(true);
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
TEST(testStereoVisionFrontEnd, getRelativePoseBodyStereo) {
  // How to test this in the simplest possible way?
  // Compute everything twice?
  ref_stereo_frame->setIsKeyframe(true);

  StereoVisionFrontEnd st;
  st.stereoFrame_lkf_ = make_shared<StereoFrame>(*ref_stereo_frame);
  st.trackerStatusSummary_.lkf_T_k_stereo_ = Pose3(
      Rot3::Expmap(Vector3(0.1, -0.1, 0.2)), Vector3(0.1, 0.1, 0.1));
  ref_stereo_frame->setIsRectified(true);
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
TEST(testStereoVisionFrontEnd, getSmartStereoMeasurements) {
  ClearStereoFrame(ref_stereo_frame);
  ref_stereo_frame->setIsKeyframe(true);
  ref_stereo_frame->setIsRectified(true);

  StereoVisionFrontEnd st;

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
    ref_stereo_frame->right_keypoints_status_.push_back(Kstatus::VALID);
  }

  // right keypoints invalid!
  for (int i = 0; i < num_right_missing; i++) {
    double uL = rand() % 800;
    double uR = uL + (rand() % 80 - 40);
    double v = rand() % 600;
    ref_stereo_frame->getLeftFrameMutable()->landmarks_.push_back(i + num_valid);
    ref_stereo_frame->getLeftFrameMutable()->scores_.push_back(1.0);
    ref_stereo_frame->left_keypoints_rectified_.push_back(cv::Point2f(uL, v));
    ref_stereo_frame->right_keypoints_rectified_.push_back(cv::Point2f(uL, v));
    ref_stereo_frame->right_keypoints_status_.push_back(Kstatus::NO_RIGHT_RECT);
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
    ref_stereo_frame->right_keypoints_status_.push_back(Kstatus::VALID);
  }

  FillStereoFrame(ref_stereo_frame);
  // Call the function!
  SmartStereoMeasurements ssm;
  ssm = st.getSmartStereoMeasurements(*ref_stereo_frame);

  // Verify the correctness of the results!
  EXPECT(ssm.size() == num_valid + num_right_missing);
  set<int> landmark_set;

  for (auto s : ssm) {
    // No order requirement for the entries in ssm.
    // To avoid searching for the location for a landmark, the data is synthesized
    // following a simple convention:
    //         landmark_[i] = i; for landmark_[i] != -1;
    int landmark_id = s.first;
    EXPECT(ref_stereo_frame->left_keypoints_rectified_[landmark_id].x
        == s.second.uL());
    EXPECT(ref_stereo_frame->left_keypoints_rectified_[landmark_id].y
        == s.second.v());
    if (ref_stereo_frame->right_keypoints_status_[landmark_id]
                                                  == Kstatus::VALID) {
      EXPECT(ref_stereo_frame->right_keypoints_rectified_[landmark_id].x
          == s.second.uR());
    } else {
      EXPECT(isnan(s.second.uR()));
    }


    // Verify that there is no replicated entries in landmark_set.
    EXPECT(landmark_set.find(landmark_id) == landmark_set.end());
    landmark_set.insert(landmark_id);
  }

}
/* ************************************************************************* */
TEST(testStereoVisionFrontEnd, processFirstFrame) {
  // Things to test:
  // 1. Feature detection (from tracker)
  // 2. results from sparseStereoMatching

  // Load a synthetic stereo pair from MATLAB with known ground-truth
  CameraParams cam_params_left, cam_params_right;
  string matlab_syn_path(DATASET_PATH + string("/ForStereoTracker"));
  cam_params_left.parseYAML(matlab_syn_path + "/camLeft.yaml");
  cam_params_right.parseYAML(matlab_syn_path + "/camRight.yaml");

  string img_name_left = matlab_syn_path + "/img_distort_left.png";
  string img_name_right = matlab_syn_path + "/img_distort_right.png";

  Pose3 camL_Pose_camR = cam_params_left.body_Pose_cam_.between(
      cam_params_right.body_Pose_cam_);

  // cout << "camL_Pose_camR = " << camL_Pose_camR << endl;
  // Assert certain properties of the synthetic data
  // Same intrinsics
  for (int i = 0; i < 4; i++) {
    EXPECT(cam_params_left.intrinsics_[i] == cam_params_right.intrinsics_[i]);
  }

  // Already rectified
  Vector3 T = camL_Pose_camR.translation();
  EXPECT(T(0) > 0);
  EXPECT_DOUBLES_EQUAL(0, T(1), 1e-4);
  EXPECT_DOUBLES_EQUAL(0, T(2), 1e-4);

  // Strangely, StereoFrame.cpp requires that the baseline is between 0.1,
  // and 0.12. Therefore, I have to uniformly scale the whole 3D scene to
  // meet this requirement.
  double scale = T(0) / 0.11;
  camL_Pose_camR = Pose3(camL_Pose_camR.rotation(),
      camL_Pose_camR.translation() / scale);

  VioFrontEndParams p = VioFrontEndParams(); // default
  p.min_distance_ = 0.05;
  p.quality_level_ = 0.1;

  StereoFrame first_stereo_frame(0, 0,
                                 UtilsOpenCV::ReadAndConvertToGrayScale(
                                   img_name_left,
                                   p.getStereoMatchingParams().equalize_image_),
                                 cam_params_left,
                                 UtilsOpenCV::ReadAndConvertToGrayScale(
                                   img_name_right,
                                   p.getStereoMatchingParams().equalize_image_),
                                 cam_params_right,
                                 camL_Pose_camR,
                                 p.getStereoMatchingParams());

  first_stereo_frame.getLeftFrameMutable()->cam_param_.body_Pose_cam_ = Pose3(
      first_stereo_frame.getLeftFrame().cam_param_.body_Pose_cam_.rotation(),
      first_stereo_frame.getLeftFrame().cam_param_.body_Pose_cam_.translation()
      / scale);

  first_stereo_frame.getRightFrameMutable()->cam_param_.body_Pose_cam_ = Pose3(
      first_stereo_frame.getRightFrame().cam_param_.body_Pose_cam_.rotation(),
      first_stereo_frame.getRightFrame().cam_param_.body_Pose_cam_.translation()
      / scale);

  // Load the expected corners
  vector<Point2> left_distort_corners = LoadCorners(
        matlab_syn_path + "/corners_normal_left.txt");

  // Call StereoVisionFrontEnd::Process first frame!
  StereoVisionFrontEnd st(p);
  st.processFirstStereoFrame(first_stereo_frame);

  // Check the following results:
  // 1. Feature Detection
  // 2. SparseStereoMatching

  // Check feature detection results!
  // landmarks_, landmarksAge_, keypoints_, versors_
  const Frame& left_frame = st.stereoFrame_km1_->getLeftFrame();
  const int num_corners = left_frame.landmarks_.size();
  EXPECT(num_corners == left_frame.landmarksAge_.size());
  EXPECT(num_corners == left_frame.keypoints_.size());
  EXPECT(num_corners == left_frame.versors_.size());
  for (auto lm : left_frame.landmarksAge_) {
    EXPECT(lm == 1);
  }
  for (auto lid : left_frame.landmarks_) {
    EXPECT(lid >= 0);
  }

  vector<int> corner_id_map_frame2gt; // useful for the tests later
  corner_id_map_frame2gt.reserve(num_corners);
  for (int i = 0; i < num_corners; i++) {
    KeypointCV pt_cv = left_frame.keypoints_[i];
    Point2 pt(pt_cv.x, pt_cv.y);
    int idx = FindCorners(pt, left_distort_corners);
    EXPECT(idx >= 0);
    corner_id_map_frame2gt.push_back(idx);
  }
  for (int i = 0; i < num_corners; i++) {
    Vector3 v_expect = Frame::CalibratePixel(left_frame.keypoints_[i],
        left_frame.cam_param_);
    Vector3 v_actual = left_frame.versors_[i];
    EXPECT((v_actual - v_expect).norm() < 0.1);
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
  shared_ptr<StereoFrame> sf = st.stereoFrame_km1_;
  EXPECT(sf->isKeyframe());
  EXPECT(sf->isRectified());

  // left_keypoints_rectified!
  vector<Point2> left_undistort_corners = LoadCorners(
      matlab_syn_path + "/corners_undistort_left.txt");
  vector<Point2> left_rect_corners = ConvertCornersAcrossCameras(
      left_undistort_corners,
      sf->getLeftFrame().cam_param_.calibration_,
      sf->getLeftUndistRectCamMat());
  for (int i = 0; i < num_corners; i++) {
    int idx_gt = corner_id_map_frame2gt[i];
    Point2 pt_expect = left_rect_corners[idx_gt];
    Point2 pt_actual = Point2(
        sf->left_keypoints_rectified_[i].x,
        sf->left_keypoints_rectified_[i].y);
    EXPECT(assert_equal(pt_expect, pt_actual, 2));
  }

  // right_keypoints_rectified
  vector<Point2> right_undistort_corners = LoadCorners(
      matlab_syn_path + "/corners_undistort_right.txt");
  vector<Point2> right_rect_corners = ConvertCornersAcrossCameras(
      right_undistort_corners,
      sf->getRightFrame().cam_param_.calibration_,
      sf->getRightUndistRectCamMat());
  for (int i = 0; i < num_corners; i++) {
    int idx_gt = corner_id_map_frame2gt[i];
    Point2 pt_expect = right_rect_corners[idx_gt];
    Point2 pt_actual = Point2(
        sf->right_keypoints_rectified_[i].x,
        sf->right_keypoints_rectified_[i].y);
    EXPECT(assert_equal(pt_expect, pt_actual, 2));
  }

  // right_keypoints_status_
  for (auto status : sf->right_keypoints_status_) {
    EXPECT(status == Kstatus::VALID);
  }

  // keypoints depth
  vector<double> depth_gt = LoadDepth(
      matlab_syn_path + "/depth_left.txt");

  for (int i = 0; i < num_corners; i++) {
    int idx_gt = corner_id_map_frame2gt[i];
    double depth_expect = depth_gt[idx_gt] / scale;
    double depth_actual = sf->keypoints_depth_[i];
    EXPECT_DOUBLES_EQUAL(depth_expect, depth_actual, 1e-2);
  }

  // keypoints 3d
  for (int i = 0; i < num_corners; i++) {
    int idx_gt = corner_id_map_frame2gt[i];
    double depth_expect = depth_gt[idx_gt] / scale;
    double depth_actual = sf->keypoints_depth_[i];
    Vector3 v_expected = Frame::CalibratePixel(
        KeypointCV(left_distort_corners[idx_gt].x(),
            left_distort_corners[idx_gt].y()),
            left_frame.cam_param_);
    v_expected =  v_expected * (depth_gt[idx_gt] / scale);
    Vector3 v_actual = sf->keypoints_3d_[i];
    EXPECT((v_expected - v_actual).norm() < 0.1);
  }
}

/* ************************************************************************* */
int main() {
  // Initialize the data!
  InitializeData();

  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
