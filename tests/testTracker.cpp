/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testTracker.cpp
 * @brief  test Tracker
 * @author Antoni Rosinol, Luca Carlone
 */

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/frontend/Tracker.h"

DECLARE_string(test_data_path);

using namespace gtsam;
using namespace std;
using namespace VIO;
using namespace cv;

/* ************************************************************************** */
// Testing data
static const double tol = 1e-7;
// TODO initializing null pointers is prone to errors!!
// TODO use test fixture!!
static Frame *ref_frame, *cur_frame;
static StereoFrame *ref_stereo_frame, *cur_stereo_frame;
static const FrameId id_ref = 0, id_cur = 1;
static const int64_t timestamp_ref = 1000, timestamp_cur = 2000;
static string stereo_test_data_path(FLAGS_test_data_path +
                                    string("/ForStereoFrame/"));
static const int repeat_times = 1;

class TestTracker : public ::testing::Test {
 protected:
  void SetUp() override {
    // Fix randomization seed (we use monte carlo runs here).
    srand(3);
    InitializeData();
  }

  // TODO deallocate dynamic memory here!
  // void TearDown() override {}

  void InitializeData() {
    CameraParams cam_params_left, cam_params_right;
    cam_params_left.parseYAML(stereo_test_data_path + "/sensorLeft.yaml");
    cam_params_right.parseYAML(stereo_test_data_path + "/sensorRight.yaml");

    string img_name_ref_left = stereo_test_data_path + "left_img_0.png";
    string img_name_ref_right = stereo_test_data_path + "right_img_0.png";

    string img_name_cur_left = stereo_test_data_path + "left_img_1.png";
    string img_name_cur_right = stereo_test_data_path + "right_img_1.png";

    // Data for testing "geometricOutlierRejectionMono"
    // !!! TODO THIS IS ALLOCATING MEMORY BUT THERE IS NO DELETE !!!
    ref_frame =
        new Frame(id_ref, timestamp_ref, cam_params_left,
                  UtilsOpenCV::ReadAndConvertToGrayScale(img_name_ref_left));
    cur_frame =
        new Frame(id_cur, timestamp_cur, cam_params_left,
                  UtilsOpenCV::ReadAndConvertToGrayScale(img_name_cur_left));

    VioFrontEndParams tp;

    ref_stereo_frame = new StereoFrame(
        id_ref,
        timestamp_ref,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_ref_left, tp.stereo_matching_params_.equalize_image_),
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_ref_right, tp.stereo_matching_params_.equalize_image_),
        cam_params_right,
        tp.stereo_matching_params_);

    ref_stereo_frame->sparseStereoMatching();  // only initialize rectification

    cur_stereo_frame = new StereoFrame(
        id_cur,
        timestamp_cur,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_cur_left, tp.stereo_matching_params_.equalize_image_),
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(
            img_name_cur_right, tp.stereo_matching_params_.equalize_image_),
        cam_params_right,
        tp.stereo_matching_params_);

    cur_stereo_frame->sparseStereoMatching();  // only initialize rectification
  }

  Vector3 IntersectVersorPlane(const Vector3& versor, const Vector3& PlaneN,
                               const double PlaneD) {
    // intersect versor with N'*x=d;
    double t = PlaneD / versor.dot(PlaneN);
    return t * versor;
  }

  void AddKeypointsVersorsToFrames(Frame* f_ref, Frame* f_cur,
                                   KeypointCV& pt_ref, KeypointCV& pt_cur,
                                   Vector3& v_ref, Vector3& v_cur) {
    // Decide the largest landmark IDs for each frame!
    int max_id;
    if (f_ref->landmarks_.size() == 0 && f_cur->landmarks_.size() == 0) {
      max_id = 0;
    } else {
      vector<LandmarkId>::iterator max_id_ref =
          max_element(f_ref->landmarks_.begin(), f_ref->landmarks_.end());
      vector<LandmarkId>::iterator max_id_cur =
          max_element(f_cur->landmarks_.begin(), f_cur->landmarks_.end());
      max_id = max(*max_id_ref, *max_id_cur);
    }

    // Add the keypoints to the frames
    f_ref->keypoints_.push_back(pt_ref);
    f_cur->keypoints_.push_back(pt_cur);

    // Add the versors to the frames
    f_ref->versors_.push_back(v_ref);
    f_cur->versors_.push_back(v_cur);

    // Assign landmark ids to them!
    f_ref->landmarks_.push_back(max_id + 1);
    f_cur->landmarks_.push_back(max_id + 1);

    f_ref->landmarksAge_.push_back(0);
    f_cur->landmarksAge_.push_back(1);
  }

  void ClearFrame(Frame* f) {
    CHECK_NOTNULL(f);
    f->keypoints_.resize(0);
    f->scores_.resize(0);
    f->landmarks_.resize(0);
    f->landmarksAge_.resize(0);
    f->versors_.resize(0);
  }

  void AddNoiseToFrame(Frame* f, const double noise_sigma) {
    CHECK_NOTNULL(f);
    default_random_engine generator;
    normal_distribution<double> distribution(0, noise_sigma);
    for (auto versor : f->versors_) {
      versor =
          versor + Vector3(distribution(generator), distribution(generator),
                           distribution(generator));
    }
  }

  void AddPlanarInliersToFrame(Frame* f_ref, Frame* f_cur,
                               const Pose3 camRef_pose_camCur,
                               const Vector3& PlaneN, const double PlaneD,
                               const int inlierNum) {
    CHECK_NOTNULL(f_ref);
    CHECK_NOTNULL(f_cur);
    // All points are on the plane: PlaneN * x = d
    for (int i = 0; i < inlierNum; i++) {
      // Randomly synthesize the point!
      KeypointCV pt_ref(rand() % f_ref->img_.cols, rand() % f_ref->img_.rows);
      // Calibrate the point
      Vector3 versor_ref = Frame::calibratePixel(pt_ref, f_ref->cam_param_);

      // Compute the intersection between the versor and the plane.
      Vector3 versor_plane = IntersectVersorPlane(versor_ref, PlaneN, PlaneD);

      // project to the current frame!
      Vector3 versor_cur =
          camRef_pose_camCur.inverse().rotation() * versor_plane +
          camRef_pose_camCur.inverse().translation();
      versor_cur = versor_cur / versor_cur.norm();

      Point2 pt_cur_gtsam = f_ref->cam_param_.distortion_->uncalibrate(
          Point2(versor_cur[0] / versor_cur[2], versor_cur[1] / versor_cur[2]));
      KeypointCV pt_cur(pt_cur_gtsam.x(), pt_cur_gtsam.y());

      AddKeypointsVersorsToFrames(f_ref, f_cur, pt_ref, pt_cur, versor_ref,
                                  versor_cur);
    }
  }

  void AddNonPlanarInliersToFrame(Frame* f_ref, Frame* f_cur,
                                  const Pose3 camRef_pose_camCur,
                                  const vector<double>& depth_range,
                                  const int inlierNum) {
    for (int i = 0; i < inlierNum; i++) {
      CHECK_NOTNULL(f_ref);
      CHECK_NOTNULL(f_cur);
      // Randomly syntheisze the point!
      KeypointCV pt_ref(rand() % f_ref->img_.cols, rand() % f_ref->img_.rows);

      // Calibrate the point
      Vector3 versor_ref = Frame::calibratePixel(pt_ref, f_ref->cam_param_);

      // Randomly generate the depth
      double depth = depth_range[0] + (depth_range[1] - depth_range[0]) *
                                          ((double)rand() / RAND_MAX);

      // project to the current frame!
      Vector3 versor_cur =
          camRef_pose_camCur.inverse().rotation() * (versor_ref * depth) +
          camRef_pose_camCur.inverse().translation();
      versor_cur = versor_cur / versor_cur.norm();

      Point2 pt_cur_gtsam = f_ref->cam_param_.distortion_->uncalibrate(
          Point2(versor_cur[0] / versor_cur[2], versor_cur[1] / versor_cur[2]));
      KeypointCV pt_cur(pt_cur_gtsam.x(), pt_cur_gtsam.y());

      AddKeypointsVersorsToFrames(f_ref, f_cur, pt_ref, pt_cur, versor_ref,
                                  versor_cur);
    }
  }

  void AddOutliersToFrame(Frame* f_ref, Frame* f_cur, Pose3 camRef_pose_camCur,
                          int outlierNum) {
    for (int i = 0; i < outlierNum; i++) {
      while (true) {
        CHECK_NOTNULL(f_ref);
        CHECK_NOTNULL(f_cur);
        // Keypoints
        KeypointCV pt_ref(rand() % f_ref->img_.cols, rand() % f_ref->img_.rows);
        KeypointCV pt_cur(rand() % f_cur->img_.cols, rand() % f_cur->img_.rows);

        // Calibrate keypoints
        Vector3 versor_ref = Frame::calibratePixel(pt_ref, f_ref->cam_param_);
        Vector3 versor_cur = Frame::calibratePixel(pt_cur, f_cur->cam_param_);

        // Check that they are indeed outliers!
        double depth = camRef_pose_camCur.translation().norm();
        Vector3 versor_proj =
            camRef_pose_camCur.inverse().rotation() * (versor_ref * depth) +
            camRef_pose_camCur.inverse().translation();
        versor_proj = versor_proj / versor_proj.norm();
        double cos_angle = versor_proj.dot(versor_cur);
        const double cos_angle_tol = 0.9;
        if (cos_angle > cos_angle_tol) {
          // randomized "inliers"
          continue;
        } else {
          // true outliers
          AddKeypointsVersorsToFrames(f_ref, f_cur, pt_ref, pt_cur, versor_ref,
                                      versor_cur);
          break;
        }
      }
    }
  }

  void AddVersorsToStereoFrames(StereoFrame* sf_ref, StereoFrame* sf_cur,
                                Vector3& v_ref, Vector3& v_cur) {
    CHECK_NOTNULL(sf_ref);
    CHECK_NOTNULL(sf_cur);
    // Decide the largest landmark IDs for each frame!
    int max_id;
    if (sf_ref->getLeftFrame().landmarks_.size() == 0 &&
        sf_cur->getLeftFrame().landmarks_.size() == 0) {
      max_id = 0;
    } else {
      vector<LandmarkId>::const_iterator max_id_ref =
          max_element(sf_ref->getLeftFrame().landmarks_.begin(),
                      sf_ref->getLeftFrame().landmarks_.end());
      vector<LandmarkId>::const_iterator max_id_cur =
          max_element(sf_cur->getLeftFrame().landmarks_.begin(),
                      sf_cur->getLeftFrame().landmarks_.end());
      max_id = max(*max_id_ref, *max_id_cur);
    }

    // Add the versors to the stereo frames
    sf_ref->keypoints_3d_.push_back(v_ref);
    sf_cur->keypoints_3d_.push_back(v_cur);

    // create ref stereo camera
    Rot3 camLrect_R_camL = UtilsOpenCV::cvMatToGtsamRot3(
        sf_ref->getLeftFrame().cam_param_.R_rectify_);
    Cal3_S2 ref_left_undist_rect_cam_mat = sf_ref->getLeftUndistRectCamMat();
    Cal3_S2Stereo::shared_ptr K_ref(new Cal3_S2Stereo(
        ref_left_undist_rect_cam_mat.fx(), ref_left_undist_rect_cam_mat.fy(),
        ref_left_undist_rect_cam_mat.skew(), ref_left_undist_rect_cam_mat.px(),
        ref_left_undist_rect_cam_mat.py(), sf_ref->getBaseline()));
    StereoCamera stereoCam = StereoCamera(Pose3(), K_ref);

    StereoPoint2 sp2 = stereoCam.project(camLrect_R_camL.rotate(Point3(v_ref)));
    sf_ref->left_keypoints_rectified_.push_back(KeypointCV(sp2.uL(), sp2.v()));
    sf_ref->right_keypoints_rectified_.push_back(KeypointCV(sp2.uR(), sp2.v()));

    // create cur stereo camera
    camLrect_R_camL = UtilsOpenCV::cvMatToGtsamRot3(
        sf_cur->getLeftFrame().cam_param_.R_rectify_);
    Cal3_S2 cur_left_undist_rect_cam_mat = sf_cur->getLeftUndistRectCamMat();
    Cal3_S2Stereo::shared_ptr K_cur(new Cal3_S2Stereo(
        cur_left_undist_rect_cam_mat.fx(), cur_left_undist_rect_cam_mat.fy(),
        cur_left_undist_rect_cam_mat.skew(), cur_left_undist_rect_cam_mat.px(),
        cur_left_undist_rect_cam_mat.py(), sf_cur->getBaseline()));
    stereoCam = StereoCamera(Pose3(), K_cur);

    sp2 = stereoCam.project(camLrect_R_camL.rotate(Point3(v_cur)));
    sf_cur->left_keypoints_rectified_.push_back(KeypointCV(sp2.uL(), sp2.v()));
    sf_cur->right_keypoints_rectified_.push_back(KeypointCV(sp2.uR(), sp2.v()));

    // depth!
    sf_ref->keypoints_depth_.push_back(v_ref.norm());
    sf_cur->keypoints_depth_.push_back(v_cur.norm());

    // Assign landmark ids to them!
    sf_ref->getLeftFrameMutable()->landmarks_.push_back(max_id + 1);
    sf_cur->getLeftFrameMutable()->landmarks_.push_back(max_id + 1);

    sf_ref->getLeftFrameMutable()->landmarksAge_.push_back(0);
    sf_cur->getLeftFrameMutable()->landmarksAge_.push_back(1);

    // Set sf->right_keypoints_status_
    sf_ref->right_keypoints_status_.push_back(KeypointStatus::VALID);
    sf_cur->right_keypoints_status_.push_back(KeypointStatus::VALID);
  }

  void AddNonPlanarInliersToStereoFrame(StereoFrame* sf_ref,
                                        StereoFrame* sf_cur,
                                        const Pose3 camLeftRef_pose_camLeftCur,
                                        const vector<double>& depth_range,
                                        const int inlierNum) {
    CHECK_NOTNULL(sf_ref);
    CHECK_NOTNULL(sf_cur);
    for (int i = 0; i < inlierNum; i++) {
      // Randomly synthesize left keypoint!
      KeypointCV pt_ref(rand() % sf_ref->getLeftFrame().img_.cols,
                        rand() % sf_ref->getLeftFrame().img_.rows);

      // Calibrate the point
      Vector3 versor_ref =
          Frame::calibratePixel(pt_ref, sf_ref->getLeftFrame().cam_param_);
      // Randomly generate the depth
      double depth = depth_range[0] + (depth_range[1] - depth_range[0]) *
                                          ((double)rand() / RAND_MAX);

      versor_ref = versor_ref * depth;

      // inverse transformation transforms point to the current frame!
      Vector3 versor_cur =
          camLeftRef_pose_camLeftCur.inverse().rotation() * versor_ref +
          camLeftRef_pose_camLeftCur.inverse()
              .translation();  // versor_cur = R' * (versor_ref - T)
      // also: R*versor_cur - versor_ref = T

      AddVersorsToStereoFrames(sf_ref, sf_cur, versor_ref, versor_cur);
    }
  }

  void ClearStereoFrame(StereoFrame* sf) {
    CHECK_NOTNULL(sf);
    ClearFrame(sf->getLeftFrameMutable());
    ClearFrame(sf->getRightFrameMutable());
    sf->keypoints_3d_.clear();
    sf->keypoints_depth_.clear();  // Might not be needed for this test?
    sf->right_keypoints_status_.clear();
    sf->left_keypoints_rectified_.clear();
    sf->right_keypoints_rectified_.clear();
  }

  void AddOutliersToStereoFrame(StereoFrame* sf_ref, StereoFrame* sf_cur,
                                Pose3 camRef_pose_camCur,
                                vector<double>& depth_range, int outlierNum) {
    CHECK_NOTNULL(sf_ref);
    CHECK_NOTNULL(sf_cur);
    for (int i = 0; i < outlierNum; i++) {
      while (true) {
        // Keypoints
        KeypointCV pt_ref(rand() % sf_ref->getLeftFrame().img_.cols,
                          rand() % sf_ref->getLeftFrame().img_.rows);
        KeypointCV pt_cur(rand() % sf_cur->getLeftFrame().img_.cols,
                          rand() % sf_cur->getLeftFrame().img_.rows);

        // Calibrate keypoints
        Vector3 versor_ref =
            Frame::calibratePixel(pt_ref, sf_ref->getLeftFrame().cam_param_);
        Vector3 versor_cur =
            Frame::calibratePixel(pt_cur, sf_cur->getLeftFrame().cam_param_);

        // Check that they are indeed outliers!
        double depth_ref =
            depth_range[0] + (depth_range[1] - depth_range[0]) *
                                 (((double)rand()) / ((double)RAND_MAX));
        double depth_cur =
            depth_range[0] + (depth_range[1] - depth_range[0]) *
                                 (((double)rand()) / ((double)RAND_MAX));

        versor_ref = versor_ref * depth_ref;
        versor_cur = versor_cur * depth_cur;
        Vector3 versor_proj =
            camRef_pose_camCur.inverse().rotation() * versor_ref +
            camRef_pose_camCur.inverse().translation();
        double cos_angle = versor_proj.dot(versor_cur) / versor_proj.norm() /
                           versor_cur.norm();
        const double cos_angle_tol = 0.9;
        if (cos_angle > cos_angle_tol) {
          // randomized "inliers"
          continue;
        } else {
          // true outliers
          AddVersorsToStereoFrames(sf_ref, sf_cur, versor_ref, versor_cur);
          break;
        }
      }
    }
  }

  void AddPlanarInliersToStereoFrame(StereoFrame* sf_ref, StereoFrame* sf_cur,
                                     const Pose3 camLeftRef_pose_camLeftCur,
                                     const Vector3& PlaneN, const double PlaneD,
                                     const int inlierNum) {
    CHECK_NOTNULL(sf_ref);
    CHECK_NOTNULL(sf_cur);
    // All points are on the plane: PlaneN * x = d
    for (int i = 0; i < inlierNum; i++) {
      // Randomly synthesize the point!
      KeypointCV pt_ref(rand() % sf_ref->getLeftFrame().img_.cols,
                        rand() % sf_ref->getLeftFrame().img_.rows);
      // Calibrate the point
      Vector3 versor_ref =
          Frame::calibratePixel(pt_ref, sf_ref->getLeftFrame().cam_param_);

      // Compute the intersection between the versor and the plane.
      Vector3 versor_plane = IntersectVersorPlane(versor_ref, PlaneN, PlaneD);

      // project to the current frame!
      Vector3 versor_cur =
          camLeftRef_pose_camLeftCur.inverse().rotation() * versor_plane +
          camLeftRef_pose_camLeftCur.inverse().translation();

      AddVersorsToStereoFrames(sf_ref, sf_cur, versor_plane, versor_cur);
    }
  }

  void AddNoiseToStereoFrame(StereoFrame* sf, const double noise_sigma) {
    CHECK_NOTNULL(sf);
    default_random_engine generator;
    normal_distribution<double> distribution(0, noise_sigma);
    for (auto versor : sf->keypoints_3d_) {
      Vector3 noise(distribution(generator), distribution(generator),
                    distribution(generator));
      versor = versor + noise;
    }
  }

  pair<Vector3, Matrix3> monteCarloSampleCovariance(
      const StereoCamera stereoCam, const StereoPoint2 stereoPoint,
      const Matrix3 stereoPtCov) {
    Vector3 meanVector = stereoCam.backproject2(stereoPoint).vector();
    Vector3 sampleMean = Vector3::Zero();

    default_random_engine generator;
    Matrix3 sampleCovariance = Matrix3::Zero();
    int nrRuns = 100000;
    for (int i = 0; i < nrRuns; i++) {  // monte carlo runs

      normal_distribution<double> pixelNoise(
          0.0,
          stereoPtCov(
              0, 0));  // 0 mean and suitable std (vioParams.smartNoiseSigma_)
      double noiseUL = pixelNoise(generator);
      double noiseUR = pixelNoise(generator);
      double noiseV = pixelNoise(generator);
      StereoPoint2 perturbedStereoPoint(stereoPoint.uL() + noiseUL,
                                        stereoPoint.uR() + noiseUR,
                                        stereoPoint.v() + noiseV);

      Vector3 sample_i = stereoCam.backproject2(perturbedStereoPoint).vector();
      sampleMean += sample_i;
      sampleCovariance +=
          (sample_i - meanVector) * (sample_i - meanVector).transpose();
    }
    sampleMean = sampleMean / double(nrRuns);
    sampleCovariance = sampleCovariance / double(nrRuns - 1);
    return make_pair(sampleMean, sampleCovariance);
  }
};

/* ************************************************************************* */
TEST_F(TestTracker, geometricOutlierRejectionMono) {
  // Start with the simplest case:
  // Noise free, no outlier, non-planar

  Rot3 R = Rot3::Expmap(Vector3(0.01, 0.01, 0.01));
  Vector3 T(1, 0, 0);
  Pose3 camRef_pose_camCur(R, T);

  // Factor 1: Noise level
  // Factor 2: Outlier ratio
  // Factor 3: Planar or Non-Planar

  // Generate Test Configurations
  // (is_planar, inlier_num, outlier_num, noise_sigma)
  vector<std::tuple<bool, int, int, double>> test_configurations;

  // Test case 1: Idea test, 3d rigid scene, 80 inliers, no outliers, no noise
  test_configurations.push_back(std::make_tuple(false, 82, 0, 0.0));
  // Test case 2: Noisy/Outlier test, 3d rigid scene, 80 inliers, 40 outliers,
  // noise_level 1
  test_configurations.push_back(std::make_tuple(false, 80, 40, 1));
  // Test case 3: Noisy/Outlier test, Planar scene, 80 inliers, 40 outliers,
  // noise_level 1
  test_configurations.push_back(std::make_tuple(true, 80, 40, 1));

  for (auto test_conf : test_configurations) {
    bool is_planar = get<0>(test_conf);
    int inlier_num = get<1>(test_conf);
    int outlier_num = get<2>(test_conf);
    double noise_sigma = get<3>(test_conf);
    for (int t = 0; t < repeat_times; t++) {
      ClearFrame(ref_frame);
      ClearFrame(cur_frame);

      VLOG(1) << "========================== \n"
              << "is_planar: " << is_planar << '\n'
              << " inlier_num: " << inlier_num << '\n'
              << " outlier_num: " << outlier_num << '\n'
              << " noise_sigma: " << noise_sigma;
      // add inliers
      if (is_planar) {
        Vector3 PlaneN(0.1, -0.1, 1);
        const double PlaneD = camRef_pose_camCur.translation().norm();
        AddPlanarInliersToFrame(ref_frame, cur_frame, camRef_pose_camCur,
                                PlaneN, PlaneD, inlier_num);
      } else {
        vector<double> depth_range;
        depth_range.push_back(camRef_pose_camCur.translation().norm());
        depth_range.push_back(10 * camRef_pose_camCur.translation().norm());
        AddNonPlanarInliersToFrame(ref_frame, cur_frame, camRef_pose_camCur,
                                   depth_range, inlier_num);
      }

      // add outliers
      AddOutliersToFrame(ref_frame, cur_frame, camRef_pose_camCur, outlier_num);
      // add noise
      if (noise_sigma != 0) {
        AddNoiseToFrame(ref_frame, noise_sigma);
        AddNoiseToFrame(cur_frame, noise_sigma);
      }

      // Perform Ransac
      VioFrontEndParams trackerParams = VioFrontEndParams();
      trackerParams.ransac_max_iterations_ = 1000;
      // trackerParams.ransac_probability_ = 0.8;
      trackerParams.ransac_randomize_ = false;
      Tracker tracker(trackerParams);
      TrackingStatus tracking_status;
      Pose3 estimated_pose;
      tie(tracking_status, estimated_pose) =
          tracker.geometricOutlierRejectionMono(ref_frame, cur_frame);

      EXPECT_EQ(tracking_status, TrackingStatus::VALID);

      // Check the correctness of the outlier rejection!
      for (int i = 0; i < inlier_num; i++) {
        EXPECT_NE(ref_frame->landmarks_[i], -1);
        EXPECT_NE(cur_frame->landmarks_[i], -1);
      }

      VLOG(1) << "outlier_num: " << outlier_num;
      for (int i = inlier_num; i < inlier_num + outlier_num; i++) {
        EXPECT_EQ(ref_frame->landmarks_[i], -1);
        EXPECT_EQ(cur_frame->landmarks_[i], -1);
      }
    }
  }
}

/* ************************************************************************* */
TEST_F(TestTracker, geometricOutlierRejectionMonoGivenRotation) {
  // Start with the simplest case:
  // Noise free, no outlier, non-planar

  Rot3 R = Rot3();  // ::Expmap(Vector3(0.01, 0.01, 0.01));
  Vector3 T(1, 0, 0);
  Pose3 camRef_pose_camCur(R, T);

  // Factor 1: Noise level
  // Factor 2: Outlier ratio
  // Factor 3: Planar or Non-Planar

  // Generate Test Configurations
  // (is_planar, inlier_num, outlier_num, noise_sigma)
  vector<std::tuple<bool, int, int, double>> test_configurations;

  // Test case 1: Idea test, 3d rigid scene, 80 inliers, no outliers, no noise
  test_configurations.push_back(std::make_tuple(false, 80, 0, 0.0));
  // Test case 2: Noisy/Outlier test, 3d rigid scene, 80 inliers, 40 outliers,
  // noise_level 1
  test_configurations.push_back(std::make_tuple(false, 80, 20, 1));
  // Test case 3: Noisy/Outlier test, Planar scene, 80 inliers, 40 outliers,
  // noise_level 1
  test_configurations.push_back(std::make_tuple(true, 80, 20, 1));

  for (auto test_conf : test_configurations) {
    bool is_planar = get<0>(test_conf);
    int inlier_num = get<1>(test_conf);
    int outlier_num = get<2>(test_conf);
    double noise_sigma = get<3>(test_conf);
    for (int t = 0; t < repeat_times; t++) {
      ClearFrame(ref_frame);
      ClearFrame(cur_frame);

      // add inliers
      if (is_planar) {
        Vector3 PlaneN(0.1, -0.1, 1);
        const double PlaneD = camRef_pose_camCur.translation().norm();
        AddPlanarInliersToFrame(ref_frame, cur_frame, camRef_pose_camCur,
                                PlaneN, PlaneD, inlier_num);
      } else {
        vector<double> depth_range;
        depth_range.push_back(camRef_pose_camCur.translation().norm());
        depth_range.push_back(10 * camRef_pose_camCur.translation().norm());
        AddNonPlanarInliersToFrame(ref_frame, cur_frame, camRef_pose_camCur,
                                   depth_range, inlier_num);
      }

      // add outliers
      AddOutliersToFrame(ref_frame, cur_frame, camRef_pose_camCur, outlier_num);
      // add noise
      if (noise_sigma != 0) {
        AddNoiseToFrame(ref_frame, noise_sigma);
        AddNoiseToFrame(cur_frame, noise_sigma);
      }

      // Perform Ransac
      Tracker tracker;
      TrackingStatus tracking_status;
      Pose3 estimated_pose;
      tie(tracking_status, estimated_pose) =
          tracker.geometricOutlierRejectionMonoGivenRotation(ref_frame,
                                                             cur_frame, R);

      EXPECT_EQ(tracking_status, TrackingStatus::VALID);

      // Check the correctness of the outlier rejection!
      for (int i = 0; i < inlier_num; i++) {
        EXPECT_NE(ref_frame->landmarks_[i], -1);
        EXPECT_NE(cur_frame->landmarks_[i], -1);
      }

      VLOG(1) << "inlier_num " << inlier_num << '\n'
              << "outlier_num " << outlier_num;
      for (int i = inlier_num; i < inlier_num + outlier_num; i++) {
        EXPECT_EQ(ref_frame->landmarks_[i], -1);
        EXPECT_EQ(cur_frame->landmarks_[i], -1);
      }
    }
  }
}

/* ************************************************************************* */
TEST_F(TestTracker, geometricOutlierRejectionStereo) {
  // Start with the simplest case:
  // Noise free, no outlier, non-planar

  CHECK(ref_stereo_frame->isRectified());
  Rot3 R = Rot3::Expmap(Vector3(0.1, 0.1, 0.1));
  Vector3 T(ref_stereo_frame->getBaseline(), 0, 0);
  Pose3 camLeftRef_pose_camLeftCur(R, T);

  // Factor 1: Noise level
  // Factor 2: Outlier ratio
  // Factor 3: Planar or Non-Planar

  // Generate Test Configurations
  // (is_planar, inlier_num, outlier_num, noise_sigma)
  vector<std::tuple<bool, int, int, double>> test_configurations;

  // Test case 1: Idea test, 3d rigid scene, 3 inliers, no outliers, no noise
  test_configurations.push_back(std::make_tuple(false, 3, 0, 0.0));
  // Test case 2: Idea test, 3d rigid scene, 80 inliers, no outliers, no noise
  test_configurations.push_back(std::make_tuple(false, 40, 0, 0.0));
  // Test case 3: Noisy/Outlier test, 3d rigid scene, 80 inliers, 40 outliers,
  // noise_level 1
  test_configurations.push_back(std::make_tuple(false, 80, 40, 0));
  // Test case 4: Noisy/Outlier test, Planar scene, 80 inliers, 40 outliers,
  // noise_level 1
  test_configurations.push_back(std::make_tuple(true, 80, 40, 0.1));

  for (size_t testId = 0; testId < test_configurations.size(); testId++) {
    auto test_conf = test_configurations.at(testId);
    bool is_planar = get<0>(test_conf);
    int inlier_num = get<1>(test_conf);
    int outlier_num = get<2>(test_conf);
    double noise_sigma = get<3>(test_conf);
    for (int t = 0; t < repeat_times; t++) {
      ClearStereoFrame(ref_stereo_frame);
      ClearStereoFrame(cur_stereo_frame);

      // Define depth_range for the synthesized 3D points.
      vector<double> depth_range;
      depth_range.push_back(camLeftRef_pose_camLeftCur.translation().norm() *
                            10);
      depth_range.push_back(camLeftRef_pose_camLeftCur.translation().norm() *
                            20);

      // add inliers
      if (is_planar) {
        Vector3 PlaneN(0, 0, 1);
        const double PlaneD =
            camLeftRef_pose_camLeftCur.translation().norm() * 20;
        AddPlanarInliersToStereoFrame(ref_stereo_frame, cur_stereo_frame,
                                      camLeftRef_pose_camLeftCur, PlaneN,
                                      PlaneD, inlier_num);
      } else {
        AddNonPlanarInliersToStereoFrame(ref_stereo_frame, cur_stereo_frame,
                                         camLeftRef_pose_camLeftCur,
                                         depth_range, inlier_num);
      }

      // add outliers
      AddOutliersToStereoFrame(ref_stereo_frame, cur_stereo_frame,
                               camLeftRef_pose_camLeftCur, depth_range,
                               outlier_num);

      // add noise
      if (noise_sigma != 0) {
        AddNoiseToStereoFrame(ref_stereo_frame, noise_sigma);
        AddNoiseToStereoFrame(cur_stereo_frame, noise_sigma);
      }

      // Perform Ransac
      VioFrontEndParams trackerParams;
      trackerParams.ransac_threshold_stereo_ = 0.3;
      Tracker tracker(trackerParams);
      TrackingStatus tracking_status;
      Pose3 estimated_pose;
      tie(tracking_status, estimated_pose) =
          tracker.geometricOutlierRejectionStereo(*ref_stereo_frame,
                                                  *cur_stereo_frame);

      // Check the correctness of the outlier rejection!
      for (int i = 0; i < inlier_num; i++) {
        EXPECT_EQ(ref_stereo_frame->right_keypoints_status_[i],
                  KeypointStatus::VALID);
        EXPECT_NE(ref_stereo_frame->keypoints_depth_[i], 0.0);
        EXPECT_GT((ref_stereo_frame->keypoints_3d_[i] - Vector3::Zero()).norm(),
                  tol);

        EXPECT_EQ(cur_stereo_frame->right_keypoints_status_[i],
                  KeypointStatus::VALID);
        EXPECT_NE(cur_stereo_frame->keypoints_depth_[i], 0.0);
        EXPECT_GT((cur_stereo_frame->keypoints_3d_[i] - Vector3::Zero()).norm(),
                  tol);
      }

      for (int i = inlier_num; i < inlier_num + outlier_num; i++) {
        EXPECT_EQ(ref_stereo_frame->right_keypoints_status_[i],
                  KeypointStatus::FAILED_ARUN);
        EXPECT_EQ(ref_stereo_frame->keypoints_depth_[i], 0.0);
        EXPECT_LT((ref_stereo_frame->keypoints_3d_[i] - Vector3::Zero()).norm(),
                  tol);

        EXPECT_EQ(cur_stereo_frame->right_keypoints_status_[i],
                  KeypointStatus::FAILED_ARUN);
        EXPECT_EQ(cur_stereo_frame->keypoints_depth_[i], 0.0);
        EXPECT_LT((cur_stereo_frame->keypoints_3d_[i] - Vector3::Zero()).norm(),
                  tol);
      }

      // Check the correctness of the estimated_pose!
      double tolPoint3;
      if (testId < 2) {
        tolPoint3 = 1e-3;  // noiseless case
        // evaluate estimated translation
        Vector3 Tactual = estimated_pose.translation();
        EXPECT_TRUE(assert_equal(T, Tactual, tolPoint3));
      } else {
        tolPoint3 = 1e-2;  // noisy case
      }

      for (int i = 0; i < inlier_num; i++) {  // compare only inliers
        Vector3 right_pt_3d_exp = estimated_pose.inverse().rotation() *
                                      ref_stereo_frame->keypoints_3d_[i] +
                                  estimated_pose.inverse().translation();
        double diff =
            (right_pt_3d_exp - cur_stereo_frame->keypoints_3d_[i]).norm();
        EXPECT_LT(diff, tolPoint3);
      }
    }
  }
}

/* ************************************************************************* */
TEST_F(TestTracker, geometricOutlierRejectionStereoGivenRotation) {
  // Start with the simplest case:
  // Noise free, no outlier, non-planar

  CHECK(ref_stereo_frame->isRectified());
  Rot3 R = Rot3::Expmap(Vector3(0.1, 0.1, 0.1));
  Vector3 T(ref_stereo_frame->getBaseline(), 0, 0);
  Pose3 camLeftRef_pose_camLeftCur(R, T);

  // Factor 1: Noise level
  // Factor 2: Outlier ratio
  // Factor 3: Planar or Non-Planar

  // Generate Test Configurations
  // (is_planar, inlier_num, outlier_num, noise_sigma)
  vector<std::tuple<bool, int, int, double>> test_configurations;

  // Test case 1: Idea test, 3d rigid scene, 3 inliers, no outliers, no noise
  test_configurations.push_back(std::make_tuple(false, 3, 0, 0.0));
  // Test case 2: Idea test, 3d rigid scene, 80 inliers, no outliers, no noise
  test_configurations.push_back(std::make_tuple(false, 40, 0, 0.0));
  // Test case 3: Noisy/Outlier test, 3d rigid scene, 80 inliers, 40 outliers,
  // noise_level 1
  test_configurations.push_back(std::make_tuple(false, 80, 40, 0));
  // Test case 4: Noisy/Outlier test, Planar scene, 80 inliers, 40 outliers,
  // noise_level 1
  test_configurations.push_back(std::make_tuple(true, 80, 40, 0.1));

  for (size_t testId = 0; testId < test_configurations.size(); testId++) {
    auto test_conf = test_configurations.at(testId);
    bool is_planar = get<0>(test_conf);
    int inlier_num = get<1>(test_conf);
    int outlier_num = get<2>(test_conf);
    double noise_sigma = get<3>(test_conf);
    for (int t = 0; t < repeat_times; t++) {
      ClearStereoFrame(ref_stereo_frame);
      ClearStereoFrame(cur_stereo_frame);

      // Define depth_range for the synthesized 3D points.
      vector<double> depth_range;
      depth_range.push_back(camLeftRef_pose_camLeftCur.translation().norm() *
                            10);
      depth_range.push_back(camLeftRef_pose_camLeftCur.translation().norm() *
                            20);

      // add inliers
      if (is_planar) {
        Vector3 PlaneN(0, 0, 1);
        const double PlaneD =
            camLeftRef_pose_camLeftCur.translation().norm() * 20;
        AddPlanarInliersToStereoFrame(ref_stereo_frame, cur_stereo_frame,
                                      camLeftRef_pose_camLeftCur, PlaneN,
                                      PlaneD, inlier_num);
      } else {
        AddNonPlanarInliersToStereoFrame(ref_stereo_frame, cur_stereo_frame,
                                         camLeftRef_pose_camLeftCur,
                                         depth_range, inlier_num);
      }

      // add outliers
      AddOutliersToStereoFrame(ref_stereo_frame, cur_stereo_frame,
                               camLeftRef_pose_camLeftCur, depth_range,
                               outlier_num);

      // add noise
      if (noise_sigma != 0) {
        AddNoiseToStereoFrame(ref_stereo_frame, noise_sigma);
        AddNoiseToStereoFrame(cur_stereo_frame, noise_sigma);
      }

      // Perform Ransac
      Tracker tracker;
      pair<TrackingStatus, Pose3> poseStatus;
      Matrix3 infoMat;
      tie(poseStatus, infoMat) =
          tracker.geometricOutlierRejectionStereoGivenRotation(
              *ref_stereo_frame, *cur_stereo_frame, R);

      TrackingStatus tracking_status = poseStatus.first;
      Pose3 estimated_pose = poseStatus.second;
      // Check the correctness of the outlier rejection!
      for (int i = 0; i < inlier_num; i++) {
        EXPECT_EQ(ref_stereo_frame->right_keypoints_status_[i],
                  KeypointStatus::VALID);
        EXPECT_NE(ref_stereo_frame->keypoints_depth_[i], 0.0);
        EXPECT_GT((ref_stereo_frame->keypoints_3d_[i] - Vector3::Zero()).norm(),
                  tol);

        EXPECT_EQ(cur_stereo_frame->right_keypoints_status_[i],
                  KeypointStatus::VALID);
        EXPECT_NE(cur_stereo_frame->keypoints_depth_[i], 0.0);
        EXPECT_GT((cur_stereo_frame->keypoints_3d_[i] - Vector3::Zero()).norm(),
                  tol);
      }

      for (int i = inlier_num; i < inlier_num + outlier_num; i++) {
        EXPECT_EQ(ref_stereo_frame->right_keypoints_status_[i],
                  KeypointStatus::FAILED_ARUN);
        EXPECT_EQ(ref_stereo_frame->keypoints_depth_[i], 0.0);
        EXPECT_LT((ref_stereo_frame->keypoints_3d_[i] - Vector3::Zero()).norm(),
                  tol);

        EXPECT_EQ(cur_stereo_frame->right_keypoints_status_[i],
                  KeypointStatus::FAILED_ARUN);
        EXPECT_EQ(cur_stereo_frame->keypoints_depth_[i], 0.0);
        EXPECT_LT((cur_stereo_frame->keypoints_3d_[i] - Vector3::Zero()).norm(),
                  tol);
      }

      // Check the correctness of the estimated_pose!
      double tolPoint3;
      if (testId < 2) {
        tolPoint3 = 1e-3;  // noiseless case
        // evaluate estimated translation
        Vector3 Tactual = estimated_pose.translation();
        EXPECT_TRUE(assert_equal(T, Tactual, tolPoint3));
      } else {
        tolPoint3 = 1e-2;  // noisy case
      }

      for (int i = 0; i < inlier_num; i++) {  // compare only inliers
        Vector3 right_pt_3d_exp = estimated_pose.inverse().rotation() *
                                      ref_stereo_frame->keypoints_3d_[i] +
                                  estimated_pose.inverse().translation();
        double diff =
            (right_pt_3d_exp - cur_stereo_frame->keypoints_3d_[i]).norm();
        EXPECT_LT(diff, tolPoint3);
      }
    }
  }
}

/* ************************************************************************* */
TEST_F(TestTracker, getPoint3AndCovariance) {
  ClearStereoFrame(ref_stereo_frame);
  // create stereo cam
  Cal3_S2 ref_left_undist_rect_cam_mat =
      ref_stereo_frame->getLeftUndistRectCamMat();
  Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(
      ref_left_undist_rect_cam_mat.fx(), ref_left_undist_rect_cam_mat.fy(),
      ref_left_undist_rect_cam_mat.skew(), ref_left_undist_rect_cam_mat.px(),
      ref_left_undist_rect_cam_mat.py(), ref_stereo_frame->getBaseline()));
  StereoCamera stereoCam =
      StereoCamera(Pose3(), K);  // in the ref frame of the left camera

  // create a stereo point:
  double xL = 379.999 / 2;  // in the middle of the image
  double v = 255.238 / 2;   // in the middle of the image
  double xR = xL - 10;      // some disparity
  StereoPoint2 stereoPoint(xL, xR, v);

  // create a 3D point in front of the camera
  Vector3 point3 = stereoCam.backproject2(stereoPoint).vector();
  int pointId = 0;  // only point

  Matrix3 stereoPtCov = Matrix3::Identity();

  // populate stereoFrame:
  ref_stereo_frame->left_keypoints_rectified_.push_back(KeypointCV(xL, v));
  ref_stereo_frame->right_keypoints_rectified_.push_back(KeypointCV(xR, v));
  ref_stereo_frame->keypoints_3d_.push_back(point3);

  // use function to get actual answer
  Vector3 f_ref_i_expected, f_ref_i_actual;
  Matrix3 cov_ref_i_expected, cov_ref_i_actual;
  tie(f_ref_i_actual, cov_ref_i_actual) = Tracker::getPoint3AndCovariance(
      *ref_stereo_frame, stereoCam, pointId, stereoPtCov);

  // use monte carlo method to get expected answer
  tie(f_ref_i_expected, cov_ref_i_expected) =
      monteCarloSampleCovariance(stereoCam, stereoPoint, stereoPtCov);

  // check monte Carlo runs against our estimate
  EXPECT_TRUE(assert_equal(f_ref_i_expected, f_ref_i_actual, 0.1));
  EXPECT_TRUE(assert_equal(cov_ref_i_expected, cov_ref_i_actual, 0.1));
  // cout << "f_ref_i_expected \n" << f_ref_i_expected << endl;
  // cout << "f_ref_i_actual \n" << f_ref_i_actual << endl;
  // cout << "cov_ref_i_actual \n" << cov_ref_i_actual << endl;
  // cout << "cov_ref_i_expected \n" << cov_ref_i_expected << endl;
}

/* ************************************************************************* */
TEST_F(TestTracker, findOutliers) {
  // Normal case:
  {
    // synthesize the data
    const int num_inliers = 100;
    const int num_outliers = 100;

    vector<int> inliers;
    vector<int> outliers_expected;

    inliers.reserve(num_inliers);
    outliers_expected.reserve(num_outliers);

    for (size_t i = 0; i < num_inliers + num_outliers; i++) {
      if (i % 2 == 0) {
        inliers.push_back(i);
      } else {
        outliers_expected.push_back(i);
      }
    }

    // only size of the matches matters.
    vector<pair<size_t, size_t>> matches_ref_cur(num_inliers + num_outliers);

    // randomly shuffle the inliers
    random_shuffle(inliers.begin(), inliers.end());

    vector<int> outliers_actual;
    Tracker::findOutliers(matches_ref_cur, inliers, &outliers_actual);

    // check that outliers_actual matches outliers_expected
    EXPECT_EQ(outliers_expected.size(), outliers_actual.size());
    for (auto i : outliers_actual) {
      EXPECT_NE(find(outliers_expected.begin(), outliers_expected.end(), i),
                outliers_expected.end());
    }
  }

  // Corner case: empty inliers
  {
    // synthesize the data
    const int num_outliers = 100;

    vector<int> inliers;
    vector<int> outliers_expected;

    outliers_expected.reserve(num_outliers);

    for (size_t i = 0; i < num_outliers; i++) {
      outliers_expected.push_back(i);
    }

    // only size of the matches matters.
    vector<pair<size_t, size_t>> matches_ref_cur(num_outliers);

    vector<int> outliers_actual;
    Tracker::findOutliers(matches_ref_cur, inliers, &outliers_actual);

    // check that outliers_actual matches outliers_expected
    EXPECT_EQ(outliers_expected.size(), outliers_actual.size());
    for (auto i : outliers_actual) {
      EXPECT_NE(find(outliers_expected.begin(), outliers_expected.end(), i),
                outliers_expected.end());
    }
  }

  // Corner case: empty outliers
  {
    // synthesize the data
    const int num_inliers = 100;
    vector<int> inliers;
    inliers.reserve(num_inliers);
    for (size_t i = 0; i < num_inliers; i++) {
      inliers.push_back(i);
    }

    // only size of the matches matters.
    vector<pair<size_t, size_t>> matches_ref_cur(num_inliers);

    // randomly shuffle the inliers
    random_shuffle(inliers.begin(), inliers.end());

    vector<int> outliers_actual;
    Tracker::findOutliers(matches_ref_cur, inliers, &outliers_actual);

    // check that outliers_actual matches outliers_expected
    EXPECT_EQ(outliers_actual.size(), 0);
  }
}

/* ************************************************************************* */
TEST_F(TestTracker, FindMatchingKeypoints) {
  // Synthesize an example!
  ClearFrame(ref_frame);
  ClearFrame(cur_frame);

  const int num_landmarks_common = 100;
  const int num_landmarks_ref = 100;
  const int num_landmarks_cur = 100;
  const int num_landmarks_invalid = 100;

  ref_frame->landmarks_.reserve(num_landmarks_common + num_landmarks_ref +
                                num_landmarks_invalid);
  cur_frame->landmarks_.reserve(num_landmarks_common + num_landmarks_cur +
                                num_landmarks_invalid);

  // landmark ids in common!
  for (int i = 0; i < num_landmarks_common; i++) {
    ref_frame->landmarks_.push_back(3 * i);
    cur_frame->landmarks_.push_back(3 * i);
  }

  // landmark ids unique to ref_frame
  for (int i = 0; i < num_landmarks_ref; i++) {
    ref_frame->landmarks_.push_back(3 * i + 1);
  }

  // landmark ids unique to ref_frame
  for (int i = 0; i < num_landmarks_cur; i++) {
    cur_frame->landmarks_.push_back(3 * i + 2);
  }

  // add a bunch of invalid landmarks to both frames
  for (int i = 0; i < num_landmarks_invalid; i++) {
    ref_frame->landmarks_.push_back(-1);
    cur_frame->landmarks_.push_back(-1);
  }

  // shuffle landmarks in both frames
  random_shuffle(ref_frame->landmarks_.begin(), ref_frame->landmarks_.end());
  random_shuffle(cur_frame->landmarks_.begin(), cur_frame->landmarks_.end());

  vector<pair<size_t, size_t>> matches_ref_cur;
  Tracker::findMatchingKeypoints(*ref_frame, *cur_frame, &matches_ref_cur);

  // Check the correctness of matches_ref_cur
  EXPECT_EQ(matches_ref_cur.size(), num_landmarks_common);
  set<int> landmarks_found;
  for (auto match_ref_cur : matches_ref_cur) {
    int l_ref = ref_frame->landmarks_[match_ref_cur.first];
    int l_cur = cur_frame->landmarks_[match_ref_cur.second];

    EXPECT_EQ(l_ref, l_cur);
    EXPECT_EQ(landmarks_found.find(l_ref), landmarks_found.end());

    landmarks_found.insert(l_ref);
  }
}

/* ************************************************************************* */
TEST_F(TestTracker, FindMatchingStereoKeypoints) {
  // Synthesize the data for test!
  ClearStereoFrame(ref_stereo_frame);
  ClearStereoFrame(cur_stereo_frame);

  const int num_landmarks_common = 100;
  const int num_landmarks_ref = 90;
  const int num_landmarks_cur = 80;
  const int num_landmarks_invalid = 70;

  ref_stereo_frame->getLeftFrameMutable()->landmarks_.reserve(
      num_landmarks_common + num_landmarks_ref + num_landmarks_invalid);
  cur_stereo_frame->getLeftFrameMutable()->landmarks_.reserve(
      num_landmarks_common + num_landmarks_cur + num_landmarks_invalid);

  // landmark ids in common!
  for (int i = 0; i < num_landmarks_common; i++) {
    ref_stereo_frame->getLeftFrameMutable()->landmarks_.push_back(3 * i);
    cur_stereo_frame->getLeftFrameMutable()->landmarks_.push_back(3 * i);
  }

  // landmark ids unique to ref_stereo_frame
  for (int i = 0; i < num_landmarks_ref; i++) {
    ref_stereo_frame->getLeftFrameMutable()->landmarks_.push_back(3 * i + 1);
  }

  // landmark ids unique to ref_stereo_frame
  for (int i = 0; i < num_landmarks_cur; i++) {
    cur_stereo_frame->getLeftFrameMutable()->landmarks_.push_back(3 * i + 2);
  }

  // add a bunch of invalid landmarks to both frames
  for (int i = 0; i < num_landmarks_invalid; i++) {
    ref_stereo_frame->getLeftFrameMutable()->landmarks_.push_back(-1);
    cur_stereo_frame->getLeftFrameMutable()->landmarks_.push_back(-1);
  }

  // shuffle landmarks in both frames
  random_shuffle(ref_stereo_frame->getLeftFrameMutable()->landmarks_.begin(),
                 ref_stereo_frame->getLeftFrameMutable()->landmarks_.end());
  random_shuffle(cur_stereo_frame->getLeftFrameMutable()->landmarks_.begin(),
                 cur_stereo_frame->getLeftFrameMutable()->landmarks_.end());

  //   Set right_keypoints_status!
  for (int i = 0; i < ref_stereo_frame->getLeftFrame().landmarks_.size(); i++) {
    int l_id = ref_stereo_frame->getLeftFrameMutable()->landmarks_[i];
    if (l_id % 6 == 0) {
      ref_stereo_frame->right_keypoints_status_.push_back(
          KeypointStatus::VALID);
    } else {
      ref_stereo_frame->right_keypoints_status_.push_back(
          KeypointStatus::NO_RIGHT_RECT);
    }
  }

  //   Set right_keypoints_status!
  for (int i = 0; i < cur_stereo_frame->getLeftFrame().landmarks_.size(); i++) {
    int l_id = cur_stereo_frame->getLeftFrameMutable()->landmarks_[i];
    if (l_id % 6 == 0) {
      cur_stereo_frame->right_keypoints_status_.push_back(
          KeypointStatus::VALID);
    } else {
      cur_stereo_frame->right_keypoints_status_.push_back(
          KeypointStatus::NO_RIGHT_RECT);
    }
  }

  vector<pair<size_t, size_t>> matches_ref_cur;
  Tracker::findMatchingStereoKeypoints(*ref_stereo_frame, *cur_stereo_frame,
                                       &matches_ref_cur);

  // Check the correctness!
  EXPECT_EQ(matches_ref_cur.size(), (num_landmarks_common + 1) / 2);
  set<int> landmarks_found;
  for (auto match_ref_cur : matches_ref_cur) {
    int l_ref = ref_stereo_frame->getLeftFrameMutable()
                    ->landmarks_[match_ref_cur.first];
    int l_cur = cur_stereo_frame->getLeftFrameMutable()
                    ->landmarks_[match_ref_cur.second];

    EXPECT_EQ(l_ref, l_cur);
    EXPECT_EQ(l_ref % 6, 0);
    EXPECT_EQ(landmarks_found.find(l_ref), landmarks_found.end());

    landmarks_found.insert(l_ref);
  }
}

/* ************************************************************************* */
TEST_F(TestTracker, mahalanobisDistance) {
  double timeBefore = 0;
  double time1 = 0, time2 = 0, time3 = 0;
  for (size_t test = 0; test < 1000; test++) {
    // generate matrix
    Matrix m = Matrix::Random(3, 5);
    Matrix3f O = (m * m.transpose()).cast<float>();
    // cout << "mm: \n" << m * m.transpose() << endl;
    // cout << "O: \n" << O << endl;

    Vector3 vd = Vector3::Random();
    Vector3f v = vd.cast<float>();

    // sol1 - SLOWER: sol2 x 2
    timeBefore = UtilsOpenCV::GetTimeInSeconds();
    Vector3f Omega_relTran_j = O.llt().solve(v);
    float innovationMahalanobisNorm1 = v.transpose() * Omega_relTran_j;
    time1 += UtilsOpenCV::GetTimeInSeconds() - timeBefore;

    // sol2 - still 0.25 seconds for 200 features
    timeBefore = UtilsOpenCV::GetTimeInSeconds();
    Matrix3f infoMatSum = O.inverse();
    float innovationMahalanobisNorm2 = v.transpose() * infoMatSum * v;
    time2 += UtilsOpenCV::GetTimeInSeconds() - timeBefore;

    // sol3 - still 0.25 seconds for 200 features
    timeBefore = UtilsOpenCV::GetTimeInSeconds();
    float dinv = 1 / (O(0, 0) * (O(1, 1) * O(2, 2) - O(1, 2) * O(2, 1)) -
                      O(1, 0) * (O(0, 1) * O(2, 2) - O(0, 2) * O(2, 1)) +
                      O(2, 0) * (O(0, 1) * O(1, 2) - O(1, 1) * O(0, 2)));
    float innovationMahalanobisNorm3 =
        dinv * v(0) *
            (v(0) * (O(1, 1) * O(2, 2) - O(1, 2) * O(2, 1)) -
             v(1) * (O(0, 1) * O(2, 2) - O(0, 2) * O(2, 1)) +
             v(2) * (O(0, 1) * O(1, 2) - O(1, 1) * O(0, 2))) +
        dinv * v(1) *
            (O(0, 0) * (v(1) * O(2, 2) - O(1, 2) * v(2)) -
             O(1, 0) * (v(0) * O(2, 2) - O(0, 2) * v(2)) +
             O(2, 0) * (v(0) * O(1, 2) - v(1) * O(0, 2))) +
        dinv * v(2) *
            (O(0, 0) * (O(1, 1) * v(2) - v(1) * O(2, 1)) -
             O(1, 0) * (O(0, 1) * v(2) - v(0) * O(2, 1)) +
             O(2, 0) * (O(0, 1) * v(1) - O(1, 1) * v(0)));
    time3 += UtilsOpenCV::GetTimeInSeconds() - timeBefore;

    EXPECT_NEAR(double(innovationMahalanobisNorm1),
                double(innovationMahalanobisNorm2), 1e-2);
    EXPECT_NEAR(double(innovationMahalanobisNorm1),
                double(innovationMahalanobisNorm3), 1e-2);
    EXPECT_NEAR(double(1 / dinv), double(O.determinant()), 1e-4);
  }
  VLOG(1) << "time1 (llt): " << time1 << '\n'
          << "time2 (x'*O*x): " << time2 << '\n'
          << "time3 (manual): " << time3;
}
