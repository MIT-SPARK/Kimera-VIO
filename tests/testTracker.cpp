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
 * @author Antoni Rosinol
 * @author Luca Carlone
 * @author Marcus Abate
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>

#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/utils/Timer.h"

DECLARE_string(test_data_path);
DECLARE_bool(display);

using namespace gtsam;
using namespace std;
using namespace VIO;
using namespace cv;

/* ************************************************************************** */
// Testing data
static const double tol = 1e-7;
// TODO initializing null pointers is prone to errors!!
// TODO use test fixture!!
static const FrameId id_ref = 0, id_cur = 1;
static const int64_t timestamp_ref = 1000, timestamp_cur = 2000;
static string stereo_test_data_path(FLAGS_test_data_path +
                                    string("/ForStereoFrame/"));
static const int repeat_times = 1;

class TestTracker : public ::testing::Test {
 public:
  TestTracker() : tracker_params_(), tracker_(nullptr) {
    // Fix randomization seed (we use monte carlo runs here).
    srand(3);
    InitializeData();

    if (FLAGS_display) {
      window_ = std::make_unique<cv::viz::Viz3d>("Test Tracker");
      window_->setBackgroundColor(cv::viz::Color::black());
    }
  }

 protected:
  void SetUp() override {}
  // TODO deallocate dynamic memory here!
  void TearDown() override {}

  void InitializeData() {
    CameraParams cam_params_left, cam_params_right;
    cam_params_left.parseYAML(stereo_test_data_path + "/sensorLeft.yaml");
    cam_params_right.parseYAML(stereo_test_data_path + "/sensorRight.yaml");

    string img_name_ref_left = stereo_test_data_path + "left_img_0.png";
    string img_name_ref_right = stereo_test_data_path + "right_img_0.png";

    string img_name_cur_left = stereo_test_data_path + "left_img_1.png";
    string img_name_cur_right = stereo_test_data_path + "right_img_1.png";

    // Data for testing "geometricOutlierRejection2d2d"
    // !!! TODO THIS IS ALLOCATING MEMORY BUT THERE IS NO DELETE !!!
    ref_frame = std::make_shared<Frame>(
        id_ref,
        timestamp_ref,
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(img_name_ref_left));
    cur_frame = std::make_shared<Frame>(
        id_cur,
        timestamp_cur,
        cam_params_left,
        UtilsOpenCV::ReadAndConvertToGrayScale(img_name_cur_left));

    FrontendParams tp;

    ref_stereo_frame = std::make_shared<StereoFrame>(
        id_ref,
        timestamp_ref,
        Frame(
            id_ref,
            timestamp_ref,
            cam_params_left,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_ref_left, tp.stereo_matching_params_.equalize_image_)),
        Frame(id_ref,
              timestamp_ref,
              cam_params_right,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  img_name_ref_right,
                  tp.stereo_matching_params_.equalize_image_)));

    cur_stereo_frame = std::make_shared<StereoFrame>(
        id_cur,
        timestamp_cur,
        Frame(
            id_cur,
            timestamp_cur,
            cam_params_left,
            UtilsOpenCV::ReadAndConvertToGrayScale(
                img_name_cur_left, tp.stereo_matching_params_.equalize_image_)),
        Frame(id_cur,
              timestamp_cur,
              cam_params_right,
              UtilsOpenCV::ReadAndConvertToGrayScale(
                  img_name_cur_right,
                  tp.stereo_matching_params_.equalize_image_)));

    stereo_camera_ =
        std::make_shared<VIO::StereoCamera>(cam_params_left, cam_params_right);
    stereo_matcher_ = std::make_unique<VIO::StereoMatcher>(
        stereo_camera_, tp.stereo_matching_params_);
    tracker_ = std::make_unique<Tracker>(
        tracker_params_, stereo_camera_->getOriginalLeftCamera());

    feature_detector_ =
        std::make_unique<VIO::FeatureDetector>(tp.feature_detector_params_);
    feature_detector_->featureDetection(&ref_stereo_frame->left_frame_);
    feature_detector_->featureDetection(&cur_stereo_frame->left_frame_);

    stereo_matcher_->sparseStereoReconstruction(ref_stereo_frame.get());
    stereo_matcher_->sparseStereoReconstruction(cur_stereo_frame.get());

    ClearStereoFrame(ref_stereo_frame.get());
    ClearStereoFrame(cur_stereo_frame.get());
  }

  Vector3 IntersectVersorPlane(const Vector3& versor,
                               const Vector3& PlaneN,
                               const double PlaneD) {
    // intersect versor with N'*x=d;
    double t = PlaneD / versor.dot(PlaneN);
    return t * versor;
  }

  void AddKeypointsVersorsToFrames(Frame* f_ref,
                                   Frame* f_cur,
                                   KeypointCV& pt_ref,
                                   KeypointCV& pt_cur,
                                   Vector3& v_ref,
                                   Vector3& v_cur) {
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

    f_ref->landmarks_age_.push_back(0);
    f_cur->landmarks_age_.push_back(1);
  }

  void ClearFrame(Frame* f) {
    CHECK_NOTNULL(f);
    f->keypoints_.resize(0);
    f->scores_.resize(0);
    f->landmarks_.resize(0);
    f->landmarks_age_.resize(0);
    f->versors_.resize(0);
  }

  void AddNoiseToFrame(Frame* f, const double noise_sigma) {
    CHECK_NOTNULL(f);
    default_random_engine generator;
    normal_distribution<double> distribution(0, noise_sigma);
    for (auto versor : f->versors_) {
      versor = versor + Vector3(distribution(generator),
                                distribution(generator),
                                distribution(generator));
    }
  }

  void AddPlanarInliersToFrame(Frame* f_ref,
                               Frame* f_cur,
                               const Pose3 camRef_pose_camCur,
                               const Vector3& PlaneN,
                               const double PlaneD,
                               const int inlierNum) {
    CHECK_NOTNULL(f_ref);
    CHECK_NOTNULL(f_cur);
    // All points are on the plane: PlaneN * x = d
    for (int i = 0; i < inlierNum; i++) {
      // Randomly synthesize the point!
      KeypointCV pt_ref(rand() % f_ref->img_.cols, rand() % f_ref->img_.rows);
      // Calibrate the point
      Vector3 versor_ref =
          UndistorterRectifier::GetBearingVector(pt_ref, f_ref->cam_param_);

      // Compute the intersection between the versor and the plane.
      Vector3 versor_plane = IntersectVersorPlane(versor_ref, PlaneN, PlaneD);

      // project to the current frame!
      Vector3 versor_cur =
          camRef_pose_camCur.inverse().rotation() * versor_plane +
          camRef_pose_camCur.inverse().translation();
      versor_cur = versor_cur / versor_cur.norm();

      gtsam::Cal3DS2 gtsam_calib;
      CameraParams::createGtsamCalibration(
          f_ref->cam_param_.distortion_coeff_mat_,
          f_ref->cam_param_.intrinsics_,
          &gtsam_calib);
      Point2 pt_cur_gtsam = gtsam_calib.uncalibrate(
          Point2(versor_cur[0] / versor_cur[2], versor_cur[1] / versor_cur[2]));
      KeypointCV pt_cur(pt_cur_gtsam.x(), pt_cur_gtsam.y());

      AddKeypointsVersorsToFrames(
          f_ref, f_cur, pt_ref, pt_cur, versor_ref, versor_cur);
    }
  }

  void AddNonPlanarInliersToFrame(Frame* f_ref,
                                  Frame* f_cur,
                                  const Pose3 camRef_pose_camCur,
                                  const vector<double>& depth_range,
                                  const int inlierNum) {
    for (int i = 0; i < inlierNum; i++) {
      CHECK_NOTNULL(f_ref);
      CHECK_NOTNULL(f_cur);
      // Randomly syntheisze the point!
      KeypointCV pt_ref(rand() % f_ref->img_.cols, rand() % f_ref->img_.rows);

      // Calibrate the point
      Vector3 versor_ref =
          UndistorterRectifier::GetBearingVector(pt_ref, f_ref->cam_param_);

      // Randomly generate the depth
      double depth = depth_range[0] + (depth_range[1] - depth_range[0]) *
                                          ((double)rand() / RAND_MAX);

      // project to the current frame!
      Vector3 versor_cur =
          camRef_pose_camCur.inverse().rotation() * (versor_ref * depth) +
          camRef_pose_camCur.inverse().translation();
      versor_cur = versor_cur / versor_cur.norm();

      gtsam::Cal3DS2 gtsam_calib;
      CameraParams::createGtsamCalibration(
          f_ref->cam_param_.distortion_coeff_mat_,
          f_ref->cam_param_.intrinsics_,
          &gtsam_calib);
      Point2 pt_cur_gtsam = gtsam_calib.uncalibrate(
          Point2(versor_cur[0] / versor_cur[2], versor_cur[1] / versor_cur[2]));
      KeypointCV pt_cur(pt_cur_gtsam.x(), pt_cur_gtsam.y());

      AddKeypointsVersorsToFrames(
          f_ref, f_cur, pt_ref, pt_cur, versor_ref, versor_cur);
    }
  }

  void AddOutliersToFrame(Frame* f_ref,
                          Frame* f_cur,
                          Pose3 camRef_pose_camCur,
                          int outlierNum) {
    for (int i = 0; i < outlierNum; i++) {
      while (true) {
        CHECK_NOTNULL(f_ref);
        CHECK_NOTNULL(f_cur);
        // Keypoints
        KeypointCV pt_ref(rand() % f_ref->img_.cols, rand() % f_ref->img_.rows);
        KeypointCV pt_cur(rand() % f_cur->img_.cols, rand() % f_cur->img_.rows);

        // Calibrate keypoints
        Vector3 versor_ref =
            UndistorterRectifier::GetBearingVector(pt_ref, f_ref->cam_param_);
        Vector3 versor_cur =
            UndistorterRectifier::GetBearingVector(pt_cur, f_cur->cam_param_);

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
          AddKeypointsVersorsToFrames(
              f_ref, f_cur, pt_ref, pt_cur, versor_ref, versor_cur);
          break;
        }
      }
    }
  }

  void AddVersorsToStereoFrames(StereoFrame* sf_ref,
                                StereoFrame* sf_cur,
                                Vector3& v_ref,
                                Vector3& v_cur) {
    CHECK_NOTNULL(sf_ref);
    CHECK_NOTNULL(sf_cur);
    // Decide the largest landmark IDs for each frame!
    int max_id;
    if (sf_ref->left_frame_.landmarks_.size() == 0 &&
        sf_cur->left_frame_.landmarks_.size() == 0) {
      max_id = 0;
    } else {
      vector<LandmarkId>::const_iterator max_id_ref =
          max_element(sf_ref->left_frame_.landmarks_.begin(),
                      sf_ref->left_frame_.landmarks_.end());
      vector<LandmarkId>::const_iterator max_id_cur =
          max_element(sf_cur->left_frame_.landmarks_.begin(),
                      sf_cur->left_frame_.landmarks_.end());
      max_id = max(*max_id_ref, *max_id_cur);
    }

    // Add the versors to the stereo frames
    sf_ref->keypoints_3d_.push_back(v_ref);
    sf_cur->keypoints_3d_.push_back(v_cur);

    // create ref stereo camera
    VIO::StereoCamera ref_stereo_camera(sf_ref->left_frame_.cam_param_,
                                        sf_ref->right_frame_.cam_param_);
    Rot3 camLrect_R_camL =
        UtilsOpenCV::cvMatToGtsamRot3(ref_stereo_camera.getR1());
    gtsam::StereoCamera stereoCam =
        gtsam::StereoCamera(Pose3(), ref_stereo_camera.getStereoCalib());

    StereoPoint2 sp2 = stereoCam.project(camLrect_R_camL.rotate(Point3(v_ref)));
    sf_ref->left_keypoints_rectified_.push_back(
        StatusKeypointCV(KeypointStatus::VALID, KeypointCV(sp2.uL(), sp2.v())));
    sf_ref->right_keypoints_rectified_.push_back(
        StatusKeypointCV(KeypointStatus::VALID, KeypointCV(sp2.uR(), sp2.v())));

    // create cur stereo camera
    VIO::StereoCamera cur_stereo_camera(sf_cur->left_frame_.cam_param_,
                                        sf_cur->right_frame_.cam_param_);
    camLrect_R_camL = UtilsOpenCV::cvMatToGtsamRot3(cur_stereo_camera.getR1());
    stereoCam =
        gtsam::StereoCamera(Pose3(), cur_stereo_camera.getStereoCalib());

    sp2 = stereoCam.project(camLrect_R_camL.rotate(Point3(v_cur)));
    sf_cur->left_keypoints_rectified_.push_back(
        StatusKeypointCV(KeypointStatus::VALID, KeypointCV(sp2.uL(), sp2.v())));
    sf_cur->right_keypoints_rectified_.push_back(
        StatusKeypointCV(KeypointStatus::VALID, KeypointCV(sp2.uR(), sp2.v())));

    // depth!
    sf_ref->keypoints_depth_.push_back(v_ref.norm());
    sf_cur->keypoints_depth_.push_back(v_cur.norm());

    // Assign landmark ids to them!
    sf_ref->left_frame_.landmarks_.push_back(max_id + 1);
    sf_cur->left_frame_.landmarks_.push_back(max_id + 1);

    sf_ref->left_frame_.landmarks_age_.push_back(0);
    sf_cur->left_frame_.landmarks_age_.push_back(1);
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
      KeypointCV pt_ref(rand() % sf_ref->left_frame_.img_.cols,
                        rand() % sf_ref->left_frame_.img_.rows);

      // Calibrate the point
      Vector3 versor_ref = UndistorterRectifier::GetBearingVector(
          pt_ref, sf_ref->left_frame_.cam_param_);
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
    ClearFrame(&sf->left_frame_);
    ClearFrame(&sf->right_frame_);
    sf->keypoints_3d_.clear();
    sf->keypoints_depth_.clear();
    sf->left_keypoints_rectified_.clear();
    sf->right_keypoints_rectified_.clear();
  }

  void AddOutliersToStereoFrame(StereoFrame* sf_ref,
                                StereoFrame* sf_cur,
                                Pose3 camRef_pose_camCur,
                                vector<double>& depth_range,
                                int outlierNum) {
    CHECK_NOTNULL(sf_ref);
    CHECK_NOTNULL(sf_cur);
    for (int i = 0; i < outlierNum; i++) {
      while (true) {
        // Keypoints
        KeypointCV pt_ref(rand() % sf_ref->left_frame_.img_.cols,
                          rand() % sf_ref->left_frame_.img_.rows);
        KeypointCV pt_cur(rand() % sf_cur->left_frame_.img_.cols,
                          rand() % sf_cur->left_frame_.img_.rows);

        // Calibrate keypoints
        Vector3 versor_ref = UndistorterRectifier::GetBearingVector(
            pt_ref, sf_ref->left_frame_.cam_param_);
        Vector3 versor_cur = UndistorterRectifier::GetBearingVector(
            pt_cur, sf_cur->left_frame_.cam_param_);

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

  void AddPlanarInliersToStereoFrame(StereoFrame* sf_ref,
                                     StereoFrame* sf_cur,
                                     const Pose3 camLeftRef_pose_camLeftCur,
                                     const Vector3& PlaneN,
                                     const double PlaneD,
                                     const int inlierNum) {
    CHECK_NOTNULL(sf_ref);
    CHECK_NOTNULL(sf_cur);
    // All points are on the plane: PlaneN * x = d
    for (int i = 0; i < inlierNum; i++) {
      // Randomly synthesize the point!
      KeypointCV pt_ref(rand() % sf_ref->left_frame_.img_.cols,
                        rand() % sf_ref->left_frame_.img_.rows);
      // Calibrate the point
      Vector3 versor_ref = UndistorterRectifier::GetBearingVector(
          pt_ref, sf_ref->left_frame_.cam_param_);

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
    for (int i = 0; i < sf->keypoints_3d_.size(); i++) {
      Vector3 noise(distribution(generator),
                    distribution(generator),
                    distribution(generator));
      sf->keypoints_3d_.at(i) = sf->keypoints_3d_.at(i) + noise;
    }
  }

  pair<Vector3, Matrix3> monteCarloSampleCovariance(
      const gtsam::StereoCamera stereoCam,
      const StereoPoint2 stereoPoint,
      const Matrix3 stereoPtCov) {
    Vector3 meanVector = stereoCam.backproject2(stereoPoint);
    Vector3 sampleMean = Vector3::Zero();

    default_random_engine generator;
    Matrix3 sampleCovariance = Matrix3::Zero();
    int nrRuns = 1000000;
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

      Vector3 sample_i = stereoCam.backproject2(perturbedStereoPoint);
      sampleMean += sample_i;
      sampleCovariance +=
          (sample_i - meanVector) * (sample_i - meanVector).transpose();
    }
    sampleMean = sampleMean / double(nrRuns);
    sampleCovariance = sampleCovariance / double(nrRuns - 1);
    return make_pair(sampleMean, sampleCovariance);
  }

  // THIS IS COPIED FROM testOpticalFlow
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
    CHECK(window_);
    // Display 3D rays from cam origin to lmks.
    if (display_text) {
      window_->showWidget(
          "Ray Label " + id,
          cv::viz::WText3D(id, lmk, text_thickness, true, color));
    }
    window_->showWidget("Ray " + id,
                        cv::viz::WLine(cam_world_origin, lmk, color));
  }

  void visualizePointCloud(const std::string& id, const cv::Mat& pointcloud) {
    CHECK(window_);
    cv::viz::WCloud cloud(pointcloud, cv::viz::Color::red());
    cloud.setRenderingProperty(cv::viz::POINT_SIZE, 6);
    window_->showWidget(id, cloud);
  }

  void visualizeScene(const CameraParams& camera_1_params,
                      const CameraParams& camera_2_params,
                      const KeypointsCV& cam_1_kpts,
                      const KeypointsCV& cam_2_kpts,
                      const LandmarksCV& lmks) {
    if (FLAGS_display) {
      const cv::Matx33d& K_1 = camera_1_params.K_;
      const cv::Matx33d& K_2 = camera_2_params.K_;
      const gtsam::Pose3& cam_1_pose = camera_1_params.body_Pose_cam_;
      const gtsam::Pose3& cam_2_pose = camera_2_params.body_Pose_cam_;

      // Visualize world coords.
      window_->showWidget("World Coordinates", cv::viz::WCoordinateSystem(0.5));

      // Visualize left/right cameras
      const auto& cam_1_cv_pose =
          UtilsOpenCV::gtsamPose3ToCvAffine3d(cam_1_pose);
      const auto& cam_2_cv_pose =
          UtilsOpenCV::gtsamPose3ToCvAffine3d(cam_2_pose);
      // Camera Coordinate axes
      cv::viz::WCameraPosition cpw1(0.2);
      cv::viz::WCameraPosition cpw2(0.1);  // Second camera a bit smaller
      window_->showWidget("Cam 1 Coordinates", cpw1, cam_1_cv_pose);
      window_->showWidget("Cam 2 Coordinates", cpw2, cam_2_cv_pose);

      // Visualize landmarks
      cv::Mat pointcloud = cv::Mat(0, 3, CV_64FC1);
      cv::Point3f cam_1_position =
          UtilsOpenCV::gtsamVector3ToCvPoint3(cam_1_pose.translation());
      cv::Point3f cam_2_position =
          UtilsOpenCV::gtsamVector3ToCvPoint3(cam_2_pose.translation());
      for (size_t i = 0u; i < lmks.size(); i++) {
        cv::Point3f lmk_cv = lmks[i];
        visualizeRay(lmk_cv, "lmk-cam1" + std::to_string(i), cam_1_position);
        visualizeRay(lmk_cv,
                     "lmk-cam2" + std::to_string(i),
                     cam_2_position,
                     0.2,
                     cv::viz::Color::red());
        pointcloud.push_back(cv::Mat(lmk_cv).reshape(1).t());
      }
      pointcloud = pointcloud.reshape(3, lmks.size());
      visualizePointCloud("Scene Landmarks", pointcloud);

      cv::Mat cam_1_img = cv::Mat(
          camera_1_params.image_size_, CV_8UC3, cv::Scalar(255u, 0u, 0u));
      cv::Mat cam_2_img = cv::Mat(
          camera_2_params.image_size_, CV_8UC3, cv::Scalar(255u, 0u, 0u));

      // Color image 1 with pixel projections
      drawPixelsOnImg(cam_1_kpts, cam_1_img, cv::viz::Color::brown(), 6u, 225u);

      // Color image 2 with pixel reprojections
      drawPixelsOnImg(cam_2_kpts, cam_2_img, cv::viz::Color::green(), 3u, 125u);

      cv::imshow("AHA1", cam_1_img);
      cv::imshow("AHA2", cam_2_img);
      cv::waitKey(0);

      // Camera frustums
      // cv::viz::WCameraPosition cpw_1_frustum(K, cam_1_img, 2.0);
      cv::viz::WCameraPosition cpw_1_frustum(
          K_1, cam_1_img, 1.0, cv::viz::Color::red());
      cv::viz::WCameraPosition cpw_2_frustum(
          K_2, cam_2_img, 1.0, cv::viz::Color::green());
      window_->showWidget("Cam 1 Frustum", cpw_1_frustum, cam_1_cv_pose);
      window_->showWidget("Cam 2 Frustum", cpw_2_frustum, cam_2_cv_pose);

      // Finally, spin
      spinDisplay();
    } else {
      LOG(WARNING) << "Requested scene visualization but display gflag is "
                   << "set to false.";
    }
  }

  // Display 3D window
  void spinDisplay() {
    CHECK(window_);
    window_->spin();
  }

 protected:
  // Perform Ransac
  TrackerParams tracker_params_;
  Tracker::UniquePtr tracker_;
  VIO::FeatureDetector::UniquePtr feature_detector_;

  Frame::Ptr ref_frame, cur_frame;
  StereoFrame::Ptr ref_stereo_frame, cur_stereo_frame;
  VIO::StereoCamera::ConstPtr stereo_camera_;
  VIO::StereoMatcher::UniquePtr stereo_matcher_;

 private:
  std::unique_ptr<cv::viz::Viz3d> window_;
};

/* ************************************************************************* */
TEST_F(TestTracker, geometricOutlierRejection2d2d) {
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
      ClearFrame(ref_frame.get());
      ClearFrame(cur_frame.get());

      VLOG(1) << "========================== \n"
              << "is_planar: " << is_planar << '\n'
              << " inlier_num: " << inlier_num << '\n'
              << " outlier_num: " << outlier_num << '\n'
              << " noise_sigma: " << noise_sigma;
      // add inliers
      if (is_planar) {
        Vector3 PlaneN(0.1, -0.1, 1);
        const double PlaneD = camRef_pose_camCur.translation().norm();
        AddPlanarInliersToFrame(ref_frame.get(),
                                cur_frame.get(),
                                camRef_pose_camCur,
                                PlaneN,
                                PlaneD,
                                inlier_num);
      } else {
        vector<double> depth_range;
        depth_range.push_back(camRef_pose_camCur.translation().norm());
        depth_range.push_back(10 * camRef_pose_camCur.translation().norm());
        AddNonPlanarInliersToFrame(ref_frame.get(),
                                   cur_frame.get(),
                                   camRef_pose_camCur,
                                   depth_range,
                                   inlier_num);
      }

      // add outliers
      AddOutliersToFrame(
          ref_frame.get(), cur_frame.get(), camRef_pose_camCur, outlier_num);
      // add noise
      if (noise_sigma != 0) {
        AddNoiseToFrame(ref_frame.get(), noise_sigma);
        AddNoiseToFrame(cur_frame.get(), noise_sigma);
      }

      // Perform Ransac
      TrackerParams trackerParams = TrackerParams();
      trackerParams.ransac_use_2point_mono_ = false;
      trackerParams.ransac_max_iterations_ = 1000;
      // trackerParams.ransac_probability_ = 0.8;
      trackerParams.ransac_randomize_ = false;
      Tracker tracker(trackerParams, stereo_camera_->getOriginalLeftCamera());
      TrackingStatus tracking_status;
      Pose3 estimated_pose;
      tie(tracking_status, estimated_pose) =
          tracker.geometricOutlierRejection2d2d(ref_frame.get(),
                                                cur_frame.get());

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
TEST_F(TestTracker, geometricOutlierRejection2d2dGivenRotation) {
  TrackerParams tracker_params = TrackerParams();
  tracker_params.ransac_use_1point_stereo_ = true;
  tracker_params.ransac_randomize_ = false;
  Tracker tracker(tracker_params, stereo_camera_->getOriginalLeftCamera());
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
      ClearFrame(ref_frame.get());
      ClearFrame(cur_frame.get());

      // add inliers
      if (is_planar) {
        Vector3 PlaneN(0.1, -0.1, 1);
        const double PlaneD = camRef_pose_camCur.translation().norm();
        AddPlanarInliersToFrame(ref_frame.get(),
                                cur_frame.get(),
                                camRef_pose_camCur,
                                PlaneN,
                                PlaneD,
                                inlier_num);
      } else {
        vector<double> depth_range;
        depth_range.push_back(camRef_pose_camCur.translation().norm());
        depth_range.push_back(10 * camRef_pose_camCur.translation().norm());
        AddNonPlanarInliersToFrame(ref_frame.get(),
                                   cur_frame.get(),
                                   camRef_pose_camCur,
                                   depth_range,
                                   inlier_num);
      }

      // add outliers
      AddOutliersToFrame(
          ref_frame.get(), cur_frame.get(), camRef_pose_camCur, outlier_num);
      // add noise
      if (noise_sigma != 0) {
        AddNoiseToFrame(ref_frame.get(), noise_sigma);
        AddNoiseToFrame(cur_frame.get(), noise_sigma);
      }

      // Perform Ransac
      TrackingStatus tracking_status;
      Pose3 estimated_pose;
      tie(tracking_status, estimated_pose) =
          tracker.geometricOutlierRejection2d2d(
              ref_frame.get(), cur_frame.get(), camRef_pose_camCur);

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
TEST_F(TestTracker, geometricOutlierRejection3d3d) {
  // Start with the simplest case:
  // Noise free, no outlier, non-planar

  CHECK(ref_stereo_frame->isRectified());
  Rot3 R = Rot3::Expmap(Vector3(0.1, 0.1, 0.1));
  Vector3 T(stereo_camera_->getBaseline(), 0, 0);
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
  test_configurations.push_back(std::make_tuple(true, 80, 40, 0.01));

  for (size_t testId = 0; testId < test_configurations.size(); testId++) {
    auto test_conf = test_configurations.at(testId);
    bool is_planar = get<0>(test_conf);
    int inlier_num = get<1>(test_conf);
    int outlier_num = get<2>(test_conf);
    double noise_sigma = get<3>(test_conf);
    for (int t = 0; t < repeat_times; t++) {
      ClearStereoFrame(ref_stereo_frame.get());
      ClearStereoFrame(cur_stereo_frame.get());

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
        AddPlanarInliersToStereoFrame(ref_stereo_frame.get(),
                                      cur_stereo_frame.get(),
                                      camLeftRef_pose_camLeftCur,
                                      PlaneN,
                                      PlaneD,
                                      inlier_num);
      } else {
        AddNonPlanarInliersToStereoFrame(ref_stereo_frame.get(),
                                         cur_stereo_frame.get(),
                                         camLeftRef_pose_camLeftCur,
                                         depth_range,
                                         inlier_num);
      }

      // add outliers
      AddOutliersToStereoFrame(ref_stereo_frame.get(),
                               cur_stereo_frame.get(),
                               camLeftRef_pose_camLeftCur,
                               depth_range,
                               outlier_num);

      // add noise
      if (noise_sigma != 0) {
        AddNoiseToStereoFrame(ref_stereo_frame.get(), noise_sigma);
        AddNoiseToStereoFrame(cur_stereo_frame.get(), noise_sigma);
      }

      TrackerParams trackerParams;
      trackerParams.ransac_threshold_stereo_ = 0.3;
      trackerParams.ransac_randomize_ = false;
      Tracker tracker(trackerParams, stereo_camera_->getOriginalLeftCamera());
      TrackingStatus tracking_status;
      Pose3 estimated_pose;
      tie(tracking_status, estimated_pose) =
          tracker.geometricOutlierRejection3d3d(ref_stereo_frame.get(),
                                                cur_stereo_frame.get());

      // Check the correctness of the outlier rejection!
      for (int i = 0; i < inlier_num; i++) {
        EXPECT_EQ(ref_stereo_frame->right_keypoints_rectified_.at(i).first,
                  KeypointStatus::VALID);
        EXPECT_NE(ref_stereo_frame->keypoints_depth_.at(i), 0.0);
        EXPECT_GT(
            (ref_stereo_frame->keypoints_3d_.at(i) - Vector3::Zero()).norm(),
            tol);

        EXPECT_EQ(cur_stereo_frame->right_keypoints_rectified_.at(i).first,
                  KeypointStatus::VALID);
        EXPECT_NE(ref_stereo_frame->keypoints_depth_.at(i), 0.0);
        EXPECT_GT(
            (cur_stereo_frame->keypoints_3d_.at(i) - Vector3::Zero()).norm(),
            tol);
      }

      for (int i = inlier_num; i < inlier_num + outlier_num; i++) {
        EXPECT_EQ(ref_stereo_frame->right_keypoints_rectified_.at(i).first,
                  KeypointStatus::FAILED_ARUN);
        EXPECT_EQ(ref_stereo_frame->keypoints_depth_.at(i), 0.0);
        EXPECT_LT(
            (ref_stereo_frame->keypoints_3d_.at(i) - Vector3::Zero()).norm(),
            tol);

        EXPECT_EQ(cur_stereo_frame->right_keypoints_rectified_.at(i).first,
                  KeypointStatus::FAILED_ARUN);
        EXPECT_EQ(ref_stereo_frame->keypoints_depth_.at(i), 0.0);
        EXPECT_LT(
            (cur_stereo_frame->keypoints_3d_.at(i) - Vector3::Zero()).norm(),
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
        tolPoint3 = 1e-1;  // noisy case
      }

      for (int i = 0; i < inlier_num; i++) {  // compare only inliers
        Vector3 right_pt_3d_exp = estimated_pose.inverse().rotation() *
                                      ref_stereo_frame->keypoints_3d_.at(i) +
                                  estimated_pose.inverse().translation();
        double diff =
            (right_pt_3d_exp - cur_stereo_frame->keypoints_3d_.at(i)).norm();
        EXPECT_LT(diff, tolPoint3);
      }
    }
  }
}

/* ************************************************************************* */
TEST_F(TestTracker, geometricOutlierRejection3d3dGivenRotation) {
  // Start with the simplest case:
  // Noise free, no outlier, non-planar

  CHECK(ref_stereo_frame->isRectified());
  Rot3 R = Rot3::Expmap(Vector3(0.1, 0.1, 0.1));
  Vector3 T(stereo_camera_->getBaseline(), 0, 0);
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
  test_configurations.push_back(std::make_tuple(true, 80, 40, 0.01));

  for (size_t testId = 0; testId < test_configurations.size(); testId++) {
    auto test_conf = test_configurations.at(testId);
    bool is_planar = get<0>(test_conf);
    int inlier_num = get<1>(test_conf);
    int outlier_num = get<2>(test_conf);
    double noise_sigma = get<3>(test_conf);
    for (int t = 0; t < repeat_times; t++) {
      ClearStereoFrame(ref_stereo_frame.get());
      ClearStereoFrame(cur_stereo_frame.get());

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
        AddPlanarInliersToStereoFrame(ref_stereo_frame.get(),
                                      cur_stereo_frame.get(),
                                      camLeftRef_pose_camLeftCur,
                                      PlaneN,
                                      PlaneD,
                                      inlier_num);
      } else {
        AddNonPlanarInliersToStereoFrame(ref_stereo_frame.get(),
                                         cur_stereo_frame.get(),
                                         camLeftRef_pose_camLeftCur,
                                         depth_range,
                                         inlier_num);
      }

      // add outliers
      AddOutliersToStereoFrame(ref_stereo_frame.get(),
                               cur_stereo_frame.get(),
                               camLeftRef_pose_camLeftCur,
                               depth_range,
                               outlier_num);

      // add noise
      if (noise_sigma != 0) {
        AddNoiseToStereoFrame(ref_stereo_frame.get(), noise_sigma);
        AddNoiseToStereoFrame(cur_stereo_frame.get(), noise_sigma);
      }

      // Perform Ransac
      pair<TrackingStatus, Pose3> poseStatus;
      Matrix3 infoMat;
      tie(poseStatus, infoMat) =
          tracker_->geometricOutlierRejection3d3dGivenRotation(
              *ref_stereo_frame,
              *cur_stereo_frame,
              stereo_camera_->getGtsamStereoCam(),
              R);

      TrackingStatus tracking_status = poseStatus.first;
      Pose3 estimated_pose = poseStatus.second;
      // Check the correctness of the outlier rejection!
      for (int i = 0; i < inlier_num; i++) {
        EXPECT_EQ(ref_stereo_frame->right_keypoints_rectified_.at(i).first,
                  KeypointStatus::VALID);
        EXPECT_NE(ref_stereo_frame->keypoints_depth_.at(i), 0.0);
        EXPECT_GT(
            (ref_stereo_frame->keypoints_3d_.at(i) - Vector3::Zero()).norm(),
            tol);

        EXPECT_EQ(cur_stereo_frame->right_keypoints_rectified_.at(i).first,
                  KeypointStatus::VALID);
        EXPECT_NE(ref_stereo_frame->keypoints_depth_.at(i), 0.0);
        EXPECT_GT(
            (cur_stereo_frame->keypoints_3d_.at(i) - Vector3::Zero()).norm(),
            tol);
      }

      for (int i = inlier_num; i < inlier_num + outlier_num; i++) {
        EXPECT_EQ(ref_stereo_frame->right_keypoints_rectified_.at(i).first,
                  KeypointStatus::FAILED_ARUN);
        EXPECT_EQ(ref_stereo_frame->keypoints_depth_.at(i), 0.0);
        EXPECT_LT(
            (ref_stereo_frame->keypoints_3d_.at(i) - Vector3::Zero()).norm(),
            tol);

        EXPECT_EQ(cur_stereo_frame->right_keypoints_rectified_.at(i).first,
                  KeypointStatus::FAILED_ARUN);
        EXPECT_EQ(ref_stereo_frame->keypoints_depth_.at(i), 0.0);
        EXPECT_LT(
            (cur_stereo_frame->keypoints_3d_.at(i) - Vector3::Zero()).norm(),
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
                                      ref_stereo_frame->keypoints_3d_.at(i) +
                                  estimated_pose.inverse().translation();
        double diff =
            (right_pt_3d_exp - cur_stereo_frame->keypoints_3d_.at(i)).norm();
        EXPECT_LT(diff, tolPoint3);
      }
    }
  }
}

/* ************************************************************************* */
TEST_F(TestTracker, getPoint3AndCovariance) {
  ClearStereoFrame(ref_stereo_frame.get());
  // create stereo cam
  VIO::StereoCamera ref_stereo_camera(
      ref_stereo_frame->left_frame_.cam_param_,
      ref_stereo_frame->right_frame_.cam_param_);
  gtsam::StereoCamera stereoCam =
      gtsam::StereoCamera(gtsam::Pose3(), ref_stereo_camera.getStereoCalib());

  // create a stereo point:
  double xL = 379.999 / 2;  // in the middle of the image
  double v = 255.238 / 2;   // in the middle of the image
  double xR = xL - 10;      // some disparity
  StereoPoint2 stereoPoint(xL, xR, v);

  // create a 3D point in front of the camera
  Vector3 point3 = stereoCam.backproject2(stereoPoint);
  int pointId = 0;  // only point

  Matrix3 stereoPtCov = Matrix3::Identity();

  // populate stereoFrame:
  ref_stereo_frame->left_keypoints_rectified_.push_back(
      StatusKeypointCV(KeypointStatus::VALID, KeypointCV(xL, v)));
  ref_stereo_frame->right_keypoints_rectified_.push_back(
      StatusKeypointCV(KeypointStatus::VALID, KeypointCV(xR, v)));
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
  EXPECT_TRUE(assert_equal(f_ref_i_expected, f_ref_i_actual, 0.2));
  EXPECT_TRUE(assert_equal(cov_ref_i_expected, cov_ref_i_actual, 0.2));
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
  ClearFrame(ref_frame.get());
  ClearFrame(cur_frame.get());

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
  ClearStereoFrame(ref_stereo_frame.get());
  ClearStereoFrame(cur_stereo_frame.get());

  const int num_landmarks_common = 100;
  const int num_landmarks_ref = 90;
  const int num_landmarks_cur = 80;
  const int num_landmarks_invalid = 70;

  ref_stereo_frame->left_frame_.landmarks_.reserve(
      num_landmarks_common + num_landmarks_ref + num_landmarks_invalid);
  cur_stereo_frame->left_frame_.landmarks_.reserve(
      num_landmarks_common + num_landmarks_cur + num_landmarks_invalid);

  // landmark ids in common!
  for (int i = 0; i < num_landmarks_common; i++) {
    ref_stereo_frame->left_frame_.landmarks_.push_back(3 * i);
    cur_stereo_frame->left_frame_.landmarks_.push_back(3 * i);
  }

  // landmark ids unique to ref_stereo_frame
  for (int i = 0; i < num_landmarks_ref; i++) {
    ref_stereo_frame->left_frame_.landmarks_.push_back(3 * i + 1);
  }

  // landmark ids unique to ref_stereo_frame
  for (int i = 0; i < num_landmarks_cur; i++) {
    cur_stereo_frame->left_frame_.landmarks_.push_back(3 * i + 2);
  }

  // add a bunch of invalid landmarks to both frames
  for (int i = 0; i < num_landmarks_invalid; i++) {
    ref_stereo_frame->left_frame_.landmarks_.push_back(-1);
    cur_stereo_frame->left_frame_.landmarks_.push_back(-1);
  }

  // shuffle landmarks in both frames
  random_shuffle(ref_stereo_frame->left_frame_.landmarks_.begin(),
                 ref_stereo_frame->left_frame_.landmarks_.end());
  random_shuffle(cur_stereo_frame->left_frame_.landmarks_.begin(),
                 cur_stereo_frame->left_frame_.landmarks_.end());

  //   Set right_keypoints_status!
  ref_stereo_frame->right_keypoints_rectified_.resize(
      ref_stereo_frame->left_frame_.landmarks_.size());
  for (int i = 0; i < ref_stereo_frame->left_frame_.landmarks_.size(); i++) {
    int l_id = ref_stereo_frame->left_frame_.landmarks_[i];
    if (l_id % 6 == 0) {
      ref_stereo_frame->right_keypoints_rectified_.at(i).first =
          KeypointStatus::VALID;
    } else {
      ref_stereo_frame->right_keypoints_rectified_.at(i).first =
          KeypointStatus::NO_RIGHT_RECT;
    }
  }

  //   Set right_keypoints_status!
  cur_stereo_frame->right_keypoints_rectified_.resize(
      cur_stereo_frame->left_frame_.landmarks_.size());
  for (int i = 0; i < cur_stereo_frame->left_frame_.landmarks_.size(); i++) {
    int l_id = cur_stereo_frame->left_frame_.landmarks_[i];
    if (l_id % 6 == 0) {
      cur_stereo_frame->right_keypoints_rectified_.at(i).first =
          KeypointStatus::VALID;
    } else {
      cur_stereo_frame->right_keypoints_rectified_.at(i).first =
          KeypointStatus::NO_RIGHT_RECT;
    }
  }

  vector<pair<size_t, size_t>> matches_ref_cur;
  Tracker::findMatchingStereoKeypoints(
      *ref_stereo_frame, *cur_stereo_frame, &matches_ref_cur);

  // Check the correctness!
  EXPECT_EQ(matches_ref_cur.size(), (num_landmarks_common + 1) / 2);
  set<int> landmarks_found;
  for (auto match_ref_cur : matches_ref_cur) {
    int l_ref = ref_stereo_frame->left_frame_.landmarks_[match_ref_cur.first];
    int l_cur = cur_stereo_frame->left_frame_.landmarks_[match_ref_cur.second];

    EXPECT_EQ(l_ref, l_cur);
    EXPECT_EQ(l_ref % 6, 0);
    EXPECT_EQ(landmarks_found.find(l_ref), landmarks_found.end());

    landmarks_found.insert(l_ref);
  }
}

/* ************************************************************************* */
TEST_F(TestTracker, MahalanobisDistance) {
  auto timeBefore = VIO::utils::Timer::tic();
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
    timeBefore = VIO::utils::Timer::tic();
    Vector3f Omega_relTran_j = O.llt().solve(v);
    float innovationMahalanobisNorm1 = v.transpose() * Omega_relTran_j;
    time1 += VIO::utils::Timer::toc(timeBefore).count();

    // sol2 - still 0.25 seconds for 200 features
    timeBefore = VIO::utils::Timer::tic();
    Matrix3f infoMatSum = O.inverse();
    float innovationMahalanobisNorm2 = v.transpose() * infoMatSum * v;
    time2 += VIO::utils::Timer::toc(timeBefore).count();

    // sol3 - still 0.25 seconds for 200 features
    timeBefore = VIO::utils::Timer::tic();
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
    time3 += VIO::utils::Timer::toc(timeBefore).count();

    EXPECT_NEAR(double(innovationMahalanobisNorm1),
                double(innovationMahalanobisNorm2),
                1e-2);
    EXPECT_NEAR(double(innovationMahalanobisNorm1),
                double(innovationMahalanobisNorm3),
                1e-2);
    EXPECT_NEAR(double(1 / dinv), double(O.determinant()), 1e-4);
  }
  VLOG(1) << "time1 (llt): " << time1 << '\n'
          << "time2 (x'*O*x): " << time2 << '\n'
          << "time3 (manual): " << time3;
}

TEST_F(TestTracker, FeatureTrackingRotationalOpticalFlow) {
  // Load one Euroc image

  // Create second image with homography: small rotation

  // Feed frames: detect kpts.

  // Run feature tracking

  // Check cur_frame is correctly populated (rotate features in ref_frame to
  // get the expected keypoints).
}

TEST_F(TestTracker, FeatureTrackingNoOpticalFlowPrediction) {
  // Load one Euroc image

  // Create second image with homography: small rotation

  // Feed frames: detect kpts.

  // Run feature tracking

  // Check cur_frame is correctly populated (rotate features in ref_frame to
  // get the expected keypoints).
}

TEST_F(TestTracker, FeatureTrackingRotationalOpticalFlowPredictionLargeRot) {
  // Load one Euroc image

  // Create second image with homography: LARGE rotation

  // Feed frames: detect kpts.

  // Run feature tracking

  // Check cur_frame is correctly populated (rotate features in ref_frame to
  // get the expected keypoints).
}

TEST_F(TestTracker, FeatureTrackingNoOpticalFlowPredictionLargeRot) {
  // Load one Euroc image

  // Create second image with homography: LARGE rotation

  // Feed frames: detect kpts.

  // Run feature tracking

  // Check cur_frame is correctly populated (rotate features in ref_frame to
  // get the expected keypoints).

  // SHOULD kind of fail...
  // EXPECT_NEAR kpts actual vs kpts expected (large tolerance).
}

TEST_F(TestTracker,
       FeatureTrackingRotationalOpticalFlowPredictionWithLargeRot) {}

// TODO(Toni): copy of the function from PR 420
void getBearingVectorFromUndistortedKeypoint(
    const KeypointCV& undistorted_keypoint,
    const cv::Mat& P,
    gtsam::Vector3* bearing_vector) {
  CHECK_NOTNULL(bearing_vector);
  CHECK(!P.empty());
  CHECK_EQ(P.rows, 3);
  CHECK_EQ(P.cols, 4);
  cv::Mat K_inv = P.colRange(0, 3).inv();
  // Has to be a double because K is a matrix of doubles.
  cv::Vec3d homogeneous_keypoint(
      undistorted_keypoint.x, undistorted_keypoint.y, 1.0);
  *bearing_vector =
      UtilsOpenCV::cvMatToGtsamPoint3(K_inv * cv::Mat(homogeneous_keypoint));
  // Bearing vectors have unit norm.
  bearing_vector->normalize();
  CHECK_DOUBLE_EQ(bearing_vector->norm(), 1.0);
}

TEST_F(TestTracker, PnPTracking) {
  CameraParams cam_params_left, cam_params_right;
  cam_params_left.parseYAML(stereo_test_data_path + "/sensorLeft.yaml");
  cam_params_right.parseYAML(stereo_test_data_path + "/sensorRight.yaml");

  //! Set camera looking at the unitary cube of landmarks (see below), and
  //! put it a bit back in x axis to avoid cheirality exception.
  gtsam::Pose3 W_Pose_body =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(0.0, 0.0, -2.0));
  cam_params_left.body_Pose_cam_ = W_Pose_body;
  cam_params_right.body_Pose_cam_ =
      gtsam::Pose3(gtsam::Rot3(), {1.0, .0, -2.0});
  // W_Pose_body.compose(cam_params_right.body_Pose_cam_);

  //! Setup stereo camera
  stereo_camera_ =
      std::make_shared<VIO::StereoCamera>(cam_params_left, cam_params_right);

  //! Setup tracker to use pnp
  tracker_params_.pnp_algorithm_ = Pose3d2dAlgorithm::EPNP;
  tracker_params_.min_pnp_inliers_ = 10;
  tracker_params_.ransac_threshold_pnp_ = 0.5;

  //! Create pnp tracker
  tracker_ = std::make_unique<Tracker>(tracker_params_,
                                       stereo_camera_->getOriginalLeftCamera());

  //! Populate data
  //! - with inliers:
  //!   a) Generate 3D landmarks in non-planar configuration
  //!   b) Project these to stereo_camera_ to generate inlier 2D keypoints.
  // USE buildSceneLandmarks from testOpticalFlowPredictor...
  static constexpr double kCubeSideSize = 1.0;
  LandmarksCV inlier_lmks = {
      kCubeSideSize * LandmarkCV(0.0, 0.0, 0.0),
      kCubeSideSize * LandmarkCV(0.0, 0.0, 1.0),
      kCubeSideSize * LandmarkCV(0.0, 1.0, 0.0),
      kCubeSideSize * LandmarkCV(0.0, 1.0, 1.0),
      kCubeSideSize * LandmarkCV(1.0, 0.0, 0.0),
      kCubeSideSize * LandmarkCV(1.0, 0.0, 1.0),
      kCubeSideSize * LandmarkCV(1.0, 1.0, 0.0),
      kCubeSideSize * LandmarkCV(1.0, 1.0, 1.0),
      //! Some random ones
      kCubeSideSize * LandmarkCV(0.3, 0.2, 0.2),
      kCubeSideSize * LandmarkCV(0.2, 0.1, 0.8),
      kCubeSideSize * LandmarkCV(0.4, 0.2, 0.2),
      kCubeSideSize * LandmarkCV(0.4, 0.3, 0.9),
      kCubeSideSize * LandmarkCV(-0.3, 0.3, 0.9),
      kCubeSideSize * LandmarkCV(0.8, -0.1, 0.3),
      kCubeSideSize * LandmarkCV(-0.8, -0.7, 0.3),
      kCubeSideSize * LandmarkCV(-0.2, 0.1, 0.3),
      kCubeSideSize * LandmarkCV(0.1, -0.2, 0.3),
      kCubeSideSize * LandmarkCV(-0.4, 0.3, 0.3),
      kCubeSideSize * LandmarkCV(0.3, -0.1, 0.3),
      kCubeSideSize * LandmarkCV(-0.3, -0.3, -0.2),
      kCubeSideSize * LandmarkCV(-0.6, -0.2, -0.3),
      kCubeSideSize * LandmarkCV(-0.7, -0.0, -0.1),
  };
  KeypointsCV left_inlier_kpts, right_inlier_kpts;
  gtsam::Pose3 camL_Pose_camR =
      (cam_params_left.body_Pose_cam_).between(cam_params_right.body_Pose_cam_);

  stereo_camera_->project(inlier_lmks, &left_inlier_kpts, &right_inlier_kpts);
  //!   c) Add them to stereo frame
  size_t lmk_id = 0;
  LandmarkIds lmk_ids;
  StatusKeypointsCV left_inlier_status_kpts;
  BearingVectors bearing_vectors;
  for (const auto& kpt : left_inlier_kpts) {
    VLOG(5) << "Keypoint: " << kpt;
    // What if the keypoint is out of image bounds?
    left_inlier_status_kpts.push_back(
        std::make_pair(KeypointStatus::VALID, kpt));
    lmk_ids.push_back(lmk_id++);
    gtsam::Vector3 bearing_vector;
    // TODO(Toni): use the function from PR 420
    getBearingVectorFromUndistortedKeypoint(
        kpt, stereo_camera_->getP1(), &bearing_vector);
    bearing_vectors.push_back(bearing_vector);
  }
  cur_stereo_frame->left_frame_.landmarks_ = lmk_ids;
  cur_stereo_frame->left_keypoints_rectified_ = left_inlier_status_kpts;
  cur_stereo_frame->keypoints_3d_ = bearing_vectors;

  //! - with random outliers:
  //!   a) Generate random 3D landmarks
  //!   b) Associate to random keypoints
  LandmarksCV outlier_lmks = {
      LandmarkCV(1.0, 2.3, 0.4),
      LandmarkCV(0.3, 0.3, 0.4),
      LandmarkCV(1.5, 1.3, 0.4),
  };
  KeypointsCV left_outlier_kpts = {
      KeypointCV(100, 23),
      KeypointCV(234, 223),
      KeypointCV(400, 543),
  };
  //!   c) Add them to stereo frame
  for (size_t i = 0; i < outlier_lmks.size(); i++) {
    lmk_ids.push_back(lmk_id++);  //! update lmk ids, needed for LanmdarksMap
    cur_stereo_frame->left_frame_.landmarks_.push_back(lmk_ids.back());
    cur_stereo_frame->left_keypoints_rectified_.push_back(
        std::make_pair(KeypointStatus::VALID, left_outlier_kpts[i]));
    gtsam::Vector3 bearing_vector;
    // TODO(Toni): use the function from PR 420
    getBearingVectorFromUndistortedKeypoint(
        left_outlier_kpts[i], stereo_camera_->getP1(), &bearing_vector);
    cur_stereo_frame->keypoints_3d_.push_back(bearing_vector);
  }

  //! Update tracker's map of landmarks.
  LandmarksMap landmarks_map;
  //! inlier lmks
  for (size_t i = 0; i < inlier_lmks.size(); i++) {
    landmarks_map[lmk_ids[i]] =
        gtsam::Point3(inlier_lmks[i].x, inlier_lmks[i].y, inlier_lmks[i].z);
  }
  //! outlier lmks
  for (size_t i = 0; i < outlier_lmks.size(); i++) {
    landmarks_map[lmk_ids[inlier_lmks.size() + i]] =
        gtsam::Point3(outlier_lmks[i].x, outlier_lmks[i].y, outlier_lmks[i].z);
  }
  tracker_->updateMap(landmarks_map);

  VLOG(5) << "Landmarks Map \n";
  for (const auto& lmk_id : landmarks_map) {
    VLOG(5) << lmk_id.first << ", (" << lmk_id.second.x() << ", "
            << lmk_id.second.y() << ", " << lmk_id.second.z() << ")";
  }

  //! Estimate pose with PnP
  gtsam::Pose3 best_absolute_pose;
  std::vector<int> inliers;
  EXPECT_TRUE(tracker_->pnp(*cur_stereo_frame, &best_absolute_pose, &inliers));
  //! Check inliers/outliers
  EXPECT_EQ(inliers.size(), inlier_lmks.size());

  if (FLAGS_display) {
    size_t k = 0;
    for (const auto& bearing_vector : cur_stereo_frame->keypoints_3d_) {
      const gtsam::Pose3& left_cam_pose =
          stereo_camera_->getBodyPoseLeftCamRect();
      gtsam::Point3 bearing_tip =
          left_cam_pose.translation() + gtsam::Point3(bearing_vector);
      visualizeRay(
          UtilsOpenCV::gtsamVector3ToCvPoint3(bearing_tip),
          "bearing-cam1" + std::to_string(k++),
          UtilsOpenCV::gtsamVector3ToCvPoint3(left_cam_pose.translation()),
          0.2,
          cv::viz::Color::red());
    }

    visualizeScene(cam_params_left,
                   cam_params_right,
                   left_inlier_kpts,
                   right_inlier_kpts,
                   inlier_lmks);
  }

  //! Check returned 3D pose
  gtsam::Pose3 expected = stereo_camera_->getBodyPoseLeftCamRect();
  gtsam::Rot3 expected_rot = expected.rotation();
  gtsam::Point3 expected_t = expected.translation();
  gtsam::Rot3 actual_rot = best_absolute_pose.rotation();
  gtsam::Point3 actual_t = best_absolute_pose.translation();
  static constexpr double tol = 0.00001;
  EXPECT_NEAR(expected.x(), best_absolute_pose.x(), tol);
  EXPECT_NEAR(expected.y(), best_absolute_pose.y(), tol);
  EXPECT_NEAR(expected.z(), best_absolute_pose.z(), tol);

  EXPECT_NEAR(expected_rot.toQuaternion().x(),
              best_absolute_pose.rotation().toQuaternion().x(),
              tol);
  EXPECT_NEAR(expected_rot.toQuaternion().y(),
              best_absolute_pose.rotation().toQuaternion().y(),
              tol);
  EXPECT_NEAR(expected_rot.toQuaternion().z(),
              best_absolute_pose.rotation().toQuaternion().z(),
              tol);
  EXPECT_NEAR(expected_rot.toQuaternion().w(),
              best_absolute_pose.rotation().toQuaternion().w(),
              tol);
}
