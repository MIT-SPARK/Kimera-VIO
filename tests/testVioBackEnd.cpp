/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testVioBackEnd.h
 * @brief  test VioBackEnd
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

#include "kimera-vio/backend/VioBackEnd.h"
#include "kimera-vio/common/vio_types.h"
// only for gtNavState...
#include "kimera-vio/dataprovider/DataProviderInterface-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontEndParams.h"
#include "kimera-vio/initial/InitializationBackEnd.h"
#include "kimera-vio/utils/ThreadsafeImuBuffer.h"

DECLARE_string(test_data_path);

namespace VIO {

static const double tol = 1e-7;

/* ************************************************************************* */
// Parameters
static const int num_key_frames =
    10;  // number of frames of the synthetic scene
static const gtsam::Vector3 p0(0, 0, 0);  // initial pose of the robot camera
static const gtsam::Vector3 v(1.0,
                              0,
                              0);  // velocity of the robot, per time_step
static const int time_step =
    1e9;  // elapsed time between two consecutive frames is 1 second (1e9 nsecs)
static const Timestamp t_start = 1e9;  // ImuBuffer does not allow t = 0;
static const double baseline = 0.5;
static const gtsam::imuBias::ConstantBias imu_bias(
    gtsam::Vector3(0.1, -0.1, 0.3),
    gtsam::Vector3(0.1, 0.3, -0.2));

using StereoPoses = std::vector<std::pair<gtsam::Pose3, gtsam::Pose3>>;

/* ************************************************************************* */
// Helper functions!
std::vector<Point3> CreateScene() {
  std::vector<Point3> points;
  // a scene with 8 points
  points.push_back(Point3(0, 0, 20));
  points.push_back(Point3(0, 20, 20));
  points.push_back(Point3(20, 20, 20));
  points.push_back(Point3(20, 0, 20));

  points.push_back(Point3(5, 5, 25));
  points.push_back(Point3(5, 15, 25));
  points.push_back(Point3(15, 15, 25));
  points.push_back(Point3(15, 5, 25));

  return points;
}
/* ------------------------------------------------------------------------- */
StereoPoses CreateCameraPoses(const int num_keyframes,
                              const double baseline,
                              const gtsam::Vector3 p0,
                              const gtsam::Vector3 v) {
  StereoPoses poses;
  poses.reserve(num_keyframes);
  Pose3 L_pose_R(Rot3::identity(), gtsam::Point3(baseline, 0, 0));

  // The camera is assumed to face (0, 0, 1): z-forward, y down and x to the
  // right
  for (int f_id = 0; f_id < num_keyframes; f_id++) {
    // constant velocity model along x: the robot moves to the right, keeping
    // the point in front of the camera
    gtsam::Vector3 p_offset = v * f_id * (time_step / ((double)1e9));

    Pose3 pose_left(Rot3::identity(), p0 + p_offset);
    Pose3 pose_right = pose_left.compose(L_pose_R);

    poses.push_back(std::make_pair(pose_left, pose_right));
  }
  return poses;
}

/* ------------------------------------------------------------------------- */
void CreateImuBuffer(VIO::utils::ThreadsafeImuBuffer& imu_buf,
                     const int num_frames,
                     const gtsam::Vector3 v,
                     const ImuBias imu_bias,
                     const gtsam::Vector3 n_gravity,
                     const Timestamp time_step,
                     const Timestamp t_start) {
  // Synthesize IMU measurements
  for (int f_id = 0; f_id < num_frames; f_id++) {
    Vector6 acc_gyr;
    // constant speed, no acceleration
    acc_gyr.head(3) = -n_gravity + imu_bias.accelerometer();
    // Camera axis aligned with the world axis in this example
    acc_gyr.tail(3) = imu_bias.gyroscope();
    Timestamp t = int64_t(f_id) * time_step + t_start;
    imu_buf.addMeasurement(t, acc_gyr);
    LOG(INFO) << "Timestamp: " << t;
    LOG(INFO) << "Accgyr: " << acc_gyr;
  }
}

/* ************************************************************************* */
TEST(testVio, robotMovingWithConstantVelocity) {
  // Additional parameters
  VioBackEndParams vioParams;
  vioParams.landmarkDistanceThreshold_ = 30;  // we simulate points 20m away
  vioParams.horizon_ = 100;

  ImuParams imu_params;
  imu_params.gyro_noise_ = 0.00016968;
  imu_params.acc_noise_ = 0.002;
  imu_params.gyro_walk_ = 1.9393e-05;
  imu_params.acc_walk_ = 0.003;
  imu_params.n_gravity_ = gtsam::Vector3(0.0, 0.0, -9.81);
  imu_params.imu_integration_sigma_ = 1.0;
  imu_params.nominal_rate_ = 200.0;
  // TODO(Toni): test with Combined, I think it actually fails now...
  imu_params.imu_preintegration_type_ =
      ImuPreintegrationType::kPreintegratedImuMeasurements;

  // Create 3D points
  std::vector<Point3> pts = CreateScene();
  const int num_pts = pts.size();

  // Create cameras
  double fov = M_PI / 3 * 2;
  // Create image size to initiate meaningful intrinsic camera matrix
  double img_height = 600;
  double img_width = 800;
  double fx = img_width / 2 / tan(fov / 2);
  double fy = fx;
  double s = 0;
  double u0 = img_width / 2;
  double v0 = img_height / 2;

  Cal3_S2 cam_params(fx, fy, s, u0, v0);

  // Create camera poses and IMU data
  StereoPoses poses;
  VIO::utils::ThreadsafeImuBuffer imu_buf(-1);
  poses = CreateCameraPoses(num_key_frames, baseline, p0, v);
  CreateImuBuffer(imu_buf,
                  num_key_frames,
                  v,
                  imu_bias,
                  imu_params.n_gravity_,
                  time_step,
                  t_start);

  // Create measurements
  //    using SmartStereoMeasurement = pair<LandmarkId,StereoPoint2>;
  //    using SmartStereoMeasurements = vector<SmartStereoMeasurement>;
  //    using StatusSmartStereoMeasurements =
  //    pair<TrackerStatusSummary,SmartStereoMeasurements>;
  TrackerStatusSummary tracker_status_valid;
  tracker_status_valid.kfTrackingStatus_mono_ = TrackingStatus::VALID;
  tracker_status_valid.kfTrackingStatus_stereo_ = TrackingStatus::VALID;

  std::vector<StatusStereoMeasurementsPtr> all_measurements;
  for (int i = 0; i < num_key_frames; i++) {
    gtsam::PinholeCamera<Cal3_S2> cam_left(poses[i].first, cam_params);
    gtsam::PinholeCamera<Cal3_S2> cam_right(poses[i].second, cam_params);
    SmartStereoMeasurements measurement_frame;
    for (int l_id = 0; l_id < num_pts; l_id++) {
      Point2 pt_left = cam_left.project2(pts[l_id]);
      Point2 pt_right = cam_right.project2(pts[l_id]);
      StereoPoint2 pt_lr(pt_left.x(), pt_right.x(), pt_left.y());
      EXPECT_DOUBLE_EQ(pt_left.y(), pt_right.y());
      measurement_frame.push_back(std::make_pair(l_id, pt_lr));
    }
    all_measurements.push_back(std::make_shared<StatusStereoMeasurements>(
        std::make_pair(tracker_status_valid, measurement_frame)));
  }

  // create vio
  Pose3 B_pose_camLrect;
  VioNavState initial_state = VioNavState(poses[0].first, v, imu_bias);
  StereoCalibPtr stereo_calibration =
      boost::make_shared<gtsam::Cal3_S2Stereo>(cam_params.fx(),
                                               cam_params.fy(),
                                               cam_params.skew(),
                                               cam_params.px(),
                                               cam_params.py(),
                                               baseline);
  // Create frontend.
  ImuFrontEnd imu_frontend(imu_params, imu_bias);

  // Create backend.
  std::shared_ptr<VioBackEnd> vio =
      std::make_shared<VioBackEnd>(B_pose_camLrect,
                                   stereo_calibration,
                                   vioParams,
                                   imu_params,
                                   BackendOutputParams(false, 0, false),
                                   false);
  vio->registerImuBiasUpdateCallback(std::bind(
      &ImuFrontEnd::updateBias, std::ref(imu_frontend), std::placeholders::_1));
  vio->initStateAndSetPriors(VioNavStateTimestamped(t_start, initial_state));

  // For each frame, add landmarks and optimize.
  for (FrameId k = 1; k < num_key_frames; k++) {
    // Time stamp for the current keyframe and the next frame.
    Timestamp timestamp_lkf = (k - 1) * time_step + t_start;
    Timestamp timestamp_k = k * time_step + t_start;

    // Get the IMU data
    ImuStampS imu_stamps;
    ImuAccGyrS imu_accgyr;
    CHECK(imu_buf.getImuDataInterpolatedUpperBorder(
              timestamp_lkf, timestamp_k, &imu_stamps, &imu_accgyr) ==
          VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);

    const auto& pim =
        imu_frontend.preintegrateImuMeasurements(imu_stamps, imu_accgyr);

    // process data with VIO
    vio->spinOnce(BackendInput(timestamp_k,
                               all_measurements[k],
                               tracker_status_valid.kfTrackingStatus_stereo_,
                               pim));
    // At this point the update imu bias callback should be triggered which
    // will update the imu_frontend imu bias.
    imu_frontend.resetIntegrationWithCachedBias();

    const gtsam::NonlinearFactorGraph& nlfg = vio->getFactorsUnsafe();
    size_t nrFactorsInSmoother = 0;
    for (const auto& f : nlfg) {  // count the number of nonempty factors
      if (f) nrFactorsInSmoother++;
    }
    LOG(INFO) << "at frame " << k << " nr factors: " << nrFactorsInSmoother;

    if (imu_params.imu_preintegration_type_ ==
        ImuPreintegrationType::kPreintegratedCombinedMeasurements) {
      EXPECT_EQ(nrFactorsInSmoother,
                3 + k + 8);  // 3 priors, 1 imu per time stamp, 8 smart factors
    } else {
      if (k == 1) {
        EXPECT_EQ(nrFactorsInSmoother,
                  3 + 2 * k);  // 3 priors, 1 imu + 1 between per time stamp: we
                               // do not include smart factors of length 1
      } else {
        EXPECT_EQ(nrFactorsInSmoother,
                  3 + 2 * k + 8);  // 3 priors, 1 imu + 1 between per time
                                   // stamp, 8 smart factors
      }
    }
    // Check the results!
    const gtsam::Values& results = vio->getState();

    for (FrameId f_id = 0; f_id <= k; f_id++) {
      Pose3 W_Pose_Blkf = results.at<gtsam::Pose3>(gtsam::Symbol('x', f_id));
      gtsam::Vector3 W_Vel_Blkf =
          results.at<gtsam::Vector3>(gtsam::Symbol('v', f_id));
      ImuBias imu_bias_lkf = results.at<ImuBias>(gtsam::Symbol('b', f_id));

      EXPECT_TRUE(assert_equal(poses[f_id].first, W_Pose_Blkf, tol));
      EXPECT_LT((W_Vel_Blkf - v).norm(), tol);
      EXPECT_LT((imu_bias_lkf - imu_bias).vector().norm(), tol);
    }
  }
}

/* ************************************************************************* */
// TODO(Sandro): Move this test to separate file!
TEST(testVio, robotMovingWithConstantVelocityBundleAdjustment) {
  // Additional parameters
  VioBackEndParams vioParams;
  vioParams.landmarkDistanceThreshold_ = 100;  // we simulate points 30-40m away
  vioParams.horizon_ = 100;
  vioParams.smartNoiseSigma_ = 0.001;
  vioParams.outlierRejection_ = 100;
  vioParams.betweenTranslationPrecision_ = 1;

  ImuParams imu_params;
  imu_params.gyro_noise_ = 0.00016968;
  imu_params.acc_noise_ = 0.002;
  imu_params.gyro_walk_ = 1.9393e-05;
  imu_params.acc_walk_ = 0.003;
  imu_params.n_gravity_ = gtsam::Vector3(0.0, 0.0, -9.81);
  imu_params.imu_integration_sigma_ = 1.0;
  imu_params.nominal_rate_ = 200.0;

  // Create 3D points
  std::vector<Point3> pts = CreateScene();
  const int num_pts = pts.size();

  // Create cameras
  double fov = M_PI / 3 * 2;
  // Create image size to initiate meaningful intrinsic camera matrix
  double img_height = 600;
  double img_width = 800;
  double fx = img_width / 2 / tan(fov / 2);
  double fy = fx;
  double s = 0;
  double u0 = img_width / 2;
  double v0 = img_height / 2;

  // Random noise generator for ransac pose (concatenated!)
  double rad_sigma = 0.005;
  double pos_sigma = 0.01;

  Cal3_S2 cam_params(fx, fy, s, u0, v0);

  // Create camera poses and IMU data
  StereoPoses poses;
  VIO::utils::ThreadsafeImuBuffer imu_buf(-1);
  poses = CreateCameraPoses(num_key_frames, baseline, p0, v);
  CreateImuBuffer(imu_buf,
                  num_key_frames,
                  v,
                  imu_bias,
                  imu_params.n_gravity_,
                  time_step,
                  t_start);

  // Create measurements
  TrackerStatusSummary tracker_status_valid;
  tracker_status_valid.kfTrackingStatus_mono_ = TrackingStatus::VALID;
  tracker_status_valid.kfTrackingStatus_stereo_ = TrackingStatus::VALID;

  std::vector<StatusStereoMeasurementsPtr> all_measurements;
  for (int i = 0; i < num_key_frames; i++) {
    gtsam::PinholeCamera<Cal3_S2> cam_left(poses[i].first, cam_params);
    gtsam::PinholeCamera<Cal3_S2> cam_right(poses[i].second, cam_params);
    SmartStereoMeasurements measurement_frame;
    for (int l_id = 0; l_id < num_pts; l_id++) {
      Point2 pt_left = cam_left.project2(pts[l_id]);
      Point2 pt_right = cam_right.project2(pts[l_id]);
      StereoPoint2 pt_lr(pt_left.x(), pt_right.x(), pt_left.y());
      EXPECT_DOUBLE_EQ(pt_left.y(), pt_right.y());
      measurement_frame.push_back(std::make_pair(l_id, pt_lr));
    }
    all_measurements.push_back(std::make_shared<StatusStereoMeasurements>(
        std::make_pair(tracker_status_valid, measurement_frame)));
  }

  // create vio
  Pose3 B_pose_camLrect;
  StereoCalibPtr stereo_calibration =
      boost::make_shared<gtsam::Cal3_S2Stereo>(cam_params.fx(),
                                               cam_params.fy(),
                                               cam_params.skew(),
                                               cam_params.px(),
                                               cam_params.py(),
                                               baseline);
  std::shared_ptr<InitializationBackEnd> vio =
      std::make_shared<InitializationBackEnd>(
          B_pose_camLrect,
          stereo_calibration,
          vioParams,
          imu_params,
          BackendOutputParams(false, 0, false));
  ImuFrontEnd imu_frontend(imu_params, imu_bias);

  // Create vector of input payloads
  std::vector<BackendInput::UniquePtr> input_vector;
  input_vector.clear();

  // For each frame, add landmarks.
  for (int64_t k = 1; k < num_key_frames; k++) {
    // Time stamp for the current keyframe and the next frame.
    Timestamp timestamp_lkf = (k - 1) * time_step + t_start;
    Timestamp timestamp_k = k * time_step + t_start;

    // Get the IMU data
    ImuStampS imu_stamps;
    ImuAccGyrS imu_accgyr;
    CHECK(imu_buf.getImuDataInterpolatedUpperBorder(
              timestamp_lkf, timestamp_k, &imu_stamps, &imu_accgyr) ==
          VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);

    const auto& pim =
        imu_frontend.preintegrateImuMeasurements(imu_stamps, imu_accgyr);

    // Push input payload into queue

    BackendInput::UniquePtr input = VIO::make_unique<BackendInput>(
        timestamp_k,
        all_measurements[k],
        tracker_status_valid.kfTrackingStatus_stereo_,
        pim);

    // Create artificially noisy "RANSAC" pose measurements
    gtsam::Pose3 random_pose = (poses[k - 1].first).between(poses[k].first) *
                               UtilsOpenCV::RandomPose3(rad_sigma, pos_sigma);
    input->stereo_ransac_body_pose_ = random_pose;

    // Create input vector for backend
    input_vector.push_back(std::move(input));
  }

  // Perform Bundle Adjustment
  std::vector<gtsam::Pose3> results =
      vio->addInitialVisualStatesAndOptimize(input_vector);

  CHECK_EQ(results.size(), num_key_frames - 1);

  // Check error (BA results start at origin, as convention)
  // The tolerance is on compounded error!! Not relative.
  for (int f_id = 0; f_id < (num_key_frames - 1); f_id++) {
    Pose3 W_Pose_Blkf = poses[1].first.compose(results.at(f_id));
    EXPECT_TRUE(assert_equal(
        poses[f_id + 1].first, W_Pose_Blkf, vioParams.smartNoiseSigma_));
  }
}

}  // namespace VIO
