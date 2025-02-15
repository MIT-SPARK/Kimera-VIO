/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testRegularVioBackend.h
 * @brief  test RegularVioBackend
 * @author Luca Carlone, Antoni Rosinol
 */

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>

#include "RegularVioBackend.h"

DECLARE_string(test_data_path);

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

using namespace gtsam;
using namespace std;
using namespace VIO;
using namespace cv;

static constexpr double tol = 1e-7;

/* ************************************************************************* */
// Parameters
static constexpr int num_key_frames =
    20;                             // number of frames of the synthetic scene
static const Vector3 p0(0, 0, 0);   // initial pose of the robot camera
static const Vector3 v(1.0, 0, 0);  // velocity of the robot, per time_step
static constexpr int time_step =
    1e9;  // elapsed time between two consecutive frames is 1 second (1e9 nsecs)
static constexpr Timestamp t_start = 1e9;  // ImuBuffer does not allow t = 0;
static constexpr double baseline = 0.5;
static const imuBias::ConstantBias imu_bias(Vector3(0.1, -0.1, 0.3),
                                            Vector3(0.1, 0.3, -0.2));

using StereoPoses = vector<pair<Pose3, Pose3>>;

/* ************************************************************************** */
// Helper functions!
vector<Point3> CreateScene() {
  vector<Point3> points;

  // A scene with 8 points.
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

/* -------------------------------------------------------------------------- */
StereoPoses CreateCameraPoses(const int& num_keyframes,
                              const double& baseline,
                              const Vector3& p0,
                              const Vector3& v) {
  StereoPoses poses;
  poses.reserve(num_keyframes);
  Pose3 L_pose_R(Rot3(), Vector3(baseline, 0, 0));

  // The camera is assumed to face (0, 0, 1):
  // z-forward, y down and x to the right
  for (int f_id = 0; f_id < num_keyframes; f_id++) {
    // Constant velocity model along x:
    // the robot moves to the right, keeping the point in front of the camera
    Vector3 p_offset = v * f_id * (time_step / ((double)1e9));

    Pose3 pose_left(Rot3(), p0 + p_offset);
    Pose3 pose_right = pose_left.compose(L_pose_R);

    poses.push_back(make_pair(pose_left, pose_right));
  }
  return poses;
}

/* -------------------------------------------------------------------------- */
void CreateImuBuffer(ImuFrontend& imu_buf,
                     const int num_frames,
                     const Vector3 v,
                     const ImuBias imu_bias,
                     const Vector3 n_gravity,
                     const Timestamp time_step,
                     const Timestamp t_start) {
  // Synthesize IMU measurements.
  for (int f_id = 0; f_id < num_frames; f_id++) {
    Vector6 acc_gyr;
    // Constant speed, no acceleration.
    acc_gyr.head(3) = -n_gravity + imu_bias.accelerometer();
    // Camera axis aligned with the world axis in this example.
    acc_gyr.tail(3) = imu_bias.gyroscope();
    Timestamp t = ((int64_t)f_id) * time_step + t_start;
    imu_buf.insert(t, acc_gyr);
  }
}

/* ************************************************************************** */
TEST(testRegularVio, robotMovingWithConstantVelocity) {
  // Additional parameters.
  VioBackendParams vioParams;
  vioParams.landmarkDistanceThreshold_ = 30;  // We simulate points 20m away.
  vioParams.imuIntegrationSigma_ = 1e-4;
  vioParams.nr_states_ = 100;
  vioParams.landmarkDistanceThreshold_ = 50;

  // Create 3D points
  vector<Point3> lmk_pts = CreateScene();
  const int num_lmk_pts = lmk_pts.size();

  // Create cameras.
  static const double fov = M_PI / 3 * 2;
  // Create image size to initiate meaningful intrinsic camera matrix.
  static constexpr double img_height = 600;
  static constexpr double img_width = 800;
  static const double fx = img_width / 2 / tan(fov / 2);
  static const double fy = fx;
  static constexpr double s = 0;
  static const double u0 = img_width / 2;
  static const double v0 = img_height / 2;

  Cal3_S2 cam_params(fx, fy, s, u0, v0);

  // Create camera poses and IMU data
  StereoPoses poses;
  ImuFrontend imu_buf;
  poses = CreateCameraPoses(num_key_frames, baseline, p0, v);
  CreateImuBuffer(imu_buf,
                  num_key_frames,
                  v,
                  imu_bias,
                  vioParams.n_gravity_,
                  time_step,
                  t_start);

  // Create measurements
  //    using SmartStereoMeasurement = pair<LandmarkId,StereoPoint2>;
  //    using SmartStereoMeasurements = vector<SmartStereoMeasurement>;
  //    using StatusSmartStereoMeasurements =
  //                        pair<TrackerStatusSummary, SmartStereoMeasurements>;
  TrackerStatusSummary tracker_status_valid;
  tracker_status_valid.kfTrackingStatus_mono_ = Tracker::VALID;
  tracker_status_valid.kfTrackingStatus_stereo_ = Tracker::VALID;

  // TODO add regularities vector.
  // TODO fill mesh_lmk_ids with a triplet of lmk_ids.
  LandmarkIds mesh_lmk_ids_ground_cluster;
  for (int lmk_id = 0; lmk_id < num_lmk_pts; lmk_id++) {
    mesh_lmk_ids_ground_cluster.push_back(lmk_id);
  }

  vector<StatusSmartStereoMeasurements> all_measurements;
  for (int i = 0; i < num_key_frames; i++) {
    PinholeCamera<Cal3_S2> cam_left(poses.at(i).first, cam_params);
    PinholeCamera<Cal3_S2> cam_right(poses.at(i).second, cam_params);
    SmartStereoMeasurements measurement_frame;
    for (int lmk_id = 0; lmk_id < num_lmk_pts; lmk_id++) {
      Point2 px_left = cam_left.project(lmk_pts.at(lmk_id));
      Point2 px_right = cam_right.project(lmk_pts.at(lmk_id));
      StereoPoint2 px_lr(px_left.x(), px_right.x(), px_left.y());
      EXPECT_DOUBLE_EQ(px_left.y(), px_right.y());
      measurement_frame.push_back(make_pair(lmk_id, px_lr));
    }
    all_measurements.push_back(
        make_pair(tracker_status_valid, measurement_frame));
  }

  // Create regular VIO.
  Pose3 B_pose_camLrect(Rot3(), Vector3::Zero());
  auto regular_vio = std::make_shared<RegularVioBackend>(
      B_pose_camLrect, cam_params, baseline, vioParams);

  regular_vio->initializeStateAndSetPriors(
      t_start, poses.at(0).first, v, imu_bias);

  // For each frame, add landmarks and optimize.
  for (int64_t k = 1; k < num_key_frames; k++) {
    // Timestamp for the current keyframe and the next frame.
    Timestamp timestamp_lkf = (k - 1) * time_step + t_start;
    Timestamp timestamp_k = k * time_step + t_start;

    // Get the IMU data.
    ImuStamps imu_stamps;
    ImuAccGyr imu_accgyr;
    tie(imu_stamps, imu_accgyr) =
        imu_buf.getBetweenValuesInterpolated(timestamp_lkf, timestamp_k);

    // Process data with VIO.
    regular_vio->addVisualInertialStateAndOptimize(
        timestamp_k,          // Current time for fixed lag smoother.
        all_measurements[k],  // Vision data.
        imu_stamps,
        imu_accgyr,                    // Inertial data.
        mesh_lmk_ids_ground_cluster);  // Lmk ids with regularities.

    NonlinearFactorGraph nlfg = regular_vio->smoother_->getFactors();
    size_t nrFactorsInSmoother = 0;
    for (const auto& f : nlfg) {  // Count the number of nonempty factors.
      if (f) {
        nrFactorsInSmoother++;
      }
    }
    cout << "at frame " << k << " nr factors: " << nrFactorsInSmoother << endl;

    if (VLOG_IS_ON(10)) {
      nlfg.print("PRINTING Graph at timestamp: ");
    }
#ifdef USE_COMBINED_IMU_FACTOR
    EXPECT(nrFactorsInSmoother ==
           3 + k + 8);  // 3 priors, 1 imu per time stamp, 8 smart factors
#else
    if (k == 1) {
      // 3 priors, 1 imu + 1 between per time stamp:
      // we do not include smart factors of length 1.
      EXPECT_EQ(nrFactorsInSmoother, 3 + 2 * k);
    } else if (k == 2) {
      // 3 priors, 1 imu + 1 between per time stamp, 8 smart factors.
      EXPECT_EQ(nrFactorsInSmoother, 3 + 2 * k + 8);
    } else {
      // 3 priors, 1 imu + 1 between per time stamp, 8 * k projection factors.
      EXPECT_EQ(nrFactorsInSmoother, 3 + 2 * k + 8 * k);
    }
#endif
    // Check the results!
    const Values& results = regular_vio->getState();

    for (size_t frame_id = 0; frame_id <= k; frame_id++) {
      Pose3 W_Pose_Blkf = results.at<Pose3>(Symbol('x', frame_id));
      Vector3 W_Vel_Blkf = results.at<Vector3>(Symbol('v', frame_id));
      ImuBias imu_bias_lkf = results.at<ImuBias>(Symbol('b', frame_id));

      EXPECT_TRUE(assert_equal(poses.at(frame_id).first, W_Pose_Blkf, tol));
      EXPECT_LT((W_Vel_Blkf - v).norm(), tol);
      EXPECT_LT((imu_bias_lkf - imu_bias).vector().norm(), tol);
    }
  }
}

/* ************************************************************************** */
TEST(testRegularVio, robotMovingWithConstantVelocitySmartAndProjFactor) {
  // Additional parameters.
  VioBackendParams vioParams;
  vioParams.landmarkDistanceThreshold_ = 30;  // We simulate points 20m away.
  vioParams.imuIntegrationSigma_ = 1e-4;
  vioParams.nr_states_ = 100;
  vioParams.landmarkDistanceThreshold_ = 50;

  // Create 3D points
  vector<Point3> lmk_pts = CreateScene();
  const int num_lmk_pts = lmk_pts.size();

  // Create cameras.
  static const double fov = M_PI / 3 * 2;
  // Create image size to initiate meaningful intrinsic camera matrix.
  static constexpr double img_height = 600;
  static constexpr double img_width = 800;
  static const double fx = img_width / 2 / tan(fov / 2);
  static const double fy = fx;
  static constexpr double s = 0;
  static const double u0 = img_width / 2;
  static const double v0 = img_height / 2;

  Cal3_S2 cam_params(fx, fy, s, u0, v0);

  // Create camera poses and IMU data.
  StereoPoses poses;
  ImuFrontend imu_buf;
  poses = CreateCameraPoses(num_key_frames, baseline, p0, v);
  CreateImuBuffer(imu_buf,
                  num_key_frames,
                  v,
                  imu_bias,
                  vioParams.n_gravity_,
                  time_step,
                  t_start);

  // Create measurements.
  //    using SmartStereoMeasurement = pair<LandmarkId,StereoPoint2>;
  //    using SmartStereoMeasurements = vector<SmartStereoMeasurement>;
  //    using StatusSmartStereoMeasurements =
  //                        pair<TrackerStatusSummary, SmartStereoMeasurements>;
  TrackerStatusSummary tracker_status_valid;
  tracker_status_valid.kfTrackingStatus_mono_ = Tracker::VALID;
  tracker_status_valid.kfTrackingStatus_stereo_ = Tracker::VALID;

  vector<StatusSmartStereoMeasurements> all_measurements;
  for (size_t i = 0; i < num_key_frames; i++) {
    PinholeCamera<Cal3_S2> cam_left(poses.at(i).first, cam_params);
    PinholeCamera<Cal3_S2> cam_right(poses.at(i).second, cam_params);
    SmartStereoMeasurements measurement_frame;
    for (int lmk_id = 0; lmk_id < num_lmk_pts; lmk_id++) {
      Point2 px_left = cam_left.project(lmk_pts.at(lmk_id));
      Point2 px_right = cam_right.project(lmk_pts.at(lmk_id));
      StereoPoint2 px_lr(px_left.x(), px_right.x(), px_left.y());
      EXPECT_DOUBLE_EQ(px_left.y(), px_right.y());
      measurement_frame.push_back(make_pair(lmk_id, px_lr));
    }
    all_measurements.push_back(
        make_pair(tracker_status_valid, measurement_frame));
  }

  // Create regular VIO.
  Pose3 B_pose_camLrect(Rot3(), Vector3::Zero());
  auto regular_vio = std::make_shared<RegularVioBackend>(
      B_pose_camLrect, cam_params, baseline, vioParams);

  regular_vio->initializeStateAndSetPriors(
      t_start, poses.at(0).first, v, imu_bias);

  // For each frame, add landmarks and optimize.
  for (int64_t k = 1; k < num_key_frames; k++) {
    // Timestamp for the current keyframe and the next frame.
    Timestamp timestamp_lkf = (k - 1) * time_step + t_start;
    Timestamp timestamp_k = k * time_step + t_start;

    // Get the IMU data.
    ImuStamps imu_stamps;
    ImuAccGyr imu_accgyr;
    tie(imu_stamps, imu_accgyr) =
        imu_buf.getBetweenValuesInterpolated(timestamp_lkf, timestamp_k);

    // TODO add regularities vector.
    // TODO fill mesh_lmk_ids with a triplet of lmk_ids.
    LandmarkIds mesh_lmk_ids_ground_cluster;
    if ((k - 1) < num_lmk_pts) {
      mesh_lmk_ids_ground_cluster.push_back(k - 1);
    }

    // Process data with VIO.
    regular_vio->addVisualInertialStateAndOptimize(
        timestamp_k,          // Current time for fixed lag smoother.
        all_measurements[k],  // Vision data.
        imu_stamps,
        imu_accgyr,                    // Inertial data.
        mesh_lmk_ids_ground_cluster);  // Lmk ids with regularities.

    NonlinearFactorGraph nlfg = regular_vio->smoother_->getFactors();
    size_t nrFactorsInSmoother = 0;
    for (auto f : nlfg) {  // Count the number of nonempty factors.
      if (f) nrFactorsInSmoother++;
    }
    cout << "at frame " << k << " nr factors: " << nrFactorsInSmoother << endl;

    if (VLOG_IS_ON(10)) {
      nlfg.print("PRINTING Graph at timestamp: ");
    }
#ifdef USE_COMBINED_IMU_FACTOR
    EXPECT(nrFactorsInSmoother ==
           3 + k + 8);  // 3 priors, 1 imu per time stamp, 8 smart factors
#else
    if (k == 1) {
      // 3 priors, 1 imu + 1 between per time stamp:
      // we do not include smart factors of length 1.
      EXPECT_EQ(nrFactorsInSmoother, 3 + 2 * k);
    } else if (k == 2) {
      // 3 priors, 1 imu + 1 between per time stamp,  smart factors.
      EXPECT_EQ(nrFactorsInSmoother, 3 + 2 * k + 8);
    } else {
      // 3 priors, 1 imu + 1 between per time stamp,
      // Smart Factors: max (0, 8 - k). such that when k > 8 smart factors = 0.
      // Projection Factors: min ( , ). such that when k > 8 proj factors = 8*k.
      EXPECT_EQ(nrFactorsInSmoother,
                3 + 2 * k +
                    std::max(static_cast<int64_t>(0),
                             (8 - k)) +  // TODO get it right!
                    std::min((k * k), (8 * k)));
    }
#endif
    // Check the results!
    Values results = regular_vio->getState();

    for (int frame_id = 0; frame_id <= k; frame_id++) {
      Pose3 W_Pose_Blkf = results.at<Pose3>(Symbol('x', frame_id));
      Vector3 W_Vel_Blkf = results.at<Vector3>(Symbol('v', frame_id));
      ImuBias imu_bias_lkf = results.at<ImuBias>(Symbol('b', frame_id));

      ASSERT_TRUE(
          gtsam::assert_equal(poses.at(frame_id).first, W_Pose_Blkf, tol));
      ASSERT_LT((W_Vel_Blkf - v).norm(), tol);
      ASSERT_LT((imu_bias_lkf - imu_bias).vector().norm(), tol);
    }
  }
}
