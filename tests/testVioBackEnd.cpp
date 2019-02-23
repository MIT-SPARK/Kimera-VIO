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
 * @author Luca Carlone
 */

#include <cstdlib>
#include <iostream>
#include <random>
#include <algorithm>
#include "VioBackEnd.h"
#include "ETH_parser.h" // only for gtNavState...
#include "test_config.h"
#include "utils/ThreadsafeImuBuffer.h"

// Add last, since it redefines CHECK, which is first defined by glog.
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;
using namespace VIO;
using namespace cv;

static const double tol = 1e-7;

/* ************************************************************************* */
// Parameters
static const int num_key_frames = 10; // number of frames of the synthetic scene
static const Vector3 p0(0, 0, 0); // initial pose of the robot camera
static const Vector3 v(1.0, 0, 0); // velocity of the robot, per time_step
static const int time_step = 1e9; // elapsed time between two consecutive frames is 1 second (1e9 nsecs)
static const Timestamp t_start = 1e9; // ImuBuffer does not allow t = 0;
static const double baseline = 0.5;
static const imuBias::ConstantBias imu_bias(Vector3(0.1, -0.1, 0.3),
    Vector3(0.1, 0.3, -0.2));

using StereoPoses = vector<pair<Pose3, Pose3>>;

/* ************************************************************************* */
// Helper functions!
vector<Point3> CreateScene() {
  vector<Point3> points;
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
    const double baseline, const Vector3 p0, const Vector3 v) {

  StereoPoses poses;
  poses.reserve(num_keyframes);
  Pose3 L_pose_R(Rot3::identity(),
      Vector3(baseline, 0, 0));

  // The camera is assumed to face (0, 0, 1): z-forward, y down and x to the right
  for (int f_id = 0; f_id < num_keyframes; f_id++) {
    // constant velocity model along x: the robot moves to the right, keeping the point in front of the camera
    Vector3 p_offset = v * f_id * (time_step / ((double) 1e9));

    Pose3 pose_left(Rot3::identity(), p0 + p_offset);
    Pose3 pose_right = pose_left.compose(L_pose_R);

    poses.push_back(make_pair(pose_left, pose_right));
  }
  return poses;
}

/* ------------------------------------------------------------------------- */
void CreateImuBuffer(VIO::utils::ThreadsafeImuBuffer& imu_buf, const int num_frames, const Vector3 v,
    const ImuBias imu_bias, const Vector3 n_gravity,
    const Timestamp time_step, const Timestamp t_start) {
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
TEST(testVio, GuessPoseFromIMUmeasurements) {

  for(size_t test=0; test<5; test++){
    Vector3 n_gravity;
    Vector3 a;
    switch(test)
    {
    case 0: // generic vectors
      a = Vector3(9.8, 1, 0);
      n_gravity = Vector3(0,0,-9.8);
      break;
    case 1: // already aligned vectors
      a = Vector3(0, -9.8, 0);
      n_gravity = Vector3(0, -9.8, 0);
      break;
    case 2: // opposite vectors
      a = Vector3(0,0,-9.8);
      n_gravity = Vector3(0,0,+9.8);
      break;
    case 3:
      a = Vector3(9.8, 0, 0);
      n_gravity = Vector3(0,-9.8,0);
      break;
    case 4:
      a = Vector3(9.8, -1, 0);
      n_gravity = Rot3::Expmap(Vector3(0.1, 1, 0.5)).matrix() * a;
      break;
    }

    size_t n = 10;
    ImuAccGyrS accGyroRaw;
    accGyroRaw.resize(6, n); // n identical measurements
    for(size_t i=0; i<n; i++)
      accGyroRaw.col(i) << -a , Vector3::Zero(); // we measure the opposite of gravity

    bool round = false;
    Pose3 poseActual = VioBackEnd::guessPoseFromIMUmeasurements(accGyroRaw,n_gravity,round);
    Vector3 tExpected = Vector3::Zero();
    Vector3 tActual = poseActual.translation();
    EXPECT(assert_equal(tExpected, tActual, tol));

    Unit3 n_gravityDir_actual = poseActual.rotation().rotate(Unit3(a));
    Unit3 n_gravityDir_expected = Unit3(n_gravity);
    EXPECT(assert_equal(n_gravityDir_expected, n_gravityDir_actual, tol));

    if(test>0 && test<4){ // case in which true gravity is along a single axis
      round = true;
      // check that rounding does not mess up with the previous cases
      Pose3 poseActual2 = VioBackEnd::guessPoseFromIMUmeasurements(accGyroRaw,n_gravity,round); // by rounding we should filter out perturbation
      EXPECT(assert_equal(poseActual, poseActual2, tol));

      // check that rounding filter out perturbation
      Vector3 n_gravity_perturbed = n_gravity + Vector3(-0.1,0.1,0.3);
      Pose3 poseActualRound = VioBackEnd::guessPoseFromIMUmeasurements(accGyroRaw,n_gravity_perturbed,round);
      EXPECT(assert_equal(poseActual, poseActualRound, tol));
    }
  }
}

/* ************************************************************************* */
TEST(testVio, InitializeImuBias) {
  // Synthesize the data
  const int num_measurements = 100;
  ImuAccGyrS imu_accgyr;
  imu_accgyr.resize(6, num_measurements);
  Vector3 n_gravity(1.1, 2.2, 3.3); // random numbers, just for the test
  srand(0);
  for (int i = 0; i < num_measurements; i++) {
    Vector6 rand_nums;
    for(int idx = 0; idx < 6; idx++) {
      rand_nums[idx] = (rand() / ((double) RAND_MAX)) * 3; // random number between 0 and 3
    }
    imu_accgyr.col(i) = rand_nums;
  }

  // Compute the actual bias!
  ImuBias imu_bias_actual = VioBackEnd::initImuBias(imu_accgyr, n_gravity);

  // Compute the expected value!
  Vector6 imu_mean = Vector6::Zero();
  for (int i = 0; i < num_measurements; i++) {
    imu_mean += imu_accgyr.col(i);
  }
  imu_mean = imu_mean / ((double) num_measurements);
  imu_mean.head(3) = imu_mean.head(3) + n_gravity;
  ImuBias imu_bias_expected (imu_mean.head(3), imu_mean.tail(3));

  // Compare the results!
  EXPECT(assert_equal(imu_bias_expected, imu_bias_actual, tol));
}

/* ************************************************************************* */
TEST(testVio, robotMovingWithConstantVelocity) {
  // Additional parameters
  VioBackEndParams vioParams;
  vioParams.landmarkDistanceThreshold_ = 30; // we simulate points 20m away
  vioParams.imuIntegrationSigma_ = 1e-4;
  vioParams.horizon_ = 100;

  // Create 3D points
  vector<Point3> pts = CreateScene();
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
  VIO::utils::ThreadsafeImuBuffer imu_buf (-1);
  poses = CreateCameraPoses(num_key_frames, baseline, p0, v);
  CreateImuBuffer(imu_buf, num_key_frames, v, imu_bias,
      vioParams.n_gravity_, time_step, t_start);

  // Create measurements
  //    using SmartStereoMeasurement = pair<LandmarkId,StereoPoint2>;
  //    using SmartStereoMeasurements = vector<SmartStereoMeasurement>;
  //    using StatusSmartStereoMeasurements = pair<TrackerStatusSummary,SmartStereoMeasurements>;
  TrackerStatusSummary tracker_status_valid;
  tracker_status_valid.kfTrackingStatus_mono_ = Tracker::TrackingStatus::VALID;
  tracker_status_valid.kfTrackingStatus_stereo_ = Tracker::TrackingStatus::VALID;

  vector<StatusSmartStereoMeasurements> all_measurements;
  for (int i = 0; i < num_key_frames; i++) {
    PinholeCamera<Cal3_S2> cam_left(poses[i].first, cam_params);
    PinholeCamera<Cal3_S2> cam_right(poses[i].second, cam_params);
    SmartStereoMeasurements measurement_frame;
    for (int l_id = 0; l_id < num_pts; l_id++) {
      Point2 pt_left = cam_left.project(pts[l_id]);
      Point2 pt_right = cam_right.project(pts[l_id]);
      StereoPoint2 pt_lr(pt_left.x(), pt_right.x(), pt_left.y());
      EXPECT_DOUBLES_EQUAL(pt_left.y(), pt_right.y(), 1e-7);
      measurement_frame.push_back(make_pair(l_id, pt_lr));
    }
    all_measurements.push_back(make_pair(tracker_status_valid,measurement_frame));
  }

  // create vio
  Pose3 B_pose_camLrect(Rot3::identity(), Vector3::Zero());
  std::shared_ptr<gtNavState> initial_state = std::make_shared<gtNavState>(
        poses[0].first, v, imu_bias);
  boost::shared_ptr<VioBackEnd> vio = boost::make_shared<VioBackEnd>(
        B_pose_camLrect, cam_params,
        baseline, &initial_state,
        t_start, ImuAccGyrS(), vioParams);

  ImuParams imu_params;
  imu_params.n_gravity_ = vioParams.n_gravity_;
  imu_params.imu_integration_sigma_ = vioParams.imuIntegrationSigma_;
  imu_params.acc_walk_ = vioParams.accBiasSigma_;
  imu_params.acc_noise_ = vioParams.accNoiseDensity_;
  imu_params.gyro_walk_ = vioParams.gyroBiasSigma_;
  imu_params.gyro_noise_ = vioParams.gyroNoiseDensity_;
  ImuFrontEnd imu_frontend(imu_params);

  // For each frame, add landmarks and optimize.
  for(int64_t k = 1; k < num_key_frames; k++) {
    // Time stamp for the current keyframe and the next frame.
    Timestamp timestamp_lkf = (k - 1) * time_step + t_start;
    Timestamp timestamp_k = k * time_step + t_start;

    // Get the IMU data
    ImuStampS imu_stamps;
    ImuAccGyrS imu_accgyr;
    CHECK(imu_buf.getImuDataInterpolatedUpperBorder(timestamp_lkf,
                                                    timestamp_k,
                                                    &imu_stamps,
                                                    &imu_accgyr) ==
          VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);

    const auto& pim = imu_frontend.preintegrateImuMeasurements(imu_stamps,
                                                               imu_accgyr);

    const VioBackEndInputPayload input (
          timestamp_k,
          all_measurements[k],
          tracker_status_valid.kfTrackingStatus_stereo_,
          pim);

    // process data with VIO
    vio->spinOnce(std::make_shared<VioBackEndInputPayload>(input));
    imu_frontend.updateBias(vio->getLatestImuBias());
    imu_frontend.resetIntegration();

    const NonlinearFactorGraph& nlfg = vio->getFactorsUnsafe();
    size_t nrFactorsInSmoother = 0;
    for(const auto& f : nlfg) { // count the number of nonempty factors
      if (f) nrFactorsInSmoother++;
    }
    cout << "at frame " << k << " nr factors: " << nrFactorsInSmoother << endl;

#ifdef USE_COMBINED_IMU_FACTOR
    EXPECT(nrFactorsInSmoother == 3 + k + 8); // 3 priors, 1 imu per time stamp, 8 smart factors
#else
    if(k==1){
      EXPECT(nrFactorsInSmoother == 3 + 2*k); // 3 priors, 1 imu + 1 between per time stamp: we do not include smart factors of length 1
    }else{
      EXPECT(nrFactorsInSmoother == 3 + 2*k + 8); // 3 priors, 1 imu + 1 between per time stamp, 8 smart factors
    }
#endif
    // Check the results!
    const Values& results = vio->getState();

    for (int f_id = 0; f_id <= k; f_id++) {
      Pose3 W_Pose_Blkf = results.at<Pose3>(Symbol('x', f_id));
      Vector3 W_Vel_Blkf = results.at<Vector3>(Symbol('v', f_id));
      ImuBias imu_bias_lkf = results.at<ImuBias>(Symbol('b', f_id));

      EXPECT(assert_equal(poses[f_id].first, W_Pose_Blkf, tol));
      EXPECT((W_Vel_Blkf - v).norm() < tol);
      EXPECT((imu_bias_lkf - imu_bias).vector().norm() < tol);
    }
  }
}

/* ************************************************************************* */
int main(int argc, char *argv[]) {
  // Initialize Google's flags library.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // google::SetStderrLogging(google::INFO); // Used to debug.

  TestResult tr; return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
