/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testVioBackend.h
 * @brief  test VioBackend
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>

#include "kimera-vio/backend/VioBackend.h"
#include "kimera-vio/backend/VioBackendFactory.h"
#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontendParams.h"
#include "kimera-vio/initial/InitializationBackend.h"
#include "kimera-vio/utils/ThreadsafeImuBuffer.h"

DECLARE_string(test_data_path);

namespace VIO {

using StereoPoses = std::vector<std::pair<gtsam::Pose3, gtsam::Pose3>>;

class BackendFixture : public ::testing::Test {
 public:
  BackendFixture() : backend_params_(), imu_params_() {
    // Update vio params
    // we simulate points 20m away
    backend_params_.landmarkDistanceThreshold_ = 30;
    backend_params_.nr_states_ = 100;

    // Update IMU params
    imu_params_.gyro_noise_density_ = 0.00016968;
    imu_params_.acc_noise_density_ = 0.002;
    imu_params_.gyro_random_walk_ = 1.9393e-05;
    imu_params_.acc_random_walk_ = 0.003;
    imu_params_.n_gravity_ = gtsam::Vector3(0.0, 0.0, -9.81);
    imu_params_.imu_integration_sigma_ = 1.0;
    imu_params_.nominal_sampling_time_s_ = 200.0;
    // TODO(Toni): test with Combined, I think it actually fails now...
    imu_params_.imu_preintegration_type_ =
        ImuPreintegrationType::kPreintegratedImuMeasurements;
  }

  void map_update_cb(const LandmarksMap& map) {
    // Do nothing
  }

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

 protected:
  std::vector<Point3> createScene() {
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

  void createCameraPoses(StereoPoses* stereo_poses) {
    CHECK_NOTNULL(stereo_poses);
    stereo_poses->reserve(num_keyframes_);
    Pose3 L_pose_R(Rot3(), gtsam::Point3(baseline, 0, 0));

    // The camera is assumed to face (0, 0, 1): z-forward, y down and x to the
    // right
    for (int f_id = 0; f_id < num_keyframes_; f_id++) {
      // constant velocity model along x: the robot moves to the right, keeping
      // the point in front of the camera
      gtsam::Vector3 p_offset =
          velocity_x_ * f_id * (keyframe_time_step_ / ((double)1e9));

      Pose3 pose_left(Rot3(), p0 + p_offset);
      Pose3 pose_right = pose_left.compose(L_pose_R);

      stereo_poses->push_back(std::make_pair(pose_left, pose_right));
    }
  }

  void createImuBuffer(VIO::utils::ThreadsafeImuBuffer* imu_buf) {
    CHECK_NOTNULL(imu_buf);
    // Synthesize IMU measurements
    CHECK_LT(btw_keyframe_imu_msgs_ * imu_time_step_, keyframe_time_step_);

    // Constant measurements
    Vector6 acc_gyr;
    // constant speed, no acceleration
    acc_gyr.head(3) = -imu_params_.n_gravity_ + imu_bias_.accelerometer();
    // Camera axis aligned with the world axis in this example
    acc_gyr.tail(3) = imu_bias_.gyroscope();

    // Add a very old imu measurement to avoid the imu buffer complaining that
    // there was not a message before, when querying it i.e. kDataNeverAvailable
    Timestamp t = 0u;
    imu_buf->addMeasurement(t, acc_gyr);

    // Pad some measurements before start:
    CHECK_LT(before_start_imu_msgs_ * imu_time_step_, t_start_)
        << "Negative timestamps aren't allowed";
    for (size_t k = before_start_imu_msgs_; k > 0; k--) {
      Timestamp t = t_start_ - k * imu_time_step_;
      imu_buf->addMeasurement(t, acc_gyr);
      VLOG(5) << "Timestamp: " << t;
      VLOG(5) << "Accgyr: " << acc_gyr;
    }

    VLOG(5) << "Num frames: " << num_keyframes_;
    VLOG(5) << "Num IMU msgs btw frames: " << btw_keyframe_imu_msgs_;
    for (FrameId f_id = 0u; f_id < num_keyframes_; f_id++) {
      for (size_t k = 0u; k < btw_keyframe_imu_msgs_; k++) {
        Timestamp t =
            f_id * keyframe_time_step_ + k * imu_time_step_ + t_start_;
        imu_buf->addMeasurement(t, acc_gyr);
        VLOG(5) << "Timestamp: " << t;
        VLOG(5) << "Accgyr: " << acc_gyr;
      }
    }
  }

 public:
  const double tol = 1e-7;
  //! Number of frames of the synthetic scene
  const int num_keyframes_ = 10;
  //! Initial pose of the robot camera
  const gtsam::Vector3 p0 = gtsam::Vector3(0.0, 0.0, 0.0);
  //! Velocity of the robot, per time_step
  const gtsam::Vector3 velocity_x_ = gtsam::Vector3(1.0, 0.0, 0.0);
  //! Elapsed time between two consecutive frames is 1 second (1e9 nsecs)
  const Timestamp imu_time_step_ = 1;
  const Timestamp keyframe_time_step_ = 10;
  const Timestamp t_start_ = 100;
  const double baseline = 0.5;
  const gtsam::imuBias::ConstantBias imu_bias_ =
      gtsam::imuBias::ConstantBias(gtsam::Vector3(0.1, -0.1, 0.3),
                                   gtsam::Vector3(0.1, 0.3, -0.2));

  const size_t btw_keyframe_imu_msgs_ = 3u;
  const size_t before_start_imu_msgs_ = 10u;

 public:
  BackendParams backend_params_;
  ImuParams imu_params_;
};

TEST_F(BackendFixture, initializationFromGt) {
  // Expect 3 priors on init

  // Expect imu bias set
}

TEST_F(BackendFixture, robotMovingWithConstantVelocity) {
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

  // Create 3D points
  std::vector<Point3> pts = createScene();
  const size_t num_pts = pts.size();
  // Create camera poses and IMU data
  StereoPoses poses;
  VIO::utils::ThreadsafeImuBuffer imu_buf(-1);
  createCameraPoses(&poses);
  createImuBuffer(&imu_buf);
  ASSERT_GT(imu_buf.size(), 0u);

  // Create measurements
  TrackerStatusSummary tracker_status_valid;
  tracker_status_valid.kfTrackingStatus_mono_ = TrackingStatus::VALID;
  tracker_status_valid.kfTrackingStatus_stereo_ = TrackingStatus::VALID;

  std::vector<StatusStereoMeasurementsPtr> all_measurements;
  for (const auto& pose : poses) {
    gtsam::PinholeCamera<Cal3_S2> cam_left(pose.first, cam_params);
    gtsam::PinholeCamera<Cal3_S2> cam_right(pose.second, cam_params);
    StereoMeasurements measurement_frame;
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

  StereoCalibPtr stereo_calibration(new gtsam::Cal3_S2Stereo(cam_params.fx(),
                                                             cam_params.fy(),
                                                             cam_params.skew(),
                                                             cam_params.px(),
                                                             cam_params.py(),
                                                             baseline));
  ImuFrontend imu_frontend(imu_params_, imu_bias_);

  // Create backend.
  backend_params_.initial_ground_truth_state_ =
      VioNavState(poses[0].first, velocity_x_, imu_bias_);
  gtsam::Pose3 B_pose_camLrect;
  std::shared_ptr<VioBackend> vio_backend =
      std::make_shared<VioBackend>(B_pose_camLrect,
                                   stereo_calibration,
                                   backend_params_,
                                   imu_params_,
                                   BackendOutputParams(false, 0, false),
                                   false);
  vio_backend->registerImuBiasUpdateCallback(std::bind(
      &ImuFrontend::updateBias, std::ref(imu_frontend), std::placeholders::_1));
  vio_backend->registerMapUpdateCallback(
      std::bind(&BackendFixture::map_update_cb, this, std::placeholders::_1));

  // For each frame, add landmarks and optimize.
  Timestamp timestamp_km1 = t_start_ - before_start_imu_msgs_ * imu_time_step_;
  for (FrameId k = 0u; k < num_keyframes_; k++) {
    // Time stamp for the current keyframe and the next frame.
    Timestamp timestamp_k = k * keyframe_time_step_ + t_start_;
    VLOG(5) << "Timestamp_km1: " << timestamp_km1 << '\n'
            << "Timestamp k: " << timestamp_k << '\n'
            << "Imu buf size: " << imu_buf.size();

    // Get the IMU data
    ImuStampS imu_stamps;
    ImuAccGyrS imu_accgyr;
    const auto& imu_status = imu_buf.getImuDataInterpolatedUpperBorder(
        timestamp_km1, timestamp_k, &imu_stamps, &imu_accgyr);
    EXPECT_EQ(imu_stamps.cols(), imu_accgyr.cols());
    EXPECT_TRUE(imu_status ==
                utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
    switch (imu_status) {
      case utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable: {
        LOG(WARNING) << "Waiting for IMU data...";
        break;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::kQueueShutdown: {
        LOG(WARNING)
            << "IMU buffer was shutdown. Shutting down DataProviderModule.";
        break;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable: {
        LOG(WARNING)
            << "Asking for data before start of IMU stream, from timestamp: ";
        break;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::
          kTooFewMeasurementsAvailable: {
        LOG(WARNING) << "No IMU measurements here, and IMU data stream already "
                        "passed this time region";
        break;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable: {
        VLOG(5) << "All ok!";
        break;
      }
    }
    timestamp_km1 = timestamp_k;

    const auto& pim =
        imu_frontend.preintegrateImuMeasurements(imu_stamps, imu_accgyr);

    // process data with VIO
    BackendOutput::Ptr backend_output = vio_backend->spinOnce(
        BackendInput(timestamp_k, all_measurements[k], pim, imu_accgyr));
    CHECK(backend_output);

    // At this point the update imu bias callback should be triggered which
    // will update the imu_frontend imu bias.
    imu_frontend.resetIntegrationWithCachedBias();

    // Check the number of factors
    const gtsam::NonlinearFactorGraph& nlfg = backend_output->factor_graph_;
    size_t nr_factors_in_smoother = 0u;
    for (const auto& f : nlfg) {  // count the number of nonempty factors
      if (f) {
        nr_factors_in_smoother++;
        const auto gsf = dynamic_cast<const SmartStereoFactor*>(f.get());
        if (gsf) {
          EXPECT_EQ(gsf->keys().size(),
                    k);  // each landmark is seen in every frame
          for (size_t j = 1; j <= gsf->keys().size(); j++) {
            EXPECT_EQ(gtsam::Symbol(gsf->keys().at(j - 1)),
                      gtsam::Symbol('x', j));
          }
        }
      }
    }

    VLOG(1) << "Frame: " << k << ", # of factors: " << nr_factors_in_smoother;
    if (imu_params_.imu_preintegration_type_ ==
        ImuPreintegrationType::kPreintegratedCombinedMeasurements) {
      if (k == 0) {
        EXPECT_EQ(nr_factors_in_smoother, 3);  // 3 priors
      } else {
        EXPECT_EQ(
            nr_factors_in_smoother,
            3 + k + 8);  // 3 priors, 1 imu per time stamp, 8 smart factors
      }
    } else {
      if (k == 0) {
        EXPECT_EQ(nr_factors_in_smoother, 3);  // 3 priors
      } else if (k == 1) {
        EXPECT_EQ(
            nr_factors_in_smoother,
            3 + 3 * k);  // 3 priors, 1 imu + 1 btw imu biases + 1 btw stereo
                         // poses. We do not include smart factors of length 1
      } else {
        EXPECT_EQ(nr_factors_in_smoother,
                  3 + 3 * k + 8);  // 3 priors, 1 imu + 1 btw imu biases + 1 btw
                                   // stereo poses, 8 smart factors
      }
    }

    // Check the results!
    const gtsam::Values& results = vio_backend->getState();
    for (FrameId f_id = 0u; f_id <= k; f_id++) {
      gtsam::Pose3 W_Pose_Blkf =
          results.at<gtsam::Pose3>(gtsam::Symbol('x', f_id));
      gtsam::Vector3 W_Vel_Blkf =
          results.at<gtsam::Vector3>(gtsam::Symbol('v', f_id));
      ImuBias imu_bias_lkf = results.at<ImuBias>(gtsam::Symbol('b', f_id));

      EXPECT_TRUE(assert_equal(poses[f_id].first, W_Pose_Blkf, tol));
      EXPECT_LT((W_Vel_Blkf - velocity_x_).norm(), tol);
      EXPECT_LT((imu_bias_lkf - imu_bias_).vector().norm(), tol);
    }
  }
}

// make sure you have 2x factors with odom
// make sure that these factors are between factors and match your input
TEST_F(BackendFixture, robotMovingWithConstantVelocityWithExternalOdometry) {
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

  // Create 3D points
  std::vector<Point3> pts = createScene();
  const size_t num_pts = pts.size();
  // Create camera poses and IMU data
  StereoPoses poses;
  VIO::utils::ThreadsafeImuBuffer imu_buf(-1);
  createCameraPoses(&poses);
  createImuBuffer(&imu_buf);
  ASSERT_GT(imu_buf.size(), 0u);

  // Create measurements
  TrackerStatusSummary tracker_status_valid;
  tracker_status_valid.kfTrackingStatus_mono_ = TrackingStatus::VALID;
  tracker_status_valid.kfTrackingStatus_stereo_ = TrackingStatus::VALID;

  std::vector<StatusStereoMeasurementsPtr> all_measurements;
  for (const auto& pose : poses) {
    gtsam::PinholeCamera<Cal3_S2> cam_left(pose.first, cam_params);
    gtsam::PinholeCamera<Cal3_S2> cam_right(pose.second, cam_params);
    StereoMeasurements measurement_frame;
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

  StereoCalibPtr stereo_calibration(new gtsam::Cal3_S2Stereo(cam_params.fx(),
                                                             cam_params.fy(),
                                                             cam_params.skew(),
                                                             cam_params.px(),
                                                             cam_params.py(),
                                                             baseline));
  ImuFrontend imu_frontend(imu_params_, imu_bias_);

  // Create backend.
  backend_params_.initial_ground_truth_state_ =
      VioNavState(poses[0].first, velocity_x_, imu_bias_);
  gtsam::Pose3 B_pose_camLrect;
  std::shared_ptr<VioBackend> vio_backend =
      std::make_shared<VioBackend>(B_pose_camLrect,
                                   stereo_calibration,
                                   backend_params_,
                                   imu_params_,
                                   BackendOutputParams(false, 0, false),
                                   false,
                                   VIO::OdometryParams());
  vio_backend->registerImuBiasUpdateCallback(std::bind(
      &ImuFrontend::updateBias, std::ref(imu_frontend), std::placeholders::_1));
  vio_backend->registerMapUpdateCallback(
      std::bind(&BackendFixture::map_update_cb, this, std::placeholders::_1));

  // For each frame, add landmarks and optimize.
  Timestamp timestamp_km1 = t_start_ - before_start_imu_msgs_ * imu_time_step_;
  for (FrameId k = 0u; k < num_keyframes_; k++) {
    // Time stamp for the current keyframe and the next frame.
    Timestamp timestamp_k = k * keyframe_time_step_ + t_start_;
    VLOG(5) << "Timestamp_km1: " << timestamp_km1 << '\n'
            << "Timestamp k: " << timestamp_k << '\n'
            << "Imu buf size: " << imu_buf.size();

    // Get the IMU data
    ImuStampS imu_stamps;
    ImuAccGyrS imu_accgyr;
    const auto& imu_status = imu_buf.getImuDataInterpolatedUpperBorder(
        timestamp_km1, timestamp_k, &imu_stamps, &imu_accgyr);
    EXPECT_EQ(imu_stamps.cols(), imu_accgyr.cols());
    EXPECT_TRUE(imu_status ==
                utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
    switch (imu_status) {
      case utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable: {
        LOG(WARNING) << "Waiting for IMU data...";
        break;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::kQueueShutdown: {
        LOG(WARNING)
            << "IMU buffer was shutdown. Shutting down DataProviderModule.";
        break;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable: {
        LOG(WARNING)
            << "Asking for data before start of IMU stream, from timestamp: ";
        break;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::
          kTooFewMeasurementsAvailable: {
        LOG(WARNING) << "No IMU measurements here, and IMU data stream already "
                        "passed this time region";
        break;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable: {
        VLOG(5) << "All ok!";
        break;
      }
    }
    timestamp_km1 = timestamp_k;

    const auto& pim =
        imu_frontend.preintegrateImuMeasurements(imu_stamps, imu_accgyr);

    gtsam::Pose3 external_odometry_pose(gtsam::Rot3(),
                                        gtsam::Point3(0, 0, k / 1000.0));

    // process data with VIO
    BackendOutput::Ptr backend_output = vio_backend->spinOnce(BackendInput(
        timestamp_k,
        all_measurements[k],
        pim,
        imu_accgyr,
        // Only push odometry after first one
        k > 0u ? std::optional<gtsam::Pose3>(external_odometry_pose)
               : std::nullopt,
        std::nullopt));
    CHECK(backend_output);

    // At this point the update imu bias callback should be triggered which
    // will update the imu_frontend imu bias.
    imu_frontend.resetIntegrationWithCachedBias();

    // Check the number of factors
    const gtsam::NonlinearFactorGraph& nlfg = backend_output->factor_graph_;
    size_t nr_factors_in_smoother = 0u;
    for (const auto& f : nlfg) {  // count the number of nonempty factors
      if (f) {
        nr_factors_in_smoother++;
        const auto gsf = dynamic_cast<const SmartStereoFactor*>(f.get());
        if (gsf) {
          EXPECT_EQ(gsf->keys().size(),
                    k);  // each landmark is seen in every frame
          for (size_t j = 1; j <= gsf->keys().size(); j++) {
            EXPECT_EQ(gtsam::Symbol(gsf->keys().at(j - 1)),
                      gtsam::Symbol('x', j));
          }
        }
      }
    }

    VLOG(1) << "Frame: " << k << ", # of factors: " << nr_factors_in_smoother;
    if (imu_params_.imu_preintegration_type_ ==
        ImuPreintegrationType::kPreintegratedCombinedMeasurements) {
      if (k == 0) {
        EXPECT_EQ(nr_factors_in_smoother, 3);  // 3 priors
      } else {
        EXPECT_EQ(nr_factors_in_smoother,
                  3 + k + 8 + (k - 1));  // 3 priors, 1 imu per time stamp, 8
                                         // smart factors, 1 odom between factor
      }
    } else {
      if (k == 0) {
        EXPECT_EQ(nr_factors_in_smoother, 3);  // 3 priors
      } else if (k == 1) {
        EXPECT_EQ(nr_factors_in_smoother,
                  3 + 4 * k);  // 3 priors, 1 imu + 1 btw imu biases + 1 btw
                               // stereo poses and 1 odom btw factor for each k.
                               // We do not include smart factors of length 1
      } else {
        EXPECT_EQ(nr_factors_in_smoother,
                  3 + 4 * k + 8);  // 3 priors, 1 imu + 1 btw imu biases +
                                   // 1 btw stereo poses + 1 btw odom pose per
                                   // k, 8 smart factors,
      }
    }

    // Check the results!
    const gtsam::Values& results = vio_backend->getState();
    for (FrameId f_id = 0u; f_id <= k; f_id++) {
      gtsam::Pose3 W_Pose_Blkf =
          results.at<gtsam::Pose3>(gtsam::Symbol('x', f_id));
      gtsam::Vector3 W_Vel_Blkf =
          results.at<gtsam::Vector3>(gtsam::Symbol('v', f_id));
      ImuBias imu_bias_lkf = results.at<ImuBias>(gtsam::Symbol('b', f_id));

      EXPECT_TRUE(assert_equal(poses[f_id].first, W_Pose_Blkf, tol));
      EXPECT_LT((W_Vel_Blkf - velocity_x_).norm(), tol);
      EXPECT_LT((imu_bias_lkf - imu_bias_).vector().norm(), tol);
    }
  }
}

}  // namespace VIO
