/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testOnlineAlignment.cpp
 * @brief  Unit tests for Online Alignment class.
 * @author Antoni Rosinol, Sandro Berchier, Luca Carlone
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <math.h>

#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontend.h"
#include "kimera-vio/initial/OnlineGravityAlignment.h"
#include "kimera-vio/utils/ThreadsafeImuBuffer.h"
#include "kimera-vio/utils/UtilsNumerical.h"

DECLARE_string(test_data_path);

namespace VIO {

class OnlineAlignmentFixture : public ::testing::Test {
 public:
  OnlineAlignmentFixture()
      : dataset_(nullptr),
        estimated_poses_(),
        pims_(),
        ahrs_pim_(),
        delta_t_poses_(),
        imu_params_(),
        imu_bias_(),
        bias_acc_(),
        bias_gyr_(),
        init_navstate_() {
    // Set IMU params
    initializeImuParams(&imu_params_);

    // Set IMU bias
    bias_acc_ = gtsam::Vector3::Zero();
    bias_gyr_ = gtsam::Vector3::Zero();
    imu_bias_ = ImuBias(bias_acc_, bias_gyr_);
  }

 protected:
  virtual void SetUp() override {}
  virtual void TearDown() override {}

  void initializeOnlineAlignmentData(int n_begin_data, int n_frames_data,
                                     const std::string& dataset_path) {
    int initial_k = 0;
    int final_k = 0;
    VioParams vio_params ("");
    vio_params.parallel_run_ = true;
    dataset_ = VIO::make_unique<EurocDataProvider>(
                 dataset_path, initial_k, final_k, vio_params);

    // Get GT poses and IMU pims.
    Timestamp timestamp_last_frame;
    Timestamp timestamp_frame_k;
    ImuMeasurements imu_meas;

    estimated_poses_.clear();
    pims_.clear();
    delta_t_poses_.clear();
    ahrs_pim_.clear();

    // Extract first element in the ground-truth map.
    std::map<Timestamp, VioNavState>::iterator it;
    it = dataset_->gt_data_.map_to_gt_.begin();
    // Move iterator to desired spot
    std::advance(it, n_begin_data);

    timestamp_last_frame = it->first;
    gtsam::Pose3 gt_pose_k = it->second.pose_;
    estimated_poses_.push_back(gt_pose_k);
    init_navstate_ = it->second;

    // Move to the second one
    it++;
    while (it != dataset_->gt_data_.map_to_gt_.end()) {
      // Get GT information
      timestamp_frame_k = it->first;
      gt_pose_k = it->second.pose_;

      // Get PIM information
      // TODO(Toni): fix this
      // dataset_->imu_data_.imu_buffer_.getImuDataInterpolatedUpperBorder(
      //     timestamp_last_frame,
      //     timestamp_frame_k,
      //     &imu_meas.timestamps_,
      //     &imu_meas.acc_gyr_);
      ImuFrontend imu_frontend(imu_params_, imu_bias_);
      const auto& pim = imu_frontend.preintegrateImuMeasurements(
          imu_meas.timestamps_, imu_meas.acc_gyr_);

      // AHRS Pre-integration
      // Define covariance matrices
      gtsam::Vector3 biasHat = gtsam::Vector3::Zero();
      double accNoiseVar = 0.01;
      const Matrix3 kMeasuredAccCovariance =
          accNoiseVar * gtsam::Matrix3::Identity();
      gtsam::AHRSFactor::PreintegratedMeasurements ahrs_pim(
          biasHat, kMeasuredAccCovariance);
      for (size_t i = 0; i < (imu_meas.acc_gyr_.cols() - 1); ++i) {
        double delta_t = UtilsNumerical::NsecToSec(imu_meas.timestamps_(i + 1) -
                                                imu_meas.timestamps_(i));
        gtsam::Vector3 measured_omega = imu_meas.acc_gyr_.block(3, 6, i, i + 1);
        ahrs_pim.integrateMeasurement(measured_omega, delta_t);
      }

      // Buffer for online alignment
      estimated_poses_.push_back(estimated_poses_[0].between(gt_pose_k));
      delta_t_poses_.push_back(
          UtilsNumerical::NsecToSec(timestamp_frame_k - timestamp_last_frame));
      pims_.push_back(pim);
      ahrs_pim_.push_back(ahrs_pim);
      if (pims_.size() > n_frames_data) {
        break;
      }
      // Move to next element in map
      timestamp_last_frame = timestamp_frame_k;
      it++;
    }
    // Set initial pose to identity as we compute all relative to it
    estimated_poses_[0] = gtsam::Pose3();
  }

  void initializeImuParams(ImuParams* imu_params) const {
    CHECK_NOTNULL(imu_params);
    imu_params->acc_random_walk_ = 1.0;
    imu_params->acc_noise_density_ = 1.0;
    imu_params->gyro_random_walk_ = 1.0;
    imu_params->gyro_noise_density_ = 1.0;

    // This is needed for online alignment
    imu_params->n_gravity_ << 0.0, 0.0, 0.0;
    imu_params->imu_integration_sigma_ = 1.0;
  }

 protected:
  static constexpr double tol_GB = 2e-4;
  static constexpr double tol_TB = 1e-7;
  static constexpr double tol_OGA = 1e-3;
  static constexpr double tol_RD_gv = 25e-2;
  static constexpr double tol_RD_an = 4 / 180.0 * M_PI;

  std::unique_ptr<EurocDataProvider> dataset_;
  AlignmentPoses estimated_poses_;
  AlignmentPims pims_;
  InitialAHRSPims ahrs_pim_;
  std::vector<double> delta_t_poses_;
  ImuParams imu_params_;
  ImuBias imu_bias_;
  gtsam::Vector3 bias_acc_;
  gtsam::Vector3 bias_gyr_;
  VioNavState init_navstate_;
};

/* -------------------------------------------------------------------------- */
TEST_F(OnlineAlignmentFixture, DISABLED_GyroscopeBiasEstimation) {
  // Construct ETH Parser and get data
  std::string reason = "test of gyroscope estimation";
  static const std::string data_path(FLAGS_test_data_path +
                                     "/ForOnlineAlignment/gyro_bias/");
  int n_begin = 1;
  int n_frames = 5;
  initializeOnlineAlignmentData(n_begin, n_frames, data_path);

  // Construct online alignment class with dummy gravity vector
  gtsam::Vector3 n_gravity = gtsam::Vector3::Zero();
  OnlineGravityAlignment initial_alignment(
      estimated_poses_, delta_t_poses_, pims_, n_gravity);

  // Initialize OnlineAlignment
  gtsam::Vector3 gyro_bias = imu_bias_.gyroscope();
  CHECK_DOUBLE_EQ(gyro_bias.norm(), 0.0);

  // Compute Gyroscope Bias
  CHECK(initial_alignment.estimateGyroscopeBiasOnly(&gyro_bias, false));

  // Final test check against real bias in data
  gtsam::Vector3 real_gyro_bias(0.0001, 0.0002, 0.0003);
  EXPECT_NEAR(real_gyro_bias.norm(), gyro_bias.norm(), tol_GB);
}

/* -------------------------------------------------------------------------- */
TEST_F(OnlineAlignmentFixture, DISABLED_GyroscopeBiasEstimationAHRS) {
  // Construct ETH Parser and get data
  static const std::string data_path(FLAGS_test_data_path +
                                     "/ForOnlineAlignment/gyro_bias/");

  int n_begin = 1;
  int n_frames = 5;
  initializeOnlineAlignmentData(n_begin, n_frames, data_path);

  // Construct online alignment class with dummy gravity vector
  gtsam::Vector3 n_gravity = gtsam::Vector3::Zero();
  OnlineGravityAlignment initial_alignment(
      estimated_poses_, delta_t_poses_, pims_, n_gravity, ahrs_pim_);

  // Initialize OnlineAlignment
  gtsam::Vector3 gyro_bias = imu_bias_.gyroscope();
  CHECK_DOUBLE_EQ(gyro_bias.norm(), 0.0);

  // Compute Gyroscope Bias
  CHECK(initial_alignment.estimateGyroscopeBiasOnly(&gyro_bias, true));

  // Final test check against real bias in data
  gtsam::Vector3 real_gyro_bias(0.0001, 0.0002, 0.0003);
  EXPECT_NEAR(real_gyro_bias.norm(), gyro_bias.norm(), tol_GB);
}

/* -------------------------------------------------------------------------- */
TEST_F(OnlineAlignmentFixture, CreateTangentBasis) {
  for (int i=0; i < 20; i++) {
    // Create random vector (this is not unit vector!)
    gtsam::Vector3 random_vector = UtilsOpenCV::RandomVectorGenerator(1.0);

    // Create tangent basis to random vector
    gtsam::Matrix tangent_basis =
            OnlineGravityAlignment::createTangentBasis(random_vector);

    // Check size is corrrect
    CHECK_EQ(tangent_basis.cols(), 2);
    CHECK_EQ(tangent_basis.rows(), 3);

    // Check product of matrix columns with random vector
    gtsam::Vector3 basis_vec_y(tangent_basis(0, 0),
                              tangent_basis(1, 0),
                              tangent_basis(2, 0));
    gtsam::Vector3 basis_vec_z(tangent_basis(0, 1),
                              tangent_basis(1, 1),
                              tangent_basis(2, 1));

    // Check that vector product is zero (orthogonal)
    EXPECT_NEAR(0.0, gtsam::dot(basis_vec_y, basis_vec_z), tol_TB);
    EXPECT_NEAR(0.0, gtsam::dot(basis_vec_y, random_vector), tol_TB);
    EXPECT_NEAR(0.0, gtsam::dot(basis_vec_z, random_vector), tol_TB);
  }
}

/* -------------------------------------------------------------------------- */
TEST_F(OnlineAlignmentFixture, DISABLED_OnlineGravityAlignment) {
  // Construct ETH Parser and get data
  static const std::string data_path(FLAGS_test_data_path +
                                     "/ForOnlineAlignment/alignment/");
  int n_begin = 1;
  int n_frames = 40;
  initializeOnlineAlignmentData(n_begin, n_frames, data_path);

  // Initialize OnlineAlignment
  gtsam::Vector3 gyro_bias = imu_bias_.gyroscope();
  CHECK_DOUBLE_EQ(gyro_bias.norm(), 0.0);
  gtsam::Vector3 g_iter;
  gtsam::NavState init_navstate;

  // Construct online alignment class with world gravity vector
  gtsam::Vector3 n_gravity(0.0, 0.0, -9.81);
  OnlineGravityAlignment initial_alignment(
      estimated_poses_, delta_t_poses_, pims_, n_gravity);

  // Compute online gravity alignment (without gyroscope bias estimation)
  CHECK(initial_alignment.alignVisualInertialEstimates(&gyro_bias, &g_iter,
                                                 &init_navstate, false));

  // Final test checks
  gtsam::Vector3 real_init_vel(init_navstate_.velocity_);
  gtsam::Vector3 real_body_grav(init_navstate_.pose_.rotation().transpose() *
                                n_gravity);
  gtsam::Pose3 real_init_pose(init_navstate_.pose_.rotation(),
                              gtsam::Vector3::Zero());
  LOG(INFO) << real_body_grav << " vs. " << g_iter;
  EXPECT_NEAR(n_gravity.norm(), g_iter.norm(), tol_OGA);
  EXPECT_NEAR(real_body_grav.x(), g_iter.x(), tol_OGA);
  EXPECT_NEAR(real_body_grav.y(), g_iter.y(), tol_OGA);
  EXPECT_NEAR(real_body_grav.z(), g_iter.z(), tol_OGA);
  //EXPECT_TRUE(assert_equal(real_init_pose, init_navstate.pose(), tol_OGA));
  EXPECT_NEAR(real_init_vel.norm(),
            init_navstate.velocity().norm(), tol_OGA);
  EXPECT_NEAR(real_init_vel.x(),
            init_navstate.velocity().x(), tol_OGA);
  EXPECT_NEAR(real_init_vel.y(),
            init_navstate.velocity().y(), tol_OGA);
  EXPECT_NEAR(real_init_vel.z(),
            init_navstate.velocity().z(), tol_OGA);
}

/* -------------------------------------------------------------------------- */
TEST_F(OnlineAlignmentFixture, DISABLED_GravityAlignmentRealData) {
  for (int i = 0; i < 30; i++) {
    static const std::string data_path(FLAGS_test_data_path +
                                       "/ForOnlineAlignment/real_data/");
    int n_begin = int(UtilsNumerical::RandomFloatGenerator(3000));
    int n_frames = 40;
    initializeOnlineAlignmentData(n_begin, n_frames, data_path);

    // Initialize OnlineAlignment
    gtsam::Vector3 gyro_bias = imu_bias_.gyroscope();
    CHECK_DOUBLE_EQ(gyro_bias.norm(), 0.0);
    gtsam::Vector3 g_iter;
    gtsam::NavState init_navstate;

    // Construct online alignment class with world gravity vector
    gtsam::Vector3 n_gravity(0.0, 0.0, -9.81);
    OnlineGravityAlignment initial_alignment(
        estimated_poses_, delta_t_poses_, pims_, n_gravity);

    // Compute Gyroscope Bias
    CHECK(initial_alignment.alignVisualInertialEstimates(&gyro_bias, &g_iter,
                                                         &init_navstate));

    // Final test checks
    gtsam::Vector3 real_init_vel(init_navstate_.velocity_);
    gtsam::Vector3 real_body_grav(init_navstate_.pose_.rotation().transpose() *
                                  n_gravity);
    gtsam::Pose3 real_init_pose(init_navstate_.pose_.rotation(),
                                gtsam::Vector3::Zero());

    LOG(INFO) << real_body_grav << " vs. " << g_iter;
    EXPECT_NEAR(n_gravity.norm(), g_iter.norm(), tol_RD_gv);
    EXPECT_NEAR(real_body_grav.x(), g_iter.x(), tol_RD_gv);
    EXPECT_NEAR(real_body_grav.y(), g_iter.y(), tol_RD_gv);
    EXPECT_NEAR(real_body_grav.z(), g_iter.z(), tol_RD_gv);


    EXPECT_NEAR(fabs(remainder(real_init_pose.rotation().pitch() -
      init_navstate.pose().rotation().pitch(), 2*M_PI)), 0.0, tol_RD_an);
    EXPECT_NEAR(fabs(remainder(real_init_pose.rotation().roll() -
      init_navstate.pose().rotation().roll(), 2*M_PI)), 0.0, tol_RD_an);
    // Yaw angle is irrelevant for starting pose

    // TODO(Sandro): Add velocity test in same frame
    /* EXPECT_NEAR(real_init_vel.norm(),
              init_navstate.velocity().norm(), tol_RD);
    EXPECT_NEAR(real_init_vel.x(),
              init_navstate.velocity().x(), tol_RD);
    EXPECT_NEAR(real_init_vel.y(),
              init_navstate.velocity().y(), tol_RD);
    EXPECT_NEAR(real_init_vel.z(),
              init_navstate.velocity().z(), tol_RD); */
  }
}

}  // Namespace VIO
