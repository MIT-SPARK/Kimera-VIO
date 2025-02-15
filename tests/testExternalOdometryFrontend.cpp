/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testExternalOdometryFrontend.cpp
 * @brief  test that the DataProvider and VisionImuFrontend handle external
 * odometry correctly
 * @author Nathan Hughes
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <future>
#include <limits>
#include <utility>

#include "kimera-vio/dataprovider/StereoDataProviderModule.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/FrontendInputPacketBase.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/VisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/VisionImuFrontend.h"

DECLARE_bool(use_external_odometry);

namespace VIO {

class TestExternalOdometryProvider : public ::testing::Test {
 public:
  TestExternalOdometryProvider() : last_id_(0), output_queue_("test_output") {
    test_provider_ = std::make_unique<StereoDataProviderModule>(
        &output_queue_, "test_stereo_provider", false, StereoMatchingParams());
    test_provider_->registerVioPipelineCallback(
        [this](FrontendInputPacketBase::UniquePtr packet) {
          output_queue_.push(std::move(packet));
        });
  }

  virtual ~TestExternalOdometryProvider() = default;

 protected:
  DataProviderModule::OutputQueue output_queue_;
  StereoDataProviderModule::UniquePtr test_provider_;
  FrameId last_id_;

  void addImu(Timestamp timestamp) {
    test_provider_->fillImuQueue(ImuMeasurement(timestamp, ImuAccGyr::Zero()));
  }

  void addFrame(Timestamp timestamp) {
    Frame::UniquePtr lframe =
        std::make_unique<Frame>(last_id_, timestamp, CameraParams(), cv::Mat());
    test_provider_->fillLeftFrameQueue(std::move(lframe));
    Frame::UniquePtr rframe =
        std::make_unique<Frame>(last_id_, timestamp, CameraParams(), cv::Mat());
    test_provider_->fillRightFrameQueue(std::move(rframe));
    last_id_++;
  }

  void addOdometry(Timestamp timestamp) {
    test_provider_->fillExternalOdometryQueue(
        ExternalOdomMeasurement(timestamp, gtsam::NavState()));
  }

  void spinWithTimeout(int timeout = 1000) {
    // These test cases have produced infinite loops in spin() in the past
    // Protect the testing servers with a timeout mechanism from Anton Lipov
    std::promise<bool> promisedFinished;
    auto futureResult = promisedFinished.get_future();
    std::thread(
        [this](std::promise<bool>& finished) {
          test_provider_->spin();
          finished.set_value(true);
        },
        std::ref(promisedFinished))
        .detach();
    auto waitResult = futureResult.wait_for(std::chrono::milliseconds(timeout));
    CHECK(waitResult != std::future_status::timeout) << "spin timed out";
  }
};

class TestExternalOdometryProviderEnabled
    : public TestExternalOdometryProvider {
 public:
  TestExternalOdometryProviderEnabled() : TestExternalOdometryProvider() {
    FLAGS_use_external_odometry = true;
    // remake provider now that odometry is enabled
    test_provider_ = std::make_unique<StereoDataProviderModule>(
        &output_queue_, "test_stereo_provider", false, StereoMatchingParams());
    test_provider_->registerVioPipelineCallback(
        [this](FrontendInputPacketBase::UniquePtr packet) {
          output_queue_.push(std::move(packet));
        });
  }

  virtual ~TestExternalOdometryProviderEnabled() {
    FLAGS_use_external_odometry = false;
  }
};

ImuParams makeFakeImuParams() {
  ImuParams to_return;
  to_return.gyro_noise_density_ = 1.0;
  to_return.gyro_random_walk_ = 1.0;
  to_return.acc_noise_density_ = 1.0;
  to_return.acc_random_walk_ = 1.0;
  to_return.imu_integration_sigma_ = 1.0;
  to_return.nominal_sampling_time_s_ = 1.0;
  return to_return;
}

class MockVisionImuFrontend : public VisionImuFrontend {
 public:
  explicit MockVisionImuFrontend(const OdometryParams& params)
      : VisionImuFrontend(FrontendParams(),
                          makeFakeImuParams(),
                          ImuBias(),
                          nullptr,
                          false,
                          params) {}

  using VisionImuFrontend::getExternalOdometryRelativeBodyPose;
  using VisionImuFrontend::getExternalOdometryWorldVelocity;

 protected:
  FrontendOutputPacketBase::UniquePtr bootstrapSpin(
      FrontendInputPacketBase::UniquePtr&& input) override {
    return nullptr;
  }

  FrontendOutputPacketBase::UniquePtr timeAlignmentSpin(
      FrontendInputPacketBase::UniquePtr&& input) override {
    return nullptr;
  }

  FrontendOutputPacketBase::UniquePtr nominalSpin(
      FrontendInputPacketBase::UniquePtr&& input) override {
    return nullptr;
  }
};

TEST_F(TestExternalOdometryProvider, basicSequentialCase) {
  // First frame is needed for benchmarking, send it and spin
  addImu(10);
  addFrame(11);
  spinWithTimeout();
  EXPECT_TRUE(output_queue_.empty());

  // add some measurements
  addImu(12);
  addImu(13);
  addImu(14);

  addFrame(17);
  addImu(18);  // trigger packet creation
  spinWithTimeout();

  FrontendInputPacketBase::UniquePtr result_base;
  ASSERT_TRUE(output_queue_.pop(result_base));
}

TEST_F(TestExternalOdometryProviderEnabled, noOdometryPresent) {
  // First frame is needed for benchmarking, send it and spin
  addImu(10);
  addFrame(11);
  spinWithTimeout();
  EXPECT_TRUE(output_queue_.empty());

  // add some measurements
  addImu(12);
  addImu(13);
  addImu(14);

  addFrame(17);
  addImu(18);  // trigger packet creation
  spinWithTimeout();

  FrontendInputPacketBase::UniquePtr result_base;
  // we don't have any odometry measurements, so we shouldn't
  // get a packet yet
  EXPECT_FALSE(output_queue_.pop(result_base));
}

TEST_F(TestExternalOdometryProviderEnabled, validOdometrySequence) {
  // First frame is needed for benchmarking, send it and spin
  addImu(10);
  addFrame(11);
  spinWithTimeout();
  EXPECT_TRUE(output_queue_.empty());

  addImu(12);
  addImu(13);
  addImu(14);

  addOdometry(17);  // exact match should trigger packet creation
  addFrame(17);
  addImu(18);  // trigger packet creation
  spinWithTimeout();

  FrontendInputPacketBase::UniquePtr result_base;
  ASSERT_TRUE(output_queue_.pop(result_base));
  EXPECT_TRUE(result_base->world_NavState_ext_odom_);
}

TEST_F(TestExternalOdometryProvider, validOdometrySequenceNoOdom) {
  // First frame is needed for benchmarking, send it and spin
  addImu(10);
  addFrame(11);
  spinWithTimeout();
  EXPECT_TRUE(output_queue_.empty());

  addImu(12);
  addImu(13);
  addImu(14);

  addOdometry(17);
  addFrame(17);
  addImu(18);  // trigger packet creation
  spinWithTimeout();

  FrontendInputPacketBase::UniquePtr result_base;
  ASSERT_TRUE(output_queue_.pop(result_base));
  EXPECT_FALSE(result_base->world_NavState_ext_odom_);
}

TEST_F(TestExternalOdometryProviderEnabled, inexactOdometrySequence) {
  // First frame is needed for benchmarking, send it and spin
  addImu(10);
  addFrame(11);
  spinWithTimeout();
  EXPECT_TRUE(output_queue_.empty());

  addImu(12);
  addImu(13);
  addImu(14);

  addOdometry(16);  // shouldn't trigger packet creation
  addFrame(17);
  addImu(18);

  spinWithTimeout();
  FrontendInputPacketBase::UniquePtr result_base;
  EXPECT_FALSE(output_queue_.pop(result_base));

  addOdometry(18);  // trigger packet creation
  spinWithTimeout();
  ASSERT_TRUE(output_queue_.pop(result_base));
  EXPECT_TRUE(result_base->world_NavState_ext_odom_);
}

TEST(TestExternalOdometryFrontend, ExternalOdomVelCorrect) {
  OdometryParams params;
  MockVisionImuFrontend frontend(params);

  // we don't get anything back if odometry isn't present
  FrontendInputPacketBase invalid_input(
      0, ImuStampS::Zero(1), ImuAccGyrS::Zero(6, 1));
  EXPECT_FALSE(frontend.getExternalOdometryWorldVelocity(&invalid_input));

  // having odometry means we get something back immediately
  Eigen::Vector3d expected_vel;
  expected_vel << 1.0, 2.0, 3.0;
  FrontendInputPacketBase valid_input(
      0,
      ImuStampS::Zero(1),
      ImuAccGyrS::Zero(6, 1),
      gtsam::NavState(gtsam::Rot3(), Eigen::Vector3d::Zero(), expected_vel));
  auto result = frontend.getExternalOdometryWorldVelocity(&valid_input);
  ASSERT_TRUE(result);
  EXPECT_EQ(expected_vel(0), result.value()(0));
  EXPECT_EQ(expected_vel(1), result.value()(1));
  EXPECT_EQ(expected_vel(2), result.value()(2));
}

TEST(TestExternalOdometryFrontend, ExternalOdomRelPoseIdentity) {
  OdometryParams params;
  MockVisionImuFrontend frontend(params);

  // we don't get anything back if odometry isn't present
  FrontendInputPacketBase invalid_input(
      0, ImuStampS::Zero(1), ImuAccGyrS::Zero(6, 1));
  EXPECT_FALSE(frontend.getExternalOdometryRelativeBodyPose(&invalid_input));

  // arbitrary rotation and translation
  gtsam::NavState navstate(gtsam::Rot3::Rodrigues(0.2, 0.1, -0.3),
                           (Eigen::Vector3d() << 0.4, -0.1, 0.35).finished(),
                           Eigen::Vector3d::Zero());

  FrontendInputPacketBase valid_input(
      0, ImuStampS::Zero(1), ImuAccGyrS::Zero(6, 1), navstate);
  EXPECT_FALSE(frontend.getExternalOdometryRelativeBodyPose(&valid_input));

  // add the same state to get the identity when they're internally composed
  auto result = frontend.getExternalOdometryRelativeBodyPose(&valid_input);
  ASSERT_TRUE(result);
  // TODO(nathan) replace with actual tolerance constant if we write more tests
  EXPECT_NEAR(0.0, Pose3::Logmap(result.value()).norm(), 1.0e-9);
}

}  // namespace VIO
