/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   tstDataProviderModule.cpp
 * @brief  test DataProviderModule, especially regression testing
 * @author Andrew Violette
 * @author Luca Carlone
 */

#include <future>
#include <limits>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd.h"
#include "kimera-vio/frontend/VisionFrontEndModule.h"
#include "kimera-vio/dataprovider/DataProviderModule.h"

// Timeout macros from Anton Lipov
// http://antonlipov.blogspot.com/2015/08/how-to-timeout-tests-in-gtest.html
#ifndef TEST_TIMEOUT_BEGIN
#define TEST_TIMEOUT_BEGIN                           \
  std::promise<bool> promisedFinished;               \
  auto futureResult = promisedFinished.get_future(); \
              std::thread([this](std::promise<bool>& finished) {
#define TEST_TIMEOUT_FAIL_END(X)                                 \
  finished.set_value(true);                                      \
  }, std::ref(promisedFinished)).detach();           \
  ASSERT_NE(futureResult.wait_for(std::chrono::milliseconds(X)), \
            std::future_status::timeout);
#define TEST_TIMEOUT_SUCCESS_END(X)                              \
  finished.set_value(true);                                      \
  }, std::ref(promisedFinished)).detach();           \
  ASSERT_EQ(futureResult.wait_for(std::chrono::milliseconds(X)), \
            std::future_status::timeout);
#endif
static constexpr int default_timeout = 100000;
static constexpr int no_timeout = std::numeric_limits<int>::max();

/* ************************************************************************** */
// Testing data
static const double tol = 1e-7;

class TestDataProviderModule : public ::testing::Test {
 public:
  TestDataProviderModule() {  }

 protected:
  VIO::StereoVisionFrontEndModule::InputQueue* output_queue_ = nullptr;
  VIO::StereoVisionFrontEndModule::InputQueue* dummy_queue_ = nullptr;
  VIO::DataProviderModule* data_provider_module_ = nullptr;
  static constexpr size_t kImuTestBundleSize = 6;

  void SetUp() override {
    // Clean up again in case something went wrong last time
    TearDown();

    // Set google flags to assume image is already rectified--makes dummy params
    // easier
    int fake_argc = 2;
    char** fake_argv =
        reinterpret_cast<char**>(malloc(sizeof(char*) * fake_argc));
    fake_argv[0] = "foo";
    fake_argv[1] = "--images_rectified";
    google::ParseCommandLineFlags(&fake_argc, &fake_argv, true);

    // Create the output queue
    output_queue_ = new VIO::StereoVisionFrontEndModule::InputQueue("output");

    // Create the DataProviderModule
    dummy_queue_ = new VIO::StereoVisionFrontEndModule::InputQueue("unused");
    VIO::StereoMatchingParams dummy_params;
    bool parallel = false;
    data_provider_module_ = new VIO::DataProviderModule(
        dummy_queue_, "Data Provider", parallel, dummy_params);
    data_provider_module_->registerVioPipelineCallback(
        [this](VIO::StereoImuSyncPacket::UniquePtr sync_packet) {
          output_queue_->push(std::move(sync_packet));
        });
  }

  void TearDown() override {
    if (data_provider_module_ != nullptr) {
      data_provider_module_->shutdown();
      delete data_provider_module_;
      data_provider_module_ = nullptr;
    }
    if (dummy_queue_ != nullptr) {
      delete dummy_queue_;
      dummy_queue_ = nullptr;
    }
    if (output_queue_ != nullptr) {
      delete output_queue_;
      output_queue_ = nullptr;
    }

    // Reset google flags to defaults
    int fake_argc = 2;
    char** fake_argv =
        reinterpret_cast<char**>(malloc(sizeof(char*) * fake_argc));
    fake_argv[0] = "foo";
    fake_argv[1] = "--noimages_rectified";
    google::ParseCommandLineFlags(&fake_argc, &fake_argv, true);
  }

  VIO::Timestamp fillImuQueueN(const VIO::Timestamp& prev_timestamp,
                               size_t num_imu_to_make) {
    VIO::Timestamp current_time = prev_timestamp;
    for (size_t i = 0; i < num_imu_to_make; i++) {
      data_provider_module_->fillImuQueue(
          VIO::ImuMeasurement(++current_time, VIO::ImuAccGyr::Zero()));
    }
    return current_time;
    }

    void fillLeftRightQueue(const VIO::FrameId& frame_id,
                            const VIO::Timestamp& timestamp) {
      data_provider_module_->fillLeftFrameQueue(
          makeDummyFrame(frame_id, timestamp));
      data_provider_module_->fillRightFrameQueue(
          makeDummyFrame(frame_id, timestamp));
    }

    VIO::Frame::UniquePtr makeDummyFrame(const VIO::FrameId& frame_id,
                                         const VIO::Timestamp& timestamp) {
      return VIO::make_unique<VIO::Frame>(
          frame_id, timestamp, makeDumyCameraParams(), cv::Mat());
    }

    VIO::CameraParams makeDumyCameraParams() {
      VIO::CameraParams dummy_params;
      dummy_params.R_rectify_ = cv::Mat::zeros(3, 3, CV_32F);
      dummy_params.P_ = cv::Mat::zeros(3, 3, CV_32F);
      return dummy_params;
    }
};

/* ************************************************************************* */
TEST_F(TestDataProviderModule, basicSequentialCase) {
  TEST_TIMEOUT_BEGIN

  VIO::FrameId current_id = 0;
  VIO::Timestamp current_time = 10;  // 0 has special meaning, offset by 10

  // First frame is needed for benchmarking, send it and spin
  current_time = fillImuQueueN(current_time, 1);
  fillLeftRightQueue(++current_id, ++current_time);
  data_provider_module_->spin();

  EXPECT_TRUE(output_queue_->empty());

  current_time = fillImuQueueN(current_time, kImuTestBundleSize);

  fillLeftRightQueue(++current_id, ++current_time);
  // IMU and camera streams are not necessarily in sync
  // Send an IMU packet after camera packets to signal no more IMU incoming
  current_time = fillImuQueueN(current_time, 1);
  data_provider_module_->spin();

  VIO::StereoImuSyncPacket::UniquePtr result;
  CHECK(output_queue_->pop(result));
  CHECK(result);
  EXPECT_EQ(current_time - 1, result->timestamp_);
  EXPECT_EQ(static_cast<VIO::FrameId>(2),
            result->getStereoFrame().getFrameId());
  // +1 because it interpolates to the time frame
  EXPECT_EQ(kImuTestBundleSize + 1, result->getImuStamps().size());

  TEST_TIMEOUT_FAIL_END(default_timeout)
}

/* ************************************************************************* */
TEST_F(TestDataProviderModule, noImuTest) {
  TEST_TIMEOUT_BEGIN

  VIO::FrameId current_id = 0;
  VIO::Timestamp current_time = 10;  // 0 has special meaning, offset by 10

  // First frame is needed for benchmarking, send it and spin
  current_time = fillImuQueueN(current_time, 1);
  fillLeftRightQueue(++current_id, ++current_time);
  data_provider_module_->spin();
  EXPECT_TRUE(output_queue_->empty());

  size_t num_frames_to_reject = 6;
  for (size_t i = 0; i < num_frames_to_reject; i++) {
    fillLeftRightQueue(++current_id, ++current_time);
  }
  current_time = fillImuQueueN(current_time, 1);

  // Reject all the frames-- none have valid IMU data
  for (size_t i = 0; i < num_frames_to_reject; i++) {
    data_provider_module_->spin();
  }
  EXPECT_TRUE(output_queue_->empty());

  // now, a valid frame
  fillLeftRightQueue(++current_id, ++current_time);
  current_time = fillImuQueueN(current_time, 1);
  data_provider_module_->spin();
  VIO::StereoImuSyncPacket::UniquePtr result;
  CHECK(output_queue_->pop(result));
  CHECK(result);
  EXPECT_EQ(current_time - 1, result->timestamp_);
  EXPECT_EQ(static_cast<VIO::FrameId>(num_frames_to_reject + 2),
            result->getStereoFrame().getFrameId());
  // +1 because it interpolates to the time frame
  EXPECT_EQ(1 + 1, result->getImuStamps().size());

  // We need to cover the case where two frames are adjacent after
  // initialization Do it again

  TEST_TIMEOUT_FAIL_END(default_timeout)
}

/* ************************************************************************* */
TEST_F(TestDataProviderModule, manyImuTest) {
  TEST_TIMEOUT_BEGIN

  VIO::FrameId current_id = 0;
  VIO::Timestamp current_time = 10;  // 0 has special meaning, offset by 10

  size_t base_imu_to_make = 10;
  current_time = fillImuQueueN(current_time, base_imu_to_make);
  fillLeftRightQueue(++current_id, ++current_time);
  size_t num_valid_frames = 10;
  for (size_t i = 0; i < num_valid_frames; i++) {
    current_time = fillImuQueueN(current_time, base_imu_to_make + i);
    fillLeftRightQueue(++current_id, ++current_time);
  }
  current_time = fillImuQueueN(current_time, base_imu_to_make);

  // First frame is needed for benchmarking, should produce nothing
  data_provider_module_->spin();
  EXPECT_TRUE(output_queue_->empty());

  for (size_t i = 0; i < num_valid_frames; i++) {
    data_provider_module_->spin();
    VIO::StereoImuSyncPacket::UniquePtr result;
    CHECK(output_queue_->pop(result));
    CHECK(result);
    EXPECT_EQ(static_cast<VIO::FrameId>(2 + i),
              result->getStereoFrame().getFrameId());
    // +1 because it interpolates to the time frame
    EXPECT_EQ(base_imu_to_make + i + 1, result->getImuStamps().size());
  }

  TEST_TIMEOUT_FAIL_END(default_timeout)
}

/* ************************************************************************* */
TEST_F(TestDataProviderModule, imageBeforeImuTest) {
  TEST_TIMEOUT_BEGIN

  VIO::FrameId current_id = 0;
  VIO::Timestamp current_time = 10;  // 0 has special meaning, offset by 10

  // Drop any frame that appears before the IMU packets do
  fillLeftRightQueue(++current_id, ++current_time);
  data_provider_module_->spin();
  EXPECT_TRUE(output_queue_->empty());

  // Initial frame
  current_time = fillImuQueueN(current_time, kImuTestBundleSize);
  fillLeftRightQueue(++current_id, ++current_time);
  data_provider_module_->spin();
  EXPECT_TRUE(output_queue_->empty());

  // Valid frame
  current_time = fillImuQueueN(current_time, kImuTestBundleSize);
  fillLeftRightQueue(++current_id, ++current_time);
  current_time = fillImuQueueN(current_time, 1);
  data_provider_module_->spin();

  VIO::StereoImuSyncPacket::UniquePtr result;
  CHECK(output_queue_->pop(result));
  CHECK(result);
  EXPECT_EQ(current_time - 1, result->timestamp_);
  EXPECT_EQ(static_cast<VIO::FrameId>(3),
            result->getStereoFrame().getFrameId());
  // +1 because it interpolates to the time frame
  EXPECT_EQ(kImuTestBundleSize + 1, result->getImuStamps().size());

  TEST_TIMEOUT_FAIL_END(default_timeout)
}

/* ************************************************************************* */
TEST_F(TestDataProviderModule, imageBeforeImuDelayedSpinTest) {
  TEST_TIMEOUT_BEGIN

  VIO::FrameId current_id = 0;
  VIO::Timestamp current_time = 10;  // 0 has special meaning, offset by 10

  // Drop any frame that appears before the IMU packets do
  fillLeftRightQueue(++current_id, ++current_time);

  // Initial frame
  current_time = fillImuQueueN(current_time, kImuTestBundleSize);
  fillLeftRightQueue(++current_id, ++current_time);

  // Valid frame
  current_time = fillImuQueueN(current_time, kImuTestBundleSize);
  fillLeftRightQueue(++current_id, ++current_time);
  current_time = fillImuQueueN(current_time, 1);

  // Reject first frame
  data_provider_module_->spin();
  EXPECT_TRUE(output_queue_->empty());
  // Second frame is initial frame
  data_provider_module_->spin();
  EXPECT_TRUE(output_queue_->empty());
  // Third frame should work
  data_provider_module_->spin();

  VIO::StereoImuSyncPacket::UniquePtr result;
  CHECK(output_queue_->pop(result));
  CHECK(result);
  EXPECT_EQ(current_time - 1, result->timestamp_);
  EXPECT_EQ(static_cast<VIO::FrameId>(3),
            result->getStereoFrame().getFrameId());
  // +1 because it interpolates to the time frame
  EXPECT_EQ(kImuTestBundleSize + 1, result->getImuStamps().size());

  TEST_TIMEOUT_FAIL_END(default_timeout)
}
