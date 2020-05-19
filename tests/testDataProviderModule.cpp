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

static constexpr int default_timeout = 100000;
static constexpr int no_timeout = std::numeric_limits<int>::max();

DECLARE_bool(images_rectified);

/* ************************************************************************** */
// Testing data
static const double tol = 1e-7;

class TestDataProviderModule : public ::testing::Test {
 public:
  TestDataProviderModule() {
    // Set google flags to assume image is already rectified--makes dummy params
    // easier
    FLAGS_images_rectified = true;

    // Create the output queue
    output_queue_ =
        VIO::make_unique<VIO::StereoVisionFrontEndModule::InputQueue>("output");

    // Create the DataProviderModule
    dummy_queue_ =
        VIO::make_unique<VIO::StereoVisionFrontEndModule::InputQueue>("unused");
    VIO::StereoMatchingParams dummy_params;
    bool parallel = false;
    data_provider_module_ = VIO::make_unique<VIO::DataProviderModule>(
        dummy_queue_.release(), "Data Provider", parallel, dummy_params);
    data_provider_module_->registerVioPipelineCallback(
        [this](VIO::StereoImuSyncPacket::UniquePtr sync_packet) {
          output_queue_->push(std::move(sync_packet));
        });
  }

  ~TestDataProviderModule() { FLAGS_images_rectified = false; }

 protected:
  VIO::StereoVisionFrontEndModule::InputQueue::UniquePtr output_queue_;
  VIO::StereoVisionFrontEndModule::InputQueue::UniquePtr dummy_queue_;
  VIO::DataProviderModule::UniquePtr data_provider_module_;
  static constexpr size_t kImuTestBundleSize = 6;

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

    void spinDataProviderModuleWithTimeout() {
      // These test cases have produced infinite loops in spin() in the past
      // Protect the testing servers with a timeout mechanism from Anton Lipov
      std::promise<bool> promisedFinished;
      auto futureResult = promisedFinished.get_future();
      std::thread(
          [this](std::promise<bool>& finished) {
            data_provider_module_->spin();

            finished.set_value(true);
          },
          std::ref(promisedFinished))
          .detach();
      auto waitResult =
          futureResult.wait_for(std::chrono::milliseconds(default_timeout));
      ASSERT_TRUE(waitResult != std::future_status::timeout);
    }
};

/* ************************************************************************* */
TEST_F(TestDataProviderModule, basicSequentialCase) {
  VIO::FrameId current_id = 0;
  VIO::Timestamp current_time = 10;  // 0 has special meaning, offset by 10

  // First frame is needed for benchmarking, send it and spin
  current_time = fillImuQueueN(current_time, 1);
  fillLeftRightQueue(++current_id, ++current_time);
  spinDataProviderModuleWithTimeout();

  EXPECT_TRUE(output_queue_->empty());

  current_time = fillImuQueueN(current_time, kImuTestBundleSize);

  fillLeftRightQueue(++current_id, ++current_time);
  // IMU and camera streams are not necessarily in sync
  // Send an IMU packet after camera packets to signal no more IMU incoming
  current_time = fillImuQueueN(current_time, 1);
  spinDataProviderModuleWithTimeout();

  VIO::StereoImuSyncPacket::UniquePtr result;
  CHECK(output_queue_->pop(result));
  CHECK(result);
  EXPECT_EQ(current_time - 1, result->timestamp_);
  EXPECT_EQ(static_cast<VIO::FrameId>(2),
            result->getStereoFrame().getFrameId());
  // +1 because it interpolates to the time frame
  EXPECT_EQ(kImuTestBundleSize + 1, result->getImuStamps().size());
}

/* ************************************************************************* */
TEST_F(TestDataProviderModule, noImuTest) {
  VIO::FrameId current_id = 0;
  VIO::Timestamp current_time = 10;  // 0 has special meaning, offset by 10

  // First frame is needed for benchmarking, send it and spin
  current_time = fillImuQueueN(current_time, 1);
  fillLeftRightQueue(++current_id, ++current_time);
  spinDataProviderModuleWithTimeout();
  EXPECT_TRUE(output_queue_->empty());

  size_t num_frames_to_reject = 6;
  for (size_t i = 0; i < num_frames_to_reject; i++) {
    fillLeftRightQueue(++current_id, ++current_time);
  }
  current_time = fillImuQueueN(current_time, 1);

  // Reject all the frames-- none have valid IMU data
  for (size_t i = 0; i < num_frames_to_reject; i++) {
    spinDataProviderModuleWithTimeout();
  }
  EXPECT_TRUE(output_queue_->empty());

  // now, a valid frame
  fillLeftRightQueue(++current_id, ++current_time);
  current_time = fillImuQueueN(current_time, 1);
  spinDataProviderModuleWithTimeout();
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
}

/* ************************************************************************* */
TEST_F(TestDataProviderModule, manyImuTest) {
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
  spinDataProviderModuleWithTimeout();
  EXPECT_TRUE(output_queue_->empty());

  for (size_t i = 0; i < num_valid_frames; i++) {
    spinDataProviderModuleWithTimeout();
    VIO::StereoImuSyncPacket::UniquePtr result;
    CHECK(output_queue_->pop(result));
    CHECK(result);
    EXPECT_EQ(static_cast<VIO::FrameId>(2 + i),
              result->getStereoFrame().getFrameId());
    // +1 because it interpolates to the time frame
    EXPECT_EQ(base_imu_to_make + i + 1, result->getImuStamps().size());
  }
}

/* ************************************************************************* */
TEST_F(TestDataProviderModule, imageBeforeImuTest) {
  VIO::FrameId current_id = 0;
  VIO::Timestamp current_time = 10;  // 0 has special meaning, offset by 10

  // Drop any frame that appears before the IMU packets do
  fillLeftRightQueue(++current_id, ++current_time);
  spinDataProviderModuleWithTimeout();
  EXPECT_TRUE(output_queue_->empty());

  // Initial frame
  current_time = fillImuQueueN(current_time, kImuTestBundleSize);
  fillLeftRightQueue(++current_id, ++current_time);
  spinDataProviderModuleWithTimeout();
  EXPECT_TRUE(output_queue_->empty());

  // Valid frame
  current_time = fillImuQueueN(current_time, kImuTestBundleSize);
  fillLeftRightQueue(++current_id, ++current_time);
  current_time = fillImuQueueN(current_time, 1);
  spinDataProviderModuleWithTimeout();

  VIO::StereoImuSyncPacket::UniquePtr result;
  CHECK(output_queue_->pop(result));
  CHECK(result);
  EXPECT_EQ(current_time - 1, result->timestamp_);
  EXPECT_EQ(static_cast<VIO::FrameId>(3),
            result->getStereoFrame().getFrameId());
  // +1 because it interpolates to the time frame
  EXPECT_EQ(kImuTestBundleSize + 1, result->getImuStamps().size());
}

/* ************************************************************************* */
TEST_F(TestDataProviderModule, imageBeforeImuDelayedSpinTest) {
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
  spinDataProviderModuleWithTimeout();
  EXPECT_TRUE(output_queue_->empty());
  // Second frame is initial frame
  spinDataProviderModuleWithTimeout();
  EXPECT_TRUE(output_queue_->empty());
  // Third frame should work
  spinDataProviderModuleWithTimeout();

  VIO::StereoImuSyncPacket::UniquePtr result;
  CHECK(output_queue_->pop(result));
  CHECK(result);
  EXPECT_EQ(current_time - 1, result->timestamp_);
  EXPECT_EQ(static_cast<VIO::FrameId>(3),
            result->getStereoFrame().getFrameId());
  // +1 because it interpolates to the time frame
  EXPECT_EQ(kImuTestBundleSize + 1, result->getImuStamps().size());
}
