/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testParallelMonorovider.cpp
 * @brief  test the MonoDataProvider, especially around timestamp invariants
 * @author Nathan Hughes
 */

#include <chrono>
#include <future>
#include <limits>
#include <thread>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/dataprovider/MonoDataProviderModule.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/FrontendInputPacketBase.h"

namespace VIO {

class TestParallelMonoProvider : public ::testing::Test {
 public:
  TestParallelMonoProvider() : last_id_(0), output_queue_("test_output") {
    test_provider_ = VIO::make_unique<MonoDataProviderModule>(
        &output_queue_, "test_mono_provider", true);
    test_provider_->registerVioPipelineCallback(
        [this](FrontendInputPacketBase::UniquePtr packet) {
          output_queue_.push(std::move(packet));
        });
    startSpin();
  }

  void TearDown() { stopSpin(); }

  ~TestParallelMonoProvider() = default;

 protected:
  std::unique_ptr<std::thread> spin_thread_;
  std::unique_ptr<std::thread> empty_queue_thread_;
  DataProviderModule::OutputQueue output_queue_;
  MonoDataProviderModule::UniquePtr test_provider_;
  std::future<bool> finish_future_;
  FrameId last_id_;

  void addImu(Timestamp timestamp) {
    test_provider_->fillImuQueue(ImuMeasurement(timestamp, ImuAccGyr::Zero()));
  }

  void addFrame(Timestamp timestamp) {
    Frame::UniquePtr lframe =
        VIO::make_unique<Frame>(last_id_, timestamp, CameraParams(), cv::Mat());
    test_provider_->fillLeftFrameQueue(std::move(lframe));
    last_id_++;
  }

  void waitForNonEmptyQueue(int timeout = 1000, bool empty_okay = false) {
    std::promise<bool> finish_promise;
    std::future<bool> finish_future = finish_promise.get_future();
    ASSERT_TRUE(finish_future.valid());
    empty_queue_thread_.reset(new std::thread(
        [this](std::promise<bool>&& finished) {
          while (output_queue_.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          finished.set_value(true);
        },
        std::move(finish_promise)));
    ASSERT_TRUE(finish_future.valid());
    auto wait_result =
        finish_future.wait_for(std::chrono::milliseconds(timeout));
    if (!empty_okay) {
      ASSERT_TRUE(wait_result != std::future_status::timeout);
      ASSERT_FALSE(output_queue_.empty());
    }
  }

  void startSpin() {
    std::promise<bool> finish_promise;
    finish_future_ = finish_promise.get_future();
    ASSERT_TRUE(finish_future_.valid());
    spin_thread_.reset(new std::thread(
        [this](std::promise<bool>&& finished) {
          test_provider_->spin();
          finished.set_value(true);
        },
        std::move(finish_promise)));
  }

  void stopSpin(int timeout = 1000) {
    output_queue_.shutdown();
    test_provider_->shutdown();
    ASSERT_TRUE(finish_future_.valid());
    auto waitResult =
        finish_future_.wait_for(std::chrono::milliseconds(timeout));
    CHECK(waitResult != std::future_status::timeout) << "Thread is blocked!";
    spin_thread_->join();
    empty_queue_thread_->join();
  }
};  // namespace VIO

TEST_F(TestParallelMonoProvider, basicParallelCase) {
  // First frame is needed for benchmarking
  addImu(10);
  addFrame(11);
  // this is pretty opaque; there's no guarantee that the
  // output queue will be empty sometime after this is called
  EXPECT_TRUE(output_queue_.empty());

  // add some measurements
  addImu(12);
  addImu(13);
  addImu(14);
  addFrame(17);
  addImu(18);  // trigger packet creation

  waitForNonEmptyQueue();

  FrontendInputPacketBase::UniquePtr result_base;
  ASSERT_TRUE(output_queue_.pop(result_base));
  ASSERT_TRUE(result_base != nullptr);
  EXPECT_EQ(17, result_base->timestamp_);
  // +2 because it interpolates to the time frame on both sides
  EXPECT_EQ(5, result_base->imu_stamps_.cols());

  ImuStampS expected_imu_times(1, 4);
  expected_imu_times << 11, 12, 13, 14, 17;
  EXPECT_EQ(expected_imu_times, result_base->imu_stamps_);

  MonoImuSyncPacket::UniquePtr result =
      safeCast<FrontendInputPacketBase, MonoImuSyncPacket>(
          std::move(result_base));
  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(static_cast<FrameId>(1), result->getFrame().id_);
}

TEST_F(TestParallelMonoProvider, dropFramesOlderThanImu) {
  addImu(10);
  addFrame(11);
  EXPECT_TRUE(output_queue_.empty());

  // add gap in IMU data
  addImu(16);

  // add frames that won't get processed
  for (Timestamp t = 12; t < 16; ++t) {
    addFrame(t);
    EXPECT_TRUE(output_queue_.empty());
  }

  // now, a valid frame
  addFrame(17);
  addImu(18);

  waitForNonEmptyQueue();

  FrontendInputPacketBase::UniquePtr result;
  ASSERT_TRUE(output_queue_.pop(result));
  ASSERT_TRUE(result != nullptr);

  EXPECT_EQ(17, result->timestamp_);
  EXPECT_EQ(3, result->imu_stamps_.cols());

  ImuStampS expected_imu_times(1, 2);
  expected_imu_times << 11, 16, 17;
  EXPECT_EQ(expected_imu_times, result->imu_stamps_);
}

TEST_F(TestParallelMonoProvider, imageBeforeImuTest) {
  // Drop any frame that appears before the IMU packets do
  addFrame(10);
  EXPECT_TRUE(output_queue_.empty());

  // Initial frame
  addImu(11);
  addFrame(12);
  EXPECT_TRUE(output_queue_.empty());

  // Valid frame
  addImu(13);
  addFrame(14);
  addImu(15);

  waitForNonEmptyQueue();

  FrontendInputPacketBase::UniquePtr result;
  ASSERT_TRUE(output_queue_.pop(result));
  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(14, result->timestamp_);
  EXPECT_EQ(3, result->imu_stamps_.cols());
  EXPECT_EQ(12, result->imu_stamps_(0, 0));
  EXPECT_EQ(13, result->imu_stamps_(0, 1));
}

TEST_F(TestParallelMonoProvider, DISABLED_testPartialImuSequence) {
  addImu(0);  // Get past the need for available IMU data
  addFrame(1);

  // opaque / unsound check
  EXPECT_TRUE(output_queue_.empty());

  addImu(2);
  addImu(3);
  addImu(4);
  addFrame(5);

  // sleep and wait for queue to be non-empty
  // this is also a mostly opaque check / not sound
  waitForNonEmptyQueue(10, true);
  EXPECT_TRUE(output_queue_.empty());

  addImu(5);  // add missing imu measurement

  waitForNonEmptyQueue();

  FrontendInputPacketBase::UniquePtr output;
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);

  EXPECT_EQ(5, output->imu_stamps_.cols());
  EXPECT_EQ(5, output->imu_accgyrs_.cols());
  EXPECT_EQ(1, output->imu_stamps_(0, 0));
  EXPECT_EQ(2, output->imu_stamps_(0, 1));
  EXPECT_EQ(3, output->imu_stamps_(0, 2));
  EXPECT_EQ(4, output->imu_stamps_(0, 3));
}

TEST_F(TestParallelMonoProvider, testOutOfOrderImuSequence) {
  addImu(0);  // Get past the need for available IMU data
  addFrame(1);
  addImu(2);
  addImu(4);
  addImu(3);  // should be dropped
  addImu(5);
  addImu(5);  // should be dropped
  addImu(5);  // should be dropped
  addImu(5);  // should be dropped
  addFrame(5);

  waitForNonEmptyQueue();

  FrontendInputPacketBase::UniquePtr output;
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);

  EXPECT_EQ(5, output->timestamp_);
  EXPECT_EQ(4, output->imu_stamps_.cols());
  EXPECT_EQ(4, output->imu_accgyrs_.cols());
  EXPECT_EQ(1, output->imu_stamps_(0, 0));
  EXPECT_EQ(2, output->imu_stamps_(0, 1));
  EXPECT_EQ(4, output->imu_stamps_(0, 2));
}

TEST_F(TestParallelMonoProvider, testOutOfOrderImageSequence) {
  addImu(0);  // Get past the need for available IMU data
  addFrame(3);
  addImu(2);
  addImu(3);
  addImu(4);
  addImu(5);
  addFrame(2);  // should be dropped
  addFrame(5);

  waitForNonEmptyQueue();

  FrontendInputPacketBase::UniquePtr output;
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);
  EXPECT_EQ(5, output->timestamp_);
  EXPECT_EQ(3, output->imu_stamps_.cols());
  EXPECT_EQ(3, output->imu_accgyrs_.cols());
  EXPECT_EQ(3, output->imu_stamps_(0, 0));
  EXPECT_EQ(4, output->imu_stamps_(0, 1));
  EXPECT_EQ(5, output->imu_stamps_(0, 2));
}

TEST_F(TestParallelMonoProvider, testOutOfOrderImuAndImageSequence) {
  addImu(0);  // Get past the need for available IMU data
  addFrame(3);
  addImu(2);
  addImu(4);
  addImu(3);  // should be dropped
  addImu(5);
  addImu(5);    // should be dropped
  addImu(5);    // should be dropped
  addFrame(2);  // should be dropped
  addImu(5);    // should be dropped
  addImu(5);    // should be dropped
  addFrame(5);

  waitForNonEmptyQueue();

  FrontendInputPacketBase::UniquePtr output;
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);
  EXPECT_EQ(5, output->timestamp_);
  EXPECT_EQ(3, output->imu_stamps_.cols());
  EXPECT_EQ(3, output->imu_accgyrs_.cols());
  EXPECT_EQ(3, output->imu_stamps_(0, 0));
  EXPECT_EQ(4, output->imu_stamps_(0, 1));
}

TEST_F(TestParallelMonoProvider, monoPipelineWithCoarseCorrection) {
  // this isn't threadsafe, but it doesn't matter
  test_provider_->doCoarseImuCameraTemporalSync();

  addImu(10);  // Get past the need for available IMU data
  addFrame(1);
  addImu(11);
  addImu(12);
  addImu(13);
  addFrame(3);

  waitForNonEmptyQueue();

  FrontendInputPacketBase::UniquePtr output;
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);
  EXPECT_EQ(3, output->timestamp_);
  EXPECT_EQ(3, output->imu_stamps_.cols());
  EXPECT_EQ(3, output->imu_accgyrs_.cols());
  EXPECT_EQ(1, output->imu_stamps_(0, 0));
  EXPECT_EQ(2, output->imu_stamps_(0, 1));
  EXPECT_EQ(3, output->imu_stamps_(0, 2));
}

TEST_F(TestParallelMonoProvider, monoPipelineManualTimeShift) {
  test_provider_->setImuTimeShift(10.0e-9);

  addImu(10);  // Get past the need for available IMU data
  addFrame(1);
  addImu(11);
  addImu(12);
  addImu(13);
  addFrame(3);

  waitForNonEmptyQueue();

  FrontendInputPacketBase::UniquePtr output;
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);
  EXPECT_EQ(3, output->timestamp_);
  EXPECT_EQ(3, output->imu_stamps_.cols());
  EXPECT_EQ(3, output->imu_accgyrs_.cols());
  EXPECT_EQ(1, output->imu_stamps_(0, 0));
  EXPECT_EQ(2, output->imu_stamps_(0, 1));
  EXPECT_EQ(3, output->imu_stamps_(0, 2));
}

}  // namespace VIO
