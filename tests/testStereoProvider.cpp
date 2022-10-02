/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   tstStereoProvider.cpp
 * @brief  test the StereoDataProvider, especially around timestamp invariants
 * @author Andrew Violette
 * @author Luca Carlone
 * @author Nathan Hughes
 */

#include <future>
#include <limits>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/dataprovider/StereoDataProviderModule.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/FrontendInputPacketBase.h"
#include "kimera-vio/frontend/StereoFrame.h"

namespace VIO {

class TestStereoProvider : public ::testing::Test {
 public:
  TestStereoProvider() : last_id_(0), output_queue_("test_output") {
    test_provider_ = VIO::make_unique<StereoDataProviderModule>(
        &output_queue_, "test_stereo_provider", false, StereoMatchingParams());
    test_provider_->registerVioPipelineCallback(
        [this](FrontendInputPacketBase::UniquePtr packet) {
          output_queue_.push(std::move(packet));
        });
  }

  ~TestStereoProvider() = default;

 protected:
  DataProviderModule::OutputQueue output_queue_;
  StereoDataProviderModule::UniquePtr test_provider_;
  FrameId last_id_;

  void addImu(Timestamp timestamp) {
    test_provider_->fillImuQueue(ImuMeasurement(timestamp, ImuAccGyr::Zero()));
  }

  void addFrame(Timestamp timestamp) {
    Frame::UniquePtr lframe =
        VIO::make_unique<Frame>(last_id_, timestamp, CameraParams(), cv::Mat());
    test_provider_->fillLeftFrameQueue(std::move(lframe));
    Frame::UniquePtr rframe =
        VIO::make_unique<Frame>(last_id_, timestamp, CameraParams(), cv::Mat());
    test_provider_->fillRightFrameQueue(std::move(rframe));
    last_id_++;
  }

  void addLeftFrame(Timestamp timestamp, bool increment = false) {
    Frame::UniquePtr lframe =
        VIO::make_unique<Frame>(last_id_, timestamp, CameraParams(), cv::Mat());
    test_provider_->fillLeftFrameQueue(std::move(lframe));
    if (increment) {
      last_id_++;
    }
  }

  void addRightFrame(Timestamp timestamp, bool increment = false) {
    Frame::UniquePtr rframe =
        VIO::make_unique<Frame>(last_id_, timestamp, CameraParams(), cv::Mat());
    test_provider_->fillRightFrameQueue(std::move(rframe));
    if (increment) {
      last_id_++;
    }
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

TEST_F(TestStereoProvider, basicSequentialCase) {
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
  ASSERT_TRUE(result_base != nullptr);
  EXPECT_EQ(17, result_base->timestamp_);
  // +2 because it interpolates to the time frame on both sides
  EXPECT_EQ(5, result_base->imu_stamps_.cols());

  ImuStampS expected_imu_times(1, 4);
  expected_imu_times << 11, 12, 13, 14, 17;
  EXPECT_EQ(expected_imu_times, result_base->imu_stamps_);

  StereoImuSyncPacket::UniquePtr result =
      safeCast<FrontendInputPacketBase, StereoImuSyncPacket>(
          std::move(result_base));
  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(static_cast<FrameId>(1), result->getStereoFrame().id_);
}

TEST_F(TestStereoProvider, dropFramesOlderThanImu) {
  addImu(10);
  addFrame(11);
  spinWithTimeout();
  EXPECT_TRUE(output_queue_.empty());

  // add gap in IMU data
  addImu(16);

  // add frames that won't get processed
  for (Timestamp t = 12; t < 16; ++t) {
    addFrame(t);
    spinWithTimeout();
    EXPECT_TRUE(output_queue_.empty());
  }

  // now, a valid frame
  addFrame(17);
  addImu(18);
  spinWithTimeout();

  FrontendInputPacketBase::UniquePtr result;
  ASSERT_TRUE(output_queue_.pop(result));
  ASSERT_TRUE(result != nullptr);

  // you could test the frame id here, but the timestamp
  // is a good proxy for the frame id
  EXPECT_EQ(17, result->timestamp_);
  EXPECT_EQ(3, result->imu_stamps_.cols());

  ImuStampS expected_imu_times(1, 2);
  expected_imu_times << 11, 16, 17;
  EXPECT_EQ(expected_imu_times, result->imu_stamps_);
}

TEST_F(TestStereoProvider, manyImuTest) {
  Timestamp t_curr = 10;
  Timestamp num_imu = 5;
  for (Timestamp t = 0; t < num_imu; ++t) {
    addImu(t_curr + t);
  }
  t_curr += num_imu;
  addFrame(t_curr);

  size_t num_valid_frames = 10;
  for (size_t i = 0; i < num_valid_frames; i++) {
    for (Timestamp t = 0; t < num_imu; ++t) {
      addImu(t_curr + t);
    }
    t_curr += num_imu;
    addFrame(t_curr);
  }
  addImu(t_curr + 1);

  spinWithTimeout();
  EXPECT_TRUE(output_queue_.empty());

  for (size_t i = 0; i < num_valid_frames; i++) {
    spinWithTimeout();
    FrontendInputPacketBase::UniquePtr result;
    ASSERT_TRUE(output_queue_.pop(result));
    ASSERT_TRUE(result != nullptr);

    // first frame at 15, second frame valid at 20
    EXPECT_EQ(20 + i * num_imu, result->timestamp_);
    // +1 because it interpolates to the time frame
    EXPECT_EQ(num_imu + 1, result->imu_stamps_.cols());
  }
}

TEST_F(TestStereoProvider, imageBeforeImuTest) {
  // Drop any frame that appears before the IMU packets do
  addFrame(10);
  spinWithTimeout();
  EXPECT_TRUE(output_queue_.empty());

  // Initial frame
  addImu(11);
  addFrame(12);
  spinWithTimeout();
  EXPECT_TRUE(output_queue_.empty());

  // Valid frame
  addImu(13);
  addFrame(14);
  addImu(15);
  spinWithTimeout();

  FrontendInputPacketBase::UniquePtr result;
  ASSERT_TRUE(output_queue_.pop(result));
  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(14, result->timestamp_);
  EXPECT_EQ(3, result->imu_stamps_.cols());
  EXPECT_EQ(12, result->imu_stamps_(0, 0));
  EXPECT_EQ(13, result->imu_stamps_(0, 1));
}

TEST_F(TestStereoProvider, imageBeforeImuDelayedSpinTest) {
  // Drop any frame that appears before the IMU packets do
  addFrame(10);

  // Initial frame
  addImu(11);
  addFrame(12);

  // Valid frame
  addImu(13);
  addFrame(14);
  addImu(15);

  spinWithTimeout();  // Reject first frame
  EXPECT_TRUE(output_queue_.empty());
  spinWithTimeout();  // Second frame is initial frame
  EXPECT_TRUE(output_queue_.empty());

  spinWithTimeout();  // Third frame should work
  FrontendInputPacketBase::UniquePtr result;
  ASSERT_TRUE(output_queue_.pop(result));
  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(14, result->timestamp_);
  EXPECT_EQ(3, result->imu_stamps_.cols());
  EXPECT_EQ(12, result->imu_stamps_(0, 0));
  EXPECT_EQ(13, result->imu_stamps_(0, 1));
}

TEST_F(TestStereoProvider, stereoPipelineValidImuSequence) {
  addImu(0);
  addFrame(1);

  spinWithTimeout();

  FrontendInputPacketBase::UniquePtr output;
  EXPECT_FALSE(output_queue_.pop(output));

  addImu(2);
  addImu(3);
  addImu(4);
  addFrame(3);

  spinWithTimeout();

  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);
  EXPECT_EQ(3, output->timestamp_);
  EXPECT_EQ(3, output->imu_stamps_.cols());
  EXPECT_EQ(3, output->imu_accgyrs_.cols());
}

TEST_F(TestStereoProvider, stereoPipelineInvalidImuSequence) {
  addImu(10);  // Get past the need for available IMU data
  addFrame(1);

  spinWithTimeout();

  FrontendInputPacketBase::UniquePtr output;
  EXPECT_FALSE(output_queue_.pop(output));

  addImu(11);
  addImu(12);
  addImu(13);
  addFrame(3);

  spinWithTimeout();
  EXPECT_FALSE(output_queue_.pop(output));
}

TEST_F(TestStereoProvider, testPartialImuSequence) {
  addImu(0);  // Get past the need for available IMU data
  addFrame(1);

  spinWithTimeout();
  FrontendInputPacketBase::UniquePtr output;
  EXPECT_FALSE(output_queue_.pop(output));

  addImu(2);
  addImu(3);
  addImu(4);
  addFrame(5);

  spinWithTimeout();  // earlier versions would loop forever here
  EXPECT_FALSE(output_queue_.pop(output));

  addImu(5);  // add missing imu measurement

  spinWithTimeout();
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);

  EXPECT_EQ(5, output->imu_stamps_.cols());
  EXPECT_EQ(5, output->imu_accgyrs_.cols());
  EXPECT_EQ(1, output->imu_stamps_(0, 0));
  EXPECT_EQ(2, output->imu_stamps_(0, 1));
  EXPECT_EQ(3, output->imu_stamps_(0, 2));
  EXPECT_EQ(4, output->imu_stamps_(0, 3));
}

TEST_F(TestStereoProvider, testOutOfOrderImuSequence) {
  addImu(0);  // Get past the need for available IMU data
  addFrame(1);

  spinWithTimeout();
  FrontendInputPacketBase::UniquePtr output;
  EXPECT_FALSE(output_queue_.pop(output));

  addImu(2);
  addImu(4);
  addImu(3);  // should be dropped
  addImu(5);
  addImu(5);  // should be dropped
  addImu(5);  // should be dropped
  addImu(5);  // should be dropped
  addFrame(5);

  spinWithTimeout();
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);

  EXPECT_EQ(5, output->timestamp_);
  EXPECT_EQ(4, output->imu_stamps_.cols());
  EXPECT_EQ(4, output->imu_accgyrs_.cols());
  EXPECT_EQ(1, output->imu_stamps_(0, 0));
  EXPECT_EQ(2, output->imu_stamps_(0, 1));
  EXPECT_EQ(4, output->imu_stamps_(0, 2));
}

TEST_F(TestStereoProvider, testOutOfOrderImageSequence) {
  addImu(0);  // Get past the need for available IMU data
  addFrame(3);

  spinWithTimeout();
  FrontendInputPacketBase::UniquePtr output;
  EXPECT_FALSE(output_queue_.pop(output));

  addImu(2);
  addImu(3);
  addImu(4);
  addImu(5);
  addFrame(2);  // should be dropped
  addFrame(5);

  spinWithTimeout();  // dropped out-of-order frame
  ASSERT_FALSE(output_queue_.pop(output));
  spinWithTimeout();
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);

  EXPECT_EQ(5, output->timestamp_);
  EXPECT_EQ(3, output->imu_stamps_.cols());
  EXPECT_EQ(3, output->imu_accgyrs_.cols());
  EXPECT_EQ(3, output->imu_stamps_(0, 0));
  EXPECT_EQ(4, output->imu_stamps_(0, 1));
  EXPECT_EQ(5, output->imu_stamps_(0, 2));
}

TEST_F(TestStereoProvider, testOutOfOrderManyImageSequence) {
  addImu(0);  // Get past the need for available IMU data
  addFrame(3);

  spinWithTimeout();
  FrontendInputPacketBase::UniquePtr output;
  EXPECT_FALSE(output_queue_.pop(output));

  addImu(2);
  addImu(3);
  addImu(4);
  addImu(5);
  addFrame(2);  // should be dropped
  addFrame(5);
  addImu(6);
  addFrame(7);
  addImu(8);
  addFrame(9);
  addImu(10);

  spinWithTimeout();  // dropped out-of-order frame
  ASSERT_FALSE(output_queue_.pop(output));
  spinWithTimeout();
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);

  EXPECT_EQ(5, output->timestamp_);
  EXPECT_EQ(3, output->imu_stamps_.cols());
  EXPECT_EQ(3, output->imu_accgyrs_.cols());
  EXPECT_EQ(3, output->imu_stamps_(0, 0));
  EXPECT_EQ(4, output->imu_stamps_(0, 1));
  EXPECT_EQ(5, output->imu_stamps_(0, 2));

  spinWithTimeout();
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);
  EXPECT_EQ(7, output->timestamp_);
  EXPECT_EQ(3, output->imu_stamps_.cols());
  EXPECT_EQ(3, output->imu_accgyrs_.cols());
  EXPECT_EQ(5, output->imu_stamps_(0, 0));
  EXPECT_EQ(6, output->imu_stamps_(0, 1));
  EXPECT_EQ(7, output->imu_stamps_(0, 2));

  spinWithTimeout();
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);
  EXPECT_EQ(9, output->timestamp_);
  EXPECT_EQ(3, output->imu_stamps_.cols());
  EXPECT_EQ(3, output->imu_accgyrs_.cols());
  EXPECT_EQ(7, output->imu_stamps_(0, 0));
  EXPECT_EQ(8, output->imu_stamps_(0, 1));
}

TEST_F(TestStereoProvider, testOutOfOrderImuAndImageSequence) {
  addImu(0);  // Get past the need for available IMU data
  addFrame(3);

  spinWithTimeout();
  FrontendInputPacketBase::UniquePtr output;
  EXPECT_FALSE(output_queue_.pop(output));

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

  spinWithTimeout();  // dropped out-of-order frame
  ASSERT_FALSE(output_queue_.pop(output));
  spinWithTimeout();
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);

  EXPECT_EQ(5, output->timestamp_);
  EXPECT_EQ(3, output->imu_stamps_.cols());
  EXPECT_EQ(3, output->imu_accgyrs_.cols());
  EXPECT_EQ(3, output->imu_stamps_(0, 0));
  EXPECT_EQ(4, output->imu_stamps_(0, 1));
}

TEST_F(TestStereoProvider, stereoPipelineWithCoarseCorrection) {
  test_provider_->doCoarseImuCameraTemporalSync();

  addImu(10);  // Get past the need for available IMU data
  addFrame(1);

  spinWithTimeout();

  FrontendInputPacketBase::UniquePtr output;
  EXPECT_FALSE(output_queue_.pop(output));

  addImu(11);
  addImu(12);
  addImu(13);
  addFrame(3);

  spinWithTimeout();
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);

  EXPECT_EQ(3, output->timestamp_);
  EXPECT_EQ(3, output->imu_stamps_.cols());
  EXPECT_EQ(3, output->imu_accgyrs_.cols());
  EXPECT_EQ(1, output->imu_stamps_(0, 0));
  EXPECT_EQ(2, output->imu_stamps_(0, 1));
  EXPECT_EQ(3, output->imu_stamps_(0, 2));
}

TEST_F(TestStereoProvider, stereoPipelineManualTimeShift) {
  test_provider_->setImuTimeShift(10.0e-9);

  addImu(10);  // Get past the need for available IMU data
  addFrame(1);

  spinWithTimeout();

  FrontendInputPacketBase::UniquePtr output;
  EXPECT_FALSE(output_queue_.pop(output));

  addImu(11);
  addImu(12);
  addImu(13);
  addFrame(3);

  spinWithTimeout();
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);

  EXPECT_EQ(3, output->timestamp_);
  EXPECT_EQ(3, output->imu_stamps_.cols());
  EXPECT_EQ(3, output->imu_accgyrs_.cols());
  EXPECT_EQ(1, output->imu_stamps_(0, 0));
  EXPECT_EQ(2, output->imu_stamps_(0, 1));
  EXPECT_EQ(3, output->imu_stamps_(0, 2));
}

TEST_F(TestStereoProvider, dropRightFrame) {
  addImu(0);  // Get past the need for available IMU data
  addFrame(1);

  spinWithTimeout();

  FrontendInputPacketBase::UniquePtr output;
  EXPECT_FALSE(output_queue_.pop(output));

  addImu(2);
  addImu(3);
  addImu(4);
  addLeftFrame(5);
  addImu(6);
  addImu(7);
  addFrame(8);
  addImu(9);

  spinWithTimeout();
  EXPECT_FALSE(output_queue_.pop(output));
  spinWithTimeout();
  ASSERT_TRUE(output_queue_.pop(output));
  ASSERT_TRUE(output != nullptr);

  EXPECT_EQ(8, output->timestamp_);
  EXPECT_EQ(7, output->imu_stamps_.cols());
  EXPECT_EQ(7, output->imu_accgyrs_.cols());
  EXPECT_EQ(1, output->imu_stamps_(0, 0));
  EXPECT_EQ(2, output->imu_stamps_(0, 1));
  EXPECT_EQ(3, output->imu_stamps_(0, 2));
  EXPECT_EQ(4, output->imu_stamps_(0, 3));
  EXPECT_EQ(6, output->imu_stamps_(0, 4));
  EXPECT_EQ(7, output->imu_stamps_(0, 5));
}

}  // namespace VIO
