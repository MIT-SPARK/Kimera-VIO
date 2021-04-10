/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testTemporalCalibration.cpp
 * @brief  Unit tests to ensure pipeline correctness for temporal calibration
 * @author Nathan Hughes
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/dataprovider/DataProviderModule.h"
#include "kimera-vio/dataprovider/MonoDataProviderModule.h"
#include "kimera-vio/dataprovider/StereoDataProviderModule.h"

namespace VIO {

class TestMonoProvider : public MonoDataProviderModule {
 public:
  TestMonoProvider(DataProviderModule::OutputQueue* output_queue)
      : MonoDataProviderModule(output_queue, "test_mono_data_prov", false) {
    last_id_ = 0;
  }

  void addFakeFrames(Timestamp timestamp) {
    Frame::UniquePtr frame =
        make_unique<Frame>(++last_id_, timestamp, CameraParams(), cv::Mat());
    fillLeftFrameQueue(std::move(frame));
  }

  void addImu(ImuStamp timestamp) {
    fillImuQueue(ImuMeasurement(timestamp, ImuAccGyr::Zero()));
  }

  void addImu(ImuStamp timestamp, const ImuGyr& omega, const ImuAcc& accel) {
    ImuAccGyr combined;
    combined << accel, omega;
    fillImuQueue(ImuMeasurement(timestamp, combined));
  }

 private:
  FrameId last_id_;
};

TEST(testTemporalCalibration, MonoPipelineValidImuSequence) {
  DataProviderModule::OutputQueue output_queue("test_queue");
  TestMonoProvider module(&output_queue);
  module.registerVioPipelineCallback(
      [&](FrontendInputPacketBase::UniquePtr input) {
        output_queue.push(std::move(input));
      });

  // for whatever reason, the data provider needs IMU data to start
  module.addImu(0);
  module.addFakeFrames(1);

  module.spin();

  FrontendInputPacketBase::UniquePtr output;
  EXPECT_FALSE(output_queue.pop(output));

  module.addImu(2);
  module.addImu(3);
  module.addImu(4);
  module.addFakeFrames(3);

  module.spin();

  ASSERT_TRUE(output_queue.pop(output));
  EXPECT_EQ(output->imu_stamps_.cols(), 2);
  EXPECT_EQ(output->imu_accgyrs_.cols(), 2);
}

TEST(testTemporalCalibration, MonoPipelineInvalidImuSequence) {
  DataProviderModule::OutputQueue output_queue("test_queue");
  TestMonoProvider module(&output_queue);
  module.registerVioPipelineCallback(
      [&](FrontendInputPacketBase::UniquePtr input) {
        output_queue.push(std::move(input));
      });

  // Get past the need for available IMU data
  module.addImu(10);
  module.addFakeFrames(1);

  module.spin();

  FrontendInputPacketBase::UniquePtr output;
  EXPECT_FALSE(output_queue.pop(output));

  module.addImu(11);
  module.addImu(12);
  module.addImu(13);
  module.addFakeFrames(3);

  module.spin();

  EXPECT_FALSE(output_queue.pop(output));
}

TEST(testTemporalCalibration, MonoPipelineWithCal) {
  DataProviderModule::OutputQueue output_queue("test_queue");
  TestMonoProvider module(&output_queue);
  module.registerVioPipelineCallback(
      [&](FrontendInputPacketBase::UniquePtr input) {
        output_queue.push(std::move(input));
      });
  module.doCoarseTimestampCorrection();

  // Get past the need for available IMU data
  module.addImu(10);
  module.addFakeFrames(1);

  module.spin();

  FrontendInputPacketBase::UniquePtr output;
  EXPECT_FALSE(output_queue.pop(output));

  module.addImu(11);
  module.addImu(12);
  module.addImu(13);
  module.addFakeFrames(3);

  module.spin();

  ASSERT_TRUE(output_queue.pop(output));

  EXPECT_EQ(output->imu_stamps_.cols(), 3);
  EXPECT_EQ(output->imu_accgyrs_.cols(), 3);
  EXPECT_EQ(output->imu_stamps_(0, 0), 1u);
  EXPECT_EQ(output->imu_stamps_(0, 1), 2u);
  EXPECT_EQ(output->imu_stamps_(0, 2), 3u);
}

}  // namespace VIO
