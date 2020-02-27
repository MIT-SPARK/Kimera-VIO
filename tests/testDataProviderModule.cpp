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

#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd.h"
#include "kimera-vio/frontend/VisionFrontEndModule.h"
#include "kimera-vio/dataprovider/DataProviderModule.h"

DECLARE_string(test_data_path);

/* ************************************************************************** */
// Testing data
static const double tol = 1e-7;
static VIO::StereoVisionFrontEndModule::InputQueue output_queue("test_output");

class TestDataProviderModule : public ::testing::Test {
 public:
  TestDataProviderModule() {  }

 protected:
  VIO::DataProviderModule::UniquePtr data_provider_module_;

  void SetUp() override {
    // Clear the output queue
    while (!output_queue.empty()) {
      output_queue.pop();
    }

    // Create DataProvider
    VIO::StereoVisionFrontEndModule::InputQueue dummy_queue("unused");
    VIO::StereoMatchingParams dummy_params;
    bool parallel = false;
    data_provider_module_ = VIO::make_unique<VIO::DataProviderModule>(
        &dummy_queue, "Data Provider", parallel, dummy_params);
    data_provider_module_->registerVioPipelineCallback(
        std::bind(SaveOutput, std::placeholders::_1));
  }

  void TearDown() override {
    data_provider_module_->shutdown();
  }

  static void SaveOutput(VIO::StereoImuSyncPacket::UniquePtr sync_packet) {
    output_queue.push(std::move(sync_packet));
  }

  VIO::Frame::UniquePtr MakeDummyFrame(const VIO::FrameId& frame_id,
                                       const VIO::Timestamp& timestamp) {
    return VIO::make_unique<VIO::Frame>(
        frame_id, timestamp, VIO::CameraParams(), cv::Mat());
  }
};

/* ************************************************************************* */
TEST_F(TestDataProviderModule, basicSequentialCase) {
  VIO::FrameId current_id = 0;
  VIO::Timestamp current_time = 10;
  data_provider_module_->fillLeftFrameQueue(
      MakeDummyFrame(current_id++, ++current_time));
  data_provider_module_->fillRightFrameQueue(
      MakeDummyFrame(current_id++, current_time));
  data_provider_module_->spin();

  size_t num_imu_to_make = 4;
  for (size_t i = 0; i < num_imu_to_make; i++) {
    data_provider_module_->fillImuQueue(
        VIO::ImuMeasurement(++current_time, VIO::ImuAccGyr()));
  }

  data_provider_module_->fillLeftFrameQueue(
      MakeDummyFrame(current_id++, ++current_time));
  data_provider_module_->fillRightFrameQueue(
      MakeDummyFrame(current_id++, current_time));
  data_provider_module_->spin();

  // Leave some time for data_provider_module_ to finish its work.
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));

  VIO::StereoImuSyncPacket::UniquePtr result;
  CHECK(output_queue.popBlocking(result));
  CHECK(result);
  EXPECT_EQ(current_time, result->timestamp_);
  EXPECT_EQ(static_cast<VIO::FrameId>(3),
            result->getStereoFrame().getFrameId());
  EXPECT_EQ(num_imu_to_make, result->getImuStamps().size());
}

