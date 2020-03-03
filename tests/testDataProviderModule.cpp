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
static constexpr int default_timeout = 10000;

/* ************************************************************************** */
// Testing data
static const double tol = 1e-7;
static VIO::StereoVisionFrontEndModule::InputQueue output_queue("test_output");

class TestDataProviderModule : public ::testing::Test {
 public:
  TestDataProviderModule() {  }

 protected:
  VIO::StereoVisionFrontEndModule::InputQueue* dummy_queue;
  VIO::DataProviderModule::UniquePtr data_provider_module_;

  void SetUp() override {
    // Set google flags to assume image is already rectified--makes dummy params
    // easier
    int fake_argc = 2;
    char** fake_argv =
        reinterpret_cast<char**>(malloc(sizeof(char*) * fake_argc));
    fake_argv[0] = "foo";
    fake_argv[1] = "--images_rectified";
    google::ParseCommandLineFlags(&fake_argc, &fake_argv, true);

    // Create DataProvider
    dummy_queue = new VIO::StereoVisionFrontEndModule::InputQueue("unused");
    VIO::StereoMatchingParams dummy_params;
    bool parallel = false;
    data_provider_module_ = VIO::make_unique<VIO::DataProviderModule>(
        dummy_queue, "Data Provider", parallel, dummy_params);
    data_provider_module_->registerVioPipelineCallback(
        std::bind(SaveOutput, std::placeholders::_1));
  }

  void TearDown() override {
    data_provider_module_->shutdown();
    delete dummy_queue;
    // Clear the output queue
    while (!output_queue.empty()) {
      output_queue.pop();
    }
  }

  static void SaveOutput(VIO::StereoImuSyncPacket::UniquePtr sync_packet) {
    output_queue.push(std::move(sync_packet));
  }

  VIO::Frame::UniquePtr MakeDummyFrame(const VIO::FrameId& frame_id,
                                       const VIO::Timestamp& timestamp) {
    return VIO::make_unique<VIO::Frame>(
        frame_id, timestamp, MakeDumyCameraParams(), cv::Mat());
  }

  VIO::CameraParams MakeDumyCameraParams() {
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

  // Fencepost frames with IMU data
  // data_provider_module_->fillImuQueue(
  //    VIO::ImuMeasurement(++current_time, VIO::ImuAccGyr::Zero()));

  data_provider_module_->fillLeftFrameQueue(
      MakeDummyFrame(++current_id, ++current_time));
  data_provider_module_->fillRightFrameQueue(
      MakeDummyFrame(current_id, current_time));
  data_provider_module_->spin();

  size_t num_imu_to_make = 4;
  for (size_t i = 0; i < num_imu_to_make; i++) {
    data_provider_module_->fillImuQueue(
        VIO::ImuMeasurement(++current_time, VIO::ImuAccGyr::Zero()));
  }

  data_provider_module_->fillLeftFrameQueue(
      MakeDummyFrame(++current_id, ++current_time));
  data_provider_module_->fillRightFrameQueue(
      MakeDummyFrame(current_id, current_time));
  // IMU and camera streams are not necessarily in sync
  // Send an IMU packet after camera packets to signal no more IMU incoming
  data_provider_module_->fillImuQueue(
      VIO::ImuMeasurement(++current_time, VIO::ImuAccGyr::Zero()));
  data_provider_module_->spin();

  VIO::StereoImuSyncPacket::UniquePtr result;
  CHECK(output_queue.popBlocking(result));
  CHECK(result);
  EXPECT_EQ(current_time - 1, result->timestamp_);
  EXPECT_EQ(static_cast<VIO::FrameId>(2),
            result->getStereoFrame().getFrameId());
  EXPECT_EQ(num_imu_to_make, result->getImuStamps().size());

  TEST_TIMEOUT_FAIL_END(default_timeout)
}

