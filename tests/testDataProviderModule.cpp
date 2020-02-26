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

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

//#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd.h"
#include "kimera-vio/frontend/VisionFrontEndModule.h"
#include "kimera-vio/dataprovider/DataProviderModule.h"

//#include "kimera-vio/frontend/Tracker-definitions.h"
//#include "kimera-vio/frontend/Tracker.h"

DECLARE_string(test_data_path);

//using namespace gtsam;
using namespace std;
using namespace VIO;
//using namespace cv;

/* ************************************************************************** */
// Testing data
static const double tol = 1e-7;
// TODO use test fixture!!
static StereoVisionFrontEndModule::InputQueue output_queue("test_output");

class TestDataProviderModule : public ::testing::Test {
 public:
  TestDataProviderModule() {  }

 protected:
  DataProviderModule::UniquePtr data_provider_module_;
  
  void SetUp() override {
    while(!output_queue.empty()) { output_queue.pop(); } // clear the output queue

    //! Create DataProvider
    StereoVisionFrontEndModule::InputQueue dummy_queue("unused");
    StereoMatchingParams dummy_params;
    data_provider_module_ = VIO::make_unique<DataProviderModule>(
        &dummy_queue,
        "Data Provider",
        true,
        dummy_params);
    data_provider_module_->registerVioPipelineCallback(
        std::bind(SaveOutput, std::placeholders::_1));
  }

  // TODO deallocate dynamic memory here!
  void TearDown() override {
    data_provider_module_->shutdown();
  }

  static void SaveOutput(StereoImuSyncPacket::UniquePtr sync_packet) {
    output_queue.push(std::move(sync_packet));
  }

  Frame::UniquePtr MakeDummyFrame(const FrameId& frame_id, const Timestamp& timestamp)
  {
    return VIO::make_unique<Frame>(
                  frame_id,
                  timestamp,
                  CameraParams(),
                  cv::Mat());
  }
};

/* ************************************************************************* */
TEST_F(TestDataProviderModule, parallelStereoImuSyncPacket) {
  FrameId current_id = 0;
  Timestamp current_time = 10;
  data_provider_module_->fillLeftFrameQueue(MakeDummyFrame(current_id++, ++current_time));
  data_provider_module_->fillRightFrameQueue(MakeDummyFrame(current_id++, current_time));
  
  int num_imu_to_make = 4;
  for(int i = 0; i < num_imu_to_make; i++)
  {
    data_provider_module_->fillImuQueue(ImuMeasurement(++current_time, ImuAccGyr()));
  }

  data_provider_module_->fillLeftFrameQueue(MakeDummyFrame(current_id++, ++current_time));
  data_provider_module_->fillRightFrameQueue(MakeDummyFrame(current_id++, current_time));

  // Leave some time for p to finish its work.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  StereoImuSyncPacket::UniquePtr result; 
  CHECK(output_queue.popBlocking(result));
  CHECK(result);
  EXPECT_EQ(current_time, result->timestamp_);
  EXPECT_EQ((FrameId)3, result->getStereoFrame().getFrameId());
  EXPECT_EQ(num_imu_to_make, result->getImuStamps().size());
}

