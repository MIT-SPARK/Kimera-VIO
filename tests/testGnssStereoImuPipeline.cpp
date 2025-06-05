/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testGnssStereoImuPipeline.cpp
 * @brief  test the GnssStereoImuPipeline
 * @author Elizaveta Karaseva
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <future>
#include <memory>
#include <utility>

#include "kimera-vio/dataprovider/GNSSVIODataProvider.h"
#include "kimera-vio/pipeline/GnssStereoImuPipeline.h"

DECLARE_string(test_data_path);
DECLARE_bool(visualize);

namespace VIO {

class GnssVioPipelineFixture : public ::testing::Test {
 public:
  GnssVioPipelineFixture()
      : dataset_parser_(nullptr),
        vio_pipeline_(nullptr),
        vio_params_(
            FLAGS_test_data_path + "/EurocParams",
            FLAGS_test_data_path + "/AlternateTestParams/" +
                VioParams::kPipelineFilename,
            FLAGS_test_data_path + "/AlternateTestParams/" +
                VioParams::kImuFilename,
            FLAGS_test_data_path + "/EurocParams/" +
                VioParams::kLeftCameraFilename,
            FLAGS_test_data_path + "/EurocParams/" +
                VioParams::kRightCameraFilename,
            FLAGS_test_data_path + "/EurocParams/" +
                VioParams::kFrontendFilename,
            FLAGS_test_data_path + "/AlternateTestParams/" +
                VioParams::kBackendFilename,
            FLAGS_test_data_path + "/EurocParams/" + VioParams::kLcdFilename,
            FLAGS_test_data_path + "/EurocParams/" +
                VioParams::kDisplayFilename,
            FLAGS_test_data_path + "/AlternateTestParams/" +
                VioParams::kGnssFilename) {
    FLAGS_visualize = false;
    buildOnlinePipeline(vio_params_);
  }
  ~GnssVioPipelineFixture() override { destroyPipeline(); }

 protected:
  void SetUp() override {}
  void TearDown() override {}

  void buildOnlinePipeline(const VioParams& vio_params) {
    constexpr int initial_k = 10;
    constexpr int final_k = 80;
    // Needed in order to disconnect previous pipeline in case someone calls
    // this function repeatedly within the same test.
    destroyPipeline();
    LOG(INFO) << "Building pipeline.";
    //! Mind that the dataset_parser_ has to be built before the pipeline
    //! because the backend_params are updated with the ground-truth pose
    //! when parsing the dataset.
    dataset_parser_ = std::make_unique<GNSSVIODataProvider>(
        FLAGS_test_data_path + "/MicroEurocDataset",
        initial_k,
        final_k,
        vio_params);
    vio_pipeline_ = std::make_unique<GnssStereoImuPipeline>(vio_params);
    connectGnssVioPipeline();
  }

  void buildOfflinePipeline(const VioParams& vio_params) {
    constexpr int initial_k = 10;
    constexpr int final_k = 80;
    // Needed in order to disconnect previous pipeline in case someone calls
    // this function repeatedly within the same test.
    destroyPipeline();
    LOG(INFO) << "Building pipeline.";
    vio_pipeline_ = std::make_unique<GnssStereoImuPipeline>(vio_params);
    dataset_parser_ = std::make_unique<GNSSVIODataProvider>(
        FLAGS_test_data_path + "/MicroEurocDataset",
        initial_k,
        final_k,
        vio_params);
    connectGnssVioPipelineWithBlockingIfFullQueues();
  }

  void connectGnssVioPipeline() {
    LOG(INFO) << "Connecting pipeline.";
    CHECK(dataset_parser_);
    CHECK(vio_pipeline_);

    // Register callback to shutdown data provider in case VIO pipeline
    // shutsdown.
    vio_pipeline_->registerShutdownCallback(std::bind(
        &VIO::DataProviderInterface::shutdown, dataset_parser_.get()));

    // Register callback to vio pipeline.
    dataset_parser_->registerImuSingleCallback(
        std::bind(&VIO::GnssStereoImuPipeline::fillSingleImuQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    // We use blocking variants to avoid overgrowing the input queues (use
    // the non-blocking versions with real sensor streams)
    dataset_parser_->registerLeftFrameCallback(
        std::bind(&VIO::GnssStereoImuPipeline::fillLeftFrameQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    dataset_parser_->registerRightFrameCallback(
        std::bind(&VIO::GnssStereoImuPipeline::fillRightFrameQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    dataset_parser_->registerGnssCallback(
        std::bind(&VIO::GnssStereoImuPipeline::fillGnssQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
  }

  void connectGnssVioPipelineWithBlockingIfFullQueues() {
    LOG(INFO) << "Connecting pipeline.";
    CHECK(dataset_parser_);
    CHECK(vio_pipeline_);

    // Register callback to shutdown data provider in case VIO pipeline
    // shutsdown.
    vio_pipeline_->registerShutdownCallback(std::bind(
        &VIO::DataProviderInterface::shutdown, dataset_parser_.get()));

    // Register callback to vio pipeline.
    dataset_parser_->registerImuSingleCallback(
        std::bind(&VIO::GnssStereoImuPipeline::fillSingleImuQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    // We use blocking variants to avoid overgrowing the input queues (use
    // the non-blocking versions with real sensor streams)
    dataset_parser_->registerLeftFrameCallback(
        std::bind(&VIO::GnssStereoImuPipeline::fillLeftFrameQueueBlockingIfFull,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    dataset_parser_->registerRightFrameCallback(std::bind(
        &VIO::GnssStereoImuPipeline::fillRightFrameQueueBlockingIfFull,
        vio_pipeline_.get(),
        std::placeholders::_1));
    dataset_parser_->registerGnssCallback(
        std::bind(&VIO::GnssStereoImuPipeline::fillGnssQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
  }

  void destroyPipeline() {
    LOG(INFO) << "Destroying pipeline.";
    // First destroy the VIO pipeline (since this will call the shutdown of
    // the dataset_parser)
    vio_pipeline_.reset();
    // Then destroy the dataset parser.
    dataset_parser_.reset();
  }

 protected:
  DataProviderInterface::UniquePtr dataset_parser_;
  GnssStereoImuPipeline::UniquePtr vio_pipeline_;
  VioParams vio_params_;
};

TEST_F(GnssVioPipelineFixture, OnlineSequentialStart) {
  vio_params_.parallel_run_ = false;
  buildOnlinePipeline(vio_params_);
  ASSERT_TRUE(vio_pipeline_);
  EXPECT_TRUE(vio_pipeline_->spin());
  // If this segfaults, make sure you are deleting first the vio and then the
  // dataset parser.
}

// Online processing, with non-blocking dataprovider queues.
TEST_F(GnssVioPipelineFixture, OnlineSequentialShutdown) {
  vio_params_.parallel_run_ = false;
  buildOnlinePipeline(vio_params_);
  ASSERT_TRUE(vio_pipeline_);
  vio_pipeline_->shutdown();
  EXPECT_FALSE(vio_pipeline_->spin());
}

TEST_F(GnssVioPipelineFixture, OnlineSequentialSpinOnce) {
  vio_params_.parallel_run_ = false;
  buildOnlinePipeline(vio_params_);
  ASSERT_TRUE(dataset_parser_);
  dataset_parser_->spin();
  ASSERT_TRUE(vio_pipeline_);
  vio_pipeline_->spin();
  vio_pipeline_->shutdown();
}

TEST_F(GnssVioPipelineFixture, OnlineParallelStartManualShutdown) {
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle_pipeline = std::async(std::launch::async,
                                    &VIO::GnssStereoImuPipeline::spin,
                                    vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle_pipeline.get());
}

TEST_F(GnssVioPipelineFixture, OnlineParallelSpinManualShutdown) {
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline = std::async(std::launch::async,
                                    &VIO::GnssStereoImuPipeline::spin,
                                    vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle.get());
  EXPECT_FALSE(handle_pipeline.get());
}

TEST_F(GnssVioPipelineFixture, OnlineParallelSpinShutdownWhenFinished) {
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline = std::async(std::launch::async,
                                    &VIO::GnssStereoImuPipeline::spin,
                                    vio_pipeline_.get());
  auto handle_shutdown =
      std::async(std::launch::async,
                 &VIO::GnssStereoImuPipeline::shutdownWhenFinished,
                 vio_pipeline_.get(),
                 500,
                 true);
  EXPECT_TRUE(handle_shutdown.get());
  EXPECT_FALSE(handle_pipeline.get());
  EXPECT_FALSE(handle.get());
}

// Offline processing, with blocking dataprovider queues if full.
TEST_F(GnssVioPipelineFixture, OfflineSequentialShutdown) {
  vio_params_.parallel_run_ = false;
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(vio_pipeline_);
  vio_pipeline_->shutdown();
  EXPECT_FALSE(vio_pipeline_->spin());
}

TEST_F(GnssVioPipelineFixture, OfflineSequentialSpinOnce) {
  vio_params_.parallel_run_ = false;
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(dataset_parser_);
  dataset_parser_->spin();
  ASSERT_TRUE(vio_pipeline_);
  vio_pipeline_->spin();
  vio_pipeline_->shutdown();
}

TEST_F(GnssVioPipelineFixture, OfflineParallelStartManualShutdown) {
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle_pipeline = std::async(std::launch::async,
                                    &VIO::GnssStereoImuPipeline::spin,
                                    vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle_pipeline.get());
}

TEST_F(GnssVioPipelineFixture, OfflineParallelSpinManualShutdown) {
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline = std::async(std::launch::async,
                                    &VIO::GnssStereoImuPipeline::spin,
                                    vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle.get());
  EXPECT_FALSE(handle_pipeline.get());
}

TEST_F(GnssVioPipelineFixture, OfflineParallelSpinShutdownWhenFinished) {
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline = std::async(std::launch::async,
                                    &VIO::GnssStereoImuPipeline::spin,
                                    vio_pipeline_.get());
  auto handle_shutdown =
      std::async(std::launch::async,
                 &VIO::GnssStereoImuPipeline::shutdownWhenFinished,
                 vio_pipeline_.get(),
                 500,
                 true);
  EXPECT_TRUE(handle_shutdown.get());
  EXPECT_FALSE(handle_pipeline.get());
  EXPECT_FALSE(handle.get());
}

// This tests that the VIO pipeline dies gracefully if the Backend breaks.
TEST_F(GnssVioPipelineFixture,
       OnlineParallelSpinBackendFailureGracefulShutdown) {
  // Modify vio pipeline so that the Backend fails
  vio_params_.backend_params_->nr_states_ = 1;
  vio_params_.backend_type_ = BackendType::kStereoImu;
  buildOnlinePipeline(vio_params_);
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline = std::async(std::launch::async,
                                    &VIO::GnssStereoImuPipeline::spin,
                                    vio_pipeline_.get());
  auto handle_shutdown =
      std::async(std::launch::async,
                 &VIO::GnssStereoImuPipeline::shutdownWhenFinished,
                 vio_pipeline_.get(),
                 500,
                 true);
  EXPECT_TRUE(handle_shutdown.get());
  EXPECT_FALSE(handle_pipeline.get());
  EXPECT_FALSE(handle.get());
}

// This tests that the VIO pipeline dies gracefully if the Backend breaks.
TEST_F(GnssVioPipelineFixture,
       OnlineParallelSpinRegularBackendFailureGracefulShutdown) {
  // Modify vio pipeline so that the Backend fails
  vio_params_.backend_params_->nr_states_ = 1;
  vio_params_.backend_type_ = BackendType::kStructuralRegularities;
  buildOnlinePipeline(vio_params_);
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline = std::async(std::launch::async,
                                    &VIO::GnssStereoImuPipeline::spin,
                                    vio_pipeline_.get());
  auto handle_shutdown =
      std::async(std::launch::async,
                 &VIO::GnssStereoImuPipeline::shutdownWhenFinished,
                 vio_pipeline_.get(),
                 500,
                 true);
  EXPECT_TRUE(handle_shutdown.get());
  EXPECT_FALSE(handle_pipeline.get());
  EXPECT_FALSE(handle.get());
}

}  // namespace VIO
