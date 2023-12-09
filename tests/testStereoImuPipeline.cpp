#include <future>
#include <memory>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/pipeline/StereoImuPipeline.h"

DECLARE_string(test_data_path);
DECLARE_bool(visualize);

namespace VIO {

class VioPipelineFixture : public ::testing::Test {
 public:
  VioPipelineFixture()
      : dataset_parser_(nullptr),
        vio_pipeline_(nullptr),
        vio_params_(FLAGS_test_data_path + "/EurocParams") {
    FLAGS_visualize = false;
    buildOnlinePipeline(vio_params_);
  }
  ~VioPipelineFixture() override {
    destroyPipeline();
  }

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
    dataset_parser_ = std::make_unique<EurocDataProvider>(
        FLAGS_test_data_path + "/MicroEurocDataset",
        initial_k,
        final_k,
        vio_params);
    vio_pipeline_ = std::make_unique<StereoImuPipeline>(vio_params);
    connectVioPipeline();
  }

  void buildOfflinePipeline(const VioParams& vio_params) {
    constexpr int initial_k = 10;
    constexpr int final_k = 80;
    // Needed in order to disconnect previous pipeline in case someone calls
    // this function repeatedly within the same test.
    destroyPipeline();
    LOG(INFO) << "Building pipeline.";
    vio_pipeline_ = std::make_unique<StereoImuPipeline>(vio_params);
    dataset_parser_ = std::make_unique<EurocDataProvider>(
        FLAGS_test_data_path + "/MicroEurocDataset",
        initial_k,
        final_k,
        vio_params);
    connectVioPipelineWithBlockingIfFullQueues();
  }

  void connectVioPipeline() {
    LOG(INFO) << "Connecting pipeline.";
    CHECK(dataset_parser_);
    CHECK(vio_pipeline_);

    // Register callback to shutdown data provider in case VIO pipeline
    // shutsdown.
    vio_pipeline_->registerShutdownCallback(std::bind(
        &VIO::DataProviderInterface::shutdown, dataset_parser_.get()));

    // Register callback to vio pipeline.
    dataset_parser_->registerImuSingleCallback(
        std::bind(&VIO::StereoImuPipeline::fillSingleImuQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    // We use blocking variants to avoid overgrowing the input queues (use
    // the non-blocking versions with real sensor streams)
    dataset_parser_->registerLeftFrameCallback(
        std::bind(&VIO::StereoImuPipeline::fillLeftFrameQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    dataset_parser_->registerRightFrameCallback(
        std::bind(&VIO::StereoImuPipeline::fillRightFrameQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
  }

  void connectVioPipelineWithBlockingIfFullQueues() {
    LOG(INFO) << "Connecting pipeline.";
    CHECK(dataset_parser_);
    CHECK(vio_pipeline_);

    // Register callback to shutdown data provider in case VIO pipeline
    // shutsdown.
    vio_pipeline_->registerShutdownCallback(std::bind(
        &VIO::DataProviderInterface::shutdown, dataset_parser_.get()));

    // Register callback to vio pipeline.
    dataset_parser_->registerImuSingleCallback(
        std::bind(&VIO::StereoImuPipeline::fillSingleImuQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    // We use blocking variants to avoid overgrowing the input queues (use
    // the non-blocking versions with real sensor streams)
    dataset_parser_->registerLeftFrameCallback(
        std::bind(&VIO::StereoImuPipeline::fillLeftFrameQueueBlockingIfFull,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    dataset_parser_->registerRightFrameCallback(
        std::bind(&VIO::StereoImuPipeline::fillRightFrameQueueBlockingIfFull,
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
  StereoImuPipeline::UniquePtr vio_pipeline_;
  VioParams vio_params_;
};

TEST_F(VioPipelineFixture, OnlineSequentialStart) {
  vio_params_.parallel_run_ = false;
  buildOnlinePipeline(vio_params_);
  ASSERT_TRUE(vio_pipeline_);
  EXPECT_TRUE(vio_pipeline_->spin());
  // If this segfaults, make sure you are deleting first the vio and then the
  // dataset parser.
}

// Online processing, with non-blocking dataprovider queues.
TEST_F(VioPipelineFixture, OnlineSequentialShutdown) {
  vio_params_.parallel_run_ = false;
  buildOnlinePipeline(vio_params_);
  ASSERT_TRUE(vio_pipeline_);
  vio_pipeline_->shutdown();
  EXPECT_FALSE(vio_pipeline_->spin());
}

TEST_F(VioPipelineFixture, OnlineSequentialSpinOnce) {
  vio_params_.parallel_run_ = false;
  buildOnlinePipeline(vio_params_);
  ASSERT_TRUE(dataset_parser_);
  dataset_parser_->spin();
  ASSERT_TRUE(vio_pipeline_);
  vio_pipeline_->spin();
  vio_pipeline_->shutdown();
}

TEST_F(VioPipelineFixture, OnlineSequentialSpin) {
  // TODO(Toni): remove visualizer gflags!
  vio_params_.parallel_run_ = false;
  buildOnlinePipeline(vio_params_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  while (dataset_parser_->spin() && vio_pipeline_->spin()) {
    /* well, nothing to do :) */
  };
  vio_pipeline_->shutdown();
}

TEST_F(VioPipelineFixture, OnlineParallelStartManualShutdown) {
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::StereoImuPipeline::spin, vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle_pipeline.get());
}

TEST_F(VioPipelineFixture, OnlineParallelSpinManualShutdown) {
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::StereoImuPipeline::spin, vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle.get());
  EXPECT_FALSE(handle_pipeline.get());
}

TEST_F(VioPipelineFixture, OnlineParallelSpinShutdownWhenFinished) {
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::StereoImuPipeline::spin, vio_pipeline_.get());
  auto handle_shutdown = std::async(std::launch::async,
                                    &VIO::StereoImuPipeline::shutdownWhenFinished,
                                    vio_pipeline_.get(),
                                    500, true);
  EXPECT_TRUE(handle_shutdown.get());
  EXPECT_FALSE(handle_pipeline.get());
  EXPECT_FALSE(handle.get());
}

// Offline processing, with blocking dataprovider queues if full.
TEST_F(VioPipelineFixture, OfflineSequentialShutdown) {
  vio_params_.parallel_run_ = false;
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(vio_pipeline_);
  vio_pipeline_->shutdown();
  EXPECT_FALSE(vio_pipeline_->spin());
}

TEST_F(VioPipelineFixture, OfflineSequentialSpinOnce) {
  vio_params_.parallel_run_ = false;
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(dataset_parser_);
  dataset_parser_->spin();
  ASSERT_TRUE(vio_pipeline_);
  vio_pipeline_->spin();
  vio_pipeline_->shutdown();
}

TEST_F(VioPipelineFixture, OfflineSequentialSpin) {
  // TODO(Toni): remove visualizer gflags!
  vio_params_.parallel_run_ = false;
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  while (dataset_parser_->spin() && vio_pipeline_->spin()) {
    /* well, nothing to do :) */
  };
  vio_pipeline_->shutdown();
}

TEST_F(VioPipelineFixture, OfflineParallelStartManualShutdown) {
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::StereoImuPipeline::spin, vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle_pipeline.get());
}

TEST_F(VioPipelineFixture, OfflineParallelSpinManualShutdown) {
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::StereoImuPipeline::spin, vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle.get());
  EXPECT_FALSE(handle_pipeline.get());
}

TEST_F(VioPipelineFixture, OfflineParallelSpinShutdownWhenFinished) {
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::StereoImuPipeline::spin, vio_pipeline_.get());
  auto handle_shutdown = std::async(std::launch::async,
                                    &VIO::StereoImuPipeline::shutdownWhenFinished,
                                    vio_pipeline_.get(),
                                    500, true);
  EXPECT_TRUE(handle_shutdown.get());
  EXPECT_FALSE(handle_pipeline.get());
  EXPECT_FALSE(handle.get());
}

// This tests that the VIO pipeline dies gracefully if the Backend breaks.
TEST_F(VioPipelineFixture, OfflineSequentialSpinBackendFailureGracefulShutdown) {
  // Modify vio pipeline so that the Backend fails
  vio_params_.parallel_run_ = false;
  vio_params_.backend_params_->nr_states_ = 1;
  vio_params_.backend_type_ = BackendType::kStereoImu;
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  while (dataset_parser_->spin() && vio_pipeline_->spin()) {
    /* well, nothing to do :) */
  };
  vio_pipeline_->shutdown();
}

// This tests that the VIO pipeline dies gracefully if the Backend breaks.
TEST_F(VioPipelineFixture, OnlineParallelSpinBackendFailureGracefulShutdown) {
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
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::StereoImuPipeline::spin, vio_pipeline_.get());
  auto handle_shutdown = std::async(std::launch::async,
                                    &VIO::StereoImuPipeline::shutdownWhenFinished,
                                    vio_pipeline_.get(),
                                    500, true);
  EXPECT_TRUE(handle_shutdown.get());
  EXPECT_FALSE(handle_pipeline.get());
  EXPECT_FALSE(handle.get());
}

// This tests that the VIO pipeline dies gracefully if the Backend breaks.
TEST_F(VioPipelineFixture, OnlineParallelSpinRegularBackendFailureGracefulShutdown) {
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
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::StereoImuPipeline::spin, vio_pipeline_.get());
  auto handle_shutdown = std::async(std::launch::async,
                                    &VIO::StereoImuPipeline::shutdownWhenFinished,
                                    vio_pipeline_.get(),
                                    500, true);
  EXPECT_TRUE(handle_shutdown.get());
  EXPECT_FALSE(handle_pipeline.get());
  EXPECT_FALSE(handle.get());
}

}  // namespace VIO
