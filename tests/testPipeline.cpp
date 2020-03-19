#include <future>
#include <memory>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/pipeline/Pipeline.h"

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
    buildPipeline(vio_params_);
  }

  ~VioPipelineFixture() override {}
 protected:
  void SetUp() override {}
  void TearDown() override {}

  void buildPipeline(const VioParams& vio_params) {
    constexpr int initial_k = 10;
    constexpr int final_k = 80;
    dataset_parser_ = VIO::make_unique<EurocDataProvider>(
        FLAGS_test_data_path + "/MicroEurocDataset",
        initial_k,
        final_k,
        vio_params);
    vio_pipeline_ = VIO::make_unique<Pipeline>(vio_params);
    connectVioPipeline();
  }

  void connectVioPipeline() {
    CHECK(dataset_parser_);
    CHECK(vio_pipeline_);

    // Register callback to shutdown data provider in case VIO pipeline
    // shutsdown.
    vio_pipeline_->registerShutdownCallback(std::bind(
        &VIO::DataProviderInterface::shutdown, dataset_parser_.get()));

    // Register callback to vio pipeline.
    dataset_parser_->registerImuSingleCallback(
        std::bind(&VIO::Pipeline::fillSingleImuQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    // We use blocking variants to avoid overgrowing the input queues (use
    // the non-blocking versions with real sensor streams)
    dataset_parser_->registerLeftFrameCallback(
        std::bind(&VIO::Pipeline::fillLeftFrameQueueBlocking,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    dataset_parser_->registerRightFrameCallback(
        std::bind(&VIO::Pipeline::fillRightFrameQueueBlocking,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
  }

 protected:
  DataProviderInterface::UniquePtr dataset_parser_;
  Pipeline::UniquePtr vio_pipeline_;
  VioParams vio_params_;
};

TEST_F(VioPipelineFixture, SequentialStart) {
  vio_params_.parallel_run_ = false;
  buildPipeline(vio_params_);
  ASSERT_TRUE(vio_pipeline_);
  EXPECT_TRUE(vio_pipeline_->spin());
}

TEST_F(VioPipelineFixture, SequentialShutdown) {
  vio_params_.parallel_run_ = false;
  buildPipeline(vio_params_);
  ASSERT_TRUE(vio_pipeline_);
  vio_pipeline_->shutdown();
  EXPECT_FALSE(vio_pipeline_->spin());
}

TEST_F(VioPipelineFixture, SequentialSpinOnce) {
  vio_params_.parallel_run_ = false;
  buildPipeline(vio_params_);
  ASSERT_TRUE(dataset_parser_);
  dataset_parser_->spin();
  ASSERT_TRUE(vio_pipeline_);
  vio_pipeline_->spin();
  vio_pipeline_->shutdown();
}

TEST_F(VioPipelineFixture, SequentialSpin) {
  // TODO(Toni): remove visualizer gflags!
  vio_params_.parallel_run_ = false;
  buildPipeline(vio_params_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  while (dataset_parser_->spin() && vio_pipeline_->spin()) {
    /* well, nothing to do :) */
  };
  vio_pipeline_->shutdown();
}

TEST_F(VioPipelineFixture, ParallelStartManualShutdown) {
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::Pipeline::spin, vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle_pipeline.get());
}

TEST_F(VioPipelineFixture, ParallelSpinManualShutdown) {
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::Pipeline::spin, vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle.get());
  EXPECT_FALSE(handle_pipeline.get());
}

TEST_F(VioPipelineFixture, ParallelSpinShutdownWhenFinished) {
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::Pipeline::spin, vio_pipeline_.get());
  auto handle_shutdown = std::async(std::launch::async,
                                    &VIO::Pipeline::shutdownWhenFinished,
                                    vio_pipeline_.get(), 500);
  EXPECT_FALSE(handle.get());
  EXPECT_TRUE(handle_shutdown.get());
  EXPECT_FALSE(handle_pipeline.get());
}

}  // namespace VIO
