/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   KimeraVIO.cpp
 * @brief  Example of VIO pipeline.
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include <future>
#include <memory>
#include <utility>
#include <chrono>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <depthai/depthai.hpp>

#include "kimera-vio/dataprovider/OAKDataProvider.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/pipeline/MonoImuPipeline.h"
#include "kimera-vio/pipeline/StereoImuPipeline.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"

bool enableStereo = false;

dai::Pipeline createPipeline(){
      dai::Pipeline pipeline;
      pipeline.setXLinkChunkSize(0);
      // Define sources and outputs
      auto monoLeft = pipeline.create<dai::node::MonoCamera>();
      auto monoRight = pipeline.create<dai::node::MonoCamera>();
      auto imu = pipeline.create<dai::node::IMU>();

      // enable ACCELEROMETER_RAW at 500 hz rate
      imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
      // enable GYROSCOPE_RAW at 400 hz rate
      imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);

      // min number of imu msgs in batch of X, if the host is not blocked and USB bandwidth is available
      imu->setBatchReportThreshold(8);
      // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
      // if lower or equal to batchReportThreshold then the sending is always blocking on device
      // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
      imu->setMaxBatchReports(20);

      auto xoutImu = pipeline.create<dai::node::XLinkOut>();
      auto xoutL = pipeline.create<dai::node::XLinkOut>();
      auto xoutR = pipeline.create<dai::node::XLinkOut>();
      auto xoutRectifL = pipeline.create<dai::node::XLinkOut>();
      auto xoutRectifR = pipeline.create<dai::node::XLinkOut>();

      // XLinkOut
      xoutImu->setStreamName("imu");
      // xoutRight->setStreamName("right");
      // xoutDisp->setStreamName("disparity");
      // xoutDepth->setStreamName("depth");
      xoutRectifL->setStreamName("rectLeft");
      xoutRectifR->setStreamName("rectRight");
      xoutL->setStreamName("left");
      xoutR->setStreamName("right");

      // Properties
      monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
      monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
      monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
      monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

      // stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY); // HIGH_ACCURACY
      // stereo->setRectifyEdgeFillColor(0);
      // stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_5x5);
      
      // stereo->setSubpixel(true);
      // stereo->setLeftRightCheck(true);
      // stereo->setExtendedDisparity(false);
      
      // stereo->enableDistortionCorrection(true);

    if (enableStereo){
      auto stereo = pipeline.create<dai::node::StereoDepth>();
      // stereo->syncedLeft.link(xoutLeft->input);
      // stereo->syncedRight.link(xoutRight->input);
      // stereo->disparity.link(xoutDisp->input);
      //  Linking
      monoLeft->out.link(stereo->left);
      monoRight->out.link(stereo->right);

      stereo->setRectification(false);
      // if(outputRectified) {
      stereo->syncedLeft.link(xoutL->input);
      stereo->syncedRight.link(xoutR->input);
      stereo->rectifiedLeft.link(xoutRectifL->input);
      stereo->rectifiedRight.link(xoutRectifR->input);
      
    }
    else{
      monoLeft->out.link(xoutL->input);
      monoRight->out.link(xoutR->input);
    }
      
      imu->out.link(xoutImu->input);

      // if(outputDepth) {
          // stereo->depth.link(xoutDepth->input);
      // }
      return pipeline;
    }

DEFINE_string(
    params_folder_path,
    "../params/OAK-D",
    "Path to the folder containing the yaml files with the VIO parameters.");

int main(int argc, char* argv[]) {
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Parse VIO parameters from gflags.
  VIO::VioParams vio_params(FLAGS_params_folder_path,
  "PipelineParams.yaml",
                "ImuParams.yaml",
                "LeftCameraParamsS2BNO.yaml",
                "RightCameraParamsS2BNO.yaml",
                "FrontendParams.yaml",
                "BackendParams.yaml",
                "LcdParams.yaml",
                "DisplayParams.yaml");

  // Build dataset parser.
  VIO::DataProviderInterface::Ptr dataset_parser = std::make_shared<VIO::OAKDataProvider>(vio_params);
  CHECK(dataset_parser);

  // ------------------------ VIO Pipeline Config ------------------------  //
  VIO::Pipeline::Ptr vio_pipeline;

  switch (vio_params.frontend_type_) {
    case VIO::FrontendType::kMonoImu: {
      vio_pipeline = VIO::make_unique<VIO::MonoImuPipeline>(vio_params);
    } break;
    case VIO::FrontendType::kStereoImu: {
      vio_pipeline = VIO::make_unique<VIO::StereoImuPipeline>(vio_params);
    } break;
    default: {
      LOG(FATAL) << "Unrecognized Frontend type: "
                 << VIO::to_underlying(vio_params.frontend_type_)
                 << ". 0: Mono, 1: Stereo.";
    } break;
  }

  // Register callback to shutdown data provider in case VIO pipeline
  // shutsdown.
  vio_pipeline->registerShutdownCallback(
      std::bind(&VIO::DataProviderInterface::shutdown, dataset_parser));

  // Register callback to vio pipeline.
  dataset_parser->registerImuSingleCallback(
      std::bind(&VIO::Pipeline::fillSingleImuQueue,
                vio_pipeline,
                std::placeholders::_1));
  // We use blocking variants to avoid overgrowing the input queues (use
  // the non-blocking versions with real sensor streams)
  dataset_parser->registerLeftFrameCallback(
      std::bind(&VIO::Pipeline::fillLeftFrameQueue,
                vio_pipeline,
                std::placeholders::_1));

  if (vio_params.frontend_type_ == VIO::FrontendType::kStereoImu) {
    VIO::StereoImuPipeline::Ptr stereo_pipeline =
        VIO::safeCast<VIO::Pipeline, VIO::StereoImuPipeline>(
            vio_pipeline);

    dataset_parser->registerRightFrameCallback(
        std::bind(&VIO::StereoImuPipeline::fillRightFrameQueue,
                  stereo_pipeline,
                  std::placeholders::_1));
  }

  // ------------------------ The OAK's Pipeline ------------------------ //
  dai::Pipeline pipeline = createPipeline();
  auto daiDevice = std::make_shared<dai::Device>(pipeline);

  auto leftQueue = daiDevice->getOutputQueue("left", 30, false);
  auto rightQueue = daiDevice->getOutputQueue("right", 30, false);
  auto imuQueue = daiDevice->getOutputQueue("imu", 30, false);

  VIO::OAKDataProvider::Ptr oak_data_parser =
        VIO::safeCast<VIO::DataProviderInterface, VIO::OAKDataProvider>(dataset_parser);
  leftQueue->addCallback(std::bind(&VIO::OAKDataProvider::leftImageCallback, oak_data_parser, std::placeholders::_1, std::placeholders::_2));
  rightQueue->addCallback(std::bind(&VIO::OAKDataProvider::rightImageCallback, oak_data_parser, std::placeholders::_1, std::placeholders::_2));
  imuQueue->addCallback(std::bind(&VIO::OAKDataProvider::imuCallback, oak_data_parser, std::placeholders::_1, std::placeholders::_2));

// ---------------------------ASYNC Launch-------------------------------- //

  // Spin dataset.
  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;
  if (vio_params.parallel_run_) {
    auto handle = std::async(std::launch::async,
                             &VIO::DataProviderInterface::spin,
                             dataset_parser);
    auto handle_pipeline =
        std::async(std::launch::async,
                   &VIO::Pipeline::spin,
                   vio_pipeline);
    auto handle_shutdown =
        std::async(std::launch::async,
                   &VIO::Pipeline::shutdownWhenFinished,
                   vio_pipeline,
                   500,
                   true);
    vio_pipeline->spinViz();
    is_pipeline_successful = !handle.get();
    handle_shutdown.get();
    handle_pipeline.get();
  } else {
    while (dataset_parser->spin() && vio_pipeline->spin()) {
      continue;
    };
    vio_pipeline->shutdown();
    is_pipeline_successful = true;
  }

  // Output stats.
  auto spin_duration = VIO::utils::Timer::toc(tic);
  LOG(WARNING) << "Spin took: " << spin_duration.count() << " ms.";
  LOG(INFO) << "Pipeline successful? "
            << (is_pipeline_successful ? "Yes!" : "No!");

  if (is_pipeline_successful) {
    // Log overall time of pipeline run.
    VIO::PipelineLogger logger;
    logger.logPipelineOverallTiming(spin_duration);
  }

  return is_pipeline_successful ? EXIT_SUCCESS : EXIT_FAILURE;
}
