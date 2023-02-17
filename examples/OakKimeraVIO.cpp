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
#include "kimera-vio/dataprovider/OAKStereoDataProvider.h"
#include "kimera-vio/dataprovider/OAK3DFeatureDataProvider.h"

#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/pipeline/MonoImuPipeline.h"
#include "kimera-vio/pipeline/StereoImuPipeline.h"
#include "kimera-vio/pipeline/RgbdImuPipeline.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"


DEFINE_string(
    params_folder_path,
    "../params/OAK-D-mod",
    "Path to the folder containing the yaml files with the VIO parameters.");

DEFINE_bool(enable_ondevice_stereo_feature,
            false,
            "Enable Stereo and Feature tracking from On-Device");

bool enableStereoFeature = false;

dai::Pipeline createPipeline(VIO::VioParams params){
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
      imu->setBatchReportThreshold(14);
      // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
      // if lower or equal to batchReportThreshold then the sending is always blocking on device
      // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
      imu->setMaxBatchReports(25);

      auto xoutImu = pipeline.create<dai::node::XLinkOut>();
      auto xoutL = pipeline.create<dai::node::XLinkOut>();

      // XLinkOut
      xoutImu->setStreamName("imu");
      xoutL->setStreamName("left");

      // Properties
      monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
      monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
      monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
      monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    if (enableStereoFeature){
      auto stereo = pipeline.create<dai::node::StereoDepth>();
      // Stereo Config
      stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
      stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_5x5);
      stereo->setLeftRightCheck(true);
      stereo->setExtendedDisparity(false);
      stereo->setSubpixel(true);
      stereo->setDepthAlign(dai::CameraBoardSocket::LEFT);

      //  Linking
      auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
      xoutDepth->setStreamName("depth");

      stereo->depth.link(xoutDepth->input);
      monoLeft->out.link(stereo->left);
      monoRight->out.link(stereo->right);

      stereo->syncedLeft.link(xoutL->input);      

      // Feature Tracker setup
      auto featureTrackerRight = pipeline.create<dai::node::FeatureTracker>();
      monoLeft->out.link(featureTrackerRight->inputImage);

      auto numShaves = 2;
      auto numMemorySlices = 2;
      featureTrackerRight->setHardwareResources(numShaves, numMemorySlices);

      // Config Optical FLow  
      // auto featureTrackerConfig = dai::FeatureTrackerConfig();
      // featureTrackerConfig.setOpticalFlow();
      // // TODO(Saching): Use FULL control of Corner Detector in future for experimentation
      // featureTrackerConfig.setCornerDetector(dai::RawFeatureTrackerConfig::CornerDetector::Type::HARRIS);
      // featureTrackerConfig.setFeatureMaintainer(true);
      // featureTrackerRight->initialConfig.set(featureTrackerConfig.get());
      featureTrackerRight->initialConfig.setNumTargetFeatures(params.frontend_params_.feature_detector_params_.max_features_per_frame_);

      auto xoutTrackedFeaturesR = pipeline.create<dai::node::XLinkOut>();
      xoutTrackedFeaturesR->setStreamName("trackers");
      featureTrackerRight->outputFeatures.link(xoutTrackedFeaturesR->input);
    }
    else{
      auto xoutR = pipeline.create<dai::node::XLinkOut>();
      xoutR->setStreamName("right");
      monoLeft->out.link(xoutL->input);
      monoRight->out.link(xoutR->input);
    }

    imu->out.link(xoutImu->input);
    return pipeline;
    }

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

  enableStereoFeature = vio_params.frontend_params_.use_on_device_tracking_;
  std::cout << "Is on device feature enabled: " << std::boolalpha <<  enableStereoFeature << std::endl;
  // Build dataset parser.
  VIO::DataProviderInterface::Ptr dataset_parser;
  if(enableStereoFeature){
    dataset_parser = std::make_shared<VIO::OAK3DFeatureDataProvider>(vio_params);
  }
  else{
    dataset_parser = std::make_shared<VIO::OAKStereoDataProvider>(vio_params);
  }
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
    case VIO::FrontendType::kRgbdImu: {
      vio_pipeline = VIO::make_unique<VIO::RgbdImuPipeline>(vio_params);
    } break;
    default: {
      LOG(FATAL) << "Unrecognized Frontend type: "
                 << VIO::to_underlying(vio_params.frontend_type_)
                 << ". 0: Mono, 1: Stereo. 2: RGBD";
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
  else if (vio_params.frontend_type_ == VIO::FrontendType::kRgbdImu) {
    VIO::RgbdImuPipeline::Ptr rgbd_pipeline =
        VIO::safeCast<VIO::Pipeline, VIO::RgbdImuPipeline>(
            vio_pipeline);

    dataset_parser->registerDepthFrameCallback(
        std::bind(&VIO::RgbdImuPipeline::fillDepthFrameQueue,
                  rgbd_pipeline,
                  std::placeholders::_1));
    // dataset_parser->registerFeatureTrackletsCallback(
    //     std::bind(&VIO::RgbdImuPipeline::fillFeatureTrackletsQueue,
    //               rgbd_pipeline,
    //               std::placeholders::_1));

  }



  // ------------------------ The OAK's Pipeline ------------------------ //
  dai::Pipeline pipeline = createPipeline(vio_params);
  auto daiDevice = std::make_shared<dai::Device>(pipeline);

  auto leftQueue = daiDevice->getOutputQueue("left", 10, false);
  auto imuQueue = daiDevice->getOutputQueue("imu", 10, false);

  VIO::OAKDataProvider::Ptr oak_data_parser =
        VIO::safeCast<VIO::DataProviderInterface, VIO::OAKDataProvider>(dataset_parser);

// ---------------------------ASYNC Launch-------------------------------- //
  oak_data_parser->setLeftImuQueues(leftQueue, imuQueue);
  if(enableStereoFeature){
    auto depthQueue = daiDevice->getOutputQueue("depth", 10, false);
    auto featureQueue = daiDevice->getOutputQueue("trackers", 10, false);
    VIO::OAK3DFeatureDataProvider::Ptr oak_feature_data_parser =
      VIO::safeCast<VIO::DataProviderInterface, VIO::OAK3DFeatureDataProvider>(dataset_parser);
    oak_feature_data_parser->setDepthFeatureQueues(depthQueue, featureQueue);
  }
  else{
    VIO::OAKStereoDataProvider::Ptr oak_feature_data_parser =
      VIO::safeCast<VIO::DataProviderInterface, VIO::OAKStereoDataProvider>(dataset_parser);
    
    auto rightQueue = daiDevice->getOutputQueue("right", 10, false);
    oak_feature_data_parser->setRightQueue(rightQueue);
  }

  // Spin dataset.
  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;
  if (vio_params.parallel_run_) {

    // leftQueue->addCallback(std::bind(&VIO::OAKDataProvider::leftImageCallback, oak_data_parser, std::placeholders::_1, std::placeholders::_2));
    // rightQueue->addCallback(std::bind(&VIO::OAKDataProvider::rightImageCallback, oak_data_parser, std::placeholders::_1, std::placeholders::_2));
    // imuQueue->addCallback(std::bind(&VIO::OAKDataProvider::imuCallback, oak_data_parser, std::placeholders::_1, std::placeholders::_2));

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
                   800,
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
