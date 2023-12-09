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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>
#include <future>
#include <memory>
#include <utility>

#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/dataprovider/KittiDataProvider.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/pipeline/MonoImuPipeline.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/pipeline/StereoImuPipeline.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"

DEFINE_int32(dataset_type,
             0,
             "Type of parser to use:\n "
             "0: Euroc \n 1: Kitti (not supported).");
DEFINE_string(
    params_folder_path,
    "../params/Euroc",
    "Path to the folder containing the yaml files with the VIO parameters.");

int main(int argc, char* argv[]) {
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Parse VIO parameters from gflags.
  VIO::VioParams vio_params(FLAGS_params_folder_path);

  // Build dataset parser.
  VIO::DataProviderInterface::Ptr dataset_parser = nullptr;
  switch (FLAGS_dataset_type) {
    case 0: {
      switch (vio_params.frontend_type_) {
        case VIO::FrontendType::kMonoImu: {
          dataset_parser =
              std::make_unique<VIO::MonoEurocDataProvider>(vio_params);
        } break;
        case VIO::FrontendType::kStereoImu: {
          dataset_parser = std::make_unique<VIO::EurocDataProvider>(vio_params);
        } break;
        default: {
          LOG(FATAL) << "Unrecognized Frontend type: "
                     << VIO::to_underlying(vio_params.frontend_type_)
                     << ". 0: Mono, 1: Stereo.";
        }
      }
    } break;
    case 1: {
      dataset_parser = std::make_unique<VIO::KittiDataProvider>();
    } break;
    default: {
      LOG(FATAL) << "Unrecognized dataset type: " << FLAGS_dataset_type << "."
                 << " 0: EuRoC, 1: Kitti.";
    }
  }
  CHECK(dataset_parser);

  VIO::Pipeline::Ptr vio_pipeline;

  switch (vio_params.frontend_type_) {
    case VIO::FrontendType::kMonoImu: {
      vio_pipeline = std::make_unique<VIO::MonoImuPipeline>(vio_params);
    } break;
    case VIO::FrontendType::kStereoImu: {
      vio_pipeline = std::make_unique<VIO::StereoImuPipeline>(vio_params);
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
  dataset_parser->registerImuSingleCallback(std::bind(
      &VIO::Pipeline::fillSingleImuQueue, vio_pipeline, std::placeholders::_1));
  // We use blocking variants to avoid overgrowing the input queues (use
  // the non-blocking versions with real sensor streams)
  dataset_parser->registerLeftFrameCallback(std::bind(
      &VIO::Pipeline::fillLeftFrameQueue, vio_pipeline, std::placeholders::_1));

  if (vio_params.frontend_type_ == VIO::FrontendType::kStereoImu) {
    auto stereo_pipeline =
        std::dynamic_pointer_cast<VIO::StereoImuPipeline>(vio_pipeline);
    CHECK(stereo_pipeline);

    dataset_parser->registerRightFrameCallback(
        std::bind(&VIO::StereoImuPipeline::fillRightFrameQueue,
                  stereo_pipeline,
                  std::placeholders::_1));
  }

  // Spin dataset.
  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;
  if (vio_params.parallel_run_) {
    auto handle = std::async(
        std::launch::async, &VIO::DataProviderInterface::spin, dataset_parser);
    auto handle_pipeline =
        std::async(std::launch::async, &VIO::Pipeline::spin, vio_pipeline);
    auto handle_shutdown = std::async(
        std::launch::async,
        &VIO::Pipeline::waitForShutdown,
        vio_pipeline,
        [&dataset_parser]() -> bool { return !dataset_parser->hasData(); },
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
