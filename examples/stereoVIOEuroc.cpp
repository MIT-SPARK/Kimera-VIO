/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   stereoVIOEuroc.cpp
 * @brief  example of VIO pipeline running on the Euroc dataset
 * @author Antoni Rosinol, Luca Carlone, Yun Chang
 */

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "datasource/ETH_parser.h"
#include "datasource/KittiDataSource.h"
#include "LoggerMatlab.h"
#include "pipeline/Pipeline.h"
#include "utils/Statistics.h"
#include "utils/Timer.h"

#include <future>

#include "StereoImuSyncPacket.h"

DEFINE_bool(parallel_run, false, "Run parallelized pipeline.");
DEFINE_int32(dataset_type, 0,
             "Type of parser to use:\n"
             "0: EuRoC\n"
             "1: Kitti");

////////////////////////////////////////////////////////////////////////////////
// stereoVIOexample
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  VIO::DataProvider* dataset_parser;
  switch (FLAGS_dataset_type) {
    case 0 :
    {
      dataset_parser = new VIO::ETHDatasetParser();
    }
    break; 
    case 1 :
    {
      dataset_parser = new VIO::KittiDataProvider();
    }
    break; 
    default: 
    {
      CHECK(false) << "Unrecognized dataset type: " << FLAGS_dataset_type << "."
                   << " 0: EuRoC, 1: Kitti.";
    }
  }

  VIO::Pipeline vio_pipeline(dataset_parser->getParams(),
                             FLAGS_parallel_run);

  // Register callback to vio_pipeline.
  dataset_parser->registerVioCallback(
      std::bind(&VIO::Pipeline::spin, &vio_pipeline, std::placeholders::_1));

  //// Spin dataset.
  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;
  if (FLAGS_parallel_run) {
    auto handle = std::async(std::launch::async, &VIO::DataProvider::spin,
                             *dataset_parser);
    auto handle_pipeline =
        std::async(std::launch::async, &VIO::Pipeline::shutdownWhenFinished,
                   &vio_pipeline);
    vio_pipeline.spinViz();
    is_pipeline_successful = handle.get();
    handle_pipeline.get();
  } else {
    is_pipeline_successful = dataset_parser->spin();
  }
  auto spin_duration = VIO::utils::Timer::toc(tic);
  LOG(WARNING) << "Spin took: " << spin_duration.count() << " ms.";
  LOG(INFO) << "Pipeline successful? "
            << (is_pipeline_successful ? "Yes!" : "No!");
  VIO::utils::Statistics::WriteToYamlFile("StatisticsVIO.yaml");

  if (is_pipeline_successful) {
    // Log overall time of pipeline run.
    VIO::LoggerMatlab logger;
    logger.openLogFiles(11);
    logger.logPipelineOverallTiming(spin_duration);
    logger.closeLogFiles();
  }

  return is_pipeline_successful ? EXIT_SUCCESS : EXIT_FAILURE;
}
