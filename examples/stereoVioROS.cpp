/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   stereoVIOEuroc.cpp
 * @brief  Example of VIO pipeline running using ROS.
 * @author Antoni Rosinol
 */

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ETH_parser.h"
#include "pipeline/Pipeline.h"
#include "utils/Timer.h"
#include "LoggerMatlab.h"

int main(int argc, char *argv[]) {
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Ctor ETHDatasetParser, and parse dataset.
  VIO::ETHDatasetParser eth_dataset_parser;
  VIO::Pipeline vio_pipeline (&eth_dataset_parser);

  // Register callback to vio_pipeline.
  VIO::DataProvider ros_dataprovider = RostopicDataProvider();
  ros_dataprovider.registerVioCallback(
      std::bind(&VIO::Pipeline::spin, &vio_pipeline, std::placeholders::_1));

  // Spin dataset.
  auto tic = VIO::utils::Timer::tic();
  const bool is_pipeline_successful = ros_dataprovider.spin();
  auto spin_duration = VIO::utils::Timer::toc(tic);
  LOG(WARNING) << "Spin took: " << spin_duration.count() << " ms.";

  // Dataset spin has finished, shutdown VIO.
  vio_pipeline.shutdown();

  if (is_pipeline_successful) {
    // Log overall time of pipeline run.
    VIO::LoggerMatlab logger;
    logger.openLogFiles(11);
    logger.logPipelineOverallTiming(spin_duration);
    logger.closeLogFiles();
  }

  return is_pipeline_successful? EXIT_SUCCESS : EXIT_FAILURE;
}
