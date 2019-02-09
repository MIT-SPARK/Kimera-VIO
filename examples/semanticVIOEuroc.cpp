/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   semanticVIOEuroc.cpp
 * @brief  Example of semantic VIO pipeline running on the Euroc dataset.
 * @author Antoni Rosinol
 */

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ETH_parser.h"
#include "pipeline/Pipeline.h"
#include "utils/Timer.h"
#include "LoggerMatlab.h"

#include "mesh/Mesher.h"
namespace VIO {
Mesher::Mesh3DColors dummySemanticSegmentation(cv::Mat left_image,
                                               const Mesh2D& mesh_2d,
                                               const Mesh3D& mesh_3d) {
  // Dummy checks for valid data.
  CHECK(!left_image.empty());
  CHECK_GE(mesh_2d.getNumberOfUniqueVertices(), 0);
  CHECK_GE(mesh_3d.getNumberOfUniqueVertices(), 0);

  // Color all vertices in red. Each polygon will be colored according
  // to a mix of the three vertices colors I think...
  cv::Mat colors (mesh_3d.getNumberOfUniqueVertices(), 1,
                  CV_8UC3, cv::viz::Color::red());
  return colors;
}
} // End of VIO namespace.

////////////////////////////////////////////////////////////////////////////////
// semanticVIOexample
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
  // Initialize Google's flags library.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Ctor ETHDatasetParser, and parse dataset.
  VIO::ETHDatasetParser eth_dataset_parser;
  VIO::Pipeline vio_pipeline (&eth_dataset_parser);

  // Register callback to semantic segmentation.
  vio_pipeline.registerSemanticMeshSegmentationCallback(
        &VIO::dummySemanticSegmentation);

  // Register callback to vio_pipeline.
  eth_dataset_parser.registerVioCallback(
        std::bind(&VIO::Pipeline::spin, &vio_pipeline, std::placeholders::_1));

  // Spin dataset.
  auto tic = VIO::utils::Timer::tic();
  const bool is_pipeline_successful = eth_dataset_parser.spin();
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
