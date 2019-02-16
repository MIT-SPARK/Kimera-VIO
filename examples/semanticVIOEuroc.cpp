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
Mesher::Mesh3DVizProperties dummySemanticSegmentation(cv::Mat left_image,
                                                      const Mesh2D& mesh_2d,
                                                      const Mesh3D& mesh_3d) {
  // Dummy checks for valid data.
  CHECK(!left_image.empty());
  CHECK_GE(mesh_2d.getNumberOfUniqueVertices(), 0);
  CHECK_GE(mesh_3d.getNumberOfUniqueVertices(), 0);

  // Let us fill the mesh 3d viz properties structure.
  Mesher::Mesh3DVizProperties mesh_3d_viz_props;

  // Color all vertices in red. Each polygon will be colored according
  // to a mix of the three vertices colors I think...
  mesh_3d_viz_props.colors_ = cv::Mat (mesh_3d.getNumberOfUniqueVertices(), 1,
                                       CV_8UC3, cv::viz::Color::red());

  // Add texture to the mesh using the given image.
  Mesh2D::Polygon polygon;
  std::vector<Vec2d> tcoords (mesh_3d.getNumberOfPolygons());
  for (size_t i = 0; i < mesh_2d.getNumberOfPolygons(); i++) {
    CHECK(mesh_2d.getPolygon(i, &polygon)) << "Could not retrieve 2d polygon.";

    LandmarkId lmk0 = polygon.at(0).getLmkId();
    LandmarkId lmk1 = polygon.at(1).getLmkId();
    LandmarkId lmk2 = polygon.at(2).getLmkId();

    // Returns indices of points in the 3D mesh corresponding to the vertices
    // in the 2D mesh.
    int p0_id, p1_id, p2_id;
    CHECK(mesh_3d.getVertex(lmk0, nullptr, &p0_id)) << "Could not retrive 3d vertex";
    CHECK(mesh_3d.getVertex(lmk1, nullptr, &p1_id)) << "Could not retrive 3d vertex";
    CHECK(mesh_3d.getVertex(lmk2, nullptr, &p2_id)) << "Could not retrive 3d vertex";

    // Sanity check.
    CHECK_LE(p0_id, tcoords.size());
    CHECK_LE(p1_id, tcoords.size());
    CHECK_LE(p2_id, tcoords.size());

    // Get pixel coordinates of the vertices of the 2D mesh.
    const auto& px0 = polygon.at(0).getVertexPosition();
    const auto& px1 = polygon.at(1).getVertexPosition();
    const auto& px2 = polygon.at(2).getVertexPosition();

    // These pixels correspond to the tcoords in the image for the 3d mesh
    // vertices.
    tcoords.at(p0_id) = Vec2d(px0.x, px0.y);
    tcoords.at(p1_id) = Vec2d(px1.x, px1.y);
    tcoords.at(p2_id) = Vec2d(px2.x, px2.y);
  }

  mesh_3d_viz_props.texture_ = left_image;
  mesh_3d_viz_props.tcoords_ = cv::Mat(tcoords, true).reshape(2, 1);

  return mesh_3d_viz_props;
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
