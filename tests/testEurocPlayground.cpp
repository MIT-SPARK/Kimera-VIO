/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testEurocPlayground.cpp
 * @brief  Loads all Euroc GT dataset, including pointclouds, to test/play with
 * algorithms.
 * @author Antoni Rosinol
 */

#include <future>
#include <limits>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetectorParams.h"
#include "kimera-vio/mesh/Mesh.h"
#include "kimera-vio/mesh/MeshOptimization.h"
#include "kimera-vio/mesh/Mesher.h"
#include "kimera-vio/playground/EurocPlayground.h"

DECLARE_string(test_data_path);
DECLARE_bool(display);

namespace VIO {

TEST(TestEurocPlayground, DISABLED_basicEurocPlayground) {
  EurocPlayground euroc_playground(FLAGS_test_data_path + "/V1_01_easy/",
                                   FLAGS_test_data_path + "/EurocParams",
                                   50,
                                   1000,
                                   300);

  Camera::ConstPtr mono_camera = std::make_unique<Camera>(
      euroc_playground.vio_params_.camera_params_.at(0));

  // Optimize mesh using MeshOpt
  MeshOptimization mesh_opt(MeshOptimizerType::kGtsamMesh,
                            MeshColorType::kVertexFlatColor,
                            mono_camera,
                            euroc_playground.visualizer_3d_);

  if (FLAGS_display) {
    euroc_playground.visualizeGtData(true, true, true);

    // Create feature detector
    FeatureDetectorParams feature_detector_params;
    feature_detector_params.feature_detector_type_ = FeatureDetectorType::GFTT;
    FeatureDetector feature_detector(feature_detector_params);

    // Get pose with depth maps
    VisualizerOutput::UniquePtr output = std::make_unique<VisualizerOutput>();
    size_t count = 0u;
    for (MeshPackets::const_iterator it =
             euroc_playground.mesh_packets_.begin();
         it != euroc_playground.mesh_packets_.end();
         it++) {
      cv::Mat left_img_rect = it->second.left_image_rect_;
      std::vector<cv::KeyPoint> corners =
          feature_detector.rawFeatureDetection(left_img_rect);
      KeypointsCV keypoints;
      cv::KeyPoint::convert(corners, keypoints);

      const cv::Size& img_size = left_img_rect.size();
      // keypoints.clear();
      for (size_t v = 0u; v < img_size.height; v += 50) {
        for (size_t u = 0u; u < img_size.width; u += 50) {
          keypoints.push_back(KeypointCV(u, v));
        }
      }

      // Build 2D Mesh out of image keypoints using delaunay triangulation
      Mesh2D mesh_2d;
      Mesher::createMesh2D(keypoints, &mesh_2d);

      // Visualize 2D mesh
      cv::Mat mesh_2d_viz;
      cv::cvtColor(left_img_rect.clone(), mesh_2d_viz, CV_GRAY2RGB);
      MeshOptimization::draw2dMeshOnImg(mesh_2d, &mesh_2d_viz);
      for (const auto& keypoint : keypoints) {
        cv::circle(mesh_2d_viz,
                   keypoint,
                   1,
                   cv::viz::Color::blue(),
                   CV_FILLED,
                   CV_AA,
                   0);
      }
      cv::imshow("Mesh 2D", mesh_2d_viz);
      cv::waitKey(1);

      // Create pointcloud
      cv::Mat ordered_pcl = it->second.depth_map_.clone();

      mesh_opt.img_ = left_img_rect;
      MeshOptimizationInput input;
      input.mesh_2d = mesh_2d;

      cv::Mat pcl;
      cv::Mat connect;

      input.mesh_2d.getVerticesMeshToMat(&pcl);
      input.mesh_2d.getPolygonsMeshToMat(&connect);

      // LOG(INFO) << pcl;
      // LOG(INFO) << connect;

      // This doesn't make too much sense because x,y coordinates are pixels!
      // convertMesh2dTo3d(
      //     mesh_2d, cv::Mat::eye(cv::Size(3, 2), CV_32F), &input.mesh_3d);

      // input.mesh_3d.convertVerticesMeshToMat(&pcl);
      // input.mesh_3d.convertPolygonsMeshToMat(&connect);

      // LOG(INFO) << pcl;
      // LOG(INFO) << connect;

      // mesh_opt.draw3dMesh(
      //     "Mesh 3D before opt", cv::viz::Color::blue(), input.mesh_3d);
      input.pcl = ordered_pcl;

      MeshOptimizationOutput::UniquePtr out_ptr = mesh_opt.spinOnce(input);
      out_ptr->optimized_mesh_3d.getVerticesMeshToMat(&pcl);
      out_ptr->optimized_mesh_3d.getPolygonsMeshToMat(&connect);
      cv::Mat colors = out_ptr->optimized_mesh_3d.getColorsMesh();
      // Move pcl to world coordinates
      output->visualization_type_ = VisualizationType::kPointcloud;
      CHECK(euroc_playground.visualizer_3d_);
      euroc_playground.visualizer_3d_->visualizeMesh3D(
          pcl, colors, connect, &output->widgets_, cv::Mat(), cv::Mat());
      gtsam::Pose3 world_pose_body = it->second.world_pose_body_;
      CHECK_GT(output->widgets_.size(), 0u);
      output->widgets_.rbegin()->second->setPose(
          UtilsOpenCV::gtsamPose3ToCvAffine3d(world_pose_body));
    }
    euroc_playground.display_module_->spinOnce(std::move(output));
    /// Render all the mesh reconstruction
    // mesh_opt.spinDisplay();
  }
}

}  // namespace kimera
