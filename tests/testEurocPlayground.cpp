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
DEFINE_bool(display, false, "Display tests results.");

namespace VIO {

TEST(TestEurocPlayground, basicEurocPlayground) {
  EurocPlayground euroc_playground(FLAGS_test_data_path + "/V1_01_easy/",
                                   FLAGS_test_data_path + "/EurocParams",
                                   50,
                                   100);
  if (FLAGS_display) {
    euroc_playground.visualizeGtData(true, true, true);

    // Create feature detector
    FeatureDetectorParams feature_detector_params;
    feature_detector_params.feature_detector_type_ = FeatureDetectorType::GFTT;
    FeatureDetector feature_detector(feature_detector_params);

    // Get pose with depth maps
    for (MeshPackets::const_iterator it =
             euroc_playground.mesh_packets_.begin();
         it != euroc_playground.mesh_packets_.end();
         it++) {
      std::vector<cv::KeyPoint> corners =
          feature_detector.rawFeatureDetection(it->second.left_image_);
      KeypointsCV keypoints;
      cv::KeyPoint::convert(corners, keypoints);

      const cv::Size& img_size = it->second.left_image_.size();
      keypoints.push_back(KeypointCV(1u, 1u));
      keypoints.push_back(KeypointCV(1u, img_size.width - 1u));
      keypoints.push_back(KeypointCV(img_size.height - 1u, 0u));
      keypoints.push_back(KeypointCV(img_size.height - 1u, img_size.width - 1u));

      // Build 2D Mesh out of image keypoints using delaunay triangulation
      Mesh2D mesh_2d;
      Mesher::createMesh2D(keypoints, img_size, &mesh_2d);

      // Visualize 2D mesh
      cv::Mat mesh_2d_viz;
      cv::cvtColor(it->second.left_image_.clone(), mesh_2d_viz, CV_GRAY2RGB);
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
      cv::waitKey(0);

      // Create pointcloud
      cv::Mat ordered_pcl = it->second.depth_map_.clone();
      // ordered_pcl = ordered_pcl.reshape(3,
      // it->second.depth_map_.size().area());

      // Optimize mesh using MeshOpt
      MeshOptimization mesh_opt(MeshOptimizerType::kGtsamMesh, true);
      mesh_opt.img_ = mesh_2d_viz;
      MeshOptimizationInput input;
      // Set to identity because the pointcloud is in the camera's frame of
      // reference
      // already!
      euroc_playground.vio_params_.camera_params_.at(0).body_Pose_cam_ =
          gtsam::Pose3::identity();
      input.camera_params = euroc_playground.vio_params_.camera_params_.at(0);
      input.mesh_2d = mesh_2d;

      cv::Mat pcl;
      cv::Mat connect;

      input.mesh_2d.convertVerticesMeshToMat(&pcl);
      input.mesh_2d.convertPolygonsMeshToMat(&connect);

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
      input.noisy_point_cloud = ordered_pcl.t();

      MeshOptimizationOutput::UniquePtr out_ptr = mesh_opt.spinOnce(input);
      out_ptr->optimized_mesh_3d.convertVerticesMeshToMat(&pcl);
      out_ptr->optimized_mesh_3d.convertPolygonsMeshToMat(&connect);

      LOG(INFO) << pcl;
      LOG(INFO) << connect;
    }
  }
}
}
