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
                                   1000);
  if (FLAGS_display) {
    euroc_playground.visualizeGtData(true, true, true);

    // Create feature detector
    FeatureDetectorParams feature_detector_params;
    feature_detector_params.feature_detector_type_ = FeatureDetectorType::FAST;
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

      // Build 2D Mesh out of image keypoints using delaunay triangulation
      Mesh2D mesh_2d;
      Mesher::createMesh2D(keypoints, it->second.left_image_.size(), &mesh_2d);

      // Visualize 2D mesh
      cv::Mat mesh_2d_viz;
      cv::cvtColor(it->second.left_image_.clone(), mesh_2d_viz, CV_GRAY2RGB);
      for (const auto& keypoint : keypoints) {
        cv::circle(mesh_2d_viz,
                   keypoint,
                   2,
                   cv::viz::Color::red(),
                   CV_FILLED,
                   CV_AA,
                   0);
      }
      MeshOptimization::draw2dMeshOnImg(mesh_2d, &mesh_2d_viz);
      cv::imshow("Mesh 2D", mesh_2d_viz);
      cv::waitKey(0);

      // Optimize mesh using MeshOpt
      // MeshOptimization mesh_opt;

      // Only do one image for now...
      break;
    }
  }
}
}
