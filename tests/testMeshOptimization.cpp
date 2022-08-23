/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testMeshOptimization.cpp
 * @brief  test MeshOptimization implementation
 * @author Antoni Rosinol
 */

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "kimera-vio/mesh/MeshOptimization.h"
#include "kimera-vio/mesh/MeshUtils.h"

DECLARE_string(test_data_path);

namespace VIO {

class MeshOptimizationFixture : public ::testing::Test {
 public:
  MeshOptimizationFixture() {}

 protected:
  virtual void SetUp() override {}
  virtual void TearDown() override {}

 protected:
  static constexpr double tol = 1e-8;

 private:
};

TEST_F(MeshOptimizationFixture, DISABLED_testCollectTriangleDataPointsFast) {
  // Parse cam params
  CameraParams camera_params;
  camera_params.parseYAML(FLAGS_test_data_path +
                          "/EurocParams/LeftCameraParams.yaml");
  camera_params.body_Pose_cam_ = gtsam::Pose3();

  // But make the image be a 2 by 2 one for simplicity
  camera_params.image_size_ = cv::Size(2, 2);

  Camera::ConstPtr mono_camera = VIO::make_unique<Camera>(camera_params);

  // Optimize mesh using MeshOpt
  MeshOptimization mesh_opt(MeshOptimizerType::kGtsamMesh,
                            MeshColorType::kVertexSupport,
                            mono_camera);

  // Only add a point in the center of the triangle
  cv::Mat point_cloud = cv::Mat(0, 0, CV_32FC3);

  // Order matters for comparing below:
  // {0, 0} first the landmakrs inside the triangle
  // {1, 0} second the landmarks outside the triangle
  // {0, 1} Infinite points
  // {1, 1} Super-close points (depth-wise)
  KeypointsCV expected_kpts;
  expected_kpts.push_back(KeypointCV(0, 0));
  expected_kpts.push_back(KeypointCV(1, 0));
  expected_kpts.push_back(KeypointCV(0, 1));
  expected_kpts.push_back(KeypointCV(1, 1));
  VIO::Depths depths = {
      0.5, std::numeric_limits<double>::max(), 0.5, 0.0003};
  LandmarksCV expected_lmks;
  mono_camera->backProject(expected_kpts, depths, &expected_lmks);
  point_cloud.push_back(expected_lmks.at(0));
  point_cloud.push_back(expected_lmks.at(1));
  point_cloud.push_back(expected_lmks.at(2));
  point_cloud.push_back(expected_lmks.at(3));

  // the point cloud shape must be the same as the image size which is 2x2 now.
  // This is interpreted as: the pixel in {0,0} has a 3D point at expected_lmk0
  // the pixel in {0,1} has a 3D point in expected_lmk1 etc...
  // In principle, according to intrinsics, the lmks should project where the
  // to the pixels they belong to.
  point_cloud = point_cloud.reshape(3, {2, 2});

  Mesh2D mesh_2d;
  // Only add one triangle
  //  triangle vertex: *
  //  triangle edge: \
  //  datapoint inside: x
  //  datapoint outside: 0
  //
  //  |-->U
  //  |*2-------*3
  //  v|        /|
  //  V|   x   / |
  //   |      /  |
  //   |     /   |
  //   |    /    |
  //   |   /     |
  //   |  /   0  |
  //   | /       |
  //   |/        |
  //   *1-------*4
  Mesh2D::VertexType vtx1(0u, expected_kpts.at(0)); // 0,0
  Mesh2D::VertexType vtx2(1u, expected_kpts.at(1)); // 1,0
  Mesh2D::VertexType vtx3(2u, expected_kpts.at(2)); // 0,1
  // Mesh2D::VertexType vtx4(3, Vertex2D(1.0, 0.0));
  Mesh2D::Polygon tri;
  tri.push_back(vtx1);
  tri.push_back(vtx2);
  tri.push_back(vtx3);
  mesh_2d.addPolygonToMesh(tri);
  // tri = Mesh2D::Polygon();
  // tri.push_back(vtx1);
  // tri.push_back(vtx3);
  // tri.push_back(vtx4);
  // mesh_2d.addPolygonToMesh(tri);

  Vertex2D vtx1_xyz = vtx1.getVertexPosition();
  Vertex2D vtx2_xyz = vtx2.getVertexPosition();
  Vertex2D vtx3_xyz = vtx3.getVertexPosition();

  // check edgeFunctions are correct
  KeypointCV pixel_inside(0.5, 0.25);
  KeypointCV pixel_outside(0.5, 0.75);

  // cw
  EXPECT_GT(edgeFunction(pixel_inside, vtx1_xyz, vtx2_xyz), 0.0);
  EXPECT_GT(edgeFunction(pixel_inside, vtx2_xyz, vtx3_xyz), 0.0);
  EXPECT_GT(edgeFunction(pixel_inside, vtx3_xyz, vtx1_xyz), 0.0);
  EXPECT_LT(edgeFunction(pixel_outside, vtx3_xyz, vtx1_xyz), 0.0);

  // ccw
  EXPECT_LT(edgeFunction(pixel_inside, vtx2_xyz, vtx1_xyz), 0.0);
  EXPECT_LT(edgeFunction(pixel_inside, vtx3_xyz, vtx2_xyz), 0.0);
  EXPECT_LT(edgeFunction(pixel_inside, vtx1_xyz, vtx3_xyz), 0.0);
  EXPECT_GT(edgeFunction(pixel_outside, vtx1_xyz, vtx3_xyz), 0.0);

  // check sign function is correct

  // check pointInTriangle function is correct
  EXPECT_TRUE(MeshOptimization::pointInTriangle(
      pixel_inside, vtx1_xyz, vtx2_xyz, vtx3_xyz));
  EXPECT_FALSE(MeshOptimization::pointInTriangle(
      pixel_outside, vtx1_xyz, vtx2_xyz, vtx3_xyz));

  MeshOptimization::TriangleToDatapoints corresp;
  MeshOptimization::TriangleToPixels pixel_corresp;
  size_t number_of_valid_datapoints;
  mesh_opt.collectTriangleDataPointsFast(point_cloud,
                                         mesh_2d,
                                         &corresp,
                                         &pixel_corresp,
                                         &number_of_valid_datapoints);

  EXPECT_EQ(number_of_valid_datapoints, 1u);
  EXPECT_EQ(corresp.size(), 1u);
  ASSERT_EQ(corresp.size(), pixel_corresp.size());

  MeshOptimization::Datapoints datapoints = corresp.at(0u);
  LOG(ERROR) << datapoints;
  ASSERT_EQ(datapoints.size(), 1u);
  EXPECT_EQ(datapoints.at(0u), point_cloud.at<cv::Point3f>(0u, 0u));
  KeypointsCV datapoints_pixels = pixel_corresp.at(0u);
  ASSERT_EQ(datapoints_pixels.size(), 1u);
  EXPECT_EQ(datapoints_pixels.at(0u), expected_kpts.at(1u));

  // Check that the point was considered to be inside the triangle
  for (size_t tri_idx = 0u; tri_idx < mesh_2d.getNumberOfPolygons();
       tri_idx++) {
    std::vector<cv::Point3f> triangle_datapoints = corresp[tri_idx];
    EXPECT_EQ(triangle_datapoints.size(), 1u);
    //! Pixels associated to a triangle that have a depth value (datapoint,
    //! measurements)
    KeypointsCV datapoint_pixels = pixel_corresp[tri_idx];
    EXPECT_EQ(datapoint_pixels.size(), 1u);

    for (size_t i = 0u; i < datapoint_pixels.size(); i++) {
      const KeypointCV& actual_pixel = datapoint_pixels[i];
      const LandmarkCV& actual_lmk = triangle_datapoints[i];
      EXPECT_EQ(actual_lmk, expected_lmks[i]);
      KeypointCV expected_pixel;
      mono_camera->project(actual_lmk, &expected_pixel);
      EXPECT_EQ(actual_pixel, expected_pixel);
    }
  }
}

}  // namespace VIO
