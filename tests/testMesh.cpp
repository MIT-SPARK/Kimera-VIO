/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testMesh.cpp
 * @brief  test Mesh implementation
 * @author Antoni Rosinol
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/mesh/Mesh.h"

DECLARE_string(test_data_path);
DECLARE_bool(display);

namespace VIO {

class MeshFixture : public ::testing::Test {
 public:
  MeshFixture() {}

 protected:
  virtual void SetUp() override {}
  virtual void TearDown() override {}

 protected:
};

/**
 * @brief Test for non duplicated polygons when two exactly equal polygons
 * are added
 */
TEST_F(MeshFixture, addPolygonNoDuplicates) {
  // Add polygon to the mesh and check that all internal datastructures
  // are correctly populated and coherent
  Mesh2D mesh_2d;
  Mesh2D::Polygon polygon;
  LandmarkIds lmk_ids = {1u, 2u, 3u};
  for (const auto& lmk_id : lmk_ids) {
    // Use as vertex position a "random" one (just adding lmk_id).
    Mesh2D::VertexType vertex(lmk_id, Vertex2D(1.0 + lmk_id, 1.0 + 2 * lmk_id));
    polygon.push_back(vertex);
  }
  mesh_2d.addPolygonToMesh(polygon);
  mesh_2d.addPolygonToMesh(polygon);

  // Check non-duplicated
  cv::Mat adjacency_matrix = mesh_2d.getAdjacencyMatrix();
  ASSERT_EQ(adjacency_matrix.rows, adjacency_matrix.cols);
  ASSERT_EQ(adjacency_matrix.rows, 3u);

  // Check actual entries
  EXPECT_EQ(adjacency_matrix.at<uint8_t>(0, 0), 0u);
  EXPECT_EQ(adjacency_matrix.at<uint8_t>(0, 1), 1u);
  EXPECT_EQ(adjacency_matrix.at<uint8_t>(0, 2), 1u);
  EXPECT_EQ(adjacency_matrix.at<uint8_t>(1, 0), 1u);
  EXPECT_EQ(adjacency_matrix.at<uint8_t>(1, 1), 0u);
  EXPECT_EQ(adjacency_matrix.at<uint8_t>(1, 2), 1u);
  EXPECT_EQ(adjacency_matrix.at<uint8_t>(2, 0), 1u);
  EXPECT_EQ(adjacency_matrix.at<uint8_t>(2, 1), 1u);
  EXPECT_EQ(adjacency_matrix.at<uint8_t>(2, 2), 0u);

  // Check that topology is consistent
  cv::Mat polygons_mesh;
  mesh_2d.convertPolygonsMeshToMat(&polygons_mesh);
  ASSERT_EQ(polygons_mesh.rows, 4u);
  ASSERT_EQ(polygons_mesh.cols, 1u);
  EXPECT_EQ(polygons_mesh.at<uint8_t>(0, 0), 3u);
  EXPECT_EQ(polygons_mesh.at<uint8_t>(1, 0), 0u);
  EXPECT_EQ(polygons_mesh.at<uint8_t>(2, 0), 1u);
  EXPECT_EQ(polygons_mesh.at<uint8_t>(3, 0), 2u);

  // Check that lmk ids are correct
  LandmarkIds actual_lmk_ids = mesh_2d.getLandmarkIds();
  ASSERT_EQ(actual_lmk_ids.size(), 3u);
  EXPECT_EQ(actual_lmk_ids[0], lmk_ids[0]);
  EXPECT_EQ(actual_lmk_ids[1], lmk_ids[1]);
  EXPECT_EQ(actual_lmk_ids[2], lmk_ids[2]);

  // Check that vtx ids are correct
  Mesh2D::VertexId vtx_id = 0u;
  ASSERT_TRUE(mesh_2d.getVtxIdForLmkId(lmk_ids[0], &vtx_id));
  EXPECT_EQ(vtx_id, 0u);
  ASSERT_TRUE(mesh_2d.getVtxIdForLmkId(lmk_ids[1], &vtx_id));
  EXPECT_EQ(vtx_id, 1u);
  ASSERT_TRUE(mesh_2d.getVtxIdForLmkId(lmk_ids[2], &vtx_id));
  EXPECT_EQ(vtx_id, 2u);

  // Check that vertices_mat is ok and consistent with vtx_ids
  cv::Mat vertices_mesh;
  mesh_2d.convertVerticesMeshToMat(&vertices_mesh);
  ASSERT_EQ(vertices_mesh.rows, 3u);
  ASSERT_EQ(vertices_mesh.cols, 2u);
  EXPECT_EQ(vertices_mesh.at<uint8_t>(0, 0), 1.1);
  EXPECT_EQ(vertices_mesh.at<uint8_t>(0, 1), 1.2);
  EXPECT_EQ(vertices_mesh.at<uint8_t>(1, 0), 1.2);
  EXPECT_EQ(vertices_mesh.at<uint8_t>(1, 1), 1.4);
  EXPECT_EQ(vertices_mesh.at<uint8_t>(2, 0), 1.3);
  EXPECT_EQ(vertices_mesh.at<uint8_t>(2, 1), 1.6);
}

}  // namespace VIO
