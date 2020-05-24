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
    Mesh2D::VertexType vertex(
        lmk_id,
        Vertex2D(1.0f + static_cast<float>(lmk_id),
                 1.0f + 2.0f * static_cast<float>(lmk_id)));
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
  mesh_2d.getPolygonsMeshToMat(&polygons_mesh);
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
  mesh_2d.getVerticesMeshToMat(&vertices_mesh);
  ASSERT_EQ(vertices_mesh.rows, 3u);
  ASSERT_EQ(vertices_mesh.cols, 1u);
  EXPECT_EQ(vertices_mesh.at<Vertex2D>(0, 0), Vertex2D(2.0, 3.0));
  EXPECT_EQ(vertices_mesh.at<Vertex2D>(1, 0), Vertex2D(3.0, 5.0));
  EXPECT_EQ(vertices_mesh.at<Vertex2D>(2, 0), Vertex2D(4.0, 7.0));
}

TEST_F(MeshFixture, addPolygonNominalMesh) {
  // Add polygon to the mesh and check that all internal datastructures
  // are correctly populated and coherent
  //  *--*--*
  //  | /| /|
  //  |/ |/ |
  //  *--*--*
  //  | /| /|
  //  |/ |/ |
  //  *--*--*
  //
  //  Lmk ids
  //  1--2--5
  //  |a/|c/|
  //  |/b|/d|
  //  3--4--6
  //  |e/|g/|
  //  |/f|/h|
  //  7--8--9

  //  Vtx ids (depends on which triangles where added first!)
  //  0--1--4
  //  |a/|c/|
  //  |/b|/d|
  //  2--3--5
  //  |e/|g/|
  //  |/f|/h|
  //  6--7--8
  //
  Mesh2D mesh_2d;
  Mesh2D::Polygon polygon;
  LandmarkIds lmk_ids = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u};
  // clang-format off
  LandmarkIds lmk_ids_connect = {1u, 2u, 3u,
                                 1u, 2u, 3u, // some duplicated faces
                                 2u, 3u, 4u,
                                 2u, 4u, 5u,
                                 4u, 5u, 6u,
                                 3u, 4u, 7u,
                                 3u, 4u, 7u, // some duplicated faces
                                 4u, 7u, 8u,
                                 4u, 6u, 8u,
                                 6u, 8u, 9u};
  // clang-format on

  // Generate mesh
  for (const auto& lmk_id : lmk_ids_connect) {
    // Use as vertex position a "random" one (just adding lmk_id).
    Mesh2D::VertexType vertex(
        lmk_id,
        Vertex2D(1.0f + static_cast<float>(lmk_id),
                 1.0f + 2.0f * static_cast<float>(lmk_id)));
    polygon.push_back(vertex);

    if (polygon.size() == 3) {
      mesh_2d.addPolygonToMesh(polygon);
      polygon.clear();
    }
  }

  // Check non-duplicated
  cv::Mat actual_adjacency_matrix = mesh_2d.getAdjacencyMatrix();
  ASSERT_EQ(actual_adjacency_matrix.rows, actual_adjacency_matrix.cols);
  ASSERT_EQ(actual_adjacency_matrix.rows, 9u);

  // Check actual entries
  cv::Mat expected_adjacency_matrix
      // clang-format off
                                 // 0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u
      =  (cv::Mat_<uint8_t>(9,9) << 0u, 1u, 1u, 0u, 0u, 0u, 0u, 0u, 0u,
                                    1u, 0u, 1u, 1u, 1u, 0u, 0u, 0u, 0u,
                                    1u, 1u, 0u, 1u, 0u, 0u, 1u, 0u, 0u,
                                    0u, 1u, 1u, 0u, 1u, 1u, 1u, 1u, 0u,
                                    0u, 1u, 0u, 1u, 0u, 1u, 0u, 0u, 0u,
                                    0u, 0u, 0u, 1u, 1u, 0u, 0u, 1u, 1u,
                                    0u, 0u, 1u, 1u, 0u, 0u, 0u, 1u, 0u,
                                    0u, 0u, 0u, 1u, 0u, 1u, 1u, 0u, 1u,
                                    0u, 0u, 0u, 0u, 0u, 1u, 0u, 1u, 0u);
  // clang-format on

  // Check adjacency matrices match
  EXPECT_EQ(
      cv::countNonZero(expected_adjacency_matrix != actual_adjacency_matrix),
      0);

  // Check that topology is consistent
  cv::Mat actual_polygons_mesh;
  mesh_2d.getPolygonsMeshToMat(&actual_polygons_mesh);
  ASSERT_EQ(actual_polygons_mesh.rows,
            4u * 8u);  // 8 polygons (each needs 4 entries)
  ASSERT_EQ(actual_polygons_mesh.cols, 1u);

  // clang-format off
  cv::Mat expected_polygons_mesh = (cv::Mat_<int>(4u * 8u, 1u) <<
    3, 0, 1, 2, 3, 1, 2, 3, 3, 1, 3, 4, 3, 3, 4, 5,
    3, 2, 3, 6, 3, 3, 6, 7, 3, 3, 5, 7, 3, 5, 7, 8);
  // clang-format on

  EXPECT_EQ(cv::countNonZero(expected_polygons_mesh != actual_polygons_mesh),
            0);

  // Check that lmk ids are correct
  LandmarkIds actual_lmk_ids = mesh_2d.getLandmarkIds();
  ASSERT_EQ(actual_lmk_ids.size(), lmk_ids.size());
  for (size_t i = 0u; i < lmk_ids.size(); i++) {
    EXPECT_EQ(actual_lmk_ids[i], lmk_ids[i]);
  }

  // Check that vtx ids are correct
  Mesh2D::VertexId vtx_id = 0u;
  for (size_t i = 0u; i < lmk_ids.size(); i++) {
    EXPECT_EQ(actual_lmk_ids[i], lmk_ids[i]);
    ASSERT_TRUE(mesh_2d.getVtxIdForLmkId(lmk_ids[i], &vtx_id));
    // We have set the lmk_ids to start on 1 so that vtx_id is not exactly
    // lmk_id, to make sure everything works as expected.
    EXPECT_EQ(vtx_id, lmk_ids[i] - 1u);
  }

  // Check that vertices_mat is ok and consistent with vtx_ids
  cv::Mat vertices_mesh;
  mesh_2d.getVerticesMeshToMat(&vertices_mesh);
  ASSERT_EQ(vertices_mesh.rows, 9u);
  ASSERT_EQ(vertices_mesh.cols, 1u);
  EXPECT_EQ(vertices_mesh.at<Vertex2D>(0, 0), Vertex2D(2.0, 3.0));
  EXPECT_EQ(vertices_mesh.at<Vertex2D>(1, 0), Vertex2D(3.0, 5.0));
  EXPECT_EQ(vertices_mesh.at<Vertex2D>(2, 0), Vertex2D(4.0, 7.0));
}

}  // namespace VIO
