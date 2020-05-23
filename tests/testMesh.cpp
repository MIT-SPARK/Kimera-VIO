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

#include <gtest/gtest.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include "kimera-vio/mesh/Mesh.h"

DECLARE_string(test_data_path);
DECLARE_bool(display);

namespace VIO {

class MeshFixture : public ::testing::Test {
 public:
  MeshFixture() { }

 protected:
  virtual void SetUp() override {}
  virtual void TearDown() override {}

 protected:
};

TEST_F(MeshFixture, addPolygon) {
  // Add polygon to the mesh and check that all internal datastructures
  // are correctly populated and coherent
  Mesh2D mesh_2d;
  Mesh2D::Polygon polygon;
  LandmarkIds lmk_ids = {1u, 2u, 3u};
  for (const auto& lmk_id: lmk_ids) {
    Mesh2D::VertexType vertex (lmk_id, Mesh2D::VertexPostionType(1.0, 1.0, 1.0));
    polygon.push_back(vertex);
  }
  mesh_2d.addPolygonToMesh(polygon);
}

}  // namespace VIO
