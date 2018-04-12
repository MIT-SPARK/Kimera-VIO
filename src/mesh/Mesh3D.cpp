/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Mesh3D.cpp
 * @brief  Represents a Mesh of polygons in 3D
 * @author Antoni Rosinol
 */

#include "mesh/Mesh3D.h"
#include "glog/logging.h"
#include <opencv2/core/core.hpp>

namespace VIO {

Mesh3D::Mesh3D(const size_t& polygon_dimension)
  : vertex_to_lmk_id_map_(),
    lmk_id_to_vertex_map_(),
    vertices_mesh_(cv::Mat(0, 1, CV_32FC3)),
    polygons_mesh_(cv::Mat(0, 1, CV_32SC1)),
    polygon_dimension_(polygon_dimension) {
  CHECK_GE(polygon_dimension, 3) << "A polygon must have more than 2"
                                    " vertices";
}

/* -------------------------------------------------------------------------- */
Mesh3D& Mesh3D::operator=(const Mesh3D& rhs_mesh) {
  // Check for self-assignment.
  if(&rhs_mesh == this)
    return *this;
  CHECK_EQ(polygon_dimension_, rhs_mesh.polygon_dimension_)
      << "The Mesh that you are trying to copy has different dimensions"
      << " for the polygons!";
  lmk_id_to_vertex_map_ = rhs_mesh.lmk_id_to_vertex_map_;
  vertex_to_lmk_id_map_ = rhs_mesh.vertex_to_lmk_id_map_;
  vertices_mesh_ = rhs_mesh.vertices_mesh_.clone();
  polygons_mesh_ = rhs_mesh.polygons_mesh_.clone();
}

/* -------------------------------------------------------------------------- */
void Mesh3D::addPolygonToMesh(const Polygon& polygon) {
  // Update mesh connectivity, this might duplicate faces, but it does not
  // really matter visually. It does in terms of speed and memory...
  // Specify number of point ids per face in the mesh.
  // Currently 3, as we are dealing with a triangular 3d mesh.
  CHECK_EQ(polygon.size(), polygon_dimension_)
      << "Trying to insert a polygon of different dimension than "
      << "the mesh's polygons.\n"
      << "Polygon dimension: " << polygon.size() << "\n"
      << "Mesh expected polygon dimension: " << polygon_dimension_ << ".\n";
  polygons_mesh_.push_back(static_cast<int>(polygon_dimension_));
  for (const MeshVertex& vertex: polygon) {
    updateMeshDataStructures(vertex.getLandmarkId(),
                             vertex.getVertexPosition(),
                             &vertex_to_lmk_id_map_,
                             &lmk_id_to_vertex_map_,
                             &vertices_mesh_,
                             &polygons_mesh_);
  }
}

/* -------------------------------------------------------------------------- */
// Updates mesh data structures incrementally, by adding new landmark
// if there was no previous id, or updating it if it was already present.
void Mesh3D::updateMeshDataStructures(
                          const LandmarkId& lmk_id,
                          const VertexPosition3D& lmk_position,
                          std::map<VertexId, LandmarkId>* vertex_to_lmk_id_map,
                          std::map<LandmarkId, VertexId>* lmk_id_to_vertex_map,
                          cv::Mat* vertices_mesh,
                          cv::Mat* polygon_mesh) const {
  CHECK_NOTNULL(vertex_to_lmk_id_map);
  CHECK_NOTNULL(lmk_id_to_vertex_map);
  CHECK_NOTNULL(vertices_mesh);
  CHECK_NOTNULL(polygon_mesh);

  const auto& lmk_id_to_vertex_map_end = lmk_id_to_vertex_map->end();
  const auto& vertex_1_it = lmk_id_to_vertex_map->find(lmk_id);

  int row_id_pt_1;
  // Check whether this landmark is already in the set of vertices of the
  // mesh.
  if (vertex_1_it == lmk_id_to_vertex_map_end) {
    // New landmark, create a new entrance in the set of vertices.
    // Store 3D points in map_points_3d.
    vertices_mesh->push_back(lmk_position);
    row_id_pt_1 = vertices_mesh->rows - 1;
    // Store the row in the vertices structure of this new landmark id.
    (*lmk_id_to_vertex_map)[lmk_id] = row_id_pt_1;
    (*vertex_to_lmk_id_map)[row_id_pt_1] = lmk_id;
  } else {
    // Update old landmark with new position.
    vertices_mesh->at<cv::Point3f>(vertex_1_it->second) = lmk_position;
    row_id_pt_1 = vertex_1_it->second;
  }
  // Store corresponding ids (row index) to the 3d point in map_points_3d.
  // This structure encodes the connectivity of the mesh:
  polygon_mesh->push_back(row_id_pt_1);
}

/* -------------------------------------------------------------------------- */
// Get a polygon in the mesh.
// Returns false if there is no polygon.
bool Mesh3D::getPolygon(const size_t& polygon_idx, Polygon* polygon) {
  CHECK_NOTNULL(polygon);
  if (polygon_idx >= getNumberOfPolygons()) {
    VLOG(10) << "Requested polygon number: " << polygon_idx
             << ". But there are only " << getNumberOfPolygons()
             << " polygons.";
    return false;
  };

  size_t idx_in_polygon_mesh = polygon_idx * (polygon_dimension_ + 1);
  polygon->resize(polygon_dimension_);
  for (size_t j = 0; j < polygon_dimension_; j++) {
    const int32_t& row_id_pt_j =
        polygons_mesh_.at<int32_t>(idx_in_polygon_mesh + j + 1);
    const LandmarkId& lmk_id_j  = vertex_to_lmk_id_map_.at(row_id_pt_j);
    const cv::Point3f& point_j  = vertices_mesh_.at<cv::Point3f>(row_id_pt_j);
    polygon->at(j) = MeshVertex(lmk_id_j, point_j);
  }
  return true;
}

/* -------------------------------------------------------------------------- */
void Mesh3D::getVerticesMesh(cv::Mat* vertices_mesh) const {
  CHECK_NOTNULL(vertices_mesh);
  *vertices_mesh = vertices_mesh_.clone();
}
void Mesh3D::getPolygonsMesh(cv::Mat* polygons_mesh) const {
  CHECK_NOTNULL(polygons_mesh);
  *polygons_mesh = polygons_mesh_.clone();
}

/* -------------------------------------------------------------------------- */
// Reset all data structures of the mesh.
void Mesh3D::clearMesh() {
  vertices_mesh_ = cv::Mat(0, 1, CV_32FC3);
  polygons_mesh_ = cv::Mat(0, 1, CV_32SC1);
  vertex_to_lmk_id_map_.clear();
  lmk_id_to_vertex_map_.clear();
}

} // End of VIO namespace.
