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

#include "mesh/Mesh.h"
#include <glog/logging.h>
#include <opencv2/core/core.hpp>

namespace VIO {

template<typename VertexPositionType>
Mesh<VertexPositionType>::Mesh(const size_t& polygon_dimension)
  : vertex_to_lmk_id_map_(),
    lmk_id_to_vertex_map_(),
    vertices_mesh_(0, 1, CV_32FC3),
    polygons_mesh_(0, 1, CV_32SC1),
    polygon_dimension_(polygon_dimension) {
  CHECK_GE(polygon_dimension, 3) << "A polygon must have more than 2"
                                    " vertices";
}

/* -------------------------------------------------------------------------- */
template<typename VertexPositionType>
Mesh<VertexPositionType>::Mesh(const Mesh<VertexPositionType>& rhs_mesh)
  : vertex_to_lmk_id_map_(rhs_mesh.vertex_to_lmk_id_map_),
    lmk_id_to_vertex_map_(rhs_mesh.lmk_id_to_vertex_map_),
    vertices_mesh_(rhs_mesh.vertices_mesh_.clone()), // CLONING!
    polygons_mesh_(rhs_mesh.vertices_mesh_.clone()), // CLONING!
    polygon_dimension_(rhs_mesh.polygon_dimension_) {
  VLOG(2) << "You are calling the copy ctor for a mesh... Cloning data.";
}

/* -------------------------------------------------------------------------- */
template<typename VertexPositionType>
Mesh<VertexPositionType>& Mesh<VertexPositionType>::operator=(
    const Mesh<VertexPositionType>& rhs_mesh) {
  // Check for self-assignment.
  if(&rhs_mesh == this)
    return *this;
  CHECK_EQ(polygon_dimension_, rhs_mesh.polygon_dimension_)
      << "The Mesh that you are trying to copy has different dimensions"
      << " for the polygons!";
  // Deep copy internal data.
  lmk_id_to_vertex_map_ = rhs_mesh.lmk_id_to_vertex_map_;
  vertex_to_lmk_id_map_ = rhs_mesh.vertex_to_lmk_id_map_;
  vertices_mesh_ = rhs_mesh.vertices_mesh_.clone();
  polygons_mesh_ = rhs_mesh.polygons_mesh_.clone();
  return *this;
}

/* -------------------------------------------------------------------------- */
template<typename VertexPositionType>
void Mesh<VertexPositionType>::addPolygonToMesh(const Polygon& polygon) {
  // Update mesh connectivity (this does duplicate polygons, adding twice the
  // same polygon is permitted, although it should not for efficiency).
  // This cannot be avoided by just checking that at least one of the
  // vertices of the wanna-be polygon is new, as a polygon that has no new
  // vertices can be linking in a new way three old vertices.
  CHECK_EQ(polygon.size(), polygon_dimension_)
      << "Trying to insert a polygon of different dimension than "
      << "the mesh's polygons.\n"
      << "Polygon dimension: " << polygon.size() << "\n"
      << "Mesh expected polygon dimension: " << polygon_dimension_ << ".\n";
  // Specify number of point ids per face in the mesh.
  polygons_mesh_.push_back(static_cast<int>(polygon_dimension_));
  // Loop over each vertex in the given polygon.
  for (const VertexType& vertex: polygon) {
    // Add or update vertex in the mesh, and encode its connectivity in the mesh.
    updateMeshDataStructures(vertex.getLmkId(),
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
// Provides the id of the row where the new/updated vertex is in the vertices_mesh
// data structure.
template<typename VertexPositionType>
void Mesh<VertexPositionType>::updateMeshDataStructures(
    const LandmarkId& lmk_id,
    const VertexPositionType& lmk_position,
    std::map<VertexId, LandmarkId>* vertex_to_lmk_id_map,
    std::map<LandmarkId, VertexId>* lmk_id_to_vertex_map,
    cv::Mat* vertices_mesh,
    cv::Mat* polygon_mesh) const {
  CHECK_NOTNULL(vertex_to_lmk_id_map);
  CHECK_NOTNULL(lmk_id_to_vertex_map);
  CHECK_NOTNULL(vertices_mesh);
  CHECK_NOTNULL(polygon_mesh);

  const auto& lmk_id_to_vertex_map_end = lmk_id_to_vertex_map->end();
  const auto& vertex_it = lmk_id_to_vertex_map->find(lmk_id);

  int row_id_vertex;
  // Check whether this landmark is already in the set of vertices of the
  // mesh.
  if (vertex_it == lmk_id_to_vertex_map_end) {
    // New landmark, create a new entrance in the set of vertices.
    // Store 3D points in map_points_3d.
    vertices_mesh->push_back(lmk_position);
    row_id_vertex = vertices_mesh->rows - 1;
    // Book-keeping.
    // Store the row in the vertices structure of this new landmark id.
    (*lmk_id_to_vertex_map)[lmk_id] = row_id_vertex;
    (*vertex_to_lmk_id_map)[row_id_vertex] = lmk_id;
  } else {
    // Update old landmark with new position.
    vertices_mesh->at<VertexPositionType>(vertex_it->second) = lmk_position;
    row_id_vertex = vertex_it->second;
  }
  // Store corresponding ids (row index) to the 3d point in map_points_3d.
  // This structure encodes the connectivity of the mesh:
  polygon_mesh->push_back(row_id_vertex);
}

/* -------------------------------------------------------------------------- */
// Get a polygon in the mesh.
// Returns false if there is no polygon.
template<typename VertexPositionType>
bool Mesh<VertexPositionType>::getPolygon(const size_t& polygon_idx, Polygon* polygon) const {
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
    const VertexPositionType& point_j  = vertices_mesh_.at<VertexPositionType>(row_id_pt_j);
    polygon->at(j) = Vertex<VertexPositionType>(lmk_id_j, point_j);
  }
  return true;
}

/* -------------------------------------------------------------------------- */
// Retrieve a vertex of the mesh given a LandmarkId.
// Returns true if we could find the vertex with the given landmark id
// false otherwise.
template<typename VertexPositionType>
bool Mesh<VertexPositionType>::getVertex(const LandmarkId& lmk_id,
                                         VertexPositionType* vertex) const {
  CHECK_NOTNULL(vertex);
  const auto& lmk_id_to_vertex_map_end = lmk_id_to_vertex_map_.end();
  const auto& vertex_it = lmk_id_to_vertex_map_.find(lmk_id);
  if (vertex_it == lmk_id_to_vertex_map_end) {
    // We didn't find the lmk id!
    VLOG(100) << "Lmk id: " << lmk_id << " not found in mesh.";
    return false;
  } else {
    // Return the vertex.
    *vertex = vertices_mesh_.at<VertexPositionType>(vertex_it->second);
    return true; // Meaning we found the vertex.
  }
}

/* -------------------------------------------------------------------------- */
template<typename VertexPositionType>
void Mesh<VertexPositionType>::convertVerticesMeshToMat(cv::Mat* vertices_mesh) const {
  CHECK_NOTNULL(vertices_mesh);
  *vertices_mesh = vertices_mesh_.clone();
}

/* -------------------------------------------------------------------------- */
template<typename VertexPositionType>
void Mesh<VertexPositionType>::convertPolygonsMeshToMat(cv::Mat* polygons_mesh) const {
  CHECK_NOTNULL(polygons_mesh);
  *polygons_mesh = polygons_mesh_.clone();
}

/* -------------------------------------------------------------------------- */
// Reset all data structures of the mesh.
template<typename VertexPositionType>
void Mesh<VertexPositionType>::clearMesh() {
  vertices_mesh_ = cv::Mat(0, 1, CV_32FC3);
  polygons_mesh_ = cv::Mat(0, 1, CV_32SC1);
  vertex_to_lmk_id_map_.clear();
  lmk_id_to_vertex_map_.clear();
}

// explicit instantiations
template class Mesh<Vertex2DType>;
template class Mesh<Vertex3DType>;

} // End of VIO namespace.
