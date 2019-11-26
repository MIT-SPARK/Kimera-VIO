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

#include "kimera-vio/mesh/Mesh.h"

#include <glog/logging.h>

#include <opencv2/core/core.hpp>

namespace VIO {

/**
 * param[in]: polygon_dimension number of vertices per polygon (triangle = 3).
 */
template <typename VertexPositionType>
Mesh<VertexPositionType>::Mesh(const size_t& polygon_dimension)
    : vertex_to_lmk_id_map_(),
      lmk_id_to_vertex_map_(),
      vertices_mesh_(0, 1, CV_32FC3),
      vertices_mesh_normal_(0, 1),
      normals_computed_(false),
      vertices_mesh_color_(0, 1, CV_8UC3),
      polygons_mesh_(0, 1, CV_32SC1),
      polygon_dimension_(polygon_dimension) {
  CHECK_GE(polygon_dimension, 3) << "A polygon must have more than 2"
                                    " vertices";
}

/* -------------------------------------------------------------------------- */
template <typename VertexPositionType>
Mesh<VertexPositionType>::Mesh(const Mesh<VertexPositionType>& rhs_mesh)
    : vertex_to_lmk_id_map_(rhs_mesh.vertex_to_lmk_id_map_),
      lmk_id_to_vertex_map_(rhs_mesh.lmk_id_to_vertex_map_),
      vertices_mesh_(rhs_mesh.vertices_mesh_.clone()),  // CLONING!
      vertices_mesh_normal_(
          rhs_mesh.vertices_mesh_normal_.clone()),  // CLONING!
      normals_computed_(false),
      vertices_mesh_color_(rhs_mesh.vertices_mesh_color_.clone()),  // CLONING!
      polygons_mesh_(rhs_mesh.polygons_mesh_.clone()),              // CLONING!
      polygon_dimension_(rhs_mesh.polygon_dimension_) {
  VLOG(2) << "You are calling the copy ctor for a mesh... Cloning data.";
}

/* -------------------------------------------------------------------------- */
template <typename VertexPositionType>
Mesh<VertexPositionType>& Mesh<VertexPositionType>::operator=(
    const Mesh<VertexPositionType>& rhs_mesh) {
  // Check for self-assignment.
  if (&rhs_mesh == this) return *this;
  CHECK_EQ(polygon_dimension_, rhs_mesh.polygon_dimension_)
      << "The Mesh that you are trying to copy has different dimensions"
      << " for the polygons!";
  // Deep copy internal data.
  lmk_id_to_vertex_map_ = rhs_mesh.lmk_id_to_vertex_map_;
  vertex_to_lmk_id_map_ = rhs_mesh.vertex_to_lmk_id_map_;
  vertices_mesh_ = rhs_mesh.vertices_mesh_.clone();
  vertices_mesh_normal_ = rhs_mesh.vertices_mesh_normal_.clone();
  normals_computed_ = rhs_mesh.normals_computed_;
  vertices_mesh_color_ = rhs_mesh.vertices_mesh_color_.clone();
  polygons_mesh_ = rhs_mesh.polygons_mesh_.clone();
  return *this;
}

/* -------------------------------------------------------------------------- */
template <typename VertexPositionType>
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
  // Reset flag to know if normals are valid or not.
  normals_computed_ = false;
  // Specify number of point ids per face in the mesh.
  polygons_mesh_.push_back(static_cast<int>(polygon_dimension_));
  // Loop over each vertex in the given polygon.
  for (const VertexType& vertex : polygon) {
    // Add or update vertex in the mesh, and encode its connectivity in the
    // mesh.
    updateMeshDataStructures(vertex.getLmkId(), vertex.getVertexPosition(),
                             &vertex_to_lmk_id_map_, &lmk_id_to_vertex_map_,
                             &vertices_mesh_, &vertices_mesh_normal_,
                             &vertices_mesh_color_, &polygons_mesh_);
  }
}

/* -------------------------------------------------------------------------- */
// Updates mesh data structures incrementally, by adding new landmark
// if there was no previous id, or updating it if it was already present.
// Provides the id of the row where the new/updated vertex is in the
// vertices_mesh data structure.
template <typename VertexPositionType>
void Mesh<VertexPositionType>::updateMeshDataStructures(
    const LandmarkId& lmk_id, const VertexPositionType& lmk_position,
    std::map<VertexId, LandmarkId>* vertex_to_lmk_id_map,
    std::map<LandmarkId, VertexId>* lmk_id_to_vertex_map,
    cv::Mat* vertices_mesh, cv::Mat_<VertexNormal>* vertices_mesh_normal,
    cv::Mat* vertices_mesh_color, cv::Mat* polygon_mesh,
    const VertexColorRGB& vertex_color) const {
  CHECK_NOTNULL(vertex_to_lmk_id_map);
  CHECK_NOTNULL(lmk_id_to_vertex_map);
  CHECK_NOTNULL(vertices_mesh);
  CHECK_NOTNULL(vertices_mesh_normal);
  CHECK_NOTNULL(vertices_mesh_color);
  CHECK_NOTNULL(polygon_mesh);
  DCHECK(!normals_computed_) << "Normals should be invalidated before...";

  const auto& lmk_id_to_vertex_map_end = lmk_id_to_vertex_map->end();
  const auto& vertex_it = lmk_id_to_vertex_map->find(lmk_id);

  int row_id_vertex;
  // Check whether this landmark is already in the set of vertices of the
  // mesh.
  if (vertex_it == lmk_id_to_vertex_map_end) {
    // New landmark, create a new entrance in the set of vertices.
    // Store 3D points in map_points_3d.
    vertices_mesh->push_back(lmk_position);
    vertices_mesh_normal->push_back(VertexNormal());
    vertices_mesh_color->push_back(vertex_color);
    row_id_vertex = vertices_mesh->rows - 1;
    // Book-keeping.
    // Store the row in the vertices structure of this new landmark id.
    (*lmk_id_to_vertex_map)[lmk_id] = row_id_vertex;
    (*vertex_to_lmk_id_map)[row_id_vertex] = lmk_id;
  } else {
    // Update old landmark with new position.
    // But don't update the color information... Or should we?
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
// TODO(Toni) this is constructing polygons on the fly, but we should instead
// store polygons, not cv::Mats
template <typename VertexPositionType>
bool Mesh<VertexPositionType>::getPolygon(const size_t& polygon_idx,
                                          Polygon* polygon) const {
  CHECK_NOTNULL(polygon);
  if (polygon_idx >= getNumberOfPolygons()) {
    VLOG(10) << "Requested polygon number: " << polygon_idx
             << ". But there are only " << getNumberOfPolygons()
             << " polygons.";
    return false;
  };

  DCHECK_EQ(vertices_mesh_.rows, vertices_mesh_normal_.rows);
  DCHECK_EQ(vertices_mesh_.rows, vertices_mesh_color_.rows);
  size_t idx_in_polygon_mesh = polygon_idx * (polygon_dimension_ + 1);
  polygon->resize(polygon_dimension_);
  for (size_t j = 0; j < polygon_dimension_; j++) {
    const int32_t& row_id_pt_j =
        polygons_mesh_.at<int32_t>(idx_in_polygon_mesh + j + 1);
    DCHECK(vertex_to_lmk_id_map_.find(row_id_pt_j) !=
           vertex_to_lmk_id_map_.end());
    DCHECK_LT(row_id_pt_j, vertices_mesh_.rows);
    polygon->at(j) = Vertex<VertexPositionType>(
        vertex_to_lmk_id_map_.at(row_id_pt_j),
        vertices_mesh_.at<VertexPositionType>(row_id_pt_j),
        vertices_mesh_normal_.at<VertexNormal>(row_id_pt_j),
        vertices_mesh_color_.at<VertexColorRGB>(row_id_pt_j));
  }
  return true;
}

/* -------------------------------------------------------------------------- */
// Retrieve a vertex of the mesh given a LandmarkId.
// Returns true if we could find the vertex with the given landmark id
// false otherwise.
template <typename VertexPosition>
bool Mesh<VertexPosition>::getVertex(const LandmarkId& lmk_id,
                                     Vertex<VertexPosition>* vertex,
                                     VertexId* vertex_id) const {
  CHECK(vertex != nullptr || vertex_id != nullptr)
      << "No output requested, are your sure you want to use this function?";
  const auto& lmk_id_to_vertex_map_end = lmk_id_to_vertex_map_.end();
  const auto& vertex_it = lmk_id_to_vertex_map_.find(lmk_id);
  if (vertex_it == lmk_id_to_vertex_map_end) {
    // We didn't find the lmk id!
    VLOG(100) << "Lmk id: " << lmk_id << " not found in mesh.";
    return false;
  } else {
    // Construct and Return the vertex.
    const VertexId& vtx_id = vertex_it->second;
    DCHECK_EQ(vertices_mesh_.rows, vertices_mesh_normal_.rows);
    DCHECK_EQ(vertices_mesh_.rows, vertices_mesh_color_.rows);
    DCHECK_LT(vtx_id, vertices_mesh_.rows);
    if (vertex_id != nullptr) *vertex_id = vtx_id;
    if (vertex != nullptr)
      *vertex = Vertex<VertexPosition>(
          vertex_to_lmk_id_map_.at(vtx_id),
          vertices_mesh_.at<VertexPosition>(vtx_id),
          vertices_mesh_normal_.at<VertexNormal>(vtx_id),
          vertices_mesh_color_.at<VertexColorRGB>(vtx_id));
    return true;  // Meaning we found the vertex.
  }
}

/* -------------------------------------------------------------------------- */
// Retrieve per vertex normals of the mesh.
template <typename VertexPositionType>
void Mesh<VertexPositionType>::computePerVertexNormals() {
  CHECK_EQ(polygon_dimension_, 3) << "Normals are only valid for dim 3 meshes.";
  LOG_IF(ERROR, normals_computed_) << "Normals have been computed already...";

  size_t n_vtx = getNumberOfUniqueVertices();
  std::vector<int> counts(n_vtx, 0);

  // Set all per-vertex normals in mesh to 0, since we want to average per-face
  // normals.
  clearVertexNormals();

  // Walk through triangles and compute averaged vertex normals.
  Polygon polygon;
  for (size_t i = 0; i < getNumberOfPolygons(); i++) {
    CHECK(getPolygon(i, &polygon)) << "Could not retrieve polygon.";
    DCHECK_EQ(polygon.size(), 3);
    // TODO(Toni): it would be better if we could do a polygon.getNormal();
    const VertexPositionType& p1 = polygon.at(0).getVertexPosition();
    const VertexPositionType& p2 = polygon.at(1).getVertexPosition();
    const VertexPositionType& p3 = polygon.at(2).getVertexPosition();

    // Outward-facing normal.
    VertexPositionType v21(p2 - p1);
    VertexPositionType v31(p3 - p1);
    VertexNormal normal(v21.cross(v31));

    // Normalize.
    double norm = cv::norm(normal);
    DCHECK_GT(norm, 0.0);
    normal /= norm;

    // TODO(Toni): Store normals at this point on a per-face basis.

    // Sanity check
    static constexpr double epsilon = 1e-3;  // 2.5 degrees aperture.
    DCHECK_LE(std::fabs(v21.ddot(v31)), 1.0 - epsilon)
        << "Cross product of aligned vectors.";

    // Compute per vertex averaged normals.
    /// Indices of vertices
    const VertexId& p1_idx = lmk_id_to_vertex_map_.at(polygon.at(0).getLmkId());
    const VertexId& p2_idx = lmk_id_to_vertex_map_.at(polygon.at(1).getLmkId());
    const VertexId& p3_idx = lmk_id_to_vertex_map_.at(polygon.at(2).getLmkId());
    /// Sum of normals per vertex
    vertices_mesh_normal_.at<VertexNormal>(p1_idx) += normal;
    vertices_mesh_normal_.at<VertexNormal>(p2_idx) += normal;
    vertices_mesh_normal_.at<VertexNormal>(p3_idx) += normal;
    /// Increase counts of normals added per vertex
    counts[p1_idx]++;
    counts[p2_idx]++;
    counts[p3_idx]++;
  }

  DCHECK_EQ(counts.size(), vertices_mesh_normal_.rows);
  // Average and normalize normals.
  // clang-format off
  // #pragma omp parallel for num_threads(params.omp_num_threads) schedule(static, params.omp_chunk_size)
  // clang-format on
  for (int i = 0; i < vertices_mesh_normal_.rows; i++) {
    VertexNormal& normal = vertices_mesh_normal_.at<VertexNormal>(i);
    // Average
    normal /= counts[i];
    // Normalize
    double norm = cv::norm(normal);
    DCHECK_GT(norm, 0.0);
    normal /= norm;
  }
  return;
}

/* -------------------------------------------------------------------------- */
// Retrieve a vertex of the mesh given a LandmarkId.
// Returns true if we could find the vertex with the given landmark id
// false otherwise.
// NOT THREADSAFE.
template <typename VertexPositionType>
bool Mesh<VertexPositionType>::setVertexColor(
    const LandmarkId& lmk_id, const VertexColorRGB& vertex_color) {
  const auto& lmk_id_to_vertex_map_end = lmk_id_to_vertex_map_.end();
  const auto& vertex_it = lmk_id_to_vertex_map_.find(lmk_id);
  if (vertex_it == lmk_id_to_vertex_map_end) {
    // We didn't find the lmk id!
    VLOG(100) << "Lmk id: " << lmk_id << " not found in mesh.";
    return false;
  } else {
    // Color the vertex.
    vertices_mesh_color_.at<VertexColorRGB>(vertex_it->second) = vertex_color;
    return true;  // Meaning we found the vertex.
  }
}

/* -------------------------------------------------------------------------- */
template <typename VertexPositionType>
void Mesh<VertexPositionType>::convertVerticesMeshToMat(
    cv::Mat* vertices_mesh) const {
  CHECK_NOTNULL(vertices_mesh);
  *vertices_mesh = vertices_mesh_.clone();
}

/* -------------------------------------------------------------------------- */
template <typename VertexPositionType>
void Mesh<VertexPositionType>::convertPolygonsMeshToMat(
    cv::Mat* polygons_mesh) const {
  CHECK_NOTNULL(polygons_mesh);
  *polygons_mesh = polygons_mesh_.clone();
}

/* -------------------------------------------------------------------------- */
// Reset all data structures of the mesh.
template <typename VertexPositionType>
void Mesh<VertexPositionType>::clearMesh() {
  vertices_mesh_ = cv::Mat(0, 1, CV_32FC3);
  vertices_mesh_normal_ = cv::Mat_<VertexNormal>(0, 1);
  vertices_mesh_color_ = cv::Mat(0, 1, CV_8UC3);
  polygons_mesh_ = cv::Mat(0, 1, CV_32SC1);
  vertex_to_lmk_id_map_.clear();
  lmk_id_to_vertex_map_.clear();
}

// explicit instantiations
template class Mesh<Vertex2D>;
template class Mesh<Vertex3D>;

}  // namespace VIO
