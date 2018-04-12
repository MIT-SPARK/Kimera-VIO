/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Mesher.h
 * @brief  Represents a Mesh of polygons in 3D
 * @author Antoni Rosinol
 */

#pragma once

#include <map>
#include <vector>
#include <opencv2/core/mat.hpp>

#include "UtilsOpenCV.h"

// Class defining the concept of a polygonal mesh.
namespace VIO {
class Mesh3D {
public:
  // Default constructor.
  Mesh3D(const size_t& polygon_dimension = 3);

  // Delete copy constructor.
  Mesh3D(const Mesh3D& rhs_mesh) = delete;

  // Copy assignement operator.
  // Performs a deep copy (clones) the data members!
  Mesh3D& operator=(const Mesh3D& mesh);

  // Delete move constructor.
  Mesh3D(Mesh3D&& mesh) = delete;

  // Delete move assignement operator.
  Mesh3D& operator=(Mesh3D&& mesh) = delete;

  // Destructor.
  ~Mesh3D() = default;

public:
  typedef int VertexId;
  typedef cv::Point3f VertexPosition3D;

  typedef std::map<VertexId, LandmarkId> VertexToLmkIdMap;
  typedef std::map<LandmarkId, VertexId> LmkIdToVertexMap;

  struct MeshVertex {
  public:
    MeshVertex()
      : lmk_id_(-1),
        vertex_position_() {}

    MeshVertex(const LandmarkId& lmk_id,
               const VertexPosition3D& vertex_position)
      : lmk_id_(lmk_id),
        vertex_position_(vertex_position) {}

    MeshVertex(const MeshVertex& rhs_mesh_vertex) = default;
    MeshVertex& operator=(const MeshVertex& rhs_mesh_vertex) = default;

    MeshVertex(MeshVertex&& rhs_mesh_vertex) = default;
    MeshVertex& operator=(MeshVertex&& rhs_mesh_vertex) = default;

    ~MeshVertex() = default;

    /// Getters
    inline const VertexPosition3D getVertexPosition() const {
      return vertex_position_;
    }
    inline const LandmarkId getLandmarkId() const {
      return lmk_id_;
    }

  private:
    /// Members
    LandmarkId lmk_id_;
    VertexPosition3D vertex_position_;
  };
  typedef std::vector<MeshVertex> Polygon;

public:

  void addPolygonToMesh(const Polygon& polygon);

  void clearMesh();

  /// Getters
  inline size_t getNumberOfPolygons() const {
    return static_cast<size_t>(polygons_mesh_.rows / (polygon_dimension_ + 1));
  }
  inline size_t getMeshPolygonDimension() const {
    return polygon_dimension_;
  }

  void getVerticesMesh(cv::Mat* vertices_mesh) const;
  void getPolygonsMesh(cv::Mat* polygons_mesh) const;

  bool getPolygon(const size_t& polygon_idx, Polygon* polygon);

private:
  /// Members
  // Bimap.
  // Vertex to LmkId Map
  VertexToLmkIdMap vertex_to_lmk_id_map_;

  // LmkId to Vertex Map
  LmkIdToVertexMap lmk_id_to_vertex_map_;

  // Vertices 3D. // map of lmkid to vertex
  // Set of (non-repeated) points = valid landmark positions.
  // Format: n rows (one for each n points), with each row being a cv::Point3f.
  cv::Mat vertices_mesh_;

  // Connectivity.
  // Set of polygons.
  cv::Mat polygons_mesh_;

  // Number of vertices per polygon.
  const size_t polygon_dimension_;

  /// Functions
  void updateMeshDataStructures(
      const LandmarkId& lmk_id,
      const VertexPosition3D& lmk_position,
      std::map<VertexId, LandmarkId>* vertex_to_lmk_id_map,
      std::map<LandmarkId, VertexId>* lmk_id_to_vertex_map,
      cv::Mat* vertices_mesh,
      cv::Mat* polygon_mesh) const;
};

} // End of VIO namespace.
