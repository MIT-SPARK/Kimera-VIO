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

namespace VIO {
// Class defining the concept of a polygonal mesh.
class Mesh3D {
public:
  // Default constructor.
  Mesh3D(const size_t& polygon_dimension = 3);

  // Delete copy constructor.
  Mesh3D(const Mesh3D& rhs_mesh) = delete;

  // Copy assignement operator.
  // Performs a deep copy (clones) the data members.
  Mesh3D& operator=(const Mesh3D& mesh);

  // Delete move constructor.
  Mesh3D(Mesh3D&& mesh) = delete;

  // Delete move assignement operator.
  Mesh3D& operator=(Mesh3D&& mesh) = delete;

  // Destructor.
  ~Mesh3D() = default;

public:
  typedef cv::Point3f VertexPosition3D;

private:
  // Vertex id (for internal processing).
  typedef int VertexId;
  // Maps (for internal processing).
  typedef std::map<VertexId, LandmarkId> VertexToLmkIdMap;
  typedef std::map<LandmarkId, VertexId> LmkIdToVertexMap;

public:
  struct Vertex {
  public:
    Vertex()
      : lmk_id_(-1),
        vertex_position_() {}

    Vertex(const LandmarkId& lmk_id,
           const VertexPosition3D& vertex_position)
      : lmk_id_(lmk_id),
        vertex_position_(vertex_position) {}

    // Make explicit that we are using the default copy constructor and
    // copy assignement operator.
    Vertex(const Vertex& rhs_mesh_vertex) = default;
    Vertex& operator=(const Vertex& rhs_mesh_vertex) = default;

    // Make explicit that we are using the default move constructor and
    // move assignement operator.
    Vertex(Vertex&& rhs_mesh_vertex) = default;
    Vertex& operator=(Vertex&& rhs_mesh_vertex) = default;

    // Default destructor.
    ~Vertex() = default;

    /// Getters.
    inline const VertexPosition3D& getVertexPosition() const {
      return vertex_position_;
    }
    inline const LandmarkId& getLmkId() const {
      return lmk_id_;
    }
    /// Setters.
    void setVertexPosition(const Mesh3D::VertexPosition3D& position) {
      vertex_position_ = position;
    }

  private:
    /// Members
    LandmarkId lmk_id_;
    VertexPosition3D vertex_position_;
  };
  // We define a polygon of the mesh as a set of mesh vertices.
  typedef std::vector<Vertex> Polygon;

public:
  // Adds a new polygon into the mesh, updates the internal data structures.
  void addPolygonToMesh(const Polygon& polygon);

  // Completely clears the mesh.
  void clearMesh();

  /// Getters
  inline size_t getNumberOfPolygons() const {
    return static_cast<size_t>(polygons_mesh_.rows / (polygon_dimension_ + 1));
  }
  // TODO needs to be generalized to aleatory polygonal meshes.
  // Currently it only allows polygons of same size.
  inline size_t getMeshPolygonDimension() const {
    return polygon_dimension_;
  }

  // Retrieve the mesh data structures.
  void convertVerticesMeshToMat(cv::Mat* vertices_mesh) const;
  void convertPolygonsMeshToMat(cv::Mat* polygons_mesh) const;

  // Retrieve a single polygon in the mesh.
  // Iterate over the total number of polygons (given by getNumberOfPolygons)
  // to retrieve one polygon at a time.
  bool getPolygon(const size_t& polygon_idx, Polygon* polygon) const;

private:
  /// TODO change internal structures for the mesh with std::vector<Polygon>.

  /// Members
  /// TODO maybe use bimap.
  // Vertex to LmkId Map
  VertexToLmkIdMap vertex_to_lmk_id_map_;

  // LmkId to Vertex Map
  LmkIdToVertexMap lmk_id_to_vertex_map_;

  // Vertices 3D.
  // Set of (non-repeated) 3d points.
  // Format: n rows (one for each point), with each row being a cv::Point3f.
  cv::Mat vertices_mesh_;

  // Connectivity of the mesh.
  // Set of polygons.
  // Raw integer list of the form: (n,id1_a,id2_a,...,idn_a,
  // n,id1_b,id2_b,...,idn_b, ..., n, ... idn_x)
  // where n is the number of points per polygon, and id is a zero-offset
  // index into the associated row in vertices_mesh_.
  cv::Mat polygons_mesh_;

  // Number of vertices per polygon.
  const size_t polygon_dimension_;

  /// Functions
  // Updates internal structures to add a vertex.
  // Used by addPolygonToMesh, it is not supposed to be used by the end user.
  void updateMeshDataStructures(
      const LandmarkId& lmk_id,
      const VertexPosition3D& lmk_position,
      std::map<VertexId, LandmarkId>* vertex_to_lmk_id_map,
      std::map<LandmarkId, VertexId>* lmk_id_to_vertex_map,
      cv::Mat* vertices_mesh,
      cv::Mat* polygon_mesh) const;
};

} // End of VIO namespace.
