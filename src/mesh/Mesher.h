/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Mesher.h
 * @brief  Build and visualize 2D mesh from Frame
 * @author Luca Carlone, AJ Haeffner, Antoni Rosinol
 */

#ifndef Mesher_H_
#define Mesher_H_

#include "StereoFrame.h"

#include <stdlib.h>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>

#include <opengv/point_cloud/methods.hpp>

#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz/vizcore.hpp>

namespace VIO {

// Class defining the concept of a polygonal mesh.
class Mesh3D {
public:
  typedef int VertexId;
  typedef cv::Point3f VertexPosition3D;

  typedef std::map<VertexId, LandmarkId> VertexToLmkIdMap;
  typedef std::map<LandmarkId, VertexId> LmkIdToVertexMap;

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

  void addPolygonToMesh(const Polygon& polygon);

  void clearMesh();

  /// Getters
  inline size_t getNumberOfPolygons() const {
    return std::round(polygons_mesh_.rows / (polygon_dimension_ + 1));
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

class Mesher {
public:

  // map a lmk id to a row in map_points_3d_
  using LandmarkIdToMapPointId = std::unordered_map<LandmarkId, size_t>;

  // map a keypoint (without lmk id) to a row in map_points_3d_
  using KeypointToMapPointId = std::vector<std::pair<KeypointCV, size_t>>;

  // Maps a lmk id to its corresponding row in map_points_3d_.
  LandmarkIdToMapPointId lmk_id_to_map_point_id_;
  KeypointToMapPointId keypoint_to_map_point_id_;

  Mesher()
    : mesh_() {}


  /* ------------------------------------------------------------------------ */
  // Reduce the 3D mesh to the current VIO lmks only.
  void reducePolygonMeshToTimeHorizon(
      const std::map<LandmarkId, gtsam::Point3>& points_with_id_map);

  /* ------------------------------------------------------------------------ */
  // Update mesh: update structures keeping memory of the map before visualization
  void updateMesh3D(
      const std::vector<std::pair<LandmarkId, gtsam::Point3>>& pointsWithIdVIO,
      std::shared_ptr<StereoFrame> stereoFrame,
      const gtsam::Pose3& leftCameraPose,
      const Mesh2Dtype& mesh2Dtype = Mesh2Dtype::VALIDKEYPOINTS,
      const float& maxGradInTriangle = 50,
      const double& minRatioBetweenLargestAnSmallestSide = 0,
      const double& min_elongation_ratio = 0.5,
      const double& maxTriangleSide = 10);


  /* ------------------------------------------------------------------------ */
  // Perform Mesh clustering.
  void clusterMesh(std::vector<TriangleCluster>* clusters);

  /* ------------------------------------------------------------------------ */
  // Extract lmk ids from triangle cluster.
  void extractLmkIdsFromTriangleCluster(const TriangleCluster& triangle_cluster,
                                        LandmarkIds* lmk_ids);

  void getVerticesMesh(cv::Mat* vertices_mesh) const;
  void getPolygonsMesh(cv::Mat* polygons_mesh) const;

private:
  Mesh3D mesh_;

private:
  /* ------------------------------------------------------------------------ */
  // for a triangle defined by the 3d points mapPoints3d_.at(rowId_pt1), mapPoints3d_.at(rowId_pt2),
  // mapPoints3d_.at(rowId_pt3), compute ratio between largest side and smallest side (how elongated it is)
  double getRatioBetweenSmallestAndLargestSide(
      const Mesh3D::VertexPosition3D& p1,
      const Mesh3D::VertexPosition3D& p2,
      const Mesh3D::VertexPosition3D& p3,
      boost::optional<double &> d12_out,
      boost::optional<double &> d23_out,
      boost::optional<double &> d31_out,
      boost::optional<double &> minSide_out = boost::none,
      boost::optional<double &> maxSide_out = boost::none) const;

  /* ------------------------------------------------------------------------ */
  // for a triangle defined by the 3d points mapPoints3d_.at(rowId_pt1), mapPoints3d_.at(rowId_pt2),
  // mapPoints3d_.at(rowId_pt3), compute ratio between largest side and smallest side (how elongated it is)
  double getRatioBetweenTangentialAndRadialDisplacement(
      const Mesh3D::VertexPosition3D& p1,
      const Mesh3D::VertexPosition3D& p2,
      const Mesh3D::VertexPosition3D& p3,
      const gtsam::Pose3& leftCameraPose) const;

  /* ------------------------------------------------------------------------ */
  // Try to reject bad triangles, corresponding to outliers
  void filterOutBadTriangles(const gtsam::Pose3& leftCameraPose,
                             double minRatioBetweenLargestAnSmallestSide,
                             double min_elongation_ratio,
                             double maxTriangleSide);

  /* ------------------------------------------------------------------------ */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  void populate3dMesh(const std::vector<cv::Vec6f>& triangulation2D,
                                  const Frame& frame,
                                  cv::Mat* polygon) const;

  /* -------------------------------------------------------------------------- */
  // Create a 3D mesh from 2D corners in an image, coded as a Frame class
  void populate3dMeshTimeHorizon(
      const std::vector<cv::Vec6f>& triangulation2D,
      const std::map<LandmarkId, gtsam::Point3>& points_with_id_map,
      const Frame& frame);

  /* ------------------------------------------------------------------------ */
  // Calculate normals of each polygon in the mesh.
  void calculateNormals(std::vector<cv::Point3f>* normals);

  /* ------------------------------------------------------------------------ */
  // Calculate normal of a triangle.
  void calculateNormal(const Mesh3D::VertexPosition3D& p1,
                       const Mesh3D::VertexPosition3D& p2,
                       const Mesh3D::VertexPosition3D& p3,
                       cv::Point3f* normal);

  /* ------------------------------------------------------------------------ */
  // Is normal perpendicular to axis?
  bool isNormalPerpendicularToAxis(const cv::Point3f& axis,
                                   const cv::Point3f& normal,
                                   const double& tolerance) const;

  /* ------------------------------------------------------------------------ */
  // Is normal around axis?
  bool isNormalAroundAxis(const cv::Point3f& axis,
                          const cv::Point3f& normal,
                          const double& tolerance) const;

  /* ------------------------------------------------------------------------ */
  // Clusters normals given an axis, a set of normals and a
  // tolerance. The result is a vector of indices of the given set of normals
  // that are in the cluster.
  void clusterNormalsAroundAxis(const cv::Point3f& axis,
                                const std::vector<cv::Point3f>& normals,
                                const double& tolerance,
                                std::vector<int>* triangle_cluster);

  /* ------------------------------------------------------------------------ */
  // Clusters normals perpendicular to an axis. Given an axis, a set of normals
  // and a tolerance. The result is a vector of indices of the given set of
  // normals that are in the cluster.
  void clusterNormalsPerpendicularToAxis(const cv::Point3f& axis,
                                         const std::vector<cv::Point3f>& normals,
                                         const double& tolerance,
                                         std::vector<int>* cluster_normals_idx);

  /* ------------------------------------------------------------------------ */
  // Filter z component in triangle cluster.
  void clusterZComponent(
    const double& z,
    const double& tolerance,
    TriangleCluster* triangle_cluster);
};

} // namespace VIO

#endif /* Mesher_H_ */


