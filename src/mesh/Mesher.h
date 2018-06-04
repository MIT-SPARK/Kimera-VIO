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
#include "mesh/Mesh3D.h"

#include <stdlib.h>

#include <opengv/point_cloud/methods.hpp>

#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz/vizcore.hpp>

namespace VIO {
class Mesher {
public:
  Mesher()
    : mesh_() {}

  /* ------------------------------------------------------------------------ */
  // Update mesh: update structures keeping memory of the map before visualization
  void updateMesh3D(
      const std::unordered_map<LandmarkId, gtsam::Point3>& pointsWithIdVIO,
      std::shared_ptr<StereoFrame> stereoFrame,
      const gtsam::Pose3& leftCameraPose,
      const float& maxGradInTriangle = 50,
      const double& minRatioBetweenLargestAnSmallestSide = 0,
      const double& min_elongation_ratio = 0.5,
      const double& maxTriangleSide = 10,
      const bool& visualize = true);

  /* ------------------------------------------------------------------------ */
  // Cluster planes from the mesh.
  void clusterPlanesFromMesh(std::vector<Plane>* planes) const;

  /* ------------------------------------------------------------------------ */
  // Clones underlying data structures encoding the mesh.
  void getVerticesMesh(cv::Mat* vertices_mesh) const;
  void getPolygonsMesh(cv::Mat* polygons_mesh) const;

private:
  Mesh3D mesh_;

private:
  /* ------------------------------------------------------------------------ */
  // Reduce the 3D mesh to the current VIO lmks only.
  void reducePolygonMeshToTimeHorizon(
      const std::unordered_map<LandmarkId, gtsam::Point3>& points_with_id_map,
      const gtsam::Pose3& leftCameraPose,
      double min_ratio_largest_smallest_side,
      double max_triangle_side);


  /* ------------------------------------------------------------------------ */
  // For a triangle defined by the 3d points p1, p2, and p3
  // compute ratio between largest side and smallest side (how elongated it is).
  double getRatioBetweenSmallestAndLargestSide(
      const double& d12,
      const double& d23,
      const double& d31,
      boost::optional<double &> minSide_out = boost::none,
      boost::optional<double &> maxSide_out = boost::none) const;

  /* ------------------------------------------------------------------------ */
  // For a triangle defined by the 3d points p1, p2, and p3
  // compute ratio between largest side and smallest side (how elongated it is)
  double getRatioBetweenTangentialAndRadialDisplacement(
      const Mesh3D::VertexPosition3D& p1,
      const Mesh3D::VertexPosition3D& p2,
      const Mesh3D::VertexPosition3D& p3,
      const gtsam::Pose3& leftCameraPose) const;

  /* ------------------------------------------------------------------------ */
  // Try to reject bad triangles, corresponding to outliers.
  void filterOutBadTriangles(const gtsam::Pose3& leftCameraPose,
                             double minRatioBetweenLargestAnSmallestSide,
                             double min_elongation_ratio,
                             double maxTriangleSide);

  /* -------------------------------------------------------------------------- */
  // Create a 3D mesh from a 2d mesh in pixel coordinates.
  // The 3D mesh is constructed by finding the 3D landmark corresponding to the
  // pixel in the 2d mesh. The correspondence is found using the frame parameter.
  // The 3D mesh contains, at any given time, only points that are in
  // points_with_id_map.
  void populate3dMeshTimeHorizon(const std::vector<cv::Vec6f>& mesh_2d,
      const std::unordered_map<LandmarkId, gtsam::Point3>& points_with_id_map,
      const Frame& frame,
      const gtsam::Pose3& leftCameraPose,
      double min_ratio_largest_smallest_side,
      double min_elongation_ratio,
      double max_triangle_side);

  /* ------------------------------------------------------------------------ */
  // Calculate normals of each polygon in the mesh.
  void calculateNormals(std::vector<cv::Point3f>* normals);

  /* ------------------------------------------------------------------------ */
  // Calculate normal of a triangle, and return whether it was possible or not.
  // Calculating the normal of aligned points in 3D is not possible...
  bool calculateNormal(const Mesh3D::VertexPosition3D& p1,
                       const Mesh3D::VertexPosition3D& p2,
                       const Mesh3D::VertexPosition3D& p3,
                       cv::Point3f* normal) const;

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
  void clusterAtDistanceFromPlane(const double& plane_distance,
                                  const cv::Point3f& plane_normal,
                                  const double& distance_tolerance,
                                  TriangleCluster* triangle_cluster) const;

  /* ------------------------------------------------------------------------ */
  // Try to reject bad triangles, corresponding to outliers.
  bool isBadTriangle(const Mesh3D::Polygon& polygon,
                     const gtsam::Pose3& left_camera_pose,
                     const double& min_ratio_between_largest_an_smallest_side,
                     const double& min_elongation_ratio,
                     const double& max_triangle_side) const;

  /* ------------------------------------------------------------------------ */
  // Perform Mesh clustering of triangles on a plane given by a normal and a
  // distance to the origin.
  void clusterTrianglesOnPlane(TriangleCluster* clusters,
                               const gtsam::Unit3& plane_normal,
                               const double& plane_distance,
                               const double& normal_tolerance,
                               const double& distance_tolerance) const;

  /* ------------------------------------------------------------------------ */
  // Segment planes in the mesh, by using initial plane seeds.
  void segmentPlanesInMesh(std::vector<Plane>* seed_planes,
                           const double& normal_tolerance,
                           const double& distance_tolerance) const;

  /* ------------------------------------------------------------------------ */
  // Segment planes in the mesh, without having initial plane seeds.
  void segmentPlanesInMeshNaive(std::vector<Plane>* segmented_planes) const;

  /* ------------------------------------------------------------------------ */
  // Data association between planes.
  void associatePlanes(const std::vector<Plane>& segmented_planes,
                       std::vector<Plane>* planes,
                       const double& normal_tolerance,
                       const double& distance_tolerance) const;

  /* ------------------------------------------------------------------------ */
  // Extract lmk ids from a vector of triangle clusters.
  void extractLmkIdsFromVectorOfTriangleClusters(
      const std::vector<TriangleCluster>& triangle_cluster,
      LandmarkIds* lmk_ids) const;

  /* ------------------------------------------------------------------------ */
  // Extract lmk ids from triangle cluster.
  void extractLmkIdsFromTriangleCluster(const TriangleCluster& triangle_cluster,
                                        LandmarkIds* lmk_ids) const;
};

} // namespace VIO

#endif /* Mesher_H_ */


