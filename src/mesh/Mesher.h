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
  // For a triangle defined by the 3d points p1, p2, and p3
  // compute ratio between largest side and smallest side (how elongated it is)
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
  void populate3dMeshTimeHorizon(
      const std::vector<cv::Vec6f>& mesh_2d,
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


