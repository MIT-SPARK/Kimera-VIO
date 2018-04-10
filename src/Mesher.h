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

class Mesher {
public:
  // map a lmk id to a row in map_points_3d_
  using LandmarkIdToMapPointId = std::unordered_map<LandmarkId, size_t>;

  // map a keypoint (without lmk id) to a row in map_points_3d_
  using KeypointToMapPointId = std::vector<std::pair<KeypointCV, size_t>>;


  // Set of (non-repeated) points = valid landmark positions.
  // Format: n rows (one for each n points), with each row being a cv::Point3f.
  cv::Mat map_points_3d_;
  // Set of polygons.
  cv::Mat polygons_mesh_;

  // Maps a lmk id to its corresponding row in map_points_3d_.
  LandmarkIdToMapPointId lmk_id_to_map_point_id_;
  KeypointToMapPointId keypoint_to_map_point_id_;

  // Last id given to a point in map_points_3d_,
  // also represents number of points.
  size_t last_map_point_id_;

  Mesher()
    : map_points_3d_(cv::Mat(0, 1, CV_32FC3)),
      polygons_mesh_(cv::Mat(0, 1, CV_32SC1)),
      lmk_id_to_map_point_id_(),
      keypoint_to_map_point_id_(),
      last_map_point_id_(0) {}

  /* ------------------------------------------------------------------------ */
  // for a triangle defined by the 3d points mapPoints3d_.at(rowId_pt1), mapPoints3d_.at(rowId_pt2),
  // mapPoints3d_.at(rowId_pt3), compute ratio between largest side and smallest side (how elongated it is)
  double getRatioBetweenSmallestAndLargestSide(
      const cv::Mat& map_points_3d,
      const int rowId_pt1, const int rowId_pt2, const int rowId_pt3,
      boost::optional<double &> d12_out = boost::none,
      boost::optional<double &> d23_out = boost::none,
      boost::optional<double &> d31_out = boost::none,
      boost::optional<double &> minSide_out = boost::none,
      boost::optional<double &> maxSide_out = boost::none) const;

  /* ------------------------------------------------------------------------ */
  // Update map: update structures keeping memory of the map before visualization
  void updateMap3D(
    const std::vector<std::pair<LandmarkId, gtsam::Point3>>& points_with_id,
    const std::vector<std::pair<KeypointCV, gtsam::Point3>>& points_without_id =
                           std::vector<std::pair<KeypointCV, gtsam::Point3>>());

  /* ------------------------------------------------------------------------ */
  // Update map, just using VIO points.
  void updateMap3dTimeHorizon(
       const std::vector<std::pair<LandmarkId, gtsam::Point3>>& points_with_id);

  /* ------------------------------------------------------------------------ */
  // Reduce the 3D mesh to the current VIO lmks only.
  void reducePolygonMeshToTimeHorizon(
                  const std::map<int, LandmarkId>& vertex_to_lmk_id_map,
                  const cv::Mat& vertices_mesh,
                  const cv::Mat& polygon_mesh,
                  const std::map<LandmarkId, gtsam::Point3>& points_with_id_map,
                  std::map<int, LandmarkId>* vertex_to_lmk_id_map_output,
                  std::map<LandmarkId, int>* lmk_id_to_vertex_map_output,
                  cv::Mat* vertices_mesh_output,
                  cv::Mat* polygon_mesh_output);

  /* ------------------------------------------------------------------------ */
  // Update mesh: update structures keeping memory of the map before visualization
  std::map<int, LandmarkId> updateMesh3D(
      const std::vector<std::pair<LandmarkId, gtsam::Point3>>& pointsWithIdVIO,
      std::shared_ptr<StereoFrame> stereoFrame,
      const gtsam::Pose3& leftCameraPose,
      cv::Mat* map_points_3d,
      cv::Mat* polygons_mesh,
      const Mesh2Dtype& mesh2Dtype = Mesh2Dtype::VALIDKEYPOINTS,
      const float& maxGradInTriangle = 50,
      const double& minRatioBetweenLargestAnSmallestSide = 0,
      const double& min_elongation_ratio = 0.5,
      const double& maxTriangleSide = 10);


  /* ------------------------------------------------------------------------ */
  // Update mesh: update structures keeping memory of the map before visualization
  void updateMesh3D(std::vector<std::pair<LandmarkId, gtsam::Point3>> pointsWithId,
      Frame& frame,
      const gtsam::Pose3& leftCameraPose,
      const double& minRatioBetweenLargestAnSmallestSide = 0.0,
      const double& min_elongation_ratio = 0.5,
      const double& maxTriangleSide = 10.0);

  /* ------------------------------------------------------------------------ */
  // Perform Mesh clustering.
  void clusterMesh(
      const cv::Mat& map_points_3d,
      const cv::Mat& polygons_mesh,
      std::vector<TriangleCluster>* clusters);

  /* ------------------------------------------------------------------------ */
  // Extract lmk ids from triangle cluster.
  void extractLmkIdsFromTriangleCluster(
      const TriangleCluster& triangle_cluster,
      const std::map<int, LandmarkId>& vertex_to_id_map,
      const cv::Mat& polygons_mesh,
      LandmarkIds* lmk_ids);

  /* ------------------------------------------------------------------------ */
  // Filter z component in triangle cluster.
  void filterZComponent(
    const double& z,
    const double& tolerance,
    const cv::Mat& map_points_3d_,
    const cv::Mat& polygons_mesh,
    TriangleCluster* triangle_cluster);

private:
  /* ------------------------------------------------------------------------ */
    // for a triangle defined by the 3d points mapPoints3d_.at(rowId_pt1), mapPoints3d_.at(rowId_pt2),
  // mapPoints3d_.at(rowId_pt3), compute ratio between largest side and smallest side (how elongated it is)
  double getRatioBetweenTangentialAndRadialDisplacement(
      const cv::Mat& map_points_3d,
      const int rowId_pt1, const int rowId_pt2, const int rowId_pt3,
      const gtsam::Pose3& leftCameraPose) const;

  /* ------------------------------------------------------------------------ */
  // Searches in lmk_id_to_map_point_id_.
  bool findMapPointIdFromLandmarkId(const LandmarkId& id_pt,
                                    int* row_id) const;

  /* ------------------------------------------------------------------------ */
  // Searches in keypoint_to_map_point_id_ (the keypoints in frame).
  bool findMapPointIdFromPixel(const KeypointCV& px,
                               int* row_id) const;

  /* ------------------------------------------------------------------------ */
  // Try to reject bad triangles, corresponding to outliers
  void filterOutBadTriangles(const gtsam::Pose3& leftCameraPose,
                             const cv::Mat& map_points_3d,
                             cv::Mat* polygons_mesh,
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
  void populate3dMeshTimeHorizon(const std::vector<cv::Vec6f>& triangulation2D,
                                 const std::map<LandmarkId, gtsam::Point3>& points_with_id_map,
                                 const Frame& frame,
                                 std::map<int, LandmarkId>* vertex_to_lmk_id_map,
                                 std::map<LandmarkId, int>* lmk_id_to_vertex_map,
                                 cv::Mat* vertices_mesh,
                                 cv::Mat* polygon_mesh);

  /* -------------------------------------------------------------------------- */
  // Updates mesh data structures incrementally, by adding new landmark
  // if there was no previous id, or updating it if it was already present.
  void updateMeshDataStructures(
    const LandmarkId& id_pt_1,
    const cv::Point3f& point_1,
    std::map<int, LandmarkId>* vertex_to_lmk_id_map,
    std::map<LandmarkId, int>* lmk_id_to_vertex_map,
    cv::Mat* vertices_mesh,
    cv::Mat* polygon_mesh);

  /* ------------------------------------------------------------------------ */
  // Calculate normals of polygonMesh.
  bool calculateNormals(const cv::Mat& map_points_3d, const cv::Mat& polygons_mesh, std::vector<cv::Point3f>* normals);

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

};

} // namespace VIO

#endif /* Mesher_H_ */


