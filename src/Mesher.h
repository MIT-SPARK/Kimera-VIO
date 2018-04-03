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
#include <opengv/point_cloud/methods.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>

namespace VIO {

class Mesher {
public:
  // map a lmk id to a row in mapPoints3d_
  using LandmarkIdToMapPointId = std::unordered_map<LandmarkId, int>;

  // map a keypoint (without lmk id) to a row in mapPoints3d_
  using KeypointToMapPointId = std::vector< std::pair<KeypointCV,int>>;


  // set of (non-repeated) points = valid landmark positions
  cv::Mat mapPoints3d_;
  // set of polygons
  cv::Mat polygonsMesh_;
  // set of triangle clusters;
  std::vector<TriangleCluster> triangle_clusters_;


  // maps lmk id to corresponding 3D points
  LandmarkIdToMapPointId lmkIdToMapPointId_;
  // number of points
  int points3D_count_;
  KeypointToMapPointId keypointToMapPointId_;


  Mesher(): polygonsMesh_(cv::Mat(0,1,CV_32SC1)), points3D_count_(0) {}

  /* ------------------------------------------------------------------------ */
  // for a triangle defined by the 3d points mapPoints3d_.at(rowId_pt1), mapPoints3d_.at(rowId_pt2),
  // mapPoints3d_.at(rowId_pt3), compute ratio between largest side and smallest side (how elongated it is)
  double getRatioBetweenSmallestAndLargestSide(
      const int rowId_pt1, const int rowId_pt2, const int rowId_pt3,
      boost::optional<double &> d12_out = boost::none,
      boost::optional<double &> d23_out = boost::none,
      boost::optional<double &> d31_out = boost::none,
      boost::optional<double &> minSide_out = boost::none,
      boost::optional<double &> maxSide_out = boost::none) const;

  /* ------------------------------------------------------------------------ */
  // Update map: update structures keeping memory of the map before visualization
  void updateMap3D(
      std::vector<std::pair<LandmarkId, gtsam::Point3> > pointsWithId,
      std::vector<std::pair<KeypointCV, gtsam::Point3> > pointsWithoutId =
                          std::vector<std::pair<KeypointCV, gtsam::Point3>>());

  /* ------------------------------------------------------------------------ */
  // Update mesh: update structures keeping memory of the map before visualization
  void updateMesh3D(
      std::vector<std::pair<LandmarkId, gtsam::Point3>> pointsWithIdVIO,
      std::shared_ptr<StereoFrame> stereoFrame,
      const gtsam::Pose3& leftCameraPose,
      const Mesh2Dtype& mesh2Dtype = Mesh2Dtype::VALIDKEYPOINTS,
      float maxGradInTriangle = 50,
      double minRatioBetweenLargestAnSmallestSide = 0,
      double min_elongation_ratio = 0.5,
      double maxTriangleSide = 10);

  /* ------------------------------------------------------------------------ */
  // Update mesh: update structures keeping memory of the map before visualization
  void updateMesh3D(
      std::vector<std::pair<LandmarkId, gtsam::Point3>> pointsWithId,
      Frame& frame,
      const gtsam::Pose3& leftCameraPose,
      double minRatioBetweenLargestAnSmallestSide = 0.0,
      double min_elongation_ratio = 0.5,
      double maxTriangleSide = 10.0);

private:
  /* ------------------------------------------------------------------------ */
    // for a triangle defined by the 3d points mapPoints3d_.at(rowId_pt1), mapPoints3d_.at(rowId_pt2),
  // mapPoints3d_.at(rowId_pt3), compute ratio between largest side and smallest side (how elongated it is)
  double getRatioBetweenTangentialAndRadialDisplacement(
      const int rowId_pt1, const int rowId_pt2, const int rowId_pt3,
      const gtsam::Pose3& leftCameraPose) const;

  /* ------------------------------------------------------------------------ */
  // searches in both the keypoints in frame as well as in the
  int findRowIdFromPixel(Frame& frame, KeypointCV px) const;

  /* ------------------------------------------------------------------------ */
  // Try to reject bad triangles, corresponding to outliers
  void filterOutBadTriangles(const gtsam::Pose3& leftCameraPose,
                             double minRatioBetweenLargestAnSmallestSide,
                             double min_elongation_ratio,
                             double maxTriangleSide);

  /* ------------------------------------------------------------------------ */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  cv::Mat getTriangulationIndices(std::vector<cv::Vec6f> triangulation2D,
                                  Frame& frame) const;

  /* ------------------------------------------------------------------------ */
  // Calculate normals of polygonMesh.
  bool calculateNormals(std::vector<cv::Point3f>* normals);

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
  // Clusters normals perpendicular to an axis. Given an axis, a set of normals and a
  // tolerance. The result is a vector of indices of the given set of normals
  // that are in the cluster.
  void clusterNormalsPerpendicularToAxis(const cv::Point3f& axis,
                                         const std::vector<cv::Point3f>& normals,
                                         const double& tolerance,
                                         std::vector<int>* cluster_normals_idx);
};

} // namespace VIO

#endif /* Mesher_H_ */


