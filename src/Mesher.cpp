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

#include "Mesher.h"
#include "LoggerMatlab.h"

namespace VIO {

/* ----------------------------------------------------------------------------- */
// for a triangle defined by the 3d points mapPoints3d_.at(rowId_pt1), mapPoints3d_.at(rowId_pt2),
// mapPoints3d_.at(rowId_pt3), compute ratio between largest side and smallest side (how elongated it is)
double Mesher::getRatioBetweenSmallestAndLargestSide(
    const int rowId_pt1, const int rowId_pt2, const int rowId_pt3,
    boost::optional<double &> d12_out,
    boost::optional<double &> d23_out,
    boost::optional<double &> d31_out,
    boost::optional<double &> minSide_out,
    boost::optional<double &> maxSide_out) const{

  // get 3D points
  cv::Point3f p1 = map_points_3d_.at<cv::Point3f>(rowId_pt1);
  cv::Point3f p2 = map_points_3d_.at<cv::Point3f>(rowId_pt2);
  cv::Point3f p3 = map_points_3d_.at<cv::Point3f>(rowId_pt3);

  // measure sides:
  double d12 = double(cv::norm(p1-p2));
  double d23 = double(cv::norm(p2-p3));
  double d31 = double(cv::norm(p3-p1));
  double minSide = std::min(d12,std::min(d23,d31));
  double maxSide = std::max(d12,std::max(d23,d31));

  if(d12_out && d23_out && d31_out){ // return distances, mainly for debug
    *d12_out = d12;
    *d23_out = d23;
    *d31_out = d31;
  }
  if(minSide_out && maxSide_out){
    *minSide_out = minSide;
    *maxSide_out = maxSide;
  }
  // compute and return ratio
  return minSide / maxSide;
}

/* -------------------------------------------------------------------------- */
// for a triangle defined by the 3d points mapPoints3d_.at(rowId_pt1), mapPoints3d_.at(rowId_pt2),
// mapPoints3d_.at(rowId_pt3), compute ratio between largest side and smallest side (how elongated it is)
double Mesher::getRatioBetweenTangentialAndRadialDisplacement(
                                     const int rowId_pt1,
                                     const int rowId_pt2,
                                     const int rowId_pt3,
                                     const gtsam::Pose3& leftCameraPose) const {
  std::vector<gtsam::Point3> points;

  // get 3D points
  cv::Point3f p1 = map_points_3d_.at<cv::Point3f>(rowId_pt1);
  gtsam::Point3 p1_C = gtsam::Point3(double(p1.x),double(p1.y),double(p1.z));
  points.push_back(leftCameraPose.transform_to(p1_C)); // checks elongation in *camera frame*

  cv::Point3f p2 = map_points_3d_.at<cv::Point3f>(rowId_pt2);
  gtsam::Point3 p2_C = gtsam::Point3(double(p2.x),double(p2.y),double(p2.z));
  points.push_back(leftCameraPose.transform_to(p2_C)); // checks elongation in *camera frame*

  cv::Point3f p3 = map_points_3d_.at<cv::Point3f>(rowId_pt3);
  gtsam::Point3 p3_C = gtsam::Point3(double(p3.x),double(p3.y),double(p3.z));
  points.push_back(leftCameraPose.transform_to(p3_C)); // checks elongation in *camera frame*

  return UtilsGeometry::getRatioBetweenTangentialAndRadialDisplacement(points);
}

/* ------------------------------------------------------------------------ */
// Searches in lmk_id_to_map_point_id_.
bool Mesher::findMapPointIdFromLandmarkId(const LandmarkId& id_pt,
                                          int* row_id) const {
  CHECK_NOTNULL(row_id);

  if (id_pt == -1) { // Landmark id not valid.
    return false;
  }

  // If we found the point in the frame (it has a valid lmk id).
  auto it = lmk_id_to_map_point_id_.find(id_pt); // get row ids

  if (it != lmk_id_to_map_point_id_.end()) {
    *row_id = it->second; // lmk_id_to_map_point_id_.at(id_pt)
  } else {
    // this case can actually happen since some points have lmk ids,
    // but do not have 3D in vio hence they do not show up in updateMap3D.
    throw std::runtime_error("findRowIdFromLandmarkId: lmk id was not"
                             " found in lmk_id_to_map_point_id_");
  }

  return true;
}

/* ------------------------------------------------------------------------ */
// Searches in keypoint_to_map_point_id_ (the keypoints in frame).
bool Mesher::findMapPointIdFromPixel(const KeypointCV& px,
                                     int* row_id) const {
  CHECK_NOTNULL(row_id);

  for (size_t i = 0; i < keypoint_to_map_point_id_.size(); i++) {
    if (keypoint_to_map_point_id_.at(i).first.x == px.x &&
        keypoint_to_map_point_id_.at(i).first.y == px.y) {
      *row_id = keypoint_to_map_point_id_.at(i).second;
      return true;
    } else {
      throw std::runtime_error("findRowIdFromPixel: kpt not found.");
    }
  }

  return false;
}

/* ----------------------------------------------------------------------------- */
// Try to reject bad triangles, corresponding to outliers
void Mesher::filterOutBadTriangles(const gtsam::Pose3& leftCameraPose,
                                   double minRatioBetweenLargestAnSmallestSide,
                                   double min_elongation_ratio,
                                   double maxTriangleSide) {
  double ratioSides_i = 1.0,ratioTangentialRadial_i = 1.0, maxTriangleSide_i = 2.0;
  cv::Mat tmpPolygons = polygons_mesh_.clone();
  polygons_mesh_ = cv::Mat(0,1,CV_32SC1); // reset

  for (size_t i = 0;i < tmpPolygons.rows;i = i + 4) { // for each polygon
    // get polygon vertices:
    if (tmpPolygons.at<int32_t>(i) != 3) {
      throw std::runtime_error("filterOutBadTriangles: expecting 3 vertices in triangle");
    }

    int rowId_pt1 = tmpPolygons.at<int32_t>(i+1);
    int rowId_pt2 = tmpPolygons.at<int32_t>(i+2);
    int rowId_pt3 = tmpPolygons.at<int32_t>(i+3);

    // check geometric dimensions
    double d12, d23, d31;
    if (minRatioBetweenLargestAnSmallestSide > 0.0){ // if threshold is disabled, avoid computation
      ratioSides_i = getRatioBetweenSmallestAndLargestSide(rowId_pt1,rowId_pt2,rowId_pt3,d12,d23,d31);
    }
    if (min_elongation_ratio > 0.0){ // if threshold is disabled, avoid computation
      ratioTangentialRadial_i = getRatioBetweenTangentialAndRadialDisplacement(rowId_pt1,rowId_pt2,rowId_pt3, leftCameraPose);
    }
    if (maxTriangleSide > 0.0){ // if threshold is disabled, avoid computation
      std::vector<double> sidesLen; sidesLen.push_back(d12);sidesLen.push_back(d23);sidesLen.push_back(d31);
      maxTriangleSide_i = *std::max_element(sidesLen.begin(), sidesLen.end());
    }
    // check if triangle is not elongated
    if ( (ratioSides_i >= minRatioBetweenLargestAnSmallestSide) &&
         (ratioTangentialRadial_i >= min_elongation_ratio) &&
         maxTriangleSide_i <= maxTriangleSide) {
      polygons_mesh_.push_back(3); // add rows
      polygons_mesh_.push_back(rowId_pt1); // row in mapPoints3d_
      polygons_mesh_.push_back(rowId_pt2); // row in mapPoints3d_
      polygons_mesh_.push_back(rowId_pt3); // row in mapPoints3d_
    }
  }
}

/* -------------------------------------------------------------------------- */
// Create a 2D mesh from 2D corners in an image, coded as a Frame class
void Mesher::populate3dMesh(const std::vector<cv::Vec6f>& triangulation2D,
                            const Frame& frame,
                            cv::Mat* polygon) const {
  CHECK_NOTNULL(polygon);
  // Raw integer list of the form: (n,id1,id2,...,idn, n,id1,id2,...,idn, ...)
  // where n is the number of points in the polygon, and id is a zero-offset
  // index into an associated cloud.

  // Populate polygons with indices:
  // note: we restrict to valid triangles in which each landmark has a 3D point
  for (size_t i = 0; i < triangulation2D.size(); i++) { // TODO: this is doing a lot of computation
    const cv::Vec6f& t = triangulation2D[i];

    // get lmk ids mapPoints3d_ (iterators)
    const cv::Point2f pixel_1 (t[0], t[1]);
    const cv::Point2f pixel_2 (t[2], t[3]);
    const cv::Point2f pixel_3 (t[4], t[5]);

    const LandmarkId id_pt1 (frame.findLmkIdFromPixel(pixel_1));
    const LandmarkId id_pt2 (frame.findLmkIdFromPixel(pixel_2));
    const LandmarkId id_pt3 (frame.findLmkIdFromPixel(pixel_3));

    int row_id_pt1, row_id_pt2, row_id_pt3;
    if (!findMapPointIdFromLandmarkId(id_pt1, &row_id_pt1)) {
      findMapPointIdFromPixel(pixel_1, &row_id_pt1);
    }
    if (!findMapPointIdFromLandmarkId(id_pt2, &row_id_pt2)) {
      findMapPointIdFromPixel(pixel_2, &row_id_pt2);
    }
    if (!findMapPointIdFromLandmarkId(id_pt3, &row_id_pt3)) {
      findMapPointIdFromPixel(pixel_3, &row_id_pt3);
    }

    polygon->push_back(3); // Add rows.
    polygon->push_back(row_id_pt1); // Row in map_points_3d_.
    polygon->push_back(row_id_pt2); // Row in map_points_3d_.
    polygon->push_back(row_id_pt3); // Row in map_points_3d_.
  }
}

/* -------------------------------------------------------------------------- */
// Calculate normals of polygonMesh.
bool Mesher::calculateNormals(std::vector<cv::Point3f>* normals) {
  if (normals == nullptr) return false;

  // Brute force, ideally only call when a new triangle appears...
  normals->clear();
  normals->resize(std::round(polygons_mesh_.rows / 4)); // TODO Assumes we have triangles...

  // Loop over all triangles (TODO: group all loopy operations, now there
  // are two or more loops over polygonsMesh around the code...
  for (size_t i = 0; i < polygons_mesh_.rows; i = i + 4) { // for each polygon
    // Get triangle vertices:
    if (polygons_mesh_.at<int32_t>(i) != 3) {
      throw std::runtime_error("CalculateNormals:"
                               " expecting 3 vertices in triangle");
    }

    // TODO Assumes we have triangles...
    int rowId_pt1 = polygons_mesh_.at<int32_t>(i + 1);
    int rowId_pt2 = polygons_mesh_.at<int32_t>(i + 2);
    int rowId_pt3 = polygons_mesh_.at<int32_t>(i + 3);

    size_t size_map_points_3d = map_points_3d_.rows;
    if (rowId_pt1 >= size_map_points_3d &&
        rowId_pt2 >= size_map_points_3d &&
        rowId_pt3 >= size_map_points_3d) {
      throw std::runtime_error("CalculateNormals: polygonsMesh corrupted.");
    }

    cv::Point3f p1 = map_points_3d_.at<cv::Point3f>(rowId_pt1);
    cv::Point3f p2 = map_points_3d_.at<cv::Point3f>(rowId_pt2);
    cv::Point3f p3 = map_points_3d_.at<cv::Point3f>(rowId_pt3);

    // Calculate vectors of the triangle.
    cv::Point3f v21 = p2 - p1;
    cv::Point3f v31 = p3 - p1;

    // Calculate normal (cross product).
    cv::Point3f normal = v21.cross(v31);

    // Normalize
    double norm = (cv::norm(v21) * cv::norm(v31));
    if (norm == 0) {
      throw std::runtime_error("CalculateNormals: norm is 0.");
    }
    normal /= norm;

    // Store normal to triangle i.
    normals->at(std::round(i / 4)) = normal; // TODO Assumes we have triangles...
  }

  return true;
}

/* ------------------------------------------------------------------------ */
// Clusters normals given an axis, a set of normals and a
// tolerance. The result is a vector of indices of the given set of normals
// that are in the cluster.
void Mesher::clusterNormalsAroundAxis(const cv::Point3f& axis,
                                      const std::vector<cv::Point3f>& normals,
                                      const double& tolerance,
                                      std::vector<int>* cluster_normals_idx) {
  size_t idx = 0;
  // TODO, this should be in the same loop as the one calculating
  // the normals...
  for (const cv::Point3f& normal: normals) {
    if (isNormalAroundAxis(axis, normal, tolerance))
      cluster_normals_idx->push_back(idx);
    idx++;
  }
}

/* ------------------------------------------------------------------------ */
// Is normal around axis?
bool Mesher::isNormalAroundAxis(const cv::Point3f& axis,
                                const cv::Point3f& normal,
                                const double& tolerance) const {
  double diff_a = cv::norm(normal - axis);
  double diff_b = cv::norm(normal + axis);
  return (((diff_a < tolerance) || //  axis and normal almost aligned
           (diff_b < tolerance)) // axis and normal in opp directions.
           ? true : false);
}

/* ------------------------------------------------------------------------ */
// Clusters normals perpendicular to an axis. Given an axis, a set of normals and a
// tolerance. The result is a vector of indices of the given set of normals
// that are in the cluster.
void Mesher::clusterNormalsPerpendicularToAxis(const cv::Point3f& axis,
                                      const std::vector<cv::Point3f>& normals,
                                      const double& tolerance,
                                      std::vector<int>* cluster_normals_idx) {
  size_t idx = 0;
  // TODO, this should be in the same loop as the one calculating
  // the normals...
  // TODO: remove logger.
  static constexpr bool log_normals = false;
  std::vector<cv::Point3f> cluster_normals;
  for (const cv::Point3f& normal: normals) {
    if (isNormalPerpendicularToAxis(axis, normal, tolerance)) {
      cluster_normals_idx->push_back(idx);
      // TODO: remove logger.
      if (log_normals) {
        cluster_normals.push_back(normal);
      }
    }
    idx++;
  }
  if (log_normals) {
    LoggerMatlab logger;
    logger.openLogFiles(4);
    logger.logNormals(cluster_normals);
    logger.closeLogFiles(4);
  }
}

/* ------------------------------------------------------------------------ */
// Is normal perpendicular to axis?
bool Mesher::isNormalPerpendicularToAxis(const cv::Point3f& axis,
                                         const cv::Point3f& normal,
                                         const double& tolerance) const {
    return ((cv::norm(normal.dot(axis)) < tolerance)? true: false);
}

/* -------------------------------------------------------------------------- */
// Update map: update structures keeping memory of the map before visualization.
void Mesher::updateMap3D(
          std::vector<std::pair<LandmarkId, gtsam::Point3>> points_with_id,
          std::vector<std::pair<KeypointCV, gtsam::Point3>> points_without_id) {

  // Update 3D points by adding new points or updating old re-observed ones
  // (for the one with lmk id).
  for (size_t i = 0; i < points_with_id.size(); i++) {
    const LandmarkId& lmk_id (points_with_id.at(i).first);
    const gtsam::Point3& point_i (points_with_id.at(i).second);

    if (lmk_id == -1) {
      throw std::runtime_error("updateMap3D: points_with_id are supposed to have"
                               " valid lmk id");
    }

    auto lmk_it = lmk_id_to_map_point_id_.find(lmk_id);
    if (lmk_it == lmk_id_to_map_point_id_.end()) { // New landmark.
      cv::Point3f new_lmk(float(point_i.x()),
                          float(point_i.y()),
                          float(point_i.z()));
      map_points_3d_.push_back(new_lmk);
      lmk_id_to_map_point_id_.insert(
                                    std::make_pair(lmk_id, last_map_point_id_));
      ++last_map_point_id_;
    } else { // Replace point for existing landmark.
      cv::Point3f* data = map_points_3d_.ptr<cv::Point3f>();
      int row_id = (*lmk_it).second;
      data[row_id].x = float(point_i.x());
      data[row_id].y = float(point_i.y());
      data[row_id].z = float(point_i.z());
    }
  }

  // Add other extra points without lmk if:
  for (size_t i = 0; i < points_without_id.size(); i++) {
    const KeypointCV& kps_i (points_without_id.at(i).first);
    const gtsam::Point3& point_i (points_without_id.at(i).second);

    cv::Point3f p(float(point_i.x()),
                  float(point_i.y()),
                  float(point_i.z()));

    map_points_3d_.push_back(p);
    keypoint_to_map_point_id_.push_back(std::pair<KeypointCV,int>
                                        (kps_i, last_map_point_id_));
    ++last_map_point_id_;
  }
}

/* ----------------------------------------------------------------------------- */
// Update mesh: update structures keeping memory of the map before visualization
void Mesher::updateMesh3D(
              std::vector<std::pair<LandmarkId, gtsam::Point3>> pointsWithIdVIO,
              std::shared_ptr<StereoFrame> stereoFrame,
              const gtsam::Pose3& leftCameraPose,
              const Mesh2Dtype& mesh2Dtype,
              float maxGradInTriangle,
              double minRatioBetweenLargestAnSmallestSide,
              double min_elongation_ratio,
              double maxTriangleSide) {

  // Build 2D mesh.
  std::vector<cv::Vec6f> mesh_2d;
  std::vector<std::pair<LandmarkId, gtsam::Point3>> pointsWithIdStereo;
  stereoFrame->createMesh2dStereo(&mesh_2d,
                                  &pointsWithIdStereo,
                                  mesh2Dtype);
  cv::Mat img_grads;
  stereoFrame->computeImgGradients(stereoFrame->left_frame_.img_,
                                   &img_grads);
  std::vector<cv::Vec6f> mesh_2d_filtered;
  stereoFrame->filterTrianglesWithGradients(img_grads, mesh_2d,
                                            &mesh_2d_filtered,
                                            maxGradInTriangle);

  // Debug.
  static constexpr bool visualize_mesh_2d = false;
  if (visualize_mesh_2d) {
    stereoFrame->visualizeMesh2DStereo(mesh_2d, 1);
  }
  static constexpr bool visualize_mesh_2d_filtered = true;
  if (visualize_mesh_2d_filtered) {
    stereoFrame->visualizeMesh2DStereo(mesh_2d_filtered, 1, "2D Mesh Filtered");
  }

  // All lmks corresponding to the kpts used for delaunay triangulation are in
  // the frame of the camera.
  // Transform 3d lmk from camera frame to global frame.
  for (std::pair<LandmarkId, gtsam::Point3>& lmk_id_to_3d_point:
                                                           pointsWithIdStereo) {
      lmk_id_to_3d_point.second = leftCameraPose.transform_from(
                                      gtsam::Point3(lmk_id_to_3d_point.second));
  }

  // Put all landmark points inside a single structure.
  std::vector<std::pair<LandmarkId, gtsam::Point3>>& points_with_id =
                                                             pointsWithIdStereo;

  // some points are both in stereo frame and in the VIO, so there is some overlap
  // But if we put the VIO points last in the following structure we make sure that
  // updateMap3D below takes the most accurate points (the ones in VIO, not stereo).
  points_with_id.insert(points_with_id.end(),
                        pointsWithIdVIO.begin(),
                        pointsWithIdVIO.end()); // order is important (VIO last)

  // Update 3D points (possibly replacing some points with new estimates).
  std::vector<std::pair<KeypointCV, gtsam::Point3>> points_without_id;
  if (mesh2Dtype == Mesh2Dtype::DENSE) {
    for (size_t i = 0; i < stereoFrame->extraStereoKeypoints_.size(); i++) { // convert to global coordinates
      KeypointCV px = stereoFrame->extraStereoKeypoints_.at(i).first;
      gtsam::Point3 point = leftCameraPose.transform_from(stereoFrame->extraStereoKeypoints_.at(i).second);
      points_without_id.push_back(std::make_pair(px, point));
    }
  }

  updateMap3D(points_with_id, points_without_id);

  VLOG(100) << "Before polygonsMesh_.size() " <<  polygons_mesh_.size << "\n";
  // Concatenate mesh in the current image to existing mesh.
  populate3dMesh(
        mesh_2d,
        stereoFrame->left_frame_,
        &polygons_mesh_);
  VLOG(100) << "After polygonsMesh_.size() " <<  polygons_mesh_.size << "\n";

  filterOutBadTriangles(leftCameraPose,
                        minRatioBetweenLargestAnSmallestSide,
                        min_elongation_ratio,
                        maxTriangleSide);

  // After filling in polygonsMesh_, we don't need this, and it must be reset.
  keypoint_to_map_point_id_.resize(0);
}

void Mesher::clusterMesh(std::vector<TriangleCluster>* clusters) {
  CHECK_NOTNULL(clusters);

  // Calculate normals of the triangles in the mesh.
  // The normals are in the world frame of reference.
  std::vector<cv::Point3f> normals;
  calculateNormals(&normals);

  // Cluster triangles oriented along z axis.
  static const cv::Point3f z_axis(0, 0, 1);

  TriangleCluster z_triangle_cluster;
  z_triangle_cluster.cluster_direction_ = z_axis;
  z_triangle_cluster.cluster_id_ = 2;

  static constexpr double normal_tolerance = 0.2; // 0.087 === 10 deg. aperture.
  clusterNormalsAroundAxis(z_axis, normals, normal_tolerance,
                           &(z_triangle_cluster.triangle_ids_));

  // Cluster triangles with normal perpendicular to z_axis, aka along equator.
  TriangleCluster equatorial_triangle_cluster;
  equatorial_triangle_cluster.cluster_direction_ = z_axis;
  equatorial_triangle_cluster.cluster_id_ = 0;

  static constexpr double normal_tolerance_perpendicular = 0.1;
  clusterNormalsPerpendicularToAxis(z_axis, normals,
                                    normal_tolerance_perpendicular,
                           &(equatorial_triangle_cluster.triangle_ids_));

  // Append clusters.
  clusters->push_back(z_triangle_cluster);
  clusters->push_back(equatorial_triangle_cluster);
}

/* -------------------------------------------------------------------------- */
// Update mesh: update structures keeping memory of the map before visualization
void Mesher::updateMesh3D(
                std::vector<std::pair<LandmarkId, gtsam::Point3> > pointsWithId,
                Frame& frame,
                const gtsam::Pose3& leftCameraPose,
                double minRatioBetweenLargestAnSmallestSide,
                double min_elongation_ratio,
                double maxTriangleSide) {
// debug:
  static constexpr bool doVisualize2Dmesh = true;

  // update 3D points (possibly replacing some points with new estimates)
  updateMap3D(pointsWithId);

  // build 2D mesh, restricted to points with lmk!=-1
  frame.createMesh2D();
  if (doVisualize2Dmesh) {
    frame.visualizeMesh2D(100);
  }

  // (do not concatenate) mesh in the current image to existing mesh
  populate3dMesh(frame.triangulation2D_,
                          frame,
                          &polygons_mesh_);

  filterOutBadTriangles(leftCameraPose,
                        minRatioBetweenLargestAnSmallestSide,
                        min_elongation_ratio,
                        maxTriangleSide);
}

} // namespace VIO
