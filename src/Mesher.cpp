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
    const cv::Mat& map_points_3d,
    const int rowId_pt1, const int rowId_pt2, const int rowId_pt3,
    boost::optional<double &> d12_out,
    boost::optional<double &> d23_out,
    boost::optional<double &> d31_out,
    boost::optional<double &> minSide_out,
    boost::optional<double &> maxSide_out) const{

  // get 3D points
  cv::Point3f p1 = map_points_3d.at<cv::Point3f>(rowId_pt1);
  cv::Point3f p2 = map_points_3d.at<cv::Point3f>(rowId_pt2);
  cv::Point3f p3 = map_points_3d.at<cv::Point3f>(rowId_pt3);

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
                                     const cv::Mat& map_points_3d,
                                     const int rowId_pt1,
                                     const int rowId_pt2,
                                     const int rowId_pt3,
                                     const gtsam::Pose3& leftCameraPose) const {
  std::vector<gtsam::Point3> points;

  // get 3D points
  cv::Point3f p1 = map_points_3d.at<cv::Point3f>(rowId_pt1);
  gtsam::Point3 p1_C = gtsam::Point3(double(p1.x),double(p1.y),double(p1.z));
  points.push_back(leftCameraPose.transform_to(p1_C)); // checks elongation in *camera frame*

  cv::Point3f p2 = map_points_3d.at<cv::Point3f>(rowId_pt2);
  gtsam::Point3 p2_C = gtsam::Point3(double(p2.x),double(p2.y),double(p2.z));
  points.push_back(leftCameraPose.transform_to(p2_C)); // checks elongation in *camera frame*

  cv::Point3f p3 = map_points_3d.at<cv::Point3f>(rowId_pt3);
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
    return false;
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
// TODO now we remove bad triangles, but the vertices of those are still in
// the data structure.
void Mesher::filterOutBadTriangles(const gtsam::Pose3& leftCameraPose,
                                   const cv::Mat& map_points_3d,
                                   cv::Mat* polygons_mesh,
                                   double minRatioBetweenLargestAnSmallestSide,
                                   double min_elongation_ratio,
                                   double maxTriangleSide) {
  CHECK_NOTNULL(polygons_mesh);

  double ratioSides_i;
  double ratioTangentialRadial_i;
  double maxTriangleSide_i;

  // Check geometric dimensions.
  double d12, d23, d31;

  cv::Mat tmpPolygons = polygons_mesh->clone();
  *polygons_mesh = cv::Mat(0, 1, CV_32SC1); // reset

  // For each polygon.
  for (size_t i = 0; i < tmpPolygons.rows; i = i + 4) {
    // Get polygon vertices:
    if (tmpPolygons.at<int32_t>(i) != 3) {
      throw std::runtime_error("filterOutBadTriangles: expecting 3 vertices in triangle");
    }

    int rowId_pt1 = tmpPolygons.at<int32_t>(i + 1);
    int rowId_pt2 = tmpPolygons.at<int32_t>(i + 2);
    int rowId_pt3 = tmpPolygons.at<int32_t>(i + 3);

    // If threshold is disabled, avoid computation.
    if (minRatioBetweenLargestAnSmallestSide > 0.0) {
      ratioSides_i = getRatioBetweenSmallestAndLargestSide(
                       map_points_3d,
                       rowId_pt1,
                       rowId_pt2,
                       rowId_pt3,
                       d12, d23, d31);
    }

    // If threshold is disabled, avoid computation.
    if (min_elongation_ratio > 0.0){
      ratioTangentialRadial_i = getRatioBetweenTangentialAndRadialDisplacement(
                                  map_points_3d,
                                  rowId_pt1, rowId_pt2, rowId_pt3,
                                  leftCameraPose);
    }

    // If threshold is disabled, avoid computation.
    if (maxTriangleSide > 0.0) {
      std::vector<double> sidesLen;
      sidesLen.push_back(d12);
      sidesLen.push_back(d23);
      sidesLen.push_back(d31);
      maxTriangleSide_i = *std::max_element(sidesLen.begin(), sidesLen.end());
    }

    // Check if triangle is not elongated.
    if ((ratioSides_i >= minRatioBetweenLargestAnSmallestSide) &&
        (ratioTangentialRadial_i >= min_elongation_ratio) &&
        (maxTriangleSide_i <= maxTriangleSide)) {
      polygons_mesh->push_back(3); // Add rows.
      polygons_mesh->push_back(rowId_pt1); // Row in mapPoints3d_.
      polygons_mesh->push_back(rowId_pt2); // Row in mapPoints3d_.
      polygons_mesh->push_back(rowId_pt3); // Row in mapPoints3d_.
    }
  }
}

/* -------------------------------------------------------------------------- */
// Create a 2D mesh from 2D corners in an image, coded as a Frame class
void Mesher::populate3dMesh(const std::vector<cv::Vec6f>& triangulation2D,
                            const Frame& frame,
                            cv::Mat* polygon_mesh) const {
  CHECK_NOTNULL(polygon_mesh);
  // *polygon = cv::Mat(0, 1, CV_32SC1); // Clear the polygon_mesh

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

    polygon_mesh->push_back(3); // Add rows.
    polygon_mesh->push_back(row_id_pt1); // Row in map_points_3d_.
    polygon_mesh->push_back(row_id_pt2); // Row in map_points_3d_.
    polygon_mesh->push_back(row_id_pt3); // Row in map_points_3d_.
  }
}

/* -------------------------------------------------------------------------- */
// Create a 3D mesh from 2D corners in an image, coded as a Frame class
void Mesher::populate3dMeshTimeHorizon(const std::vector<cv::Vec6f>& triangulation2D,
                                       const std::map<LandmarkId, gtsam::Point3>& points_with_id_map,
                                       const Frame& frame,
                                       std::map<int, LandmarkId>* vertex_to_lmk_id_map,
                                       std::map<LandmarkId, int>* lmk_id_to_vertex_map,
                                       cv::Mat* vertices_mesh,
                                       cv::Mat* polygon_mesh) {
  CHECK_NOTNULL(vertex_to_lmk_id_map);
  CHECK_NOTNULL(lmk_id_to_vertex_map);
  CHECK_NOTNULL(vertices_mesh);
  CHECK_NOTNULL(polygon_mesh);
  // Clear the polygon_mesh, equivalent to have a per-frame mesh.
  //*polygon_mesh = cv::Mat(0, 1, CV_32SC1);

  // Raw integer list of the form: (n,id1,id2,...,idn, n,id1,id2,...,idn, ...)
  // where n is the number of points in the polygon, and id is a zero-offset
  // index into an associated cloud.

  // Populate polygons with indices:
  // note: we restrict to valid triangles in which each landmark has a 3D point
  // Iterate over each face in the 2d mesh, and generate the data structure
  // for the 3d mesh: map_points_3d and polygons_mesh.
  for (size_t i = 0; i < triangulation2D.size(); i++) {
    const cv::Vec6f& t = triangulation2D[i];

    // Since the triangulation is done at the pixel coordinates level,
    // we need to find which lmk_ids correspond to the three pixel coordinates
    // chosen as a face of the mesh.
    const cv::Point2f pixel_1 (t[0], t[1]);
    const cv::Point2f pixel_2 (t[2], t[3]);
    const cv::Point2f pixel_3 (t[4], t[5]);

    // These are the lmk_ids of the three vertices of a triangular face in the
    // mesh.
    const LandmarkId id_pt_1 (frame.findLmkIdFromPixel(pixel_1));
    const LandmarkId id_pt_2 (frame.findLmkIdFromPixel(pixel_2));
    const LandmarkId id_pt_3 (frame.findLmkIdFromPixel(pixel_3));

    // We try to find which 3d points correspond to these ids.
    const auto& points_with_id_map_end = points_with_id_map.end();
    const auto& lmk_1_it = points_with_id_map.find(id_pt_1);
    const auto& lmk_2_it = points_with_id_map.find(id_pt_2);
    const auto& lmk_3_it = points_with_id_map.find(id_pt_3);

    // If we managed to find the 3d points corresponding to the three lmk_ids,
    // we then proceed to build the mesh in 3d.
    // Unless the triangle is already there, then we just update the mesh.
    if (lmk_1_it != points_with_id_map_end &&
        lmk_2_it != points_with_id_map_end &&
        lmk_3_it != points_with_id_map_end) {
      // Update mesh connectivity, this might duplicate faces, but it does not
      // really matter visually. It does in terms of speed and memory...
      // Specify number of point ids per face in the mesh.
      // Currently 3, as we are dealing with a triangular 3d mesh.
      polygon_mesh->push_back(3);

      // Check whether the triangle is already encoded in the mesh, in which case
      // no need to update the connectivity part of the mesh but just the corres-
      // ponding vertices in vertices_mesh.
      const gtsam::Point3& point_1 (lmk_1_it->second);
      cv::Point3f lmk_1(float(point_1.x()),
                        float(point_1.y()),
                        float(point_1.z()));
      updateMeshDataStructures(
            id_pt_1, lmk_1,
            vertex_to_lmk_id_map,
            lmk_id_to_vertex_map,
            vertices_mesh,
            polygon_mesh);

      // Same for pt 2.
      const gtsam::Point3& point_2 (lmk_2_it->second);
      cv::Point3f lmk_2(float(point_2.x()),
                        float(point_2.y()),
                        float(point_2.z()));
      updateMeshDataStructures(
            id_pt_2, lmk_2,
            vertex_to_lmk_id_map,
            lmk_id_to_vertex_map,
            vertices_mesh,
            polygon_mesh);

      // Same for pt 3.
      const gtsam::Point3& point_3 (lmk_3_it->second);
      cv::Point3f lmk_3(float(point_3.x()),
                        float(point_3.y()),
                        float(point_3.z()));
      updateMeshDataStructures(
            id_pt_3, lmk_3,
            vertex_to_lmk_id_map,
            lmk_id_to_vertex_map,
            vertices_mesh,
            polygon_mesh);

    } else {
      LOG(ERROR) << "Landmarks with ids : "
                 << lmk_1_it->first << " or "
                 << lmk_2_it->first << " or "
                 << lmk_3_it->first
                 << ", could not be found in points_with_id_map.\n";
    }
  }
}

void Mesher::updateMeshDataStructures(
    const LandmarkId& id_pt_1,
    const cv::Point3f& lmk_1,
    std::map<int, LandmarkId>* vertex_to_lmk_id_map,
    std::map<LandmarkId, int>* lmk_id_to_vertex_map,
    cv::Mat* vertices_mesh,
    cv::Mat* polygon_mesh) {
  CHECK_NOTNULL(vertex_to_lmk_id_map);
  CHECK_NOTNULL(lmk_id_to_vertex_map);
  CHECK_NOTNULL(vertices_mesh);
  CHECK_NOTNULL(polygon_mesh);

  const auto& lmk_id_to_vertex_map_end = lmk_id_to_vertex_map->end();
  const auto& vertex_1_it = lmk_id_to_vertex_map->find(id_pt_1);

  int row_id_pt_1;
  // Check whether this landmark is already in the set of vertices of the
  // mesh.
  if (vertex_1_it == lmk_id_to_vertex_map_end) {
    // New landmark, create a new entrance in the set of vertices.
    // Store 3D points in map_points_3d.
    vertices_mesh->push_back(lmk_1);
    row_id_pt_1 = vertices_mesh->rows - 1;
    // Store the row in the vertices structure of this new landmark id.
    (*lmk_id_to_vertex_map)[id_pt_1] = row_id_pt_1;
    (*vertex_to_lmk_id_map)[row_id_pt_1] = id_pt_1;
  } else {
    // Update old landmark with new position.
    vertices_mesh->at<cv::Point3f>(vertex_1_it->second) = lmk_1;
    row_id_pt_1 = vertex_1_it->second;
  }
  // Store corresponding ids (row index) to the 3d point in map_points_3d.
  // This structure encodes the connectivity of the mesh:
  polygon_mesh->push_back(row_id_pt_1);
}

// TODO the polygon_mesh has repeated faces...
// And this seems to slow down quite a bit the for loop!
void Mesher::reducePolygonMeshToTimeHorizon(
                  const std::map<int, LandmarkId>& vertex_to_lmk_id_map,
                  const cv::Mat& vertices_mesh,
                  const cv::Mat& polygon_mesh,
                  const std::map<LandmarkId, gtsam::Point3>& points_with_id_map,
                  std::map<int, LandmarkId>* vertex_to_lmk_id_map_output, //
                  std::map<LandmarkId, int>* lmk_id_to_vertex_map_output, //
                  cv::Mat* vertices_mesh_output,
                  cv::Mat* polygon_mesh_output) {
  CHECK_NOTNULL(vertex_to_lmk_id_map_output);
  CHECK_NOTNULL(lmk_id_to_vertex_map_output);
  CHECK_NOTNULL(vertices_mesh_output);
  CHECK_NOTNULL(polygon_mesh_output);

  // Check that output is different than input.
  CHECK_NE(vertex_to_lmk_id_map_output, &vertex_to_lmk_id_map);
  CHECK_NE(vertices_mesh_output, &vertices_mesh);
  CHECK_NE(polygon_mesh_output, &polygon_mesh);

  *vertices_mesh_output = cv::Mat(0, 1, CV_32FC3);
  *polygon_mesh_output = cv::Mat(0, 1, CV_32SC1);
  vertex_to_lmk_id_map_output->clear();
  lmk_id_to_vertex_map_output->clear();

  auto end = points_with_id_map.end();
  // Loop over each face in the mesh.
  for (int i = 0; i < polygon_mesh.rows; i = i + 4) {
    CHECK_EQ(polygon_mesh.at<int32_t>(i), 3) << "Expected triangular mesh!";
    int32_t row_id_pt1 = polygon_mesh.at<int32_t>(i + 1);
    int32_t row_id_pt2 = polygon_mesh.at<int32_t>(i + 2);
    int32_t row_id_pt3 = polygon_mesh.at<int32_t>(i + 3);

    // If the three vertices of the current face are in the set of points
    const LandmarkId& lmk_id_1 = vertex_to_lmk_id_map.at(row_id_pt1);
    const LandmarkId& lmk_id_2 = vertex_to_lmk_id_map.at(row_id_pt2);
    const LandmarkId& lmk_id_3 = vertex_to_lmk_id_map.at(row_id_pt3);
    if (points_with_id_map.find(lmk_id_1) != end &&
        points_with_id_map.find(lmk_id_2) != end &&
        points_with_id_map.find(lmk_id_3) != end) {
      // Store the new face of the mesh in the output data structures.
      /// The last row index is necessary later to encode the connectivity of the
      /// mesh.

      // Encode connectivity of the vertices of the mesh.
      polygon_mesh_output->push_back(3);

      // Save vertices of the mesh.
      // TODO CHECK THAT THE NEW VERTICES ARE NOT ALREADY THERE!!!!
      const cv::Point3f& point_1 = vertices_mesh.at<cv::Point3f>(row_id_pt1);
      updateMeshDataStructures(lmk_id_1, point_1,
                               vertex_to_lmk_id_map_output,
                               lmk_id_to_vertex_map_output,
                               vertices_mesh_output,
                               polygon_mesh_output);
      const cv::Point3f& point_2 = vertices_mesh.at<cv::Point3f>(row_id_pt2);
      updateMeshDataStructures(lmk_id_2, point_2,
                               vertex_to_lmk_id_map_output,
                               lmk_id_to_vertex_map_output,
                               vertices_mesh_output,
                               polygon_mesh_output);
      const cv::Point3f& point_3 = vertices_mesh.at<cv::Point3f>(row_id_pt3);
      updateMeshDataStructures(lmk_id_3, point_3,
                               vertex_to_lmk_id_map_output,
                               lmk_id_to_vertex_map_output,
                               vertices_mesh_output,
                               polygon_mesh_output);
    }
    // Delete the rest by not adding them to polygon_mesh_output.
  }
}

/* -------------------------------------------------------------------------- */
// Calculate normals of polygonMesh.
bool Mesher::calculateNormals(
    const cv::Mat& map_points_3d,
    const cv::Mat& polygons_mesh,
    std::vector<cv::Point3f>* normals) {
  if (normals == nullptr) return false;

  // Brute force, ideally only call when a new triangle appears...
  normals->clear();
  normals->resize(std::round(polygons_mesh.rows / 4)); // TODO Assumes we have triangles...

  // Loop over all triangles (TODO: group all loopy operations, now there
  // are two or more loops over polygonsMesh around the code...
  for (size_t i = 0; i < polygons_mesh.rows; i = i + 4) { // for each polygon
    // Get triangle vertices:
    if (polygons_mesh.at<int32_t>(i) != 3) {
      throw std::runtime_error("CalculateNormals:"
                               " expecting 3 vertices in triangle");
    }

    // TODO Assumes we have triangles...
    int rowId_pt1 = polygons_mesh.at<int32_t>(i + 1);
    int rowId_pt2 = polygons_mesh.at<int32_t>(i + 2);
    int rowId_pt3 = polygons_mesh.at<int32_t>(i + 3);

    size_t size_map_points_3d = map_points_3d.rows;
    if (rowId_pt1 >= size_map_points_3d &&
        rowId_pt2 >= size_map_points_3d &&
        rowId_pt3 >= size_map_points_3d) {
      throw std::runtime_error("CalculateNormals: polygons_mesh_ "
                               "or map_points_3d_ corrupted.");
    }

    cv::Point3f p1 = map_points_3d.at<cv::Point3f>(rowId_pt1);
    cv::Point3f p2 = map_points_3d.at<cv::Point3f>(rowId_pt2);
    cv::Point3f p3 = map_points_3d.at<cv::Point3f>(rowId_pt3);

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
   const std::vector<std::pair<LandmarkId, gtsam::Point3>>& points_with_id,
   const std::vector<std::pair<KeypointCV, gtsam::Point3>>& points_without_id) {

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

/* -------------------------------------------------------------------------- */
// Update map such that it only contains lmks in time horizon of optimization:
// update structures keeping memory of the map before visualization.
void Mesher::updateMap3dTimeHorizon(
      const std::vector<std::pair<LandmarkId, gtsam::Point3>>& points_with_id) {

  // Update 3D points by adding new points or updating old re-observed ones
  // (for the one with lmk id).
  // Clean old structures.
  lmk_id_to_map_point_id_.clear();
  map_points_3d_ = cv::Mat(0, 1, CV_32FC3);
  last_map_point_id_ = 0;
  for (size_t i = 0; i < points_with_id.size(); i++) {
    const LandmarkId& lmk_id (points_with_id.at(i).first);
    const gtsam::Point3& point_i (points_with_id.at(i).second);

    if (lmk_id == -1) {
      throw std::runtime_error("updateMap3D: points_with_id are supposed to have"
                               " valid lmk id");
    }

    // Add new lmk.
    cv::Point3f new_lmk(float(point_i.x()),
                        float(point_i.y()),
                        float(point_i.z()));
    map_points_3d_.push_back(new_lmk);
    lmk_id_to_map_point_id_.insert(
          std::make_pair(lmk_id, last_map_point_id_));
    ++last_map_point_id_;
  }
}

/* ----------------------------------------------------------------------------- */
// Update mesh: update structures keeping memory of the map before visualization
std::map<int, LandmarkId> Mesher::updateMesh3D(
       const std::vector<std::pair<LandmarkId, gtsam::Point3>>& pointsWithIdVIO,
       std::shared_ptr<StereoFrame> stereoFrame,
       const gtsam::Pose3& leftCameraPose,
       cv::Mat* map_points_3d,
       cv::Mat* polygons_mesh,
       const Mesh2Dtype& mesh2Dtype,
       const float& maxGradInTriangle,
       const double& minRatioBetweenLargestAnSmallestSide,
       const double& min_elongation_ratio,
       const double& maxTriangleSide) {
  CHECK_NOTNULL(map_points_3d);
  CHECK_NOTNULL(polygons_mesh);

  // Build 2D mesh.
  std::vector<cv::Vec6f> mesh_2d;
  std::vector<std::pair<LandmarkId, gtsam::Point3>> pointsWithIdStereo;
  //stereoFrame->createMesh2dStereo(&mesh_2d,
  //                                &pointsWithIdStereo,
  //                                mesh2Dtype);
  stereoFrame->createMesh2dVIO(&mesh_2d,
                               pointsWithIdVIO);
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
  const std::vector<std::pair<LandmarkId, gtsam::Point3>>& points_with_id =
                                                             pointsWithIdVIO;

  //points_with_id.insert(points_with_id.end(),
  //                      pointsWithIdVIO.begin(),
  //                      pointsWithIdVIO.end()); // order is important (VIO last)

  // Update 3D points (possibly replacing some points with new estimates).
  std::vector<std::pair<KeypointCV, gtsam::Point3>> points_without_id;
  if (mesh2Dtype == Mesh2Dtype::DENSE) {
    for (size_t i = 0; i < stereoFrame->extraStereoKeypoints_.size(); i++) { // convert to global coordinates
      KeypointCV px = stereoFrame->extraStereoKeypoints_.at(i).first;
      gtsam::Point3 point = leftCameraPose.transform_from(stereoFrame->extraStereoKeypoints_.at(i).second);
      points_without_id.push_back(std::make_pair(px, point));
    }
  }

  //updateMap3D(points_with_id, points_without_id);
  //updateMap3dTimeHorizon(points_with_id);

  VLOG(100) << "Before polygonsMesh_.size() " <<  polygons_mesh_.size << "\n";
  // Concatenate mesh in the current image to existing mesh.

  // Convert points_with_id to a map, otherwise following algorithms are
  // ridiculously slow.
  const std::map<LandmarkId, gtsam::Point3> points_with_id_map (points_with_id.begin(),
                                                          points_with_id.end());

  // Set of polygons.
  static std::map<int, LandmarkId> vertex_to_lmk_id_map;
  static std::map<LandmarkId, int> lmk_id_to_vertex_map;
  populate3dMeshTimeHorizon(
        mesh_2d,
        points_with_id_map,
        stereoFrame->left_frame_,
        &vertex_to_lmk_id_map,
        &lmk_id_to_vertex_map,
        map_points_3d,
        polygons_mesh);
  std::cout << "OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO\n"
            << "Polygons size is : " << polygons_mesh->rows / 4 << "\n"
            << " While Stereo points size is: " << pointsWithIdStereo.size() << "\n"
            << " While VIO points size is: " << pointsWithIdVIO.size() << std::endl;
  std::map<int, LandmarkId> vertex_to_lmk_id_map_output;
  cv::Mat map_points_3d_output;
  cv::Mat polygons_mesh_output;
  reducePolygonMeshToTimeHorizon(
                                 vertex_to_lmk_id_map,
                                 *map_points_3d,
                                 *polygons_mesh,
                                 points_with_id_map,
                                 &vertex_to_lmk_id_map_output,
                                 &lmk_id_to_vertex_map,
                                 &map_points_3d_output,
                                 &polygons_mesh_output);
  vertex_to_lmk_id_map = vertex_to_lmk_id_map_output;
  *map_points_3d = map_points_3d_output.clone();
  *polygons_mesh = polygons_mesh_output.clone();
  std::cout << "UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU\n"
            << "Polygons size is : " << polygons_mesh->rows / 4 << "\n"
            << " While Stereo points size is: " << pointsWithIdStereo.size() << "\n"
            << " While VIO points size is: " << pointsWithIdVIO.size() << std::endl;

  VLOG(100) << "After polygonsMesh_.size() " <<  polygons_mesh_.size << "\n";

  filterOutBadTriangles(leftCameraPose,
                        *map_points_3d,
                        polygons_mesh,
                        minRatioBetweenLargestAnSmallestSide,
                        min_elongation_ratio,
                        maxTriangleSide);

  // After filling in polygonsMesh_, we don't need this, and it must be reset.
  keypoint_to_map_point_id_.resize(0);

  // Quick and dirty, TODO change.
  return vertex_to_lmk_id_map;
}

void Mesher::clusterMesh(
    const cv::Mat& map_points_3d,
    const cv::Mat& polygons_mesh,
    std::vector<TriangleCluster>* clusters) {
  CHECK_NOTNULL(clusters);

  // Calculate normals of the triangles in the mesh.
  // The normals are in the world frame of reference.
  std::vector<cv::Point3f> normals;
  calculateNormals(map_points_3d, polygons_mesh, &normals);

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
                const double& minRatioBetweenLargestAnSmallestSide,
                const double& min_elongation_ratio,
                const double& maxTriangleSide) {
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
                        map_points_3d_,
                        &polygons_mesh_,
                        minRatioBetweenLargestAnSmallestSide,
                        min_elongation_ratio,
                        maxTriangleSide);
}

void Mesher::extractLmkIdsFromTriangleCluster(
    const TriangleCluster& triangle_cluster,
    const std::map<int, LandmarkId>& vertex_to_id_map,
    const cv::Mat& polygons_mesh,
    LandmarkIds* lmk_ids) {

  CHECK_NOTNULL(lmk_ids);
  lmk_ids->resize(0);

  for (const size_t& triangle_id: triangle_cluster.triangle_ids_) {
    size_t triangle_idx = std::round(triangle_id * 4);
    if (triangle_idx + 3 >= polygons_mesh.rows) {
      LOG(ERROR) << "An id in triangle_ids_ is too large.";
    }
    int32_t idx_1 = polygons_mesh.at<int32_t>(triangle_idx + 1);
    int32_t idx_2 = polygons_mesh.at<int32_t>(triangle_idx + 2);
    int32_t idx_3 = polygons_mesh.at<int32_t>(triangle_idx + 3);
    lmk_ids->push_back(vertex_to_id_map.at(idx_1));
    lmk_ids->push_back(vertex_to_id_map.at(idx_2));
    lmk_ids->push_back(vertex_to_id_map.at(idx_3));
  }
}

/* ------------------------------------------------------------------------ */
// Filter z component in triangle cluster.
void Mesher::filterZComponent(
    const double& z,
    const double& tolerance,
    const cv::Mat& map_points_3d_,
    const cv::Mat& polygons_mesh,
    TriangleCluster* triangle_cluster) {
  CHECK_NOTNULL(triangle_cluster);
  TriangleCluster triangle_cluster_output;
  triangle_cluster_output.cluster_id_ =
      triangle_cluster->cluster_id_;
  triangle_cluster_output.cluster_direction_ =
      triangle_cluster->cluster_direction_;

  for (const size_t& triangle_id: triangle_cluster->triangle_ids_) {
    size_t triangle_idx = std::round(triangle_id * 4);
    if (triangle_idx + 3 >= polygons_mesh.rows) {
      LOG(ERROR) << "An id in triangle_ids_ is too large.";
    }
    const int32_t& idx_1 = polygons_mesh.at<int32_t>(triangle_idx + 1);
    const int32_t& idx_2 = polygons_mesh.at<int32_t>(triangle_idx + 2);
    const int32_t& idx_3 = polygons_mesh.at<int32_t>(triangle_idx + 3);
    const cv::Point3f& lmk_1 = map_points_3d_.at<cv::Point3f>(idx_1);
    const cv::Point3f& lmk_2 = map_points_3d_.at<cv::Point3f>(idx_2);
    const cv::Point3f& lmk_3 = map_points_3d_.at<cv::Point3f>(idx_3);
    const double min_z = z - tolerance;
    const double max_z = z + tolerance;
    if (lmk_1.z >= min_z && lmk_1.z <= max_z &&
        lmk_2.z >= min_z && lmk_2.z <= max_z &&
        lmk_3.z >= min_z && lmk_3.z <= max_z ) {
      triangle_cluster_output.triangle_ids_.push_back(triangle_id);
    }
  }
  *triangle_cluster = triangle_cluster_output;
}

} // namespace VIO
