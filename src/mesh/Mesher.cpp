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

#include "mesh/Mesher.h"

#include "LoggerMatlab.h"

namespace VIO {

/* -------------------------------------------------------------------------- */
// for a triangle defined by the 3d points mapPoints3d_.at(rowId_pt1), mapPoints3d_.at(rowId_pt2),
// mapPoints3d_.at(rowId_pt3), compute ratio between largest side and smallest side (how elongated it is)
double Mesher::getRatioBetweenSmallestAndLargestSide(
    const Mesh3D::VertexPosition3D& p1,
    const Mesh3D::VertexPosition3D& p2,
    const Mesh3D::VertexPosition3D& p3,
    boost::optional<double &> d12_out,
    boost::optional<double &> d23_out,
    boost::optional<double &> d31_out,
    boost::optional<double &> minSide_out,
    boost::optional<double &> maxSide_out) const {

  // Measure sides.
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
    const Mesh3D::VertexPosition3D& p1,
    const Mesh3D::VertexPosition3D& p2,
    const Mesh3D::VertexPosition3D& p3,
    const gtsam::Pose3& leftCameraPose) const {
  std::vector<gtsam::Point3> points;

  // get 3D points
  gtsam::Point3 p1_C = gtsam::Point3(double(p1.x),
                                     double(p1.y),
                                     double(p1.z));
  points.push_back(leftCameraPose.transform_to(p1_C)); // checks elongation in *camera frame*

  gtsam::Point3 p2_C = gtsam::Point3(double(p2.x),
                                     double(p2.y),
                                     double(p2.z));
  points.push_back(leftCameraPose.transform_to(p2_C)); // checks elongation in *camera frame*

  gtsam::Point3 p3_C = gtsam::Point3(double(p3.x),
                                     double(p3.y),
                                     double(p3.z));
  points.push_back(leftCameraPose.transform_to(p3_C)); // checks elongation in *camera frame*

  return UtilsGeometry::getRatioBetweenTangentialAndRadialDisplacement(points);
}

/* -------------------------------------------------------------------------- */
// Try to reject bad triangles, corresponding to outliers
void Mesher::filterOutBadTriangles(const gtsam::Pose3& leftCameraPose,
                                   double minRatioBetweenLargestAnSmallestSide,
                                   double min_elongation_ratio,
                                   double maxTriangleSide) {
  double ratioSides_i;
  double ratioTangentialRadial_i;
  double maxTriangleSide_i;

  // Check geometric dimensions.
  double d12, d23, d31;

  Mesh3D mesh_output;

  // Loop over each face in the mesh.
  Mesh3D::Polygon polygon;

  for (size_t i = 0; i < mesh_.getNumberOfPolygons(); i++) {
    CHECK(mesh_.getPolygon(i, &polygon)) << "Could not retrieve polygon.";
    CHECK_EQ(polygon.size(), 3) << "Expecting 3 vertices in triangle";
    const Mesh3D::VertexPosition3D& p1 = polygon.at(0).getVertexPosition();
    const Mesh3D::VertexPosition3D& p2 = polygon.at(1).getVertexPosition();
    const Mesh3D::VertexPosition3D& p3 = polygon.at(2).getVertexPosition();

    // If threshold is disabled, avoid computation.
    if (minRatioBetweenLargestAnSmallestSide > 0.0) {
      ratioSides_i = getRatioBetweenSmallestAndLargestSide(
                      p1, p2, p3,
                      d12, d23, d31);
    }
    // If threshold is disabled, avoid computation.
    if (min_elongation_ratio > 0.0) {
      ratioTangentialRadial_i = getRatioBetweenTangentialAndRadialDisplacement(
                                  p1, p2, p3,
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
      mesh_output.addPolygonToMesh(polygon);
    }
  }

  mesh_ = mesh_output;
}

/* -------------------------------------------------------------------------- */
// Create a 3D mesh from 2D corners in an image (coded as a Frame class).
void Mesher::populate3dMeshTimeHorizon(
      const std::vector<cv::Vec6f>& mesh_2d, // cv::Vec6f assumes triangular mesh.
      const std::map<LandmarkId, gtsam::Point3>& points_with_id_map,
      const Frame& frame) {
  // Note: we restrict to valid triangles in which each landmark has a 3D point.
  // Iterate over each face in the 2d mesh, and generate the 3d mesh.

  // Create polygon and add it to the mesh.
  Mesh3D::Polygon polygon;
  polygon.resize(3);
  const auto& points_with_id_map_end = points_with_id_map.end();
  // Iterate over the 2d mesh triangles.
  for (size_t i = 0; i < mesh_2d.size(); i++) {
    const cv::Vec6f& triangle_2d = mesh_2d.at(i);

    // Iterate over each vertex (pixel) of the triangle.
    for (size_t j = 0; j < triangle_2d.rows / 2; j++) {
      // Extract pixel.
      const cv::Point2f pixel (triangle_2d[j * 2],
                               triangle_2d[j * 2 + 1]);

      // Extract landmark id corresponding to this pixel.
      const LandmarkId id_pt (frame.findLmkIdFromPixel(pixel));

      // Try to find this landmark id in points_with_id_map.
      const auto& lmk_it = points_with_id_map.find(id_pt);
      if (lmk_it != points_with_id_map_end) {
        // We found the landmark.
        // Extract 3D position of the landmark.
        const gtsam::Point3& point (lmk_it->second);
        cv::Point3f lmk(float(point.x()),
                        float(point.y()),
                        float(point.z()));
        // Add landmark as one of the vertices of the current polygon in 3D.
        polygon.at(j) = Mesh3D::Vertex(id_pt, lmk);
        static const size_t loop_end = triangle_2d.rows / 2 - 1;
        if (j == loop_end) {
          // Last iteration.
          // Save the valid triangular polygon, since it has all vertices in
          // points_with_id_map.
          mesh_.addPolygonToMesh(polygon);
        }
      } else {
        // Do not save current polygon, since it has at least one vertex that
        // is not in points_with_id_map.
        LOG(ERROR) << "Landmark with id : " << lmk_it->first
                   << ", could not be found in points_with_id_map. "
                   << "But it should have been.\n";
        break;
      }
    }
  }

  // Remove faces in the mesh that have vertices which are not in
  // points_with_id_map anymore.
  reducePolygonMeshToTimeHorizon(points_with_id_map);
}

// TODO the polygon_mesh has repeated faces...
// And this seems to slow down quite a bit the for loop!
void Mesher::reducePolygonMeshToTimeHorizon(
                const std::map<LandmarkId, gtsam::Point3>& points_with_id_map) {
  Mesh3D mesh_output;

  auto end = points_with_id_map.end();
  // Loop over each face in the mesh.
  Mesh3D::Polygon polygon;

  for (size_t i = 0; i < mesh_.getNumberOfPolygons(); i++) {
    CHECK(mesh_.getPolygon(i, &polygon)) << "Could not retrieve polygon.";
    bool save_polygon = true;
    for (const Mesh3D::Vertex& vertex: polygon) {
      if (points_with_id_map.find(vertex.getLmkId()) == end) {
        // Vertex of current polygon is not in points_with_id_map
        // Delete the polygon by not adding it to the new mesh.
        save_polygon = false;
        break;
      }
    }

    if (save_polygon) {
      mesh_output.addPolygonToMesh(polygon);
    }
  }

  mesh_ = mesh_output;
}

/* -------------------------------------------------------------------------- */
// Calculate normals of polygonMesh.
void Mesher::calculateNormals(std::vector<cv::Point3f>* normals) {
  CHECK_NOTNULL(normals);
  CHECK_EQ(mesh_.getMeshPolygonDimension(), 3)
      << "Expecting 3 vertices in triangle.";

  // Brute force, ideally only call when a new triangle appears...
  normals->clear();
  normals->resize(mesh_.getNumberOfPolygons()); // TODO Assumes we have triangles...

  // Loop over each polygon face in the mesh.
  // TODO there are far too many loops over the total number of Polygon faces...
  // Should put them all in the same loop!
  Mesh3D::Polygon polygon;
  for (size_t i = 0; i < mesh_.getNumberOfPolygons(); i++) {
    CHECK(mesh_.getPolygon(i, &polygon)) << "Could not retrieve polygon.";
    CHECK_EQ(polygon.size(), 3);
    const Mesh3D::VertexPosition3D& p1 = polygon.at(0).getVertexPosition();
    const Mesh3D::VertexPosition3D& p2 = polygon.at(1).getVertexPosition();
    const Mesh3D::VertexPosition3D& p3 = polygon.at(2).getVertexPosition();

    cv::Point3f normal;
    calculateNormal(p1, p2, p3, &normal);

    // Store normal to triangle i.
    normals->at(i) = normal;
  }
}

/* -------------------------------------------------------------------------- */
// Calculate normal of a triangle.
void Mesher::calculateNormal( const Mesh3D::VertexPosition3D& p1,
                              const Mesh3D::VertexPosition3D& p2,
                              const Mesh3D::VertexPosition3D& p3,
                              cv::Point3f* normal) {
  CHECK_NOTNULL(normal);
  // Calculate vectors of the triangle.
  cv::Point3f v21 = p2 - p1;
  cv::Point3f v31 = p3 - p1;

  // Calculate normal (cross product).
  *normal = v21.cross(v31);

  // Normalize.
  double norm = cv::norm(v21) * cv::norm(v31);
  CHECK_NE(norm, 0) << "Norm is 0.";
  *normal /= norm;
}

/* -------------------------------------------------------------------------- */
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

/* -------------------------------------------------------------------------- */
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

/* -------------------------------------------------------------------------- */
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

/* -------------------------------------------------------------------------- */
// Is normal perpendicular to axis?
bool Mesher::isNormalPerpendicularToAxis(const cv::Point3f& axis,
                                         const cv::Point3f& normal,
                                         const double& tolerance) const {
    return ((cv::norm(normal.dot(axis)) < tolerance)? true: false);
}

/* -------------------------------------------------------------------------- */
// Filter z component in triangle cluster.
void Mesher::clusterZComponent(
    const double& z,
    const double& tolerance,
    TriangleCluster* triangle_cluster) {
  CHECK_NOTNULL(triangle_cluster);
  TriangleCluster triangle_cluster_output;
  triangle_cluster_output.cluster_id_ =
      triangle_cluster->cluster_id_;
  triangle_cluster_output.cluster_direction_ =
      triangle_cluster->cluster_direction_;

  // TODO consider using erase or a list, instead of creating a new one
  // and then copying!!!
  const double min_z = z - tolerance;
  const double max_z = z + tolerance;
  Mesh3D::Polygon polygon;
  Mesh3D::VertexPosition3D lmk;
  for (const size_t& polygon_idx: triangle_cluster->triangle_ids_) {
    CHECK(mesh_.getPolygon(polygon_idx, &polygon))
        << "Polygon, with idx " << polygon_idx << ", is not in the mesh.";
    bool save_polygon = true;
    for (const Mesh3D::Vertex& vertex: polygon) {
      lmk = vertex.getVertexPosition();
      if (lmk.z <= min_z || lmk.z >= max_z) {
        // Remove current polygon from cluster.
        save_polygon = false;
        break;
      }
    }

    if (save_polygon) {
      triangle_cluster_output.triangle_ids_.push_back(polygon_idx);
    }
  }
  *triangle_cluster = triangle_cluster_output;
}

/* -------------------------------------------------------------------------- */
void Mesher::clusterMesh(std::vector<TriangleCluster>* clusters) {
  CHECK_NOTNULL(clusters);

  // Cluster triangles oriented along z axis.
  static const cv::Point3f z_axis(0, 0, 1);

  TriangleCluster z_triangle_cluster;
  z_triangle_cluster.cluster_direction_ = z_axis;
  z_triangle_cluster.cluster_id_ = 2;

  // Cluster triangles with normal perpendicular to z_axis, aka along equator.
  TriangleCluster equatorial_triangle_cluster;
  equatorial_triangle_cluster.cluster_direction_ = z_axis;
  equatorial_triangle_cluster.cluster_id_ = 0;

  // Calculate normals of the triangles in the mesh.
  // The normals are in the world frame of reference.
  std::vector<cv::Point3f> normals;
  // Brute force, ideally only call when a new triangle appears...
  normals.clear();
  normals.resize(mesh_.getNumberOfPolygons()); // TODO Assumes we have triangles...

  CHECK_EQ(mesh_.getMeshPolygonDimension(), 3)
      << "Expecting 3 vertices in triangle.";

  // Loop over each polygon face in the mesh.
  Mesh3D::Polygon polygon;
  for (size_t i = 0; i < mesh_.getNumberOfPolygons(); i++) {
    CHECK(mesh_.getPolygon(i, &polygon)) << "Could not retrieve polygon.";
    CHECK_EQ(polygon.size(), 3);
    const Mesh3D::VertexPosition3D& p1 = polygon.at(0).getVertexPosition();
    const Mesh3D::VertexPosition3D& p2 = polygon.at(1).getVertexPosition();
    const Mesh3D::VertexPosition3D& p3 = polygon.at(2).getVertexPosition();

    cv::Point3f normal;
    calculateNormal(p1, p2, p3, &normal);

    // Store normal to triangle i.
    normals.at(i) = normal;

    static constexpr double normal_tolerance = 0.2; // 0.087 === 10 deg. aperture.
    static constexpr double normal_tolerance_perpendicular = 0.1;
    if (isNormalAroundAxis(z_axis,
                           normal,
                           normal_tolerance)) {
      // Cluster Normal around z_axis.
      z_triangle_cluster.triangle_ids_.push_back(i);
    } else if (isNormalPerpendicularToAxis(z_axis,
                                           normal,
                                           normal_tolerance_perpendicular)) {
      // Cluster Normal perpendicular to z_axis.
      equatorial_triangle_cluster.triangle_ids_.push_back(i);
    }
  }

  // Append clusters.
  clusters->push_back(z_triangle_cluster);
  clusters->push_back(equatorial_triangle_cluster);

  // Only keep ground landmarks for cluster of triangles perpendicular
  // to vertical axis.
  // clusters.at(0) is therefore just the triangles on the ground plane.
  static constexpr double z = -0.1;
  static constexpr double tolerance = 0.10;
  clusterZComponent(z, tolerance,
                   &(clusters->at(0)));
}

/* -------------------------------------------------------------------------- */
// Update mesh: update structures keeping memory of the map before visualization
void Mesher::updateMesh3D(
       const std::vector<std::pair<LandmarkId, gtsam::Point3>>& pointsWithIdVIO,
       std::shared_ptr<StereoFrame> stereoFrame,
       const gtsam::Pose3& leftCameraPose,
       const float& maxGradInTriangle,
       const double& minRatioBetweenLargestAnSmallestSide,
       const double& min_elongation_ratio,
       const double& maxTriangleSide) {
  // Build 2D mesh.
  std::vector<cv::Vec6f> mesh_2d;
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

  // Convert points_with_id to a map, otherwise following algorithms are
  // ridiculously slow.
  const std::map<LandmarkId, gtsam::Point3> points_with_id_map (
        pointsWithIdVIO.begin(),
        pointsWithIdVIO.end());

  populate3dMeshTimeHorizon(
        mesh_2d,
        points_with_id_map,
        stereoFrame->left_frame_);

  filterOutBadTriangles(leftCameraPose,
                        minRatioBetweenLargestAnSmallestSide,
                        min_elongation_ratio,
                        maxTriangleSide);
}

/* -------------------------------------------------------------------------- */
void Mesher::extractLmkIdsFromTriangleCluster(
    const TriangleCluster& triangle_cluster,
    LandmarkIds* lmk_ids) {
  CHECK_NOTNULL(lmk_ids);
  lmk_ids->resize(0);

  Mesh3D::Polygon polygon;
  for (const size_t& polygon_idx: triangle_cluster.triangle_ids_) {
    CHECK(mesh_.getPolygon(polygon_idx, &polygon))
        << "Polygon, with idx " << polygon_idx << ", is not in the mesh.";
    for (const Mesh3D::Vertex& vertex: polygon) {
      lmk_ids->push_back(vertex.getLmkId());
    }
  }
}

/* -------------------------------------------------------------------------- */
void Mesher::getVerticesMesh(cv::Mat* vertices_mesh) const {
  CHECK_NOTNULL(vertices_mesh);
  mesh_.convertVerticesMeshToMat(vertices_mesh);
}
void Mesher::getPolygonsMesh(cv::Mat* polygons_mesh) const {
  CHECK_NOTNULL(polygons_mesh);
  mesh_.convertPolygonsMeshToMat(polygons_mesh);
}

} // namespace VIO
