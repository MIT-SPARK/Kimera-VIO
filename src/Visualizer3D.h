/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Visualizer.h
 * @brief  Build and visualize 2D mesh from Frame
 * @author Luca Carlone, AJ Haeffner, Antoni Rosinol
 */

#ifndef Visualizer_H_
#define Visualizer_H_

#include "UtilsOpenCV.h"
#include "LoggerMatlab.h"
#include "glog/logging.h"
#ifdef USE_CGAL
#include "Mesher_cgal.h"
#endif

namespace VIO {

enum class VisualizationType {
  POINTCLOUD, // visualize 3D VIO points  (no repeated point)
  POINTCLOUD_REPEATEDPOINTS, // visualize VIO points as point clouds (points are re-plotted at every frame)
  MESH2D, // only visualizes 2D mesh on image
  MESH2DTo3D, // get a 3D mesh from a 2D triangulation of the (right-VALID) keypoints in the left frame
  MESH2Dsparse, // visualize a 2D mesh of (right-valid) keypoints discarding triangles corresponding to non planar obstacles
  MESH2DTo3Dsparse, // same as MESH2DTo3D but filters out triangles corresponding to non planar obstacles
  MESH3D, // 3D mesh from CGAL using VIO points (requires #define USE_CGAL!)
  NONE // does not visualize map
};

class Visualizer3D {
public:
  Visualizer3D(): window_("3D Visualizer") {
    // Create window and create axes:
    window_.setBackgroundColor(background_color_);
    window_.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  }

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud using cloud widget from opencv viz.
  void visualizeMap3D(const std::vector<gtsam::Point3>& points,
                      const int waitTime = 0) {
    // based on longer example: https://docs.opencv.org/2.4/doc/tutorials/viz/transformations/transformations.html#transformations

    // No points to visualize.
    if (points.size() == 0) {
      return;
    }

    // Populate cloud structure with 3D points.
    cv::Mat pointCloud(1, points.size(), CV_32FC3);
    cv::Point3f* data = pointCloud.ptr<cv::Point3f>();
    size_t i = 0;
    for (const gtsam::Point3& point: points) {
      data[i].x = float(point.x());
      data[i].y = float(point.y());
      data[i].z = float(point.z());
      i++;
    }

    // Add to the existing map.
    cv::viz::WCloudCollection map_with_repeated_points;
    map_with_repeated_points.addCloud(pointCloud, cloud_color_);
    map_with_repeated_points.setRenderingProperty(cv::viz::POINT_SIZE, 2);

    // Plot points.
    window_.showWidget("Point cloud map", map_with_repeated_points);

    // Start event loop.
    window_.spinOnce(waitTime);
  }

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizePoints3D(
          const std::unordered_map<LandmarkId, gtsam::Point3>& points_with_id,
          const int& waitTime = 0) {

    // Sanity check dimension.
    if (points_with_id.size() == 0) {
      // No points to visualize.
      return;
    }

    // Populate cloud structure with 3D points.
    cv::Mat point_cloud(1, points_with_id.size(), CV_32FC3);
    cv::Point3f* data = point_cloud.ptr<cv::Point3f>();
    size_t i = 0;
    for (const std::pair<LandmarkId, gtsam::Point3>& id_point: points_with_id) {
      data[i].x = float(id_point.second.x());
      data[i].y = float(id_point.second.y());
      data[i].z = float(id_point.second.z());
      i++;
    }

    // Create a cloud widget.
    cv::viz::WCloud cloud_widget(point_cloud, cloud_color_);
    cloud_widget.setRenderingProperty(cv::viz::POINT_SIZE, 2);

    window_.showWidget("Point cloud.", cloud_widget);

    // Start event loop.
    window_.spinOnce(waitTime);
  }

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizePlane(
      const std::string& plane_id,
      const double& n_x,
      const double& n_y,
      const double& n_z,
      const double& d,
      const int& waitTime = 0) {
    // Create a plane widget.
    const Vec3d normal (n_x, n_y, n_z);
    const Point3d center (d * n_x, d * n_y, d * n_z);
    static const Vec3d new_yaxis (0, 1, 0);
    static const Size2d size (1.0, 1.0);
    static const cv::viz::Color plane_color = cv::viz::Color::blue();
    cv::viz::WPlane plane_widget (center, normal, new_yaxis, size, plane_color);

    window_.showWidget(plane_id, plane_widget);

    // Start event loop.
    window_.spinOnce(waitTime);
  }


  /* ------------------------------------------------------------------------ */
  // Draw a line in opencv.
  void drawLine(const std::string& line_id,
                const double& from_x,
                const double& from_y,
                const double& from_z,
                const double& to_x,
                const double& to_y,
                const double& to_z) {
    cv::Point3d pt1 (from_x, from_y, from_z);
    cv::Point3d pt2 (to_x, to_y, to_z);
    drawLine(line_id, pt1, pt2);
  }

  void drawLine(const std::string& line_id,
                const cv::Point3d& pt1, const cv::Point3d& pt2) {
    cv::viz::WLine line_widget (pt1, pt2);
    window_.showWidget(line_id, line_widget);
  }

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizeMesh3D(const cv::Mat& mapPoints3d, const cv::Mat& polygonsMesh) {
    cv::Mat colors (mapPoints3d.rows, 1, CV_8UC3, cv::viz::Color::gray());
    visualizeMesh3D(mapPoints3d, polygonsMesh, colors);
  }

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity,
  // and provide color for each polygon.
  void visualizeMesh3D(const cv::Mat& map_points_3d, const cv::Mat& colors,
                       const cv::Mat& polygons_mesh) {
    // Check data
    CHECK_EQ(map_points_3d.rows, colors.rows) << "Map points and Colors should "
                                                 "have same number of rows. One"
                                                 " color per map point.";
    // No points/mesh to visualize.
    if (map_points_3d.rows == 0 ||
        polygons_mesh.rows == 0) {
      return;
    }

    // Create a cloud widget.
    cv::viz::WMesh mesh(map_points_3d.t(), polygons_mesh, colors.t());

    // Plot mesh.
    window_.showWidget("Mesh", mesh);

    // Start event loop.
    window_.spinOnce(1);
  }

  /* ------------------------------------------------------------------------ */
  ///Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  /// Each triangle is colored depending on the cluster it is in, or gray if it
  /// is in no cluster.
  /// [in] clusters: a set of triangle clusters. The ids of the triangles must
  ///  match the order in polygons_mesh.
  /// [in] map_points_3d: set of 3d points in the mesh, format is n rows, with
  ///  three columns (x, y, z).
  /// [in] polygons_mesh: mesh faces, format is n rows, 1 column,
  ///  with [n id_a id_b id_c, ..., n /id_x id_y id_z], where n = polygon size
  ///  n=3 for triangles.
  void visualizeMesh3DWithColoredClusters(
                           const std::vector<TriangleCluster>& clusters,
                           const cv::Mat& map_points_3d,
                           const cv::Mat& polygons_mesh) {
    // Color the mesh.
    cv::Mat colors;
    colorMeshByClusters(clusters, map_points_3d, polygons_mesh, &colors);

    // Log the mesh.
    static constexpr bool log_mesh = false;
    if (log_mesh) {
      logMesh(map_points_3d, colors, polygons_mesh);
    }

    // Visualize the mesh.
    visualizeMesh3D(map_points_3d, colors, polygons_mesh);
  }

  /* ------------------------------------------------------------------------ */
  // Visualize trajectory.
  void visualizeTrajectory3D(const cv::Mat* frustum_image = nullptr){
    if(trajectoryPoses3d_.size() == 0) // no points to visualize
      return;
    // Show current camera pose.
    static const cv::Matx33d K (458, 0.0, 360,
                                0.0, 458, 240,
                                0.0, 0.0, 1.0);
    cv::viz::WCameraPosition cam_widget_ptr;
    if (frustum_image == nullptr) {
      cam_widget_ptr = cv::viz::WCameraPosition(K, 1.0, cv::viz::Color::white());
    } else {
      cv::Mat display_img;
      cv::rotate(*frustum_image, display_img, cv::ROTATE_90_CLOCKWISE);
      cam_widget_ptr = cv::viz::WCameraPosition(K, display_img,
                                                 1.0, cv::viz::Color::white());
    }
    window_.showWidget("Camera Pose with Frustum", cam_widget_ptr,
                         trajectoryPoses3d_.back());
    window_.setWidgetPose("Camera Pose with Frustum", trajectoryPoses3d_.back());

    // Create a Trajectory widget. (argument can be PATH, FRAMES, BOTH).
    cv::viz::WTrajectory trajectory_widget (trajectoryPoses3d_,
                                            cv::viz::WTrajectory::PATH,
                                            1.0, cv::viz::Color::red());
    window_.showWidget("Trajectory", trajectory_widget);
    /// Start event loop.
    window_.spinOnce(1);
  }

  /* ------------------------------------------------------------------------ */
  // Remove widget.
  void removeWidget(const std::string& widget_id){
    try {
      window_.removeWidget(widget_id);
    } catch (const cv::Exception& e) {
      VLOG(20) << e.what();
    }
  }

  /* ------------------------------------------------------------------------ */
  // Remove line widgets from plane to lmks.
  // Point key is required to avoid duplicated lines!
  void visualizePlaneConstraints(const gtsam::Point3& normal,
                                 const double& distance,
                                 const Key& point_key,
                                 const gtsam::Point3& point) {
    const auto& point_key_to_line_id =
        point_key_to_line_id_map_.find(point_key);
    if (point_key_to_line_id ==
        point_key_to_line_id_map_.end()) {
      // We have never drawn this line.
      // Store line nr (as line id).
      point_key_to_line_id_map_[point_key] = line_nr_;
      std::string line_id =  "Line " + line_nr_;
      // Draw it.
      drawLineFromPlaneToPoint(line_id,
                               normal.x(), normal.y(), normal.z(), distance,
                               point.x(), point.y(), point.z());
      // Augment line_nr for next line_id.
      line_nr_++;
    } else {
      // We have drawn this line before.
      // Update line.
      updateLineFromPlaneToPoint(
            "Line " + point_key_to_line_id->second,
            normal.x(), normal.y(), normal.z(), distance,
            point.x(), point.y(), point.z());

    }
  }

  /* ------------------------------------------------------------------------ */
  // Remove line widgets from plane to lmks.
  void removePlaneConstraintsViz() {
    for (size_t i = 0; i < line_nr_; i++) {
      removeWidget("Line " + i);
    }
    line_nr_ = 0;
    point_key_to_line_id_map_.clear();
  }

  /* ------------------------------------------------------------------------ */
  // Add pose to the previous trajectory.
  void addPoseToTrajectory(gtsam::Pose3 current_pose_gtsam){
    trajectoryPoses3d_.push_back(UtilsOpenCV::Pose2Affine3f(current_pose_gtsam));
  }

private:
  cv::viz::Viz3d window_;
  std::vector<cv::Affine3f> trajectoryPoses3d_;
  cv::viz::Color cloud_color_ = cv::viz::Color::white();
  cv::viz::Color background_color_ = cv::viz::Color::black();

  size_t line_nr_ = 0;
  std::map<Key, size_t> point_key_to_line_id_map_;

  /* ------------------------------------------------------------------------ */
  // Log mesh to ply file.
  void logMesh(const cv::Mat& map_points_3d, const cv::Mat& colors,
               const cv::Mat& polygons_mesh) {
    /// Log the mesh in a ply file.
    LoggerMatlab logger;
    logger.openLogFiles(10);
    logger.logMesh(map_points_3d, colors, polygons_mesh);
    logger.closeLogFiles(10);
  }

  /* ------------------------------------------------------------------------ */
  // Input the mesh points and triangle clusters, and
  // output colors matrix for mesh visualizer.
  void colorMeshByClusters(const std::vector<TriangleCluster>& clusters,
                           const cv::Mat& map_points_3d,
                           const cv::Mat& polygons_mesh,
                           cv::Mat* colors) {
    CHECK_NOTNULL(colors);
    *colors = cv::Mat(map_points_3d.rows, 1, CV_8UC3,
                      cv::viz::Color::gray());
    // The code below assumes triangles as polygons.
    static constexpr bool log_landmarks = false;
    cv::Mat points;
    for (const TriangleCluster& cluster: clusters) {
      // Decide color for cluster.
      cv::viz::Color cluster_color = cv::viz::Color::gray();
      switch (cluster.cluster_id_) {
      case 0: {
        cluster_color = cv::viz::Color::red();
        break;
      }
      case 1: {
        cluster_color = cv::viz::Color::green();
        break;
      }
      case 2:{
        cluster_color = cv::viz::Color::blue();
        break;
      }
      default :{
        break;
      }
      }

      for (const size_t& triangle_id: cluster.triangle_ids_) {
        size_t triangle_idx = std::round(triangle_id * 4);
        if (triangle_idx + 3 >= polygons_mesh.rows) {
          throw std::runtime_error("Visualizer3D: an id in triangle_ids_ is"
                                   " too large.");
        }
        int32_t idx_1 = polygons_mesh.at<int32_t>(triangle_idx + 1);
        int32_t idx_2 = polygons_mesh.at<int32_t>(triangle_idx + 2);
        int32_t idx_3 = polygons_mesh.at<int32_t>(triangle_idx + 3);
        colors->row(idx_1) = cluster_color;
        colors->row(idx_2) = cluster_color;
        colors->row(idx_3) = cluster_color;
        // Debug TODO remove: logging triangles perpendicular to z_axis.
        if (cluster.cluster_id_ == 2 && log_landmarks) {
          points.push_back(map_points_3d.row(idx_1));
          points.push_back(map_points_3d.row(idx_2));
          points.push_back(map_points_3d.row(idx_3));
        }
      }
    }

    // Debug TODO remove
    /// Store in file
    if (log_landmarks) {
      LoggerMatlab logger;
      logger.openLogFiles(3);
      logger.logLandmarks(points);
      logger.closeLogFiles(3);
    }
  }

  /* ------------------------------------------------------------------------ */
  // Draw a line from lmk to plane center.
  void drawLineFromPlaneToPoint(
      const std::string& line_id,
      const double& plane_n_x,
      const double& plane_n_y,
      const double& plane_n_z,
      const double& plane_d,
      const double& point_x,
      const double& point_y,
      const double& point_z) {
    const cv::Point3d center (plane_d * plane_n_x,
                              plane_d * plane_n_y,
                              plane_d * plane_n_z);
    const cv::Point3d point(point_x, point_y, point_z);
    drawLine(line_id, center, point);
  }

  /* ------------------------------------------------------------------------ */
  // Update line from lmk to plane center.
  void updateLineFromPlaneToPoint(
      const std::string& line_id,
      const double& plane_n_x,
      const double& plane_n_y,
      const double& plane_n_z,
      const double& plane_d,
      const double& point_x,
      const double& point_y,
      const double& point_z) {
    removeWidget(line_id);
    drawLineFromPlaneToPoint(line_id, plane_n_x, plane_n_y, plane_n_z,
                             plane_d, point_x, point_y, point_z);
  }

};
} // namespace VIO
#endif /* Visualizer_H_ */


