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
 * @author Luca Carlone, AJ Haeffner
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
  MESH2DTo3Ddense, // dense triangulation of stereo corners (only a subset are VIO keypoints)
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

    if(points.size() == 0) // no points to visualize
      return;

    // Populate cloud structure with 3D points.
    cv::Mat pointCloud(1, points.size(), CV_32FC3);
    cv::Point3f* data = pointCloud.ptr<cv::Point3f>();
    for(size_t i = 0; i < points.size();i++){
      data[i].x = float(points.at(i).x());
      data[i].y = float(points.at(i).y());
      data[i].z = float(points.at(i).z());
    }

    // Add to the existing map.
    cv::viz::WCloudCollection map_with_repeated_points;
    map_with_repeated_points.addCloud(pointCloud, cloud_color_);
    map_with_repeated_points.setRenderingProperty(cv::viz::POINT_SIZE, 2);

    // Plot points.
    window_.showWidget("point cloud map", map_with_repeated_points);

    /// Start event loop.
    window_.spinOnce(waitTime);
  }

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizePoints3D(
          const std::vector<std::pair<LandmarkId, gtsam::Point3>>& pointsWithId,
          const cv::Mat& map_points_3d, const int& waitTime = 0) {

    // Sanity check dimension.
    if(pointsWithId.size() == 0) // no points to visualize
      return;

    // Create a cloud widget.
    cv::viz::WCloud cloud_widget(map_points_3d, cloud_color_);
    cloud_widget.setRenderingProperty(cv::viz::POINT_SIZE, 2);

    window_.showWidget("point cloud map", cloud_widget);
    /// Start event loop.
    window_.spinOnce(waitTime);
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
    if(map_points_3d.rows == 0 || polygons_mesh.rows == 0)
      return;

    // Create a cloud widget.
    cv::viz::WMesh mesh(map_points_3d.t(), polygons_mesh, colors.t());
    window_.showWidget("Mesh", mesh); // plot mesh
    /// Start event loop.
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
  // Add pose to the previous trajectory.
  void addPoseToTrajectory(gtsam::Pose3 current_pose_gtsam){
    trajectoryPoses3d_.push_back(UtilsOpenCV::Pose2Affine3f(current_pose_gtsam));
  }

private:
  cv::viz::Viz3d window_;
  std::vector<cv::Affine3f> trajectoryPoses3d_;
  cv::viz::Color cloud_color_ = cv::viz::Color::white();
  cv::viz::Color background_color_ = cv::viz::Color::black();

  /* ----------------------------------------------------------------------------- */
  // Log mesh to ply file.
  void logMesh(const cv::Mat& map_points_3d, const cv::Mat& colors,
               const cv::Mat& polygons_mesh) {
    /// Log the mesh in a ply file.
    LoggerMatlab logger;
    logger.openLogFiles(10);
    logger.logMesh(map_points_3d, colors, polygons_mesh);
    logger.closeLogFiles(10);
  }

  /* ----------------------------------------------------------------------------- */
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

};
} // namespace VIO
#endif /* Visualizer_H_ */


