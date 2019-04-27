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
 * @author Antoni Rosinol, AJ Haeffner, Luca Carlone
 */

#pragma once

#include <vector>

#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "utils/ThreadsafeQueue.h"
#include "mesh/Mesher.h"
#include "UtilsOpenCV.h"
#include "VioBackEnd.h"

namespace VIO {

enum class VisualizationType {
  POINTCLOUD, // visualize 3D VIO points  (no repeated point)
  POINTCLOUD_REPEATEDPOINTS, // visualize VIO points as point clouds (points are re-plotted at every frame)
  MESH2D, // only visualizes 2D mesh on image
  MESH2Dsparse, // visualize a 2D mesh of (right-valid) keypoints discarding triangles corresponding to non planar obstacles
  MESH2DTo3Dsparse, // same as MESH2DTo3D but filters out triangles corresponding to non planar obstacles
  NONE // does not visualize map
};

template<class T>
static bool getEstimateOfKey(const gtsam::Values& state,
                             const gtsam::Key& key,
                             T* estimate);

struct VisualizerInputPayload {
  VisualizerInputPayload(
      const VisualizationType& visualization_type,
      int backend_type,
      const gtsam::Pose3& pose,
      const std::vector<cv::Vec6f>& mesh_2d,
      const Frame& left_stero_keyframe,
      const MesherOutputPayload& mesher_output_payload,
      const VioBackEnd::PointsWithIdMap& points_with_id_VIO,
      const VioBackEnd::LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map,
      const std::vector<Plane>& planes,
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& values,
      const vector<Point3>& points_3d,
      const Timestamp& timestamp_k);

  const VisualizationType visualization_type_;
  const int backend_type_;
  const gtsam::Pose3 pose_;
  const std::vector<cv::Vec6f> mesh_2d_;
  const Frame left_stereo_keyframe_;
  const MesherOutputPayload mesher_output_payload_;
  const VioBackEnd::PointsWithIdMap points_with_id_VIO_;
  const VioBackEnd::LmkIdToLmkTypeMap lmk_id_to_lmk_type_map_;
  const std::vector<Plane> planes_;
  const gtsam::NonlinearFactorGraph graph_;
  const gtsam::Values values_;
  const vector<Point3> points_3d_;
  const Timestamp timestamp_k_;
};

struct ImageToDisplay {
  ImageToDisplay () = default;
  ImageToDisplay (const std::string& name, const cv::Mat& image);

  std::string name_;
  cv::Mat image_;
};

struct VisualizerOutputPayload {
  VisualizerOutputPayload() = default;
  VisualizationType visualization_type_ = VisualizationType::NONE;
  std::vector<ImageToDisplay> images_to_display_;
  cv::viz::Viz3d window_ = cv::viz::Viz3d("3D Visualizer");
};

class Visualizer3D {
public:
  Visualizer3D();

  typedef size_t LineNr;
  typedef std::uint64_t PlaneId;
  typedef std::map<LandmarkId, size_t> LmkIdToLineIdMap;
  typedef std::map<PlaneId, LmkIdToLineIdMap> PlaneIdMap;

  // Contains internal data for Visualizer3D window.
  struct WindowData {
    cv::viz::Viz3d window_ = cv::viz::Viz3d("3D Visualizer");
    cv::viz::Color cloud_color_ = cv::viz::Color::white();
    cv::viz::Color background_color_ = cv::viz::Color::black();
    // Defines whether the user pressed a key to switch the mesh representation.
    bool user_updated_mesh_representation_ = false;
    // Stores the user set mesh representation.
    int mesh_representation_ = 0u;
  };

  /* ------------------------------------------------------------------------ */
  // Spin for Visualizer3D. Calling shutdown stops the visualizer.
  void spin(ThreadsafeQueue<VisualizerInputPayload>& input_queue,
            ThreadsafeQueue<VisualizerOutputPayload>& output_queue);

  /* ------------------------------------------------------------------------ */
  // Stops the visualization spin.
  void shutdown();

  /* ------------------------------------------------------------------------ */
  // Returns true if visualization is ready, false otherwise.
  // The actual visualization must be done in the main thread, and as such,
  // it is not done here to separate visualization preparation from display.
  bool visualize(const std::shared_ptr<VisualizerInputPayload>& input,
                 VisualizerOutputPayload* output);

  /* ------------------------------------------------------------------------ */
  // Returns true if visualization is ready, false otherwise.
  // The actual visualization must be done in the main thread, and as such,
  // it is not done here to separate visualization preparation from display.
  bool visualize(const VisualizerInputPayload& input,
                 VisualizerOutputPayload* output);

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud using cloud widget from opencv viz.
  void visualizeMap3D(const std::vector<gtsam::Point3>& points);

  /* ------------------------------------------------------------------------ */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  cv::Mat visualizeMesh2D(
      const std::vector<cv::Vec6f>& triangulation2D,
      const cv::Mat& img,
      const KeypointsCV& extra_keypoints = KeypointsCV()) const;

  /* ------------------------------------------------------------------------ */
  // Visualize 2d mesh.
  cv::Mat visualizeMesh2DStereo(
      const std::vector<cv::Vec6f>& triangulation_2D,
      const Frame& ref_frame) const;

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizePoints3D(
      const std::unordered_map<LandmarkId, gtsam::Point3>& points_with_id,
      const std::unordered_map<LandmarkId, LandmarkType>& lmk_id_to_lmk_type_map);

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizePlane(const PlaneId& plane_index,
                      const double& n_x,
                      const double& n_y,
                      const double& n_z,
                      const double& d,
                      const bool& visualize_plane_label = true,
                      const int& cluster_id = 1);

  /* ------------------------------------------------------------------------ */
  // Draw a line in opencv.
  void drawLine(const std::string& line_id,
                const double& from_x,
                const double& from_y,
                const double& from_z,
                const double& to_x,
                const double& to_y,
                const double& to_z);

  /* ------------------------------------------------------------------------ */
  void drawLine(const std::string& line_id,
                const cv::Point3d& pt1, const cv::Point3d& pt2);

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizeMesh3D(const cv::Mat& mapPoints3d, const cv::Mat& polygonsMesh);

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity,
  // and provide color for each polygon.
  void visualizeMesh3D(const cv::Mat& map_points_3d,
                       const cv::Mat& colors,
                       const cv::Mat& polygons_mesh);

  /* ------------------------------------------------------------------------ */
  ///Visualize a PLY from filename (absolute path).
  void visualizePlyMesh(const std::string& filename);

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
  /// [in] color_mesh whether to color the mesh or not
  /// [in] timestamp to store the timestamp of the mesh when logging the mesh.
  void visualizeMesh3DWithColoredClusters(
      const std::vector<Plane>& planes,
      const cv::Mat& map_points_3d,
      const cv::Mat& polygons_mesh,
      const bool& color_mesh = false,
      const Timestamp& timestamp = 0.0);

  /* ------------------------------------------------------------------------ */
  // Visualize convex hull in 2D for set of points in triangle cluster,
  // projected along the normal of the cluster.
  void visualizeConvexHull (const TriangleCluster& cluster,
                            const cv::Mat& map_points_3d,
                            const cv::Mat& polygons_mesh);

  /* ------------------------------------------------------------------------ */
  // Visualize trajectory. Adds an image to the frustum if cv::Mat is not empty.
  void visualizeTrajectory3D(const cv::Mat& frustum_image);

  /* ------------------------------------------------------------------------ */
  // Remove widget. True if successful, false if not.
  bool removeWidget(const std::string& widget_id);

  /* ------------------------------------------------------------------------ */
  // Visualize line widgets from plane to lmks.
  // Point key is required to avoid duplicated lines!
  void visualizePlaneConstraints(const PlaneId& plane_id,
                                 const gtsam::Point3& normal,
                                 const double& distance,
                                 const LandmarkId& lmk_id,
                                 const gtsam::Point3& point);

  /* ------------------------------------------------------------------------ */
  // Remove line widgets from plane to lmks, for lines that are not pointing
  // to any lmk_id in lmk_ids.
  void removeOldLines(const LandmarkIds& lmk_ids);

  /* ------------------------------------------------------------------------ */
  // Remove line widgets from plane to lmks.
  void removePlaneConstraintsViz(const PlaneId& plane_id);

  /* ------------------------------------------------------------------------ */
  // Remove plane widget.
  void removePlane(const PlaneId& plane_index,
                   const bool& remove_plane_label = true);

  /* ------------------------------------------------------------------------ */
  // Add pose to the previous trajectory.
  void addPoseToTrajectory(const gtsam::Pose3& current_pose_gtsam);

  /* ------------------------------------------------------------------------ */
  // Render window with drawn objects/widgets.
  // @param wait_time Amount of time in milliseconds for the event loop to keep running.
  // @param force_redraw If true, window renders.
  void renderWindow(int wait_time = 1, bool force_redraw = true);

  /* ------------------------------------------------------------------------ */
  // Get a screenshot of the window.
  void getScreenshot(const std::string& filename);

  /* ------------------------------------------------------------------------ */
  // Useful for when testing on servers without display screen.
  void setOffScreenRendering();

private:
  // Shutdown flag to stop the visualization spin.
  std::atomic_bool shutdown_ = {false};

  std::deque<cv::Affine3f> trajectoryPoses3d_;

  std::map<PlaneId, LineNr> plane_to_line_nr_map_;
  PlaneIdMap plane_id_map_;
  std::map<PlaneId, bool> is_plane_id_in_window_;

  // Holds all visualization data including the window which contains 3D widgets.
  WindowData window_data_;

  /* ------------------------------------------------------------------------ */
  // Log mesh to ply file.
  void logMesh(const cv::Mat& map_points_3d, const cv::Mat& colors,
               const cv::Mat& polygons_mesh, const Timestamp& timestamp,
               bool log_accumulated_mesh = false);

  /* ------------------------------------------------------------------------ */
  // Input the mesh points and triangle clusters, and
  // output colors matrix for mesh visualizer.
  // This will color the point with the color of the last plane having it.
  void colorMeshByClusters(const std::vector<Plane>& planes,
                           const cv::Mat& map_points_3d,
                           const cv::Mat& polygons_mesh,
                           cv::Mat* colors) const;

  /* ------------------------------------------------------------------------ */
  // Decide color of the cluster depending on its id.
  void getColorById(const size_t& id, cv::viz::Color* color) const;


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
      const double& point_z);

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
      const double& point_z);

  /* ------------------------------------------------------------------------ */
  // Keyboard callback.
  static void keyboardCallback(const viz::KeyboardEvent &event, void *t) {
    Visualizer3D::WindowData* window_data = (Visualizer3D::WindowData*)t;
    toggleFreezeScreenKeyboardCallback(event.action, event.code, *window_data);
    setMeshRepresentation(event.action, event.code, *window_data);
    getViewerPoseKeyboardCallback(event.action, event.code, *window_data);
    getCurrentWindowSizeKeyboardCallback(event.action, event.code, *window_data);
    getScreenshotCallback(event.action, event.code, *window_data);
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to toggle freezing screen.
  static void toggleFreezeScreenKeyboardCallback(
      const viz::KeyboardEvent::Action& action,
      const uchar code,
      Visualizer3D::WindowData& window_data) {
    if (action == cv::viz::KeyboardEvent::Action::KEY_DOWN) {
      if (code == 't') {
        LOG(WARNING) << "Pressing " << code << " toggles freezing screen.";
        static bool freeze = false;
        freeze = !freeze; // Toggle.
        window_data.window_.spinOnce(1, true);
        while(!window_data.window_.wasStopped()) {
          if (freeze) {
            window_data.window_.spinOnce(1, true);
          } else {
            break;
          }
        }
      }
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to set mesh representation.
  static void setMeshRepresentation(
      const viz::KeyboardEvent::Action& action,
      const uchar code,
      Visualizer3D::WindowData& window_data) {
    if (action == cv::viz::KeyboardEvent::Action::KEY_DOWN) {
      if (code == '0') {
        LOG(WARNING) << "Pressing " << code << " sets mesh representation to "
                                               "a point cloud.";
        window_data.user_updated_mesh_representation_ = true;
        window_data.mesh_representation_ = 0u;
      } else if (code == '1') {
        LOG(WARNING) << "Pressing " << code << " sets mesh representation to "
                                               "a mesh.";
        window_data.user_updated_mesh_representation_ = true;
        window_data.mesh_representation_ = 1u;
      } else if (code == '2') {
        LOG(WARNING) << "Pressing " << code << " sets mesh representation to "
                                               "a wireframe.";
        window_data.user_updated_mesh_representation_ = true;
        window_data.mesh_representation_ = 2u;
      }
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to get current viewer pose.
  static void getViewerPoseKeyboardCallback(
      const viz::KeyboardEvent::Action& action,
      const uchar& code,
      Visualizer3D::WindowData& window_data) {
    if (action == cv::viz::KeyboardEvent::Action::KEY_DOWN) {
      if (code == 'v') {
        LOG(INFO) << "Current viewer pose:\n"
                  << "\tRodriguez vector: " << window_data.window_.getViewerPose().rvec()
                  << "\n\tAffine matrix: " << window_data.window_.getViewerPose().matrix;
      }
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to get current screen size.
  static void getCurrentWindowSizeKeyboardCallback(
      const viz::KeyboardEvent::Action& action,
      const uchar code,
      Visualizer3D::WindowData& window_data) {
    if (action == cv::viz::KeyboardEvent::Action::KEY_DOWN) {
      if (code == 'w') {
        LOG(WARNING) << "Pressing " << code << " displays current window size:\n"
                     << "\theight: " << window_data.window_.getWindowSize().height
                     << "\twidth: " << window_data.window_.getWindowSize().width;
      }
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to get screenshot of current windodw.
  static void getScreenshotCallback(
      const viz::KeyboardEvent::Action& action,
      const uchar code,
      Visualizer3D::WindowData& window_data) {
    if (action == cv::viz::KeyboardEvent::Action::KEY_DOWN) {
      if (code == 's') {
        static int i = 0;
        std::string filename = "screenshot_3d_window" + std::to_string(i);
        LOG(WARNING) << "Pressing " << code << " takes a screenshot of the "
                                               "window, saved in: " + filename;
        window_data.window_.saveScreenshot(filename);
      }
    }
  }
};

} // namespace VIO


