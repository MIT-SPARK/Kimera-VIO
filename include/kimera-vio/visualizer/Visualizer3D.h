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

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

namespace VIO {

class Visualizer3D {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(Visualizer3D);
  KIMERA_POINTER_TYPEDEFS(Visualizer3D);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef size_t LineNr;
  typedef std::uint64_t PlaneId;
  typedef std::map<LandmarkId, size_t> LmkIdToLineIdMap;
  typedef std::map<PlaneId, LmkIdToLineIdMap> PlaneIdMap;
  typedef std::function<void(VisualizerOutput&)> DisplayCallback;

  /**
   * @brief Visualizer3D constructor
   * @param viz_type: type of 3D visualization
   * @param backend_type backend used so that we display the right info
   */
  Visualizer3D(const VisualizationType& viz_type,
               const BackendType& backend_type);
  virtual ~Visualizer3D() { LOG(INFO) << "Visualizer3D destructor"; };

  // Contains internal data for Visualizer3D window.
  struct WindowData {
    WindowData();
    ~WindowData() {
      // OpenCV 3d Viz has an issue that I can't resolve, it throws a segfault
      // at the end of the program. Probably because of memory not released.
      // See issues in opencv git:
      // https://github.com/opencv/opencv/issues/11219 and many more...
      window_.close();
    };
    cv::viz::Viz3d window_;
    cv::viz::Color cloud_color_;
    cv::viz::Color background_color_;
    // Stores the user set mesh representation.
    int mesh_representation_;
    int mesh_shading_;
    bool mesh_ambient_;
    bool mesh_lighting_;
  };

  /* ------------------------------------------------------------------------ */
  inline void registerMesh3dVizProperties(
      Mesh3dVizPropertiesSetterCallback cb) {
    mesh3d_viz_properties_callback_ = cb;
  }

  /**
   * \brief Returns true if visualization is ready, false otherwise.
   * The actual visualization must be done in the main thread, and as such,
   * it is not done here to separate visualization preparation from display.
   */
  virtual VisualizerOutput::UniquePtr spinOnce(const VisualizerInput& input);

  // TODO(marcus): Is there any reason the following two methods must be private?

  /* ------------------------------------------------------------------------ */
  // Visualize 2d mesh.
  static cv::Mat visualizeMesh2DStereo(
      const std::vector<cv::Vec6f>& triangulation_2D,
      const Frame& ref_frame);

  /* ------------------------------------------------------------------------ */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  static cv::Mat visualizeMesh2D(
      const std::vector<cv::Vec6f>& triangulation2D,
      const cv::Mat& img,
      const KeypointsCV& extra_keypoints = KeypointsCV());

 private:
  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks.
  void visualizePoints3D(const PointsWithIdMap& points_with_id,
                         const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map);

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizePlane(const PlaneId& plane_index, const double& n_x,
                      const double& n_y, const double& n_z, const double& d,
                      const bool& visualize_plane_label = true,
                      const int& cluster_id = 1);

  /* ------------------------------------------------------------------------ */
  // Draw a line in opencv.
  void drawLine(const std::string& line_id, const double& from_x,
                const double& from_y, const double& from_z, const double& to_x,
                const double& to_y, const double& to_z);

  /* ------------------------------------------------------------------------ */
  void drawLine(const std::string& line_id, const cv::Point3d& pt1,
                const cv::Point3d& pt2);

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizeMesh3D(const cv::Mat& mapPoints3d, const cv::Mat& polygonsMesh);

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity,
  // and provide color for each polygon.
  void visualizeMesh3D(const cv::Mat& map_points_3d, const cv::Mat& colors,
                       const cv::Mat& polygons_mesh,
                       const cv::Mat& tcoords = cv::Mat(),
                       const cv::Mat& texture = cv::Mat());

  /* ------------------------------------------------------------------------ */
  /// Visualize a PLY from filename (absolute path).
  void visualizePlyMesh(const std::string& filename);

  /* ------------------------------------------------------------------------ */
  /// Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
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
      const std::vector<Plane>& planes, const cv::Mat& map_points_3d,
      const cv::Mat& polygons_mesh,
      const bool visualize_mesh_with_colored_polygon_clusters = false,
      const Timestamp& timestamp = 0.0);

  /* ------------------------------------------------------------------------ */
  // Visualize convex hull in 2D for set of points in triangle cluster,
  // projected along the normal of the cluster.
  void visualizeConvexHull(const TriangleCluster& cluster,
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
  // @param wait_time Amount of time in milliseconds for the event loop to keep
  // running.
  // @param force_redraw If true, window renders.
  void renderWindow(int wait_time = 1, bool force_redraw = true);

  /* ------------------------------------------------------------------------ */
  // Get a screenshot of the window.
  void getScreenshot(const std::string& filename);

  /* ------------------------------------------------------------------------ */
  // Useful for when testing on servers without display screen.
  void setOffScreenRendering();

  /* ------------------------------------------------------------------------ */
  // Record video sequence at a hardcoded directory relative to executable.
  void recordVideo();

  /* ------------------------------------------------------------------------ */
  static Mesh3DVizProperties texturizeMesh3D(const Timestamp& image_timestamp,
                                             const cv::Mat& texture_image,
                                             const Mesh2D& mesh_2d,
                                             const Mesh3D& mesh_3d) {
    // Dummy checks for valid data.
    CHECK(!texture_image.empty());
    CHECK_GE(mesh_2d.getNumberOfUniqueVertices(), 0);
    CHECK_GE(mesh_3d.getNumberOfUniqueVertices(), 0);

    // Let us fill the mesh 3d viz properties structure.
    Mesh3DVizProperties mesh_3d_viz_props;

    // Color all vertices in red. Each polygon will be colored according
    // to a mix of the three vertices colors I think...
    mesh_3d_viz_props.colors_ = cv::Mat(mesh_3d.getNumberOfUniqueVertices(), 1,
                                        CV_8UC3, cv::viz::Color::red());

    // Add texture to the mesh using the given image.
    // README: tcoords specify the texture coordinates of the 3d mesh wrt 2d
    // image. As a small hack, we not only use the left_image as texture but we
    // also horizontally concatenate a white image so we can set a white texture
    // to those 3d mesh faces which should not have a texture. Below we init all
    // tcoords to 0.99 (1.0) gives a weird texture... Meaning that all faces
    // start with a default white texture, and then we change that texture to
    // the right texture for each 2d triangle that has a corresponding 3d face.
    Mesh2D::Polygon polygon;
    std::vector<cv::Vec2d> tcoords(mesh_3d.getNumberOfUniqueVertices(),
                                   cv::Vec2d(0.9, 0.9));
    for (size_t i = 0; i < mesh_2d.getNumberOfPolygons(); i++) {
      CHECK(mesh_2d.getPolygon(i, &polygon))
          << "Could not retrieve 2d polygon.";

      const LandmarkId& lmk0 = polygon.at(0).getLmkId();
      const LandmarkId& lmk1 = polygon.at(1).getLmkId();
      const LandmarkId& lmk2 = polygon.at(2).getLmkId();

      // Returns indices of points in the 3D mesh corresponding to the vertices
      // in the 2D mesh.
      int p0_id, p1_id, p2_id;
      if (mesh_3d.getVertex(lmk0, nullptr, &p0_id) &&
          mesh_3d.getVertex(lmk1, nullptr, &p1_id) &&
          mesh_3d.getVertex(lmk2, nullptr, &p2_id)) {
        // Sanity check.
        CHECK_LE(p0_id, tcoords.size());
        CHECK_LE(p1_id, tcoords.size());
        CHECK_LE(p2_id, tcoords.size());

        // Get pixel coordinates of the vertices of the 2D mesh.
        const auto& px0 = polygon.at(0).getVertexPosition();
        const auto& px1 = polygon.at(1).getVertexPosition();
        const auto& px2 = polygon.at(2).getVertexPosition();

        // These pixels correspond to the tcoords in the image for the 3d mesh
        // vertices.
        VLOG(100) << "Pixel: with id: " << p0_id << ", x: " << px0.x
                  << ", y: " << px0.y;
        // We divide by 2.0 to account for fake default texture padded to the
        // right of the texture_image.
        tcoords.at(p0_id) = cv::Vec2d(px0.x / texture_image.cols / 2.0,
                                      px0.y / texture_image.rows);
        tcoords.at(p1_id) = cv::Vec2d(px1.x / texture_image.cols / 2.0,
                                      px1.y / texture_image.rows);
        tcoords.at(p2_id) = cv::Vec2d(px2.x / texture_image.cols / 2.0,
                                      px2.y / texture_image.rows);
        mesh_3d_viz_props.colors_.row(p0_id) = cv::viz::Color::white();
        mesh_3d_viz_props.colors_.row(p1_id) = cv::viz::Color::white();
        mesh_3d_viz_props.colors_.row(p2_id) = cv::viz::Color::white();
      } else {
        // If we did not find a corresponding 3D triangle for the 2D triangle
        // leave tcoords and colors to the default values.
        LOG_EVERY_N(ERROR, 1000) << "Polygon in 2d mesh did not have a "
                                    "corresponding polygon in 3d mesh!";
      }
    }

    // Add a column with a fixed color at the end so that we can specify an
    // "invalid" or "default" texture for those points which we do not want to
    // texturize.
    static cv::Mat default_texture(texture_image.rows, texture_image.cols,
                                   texture_image.type(),
                                   cv::viz::Color::white());
    CHECK_EQ(texture_image.dims, default_texture.dims);
    CHECK_EQ(texture_image.rows, default_texture.rows);
    CHECK_EQ(texture_image.type(), default_texture.type());

    cv::Mat texture;
    // Padding actual texture with default texture, a bit hacky, but works.
    cv::hconcat(texture_image, default_texture, texture);
    mesh_3d_viz_props.texture_ = texture;

    mesh_3d_viz_props.tcoords_ = cv::Mat(tcoords, true).reshape(2);
    CHECK_EQ(mesh_3d_viz_props.tcoords_.size().height,
             mesh_3d.getNumberOfUniqueVertices());

    return mesh_3d_viz_props;
  }

 private:
  // Flags for visualization behaviour.
  const VisualizationType visualization_type_;
  const BackendType backend_type_;

  // Callbacks.
  // Mesh 3d visualization properties setter callback.
  Mesh3dVizPropertiesSetterCallback mesh3d_viz_properties_callback_;

  std::deque<cv::Affine3d> trajectory_poses_3d_;

  std::map<PlaneId, LineNr> plane_to_line_nr_map_;
  PlaneIdMap plane_id_map_;
  std::map<PlaneId, bool> is_plane_id_in_window_;

  // Holds all visualization data including the window which contains 3D
  // widgets.
  WindowData window_data_;

  //! Logging instance.
  std::unique_ptr<VisualizerLogger> logger_;

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
                           const cv::Mat& polygons_mesh, cv::Mat* colors) const;

  /* ------------------------------------------------------------------------ */
  // Decide color of the cluster depending on its id.
  void getColorById(const size_t& id, cv::viz::Color* color) const;

  /* ------------------------------------------------------------------------ */
  // Draw a line from lmk to plane center.
  void drawLineFromPlaneToPoint(const std::string& line_id,
                                const double& plane_n_x,
                                const double& plane_n_y,
                                const double& plane_n_z, const double& plane_d,
                                const double& point_x, const double& point_y,
                                const double& point_z);

  /* ------------------------------------------------------------------------ */
  // Update line from lmk to plane center.
  void updateLineFromPlaneToPoint(const std::string& line_id,
                                  const double& plane_n_x,
                                  const double& plane_n_y,
                                  const double& plane_n_z,
                                  const double& plane_d, const double& point_x,
                                  const double& point_y, const double& point_z);

  /* ------------------------------------------------------------------------ */
  // Keyboard callback.
  static void keyboardCallback(const cv::viz::KeyboardEvent& event, void* t);

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to toggle freezing screen.
  static void toggleFreezeScreenKeyboardCallback(
      const uchar code, Visualizer3D::WindowData& window_data) {
    if (code == 't') {
      LOG(WARNING) << "Pressing " << code << " toggles freezing screen.";
      static bool freeze = false;
      freeze = !freeze;  // Toggle.
      window_data.window_.spinOnce(1, true);
      while (!window_data.window_.wasStopped()) {
        if (freeze) {
          window_data.window_.spinOnce(1, true);
        } else {
          break;
        }
      }
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to set mesh representation.
  static void setMeshRepresentation(const uchar code,
                                    Visualizer3D::WindowData& window_data) {
    if (code == '0') {
      LOG(WARNING) << "Pressing " << code
                   << " sets mesh representation to "
                      "a point cloud.";
      window_data.mesh_representation_ = 0u;
    } else if (code == '1') {
      LOG(WARNING) << "Pressing " << code
                   << " sets mesh representation to "
                      "a mesh.";
      window_data.mesh_representation_ = 1u;
    } else if (code == '2') {
      LOG(WARNING) << "Pressing " << code
                   << " sets mesh representation to "
                      "a wireframe.";
      window_data.mesh_representation_ = 2u;
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to set mesh shading.
  static void setMeshShadingCallback(const uchar code,
                                     Visualizer3D::WindowData& window_data) {
    if (code == '4') {
      LOG(WARNING) << "Pressing " << code
                   << " sets mesh shading to "
                      "flat.";
      window_data.mesh_shading_ = 0u;
    } else if (code == '5') {
      LOG(WARNING) << "Pressing " << code
                   << " sets mesh shading to "
                      "Gouraud.";
      window_data.mesh_shading_ = 1u;
    } else if (code == '6') {
      LOG(WARNING) << "Pressing " << code
                   << " sets mesh shading to "
                      "Phong.";
      window_data.mesh_shading_ = 2u;
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to set mesh ambient.
  static void setMeshAmbientCallback(const uchar code,
                                     Visualizer3D::WindowData& window_data) {
    if (code == 'a') {
      window_data.mesh_ambient_ = !window_data.mesh_ambient_;
      LOG(WARNING) << "Pressing " << code << " toggles mesh ambient."
                   << " Now set to " << window_data.mesh_ambient_;
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to set mesh lighting.
  static void setMeshLightingCallback(const uchar code,
                                      Visualizer3D::WindowData& window_data) {
    if (code == 'l') {
      window_data.mesh_lighting_ = !window_data.mesh_lighting_;
      LOG(WARNING) << "Pressing " << code << " toggles mesh lighting."
                   << " Now set to " << window_data.mesh_lighting_;
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to get current viewer pose.
  static void getViewerPoseKeyboardCallback(
      const uchar& code, Visualizer3D::WindowData& window_data) {
    if (code == 'v') {
      LOG(INFO) << "Current viewer pose:\n"
                << "\tRodriguez vector: "
                << window_data.window_.getViewerPose().rvec()
                << "\n\tAffine matrix: "
                << window_data.window_.getViewerPose().matrix;
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to get current screen size.
  static void getCurrentWindowSizeKeyboardCallback(
      const uchar code, Visualizer3D::WindowData& window_data) {
    if (code == 'w') {
      LOG(WARNING) << "Pressing " << code << " displays current window size:\n"
                   << "\theight: " << window_data.window_.getWindowSize().height
                   << "\twidth: " << window_data.window_.getWindowSize().width;
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to get screenshot of current windodw.
  static void getScreenshotCallback(const uchar code,
                                    Visualizer3D::WindowData& window_data) {
    if (code == 's') {
      static int i = 0;
      std::string filename = "screenshot_3d_window" + std::to_string(i);
      LOG(WARNING) << "Pressing " << code
                   << " takes a screenshot of the "
                      "window, saved in: " +
                          filename;
      window_data.window_.saveScreenshot(filename);
    }
  }
};

}  // namespace VIO
