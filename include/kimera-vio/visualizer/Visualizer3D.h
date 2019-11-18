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
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/mesh/Mesher.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/UtilsGTSAM.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace VIO {

enum class VisualizationType {
  MESH2DTo3Dsparse,  // same as MESH2DTo3D but filters out triangles
                     // corresponding to non planar obstacles
  POINTCLOUD,        // visualize 3D VIO points  (no repeated point)
  NONE               // does not visualize map
};

struct VisualizerInput {
  KIMERA_POINTER_TYPEDEFS(VisualizerInput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisualizerInput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VisualizerInput(const gtsam::Pose3& pose,
                  const StereoFrame& stereo_keyframe,
                  const MesherOutput::Ptr& mesher_output_payload,
                  const PointsWithIdMap& points_with_id_VIO,
                  const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map,
                  const std::vector<Plane>& planes,
                  const gtsam::NonlinearFactorGraph& graph,
                  const gtsam::Values& values);

  const gtsam::Pose3 pose_;
  const StereoFrame stereo_keyframe_;
  const MesherOutput::Ptr mesher_output_payload_;
  const PointsWithIdMap points_with_id_VIO_;
  const LmkIdToLmkTypeMap lmk_id_to_lmk_type_map_;
  const std::vector<Plane> planes_;
  const gtsam::NonlinearFactorGraph graph_;
  const gtsam::Values values_;
};

struct ImageToDisplay {
  ImageToDisplay() = default;
  ImageToDisplay(const std::string& name, const cv::Mat& image);

  std::string name_;
  cv::Mat image_;
};

struct VisualizerOutput {
  KIMERA_POINTER_TYPEDEFS(VisualizerOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisualizerOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VisualizerOutput() = default;
  ~VisualizerOutput() = default;
  VisualizationType visualization_type_ = VisualizationType::NONE;
  std::vector<ImageToDisplay> images_to_display_;
  cv::viz::Viz3d window_ = cv::viz::Viz3d("3D Visualizer");
};

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
      Mesher::Mesh3dVizPropertiesSetterCallback cb) {
    mesh3d_viz_properties_callback_ = cb;
  }

  /**
   * \brief Returns true if visualization is ready, false otherwise.
   * The actual visualization must be done in the main thread, and as such,
   * it is not done here to separate visualization preparation from display.
   */
  virtual VisualizerOutput::Ptr spinOnce(const VisualizerInput& input);

 private:
  /* ------------------------------------------------------------------------ */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  static cv::Mat visualizeMesh2D(
      const std::vector<cv::Vec6f>& triangulation2D,
      const cv::Mat& img,
      const KeypointsCV& extra_keypoints = KeypointsCV());

  /* ------------------------------------------------------------------------ */
  // Visualize 2d mesh.
  static cv::Mat visualizeMesh2DStereo(
      const std::vector<cv::Vec6f>& triangulation_2D,
      const Frame& ref_frame);

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
  static Mesher::Mesh3DVizProperties texturizeMesh3D(
      const Timestamp& image_timestamp, const cv::Mat& texture_image,
      const Mesh2D& mesh_2d, const Mesh3D& mesh_3d) {
    // Dummy checks for valid data.
    CHECK(!texture_image.empty());
    CHECK_GE(mesh_2d.getNumberOfUniqueVertices(), 0);
    CHECK_GE(mesh_3d.getNumberOfUniqueVertices(), 0);

    // Let us fill the mesh 3d viz properties structure.
    Mesher::Mesh3DVizProperties mesh_3d_viz_props;

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
  Mesher::Mesh3dVizPropertiesSetterCallback mesh3d_viz_properties_callback_;

  std::deque<cv::Affine3f> trajectoryPoses3d_;

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

enum class VisualizerType {
  //! OpenCV 3D viz, uses VTK underneath the hood.
  OpenCV = 0u,
};

class VisualizerFactory {
 public:
  KIMERA_POINTER_TYPEDEFS(VisualizerFactory);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisualizerFactory);
  VisualizerFactory() = delete;
  virtual ~VisualizerFactory() = default;

  static Visualizer3D::UniquePtr createVisualizer(
      const VisualizerType visualizer_type,
      const VisualizationType& viz_type,
      const BackendType& backend_type) {
    switch (visualizer_type) {
      case VisualizerType::OpenCV: {
        return VIO::make_unique<Visualizer3D>(viz_type, backend_type);
      }
      default: {
        LOG(FATAL) << "Requested visualizer type is not supported.\n"
                   << "Currently supported visualizer types:\n"
                   << "0: OpenCV 3D viz\n 1: Pangolin (not supported yet)\n"
                   << " but requested visualizer: "
                   << static_cast<int>(visualizer_type);
      }
    }
  }
};

class VisualizerModule
    : public MIMOPipelineModule<VisualizerInput, VisualizerOutput> {
 public:
  KIMERA_POINTER_TYPEDEFS(VisualizerModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisualizerModule);
  using VizFrontendInput = StereoFrontEndOutputPayload::Ptr;
  using VizBackendInput = VioBackEndOutputPayload::Ptr;
  using VizMesherInput = MesherOutput::Ptr;

  VisualizerModule(bool parallel_run, Visualizer3D::UniquePtr visualizer)
      : MIMOPipelineModule<VisualizerInput, VisualizerOutput>("Visualizer",
                                                              parallel_run),
        frontend_queue_(""),
        backend_queue_(""),
        mesher_queue_(""),
        visualizer_(std::move(visualizer)){};
  virtual ~VisualizerModule() = default;

  //! Callbacks to fill queues: they should be all lighting fast.
  inline void fillFrontendQueue(const VizFrontendInput& frontend_payload) {
    frontend_queue_.push(frontend_payload);
  }
  inline void fillBackendQueue(const VizBackendInput& backend_payload) {
    backend_queue_.push(backend_payload);
  }
  inline void fillMesherQueue(const VizMesherInput& mesher_payload) {
    mesher_queue_.push(mesher_payload);
  }

 protected:
  //! Synchronize input queues. Currently doing it in a crude way:
  //! Pop blocking the payload that should be the last to be computed,
  //! then loop over the other queues until you get a payload that has exactly
  //! the same timestamp. Guaranteed to sync messages unless the assumption
  //! on the order of msg generation is broken.
  virtual inline InputPtr getInputPacket() override {
    bool queue_state = false;
    VizMesherInput mesher_payload = nullptr;
    if (PIO::parallel_run_) {
      queue_state = mesher_queue_.popBlocking(mesher_payload);
    } else {
      queue_state = mesher_queue_.pop(mesher_payload);
    }
    if (!queue_state) {
      LOG_IF(WARNING, PIO::parallel_run_)
          << "Module: " << name_id_ << " - Mesher queue is down";
      VLOG_IF(1, !PIO::parallel_run_)
          << "Module: " << name_id_ << " - Mesher queue is empty or down";
      return nullptr;
    }
    CHECK(mesher_payload);
    const Timestamp& timestamp = mesher_payload->timestamp_;

    // Look for the synchronized packet in frontend payload queue
    // This should always work, because it should not be possible to have
    // a backend payload without having a frontend one first!
    Timestamp frontend_payload_timestamp =
        std::numeric_limits<Timestamp>::max();
    VizFrontendInput frontend_payload = nullptr;
    while (timestamp != frontend_payload_timestamp) {
      // Pop will remove messages until the queue is empty.
      // This assumes the mesher ends processing after the frontend queue
      // has been filled (it could happen that frontend_queue_ didn't receive
      // the msg before mesher finishes, but that is very unlikely, unless
      // the queue is blocked by someone...)
      if (!frontend_queue_.pop(frontend_payload)) {
        // We had a mesher input but no frontend input, something's wrong.
        // We assume mesher runs after frontend.
        LOG(ERROR) << name_id_
                   << "'s frontend payload queue is empty or "
                      "has been shutdown.";
        return nullptr;
      }
      if (frontend_payload) {
        frontend_payload_timestamp =
            frontend_payload->stereo_frame_lkf_.getTimestamp();
      } else {
        LOG(WARNING) << "Missing frontend payload for Module: " << name_id_;
      }
    }
    CHECK(frontend_payload);

    Timestamp backend_payload_timestamp = std::numeric_limits<Timestamp>::max();
    VizBackendInput backend_payload = nullptr;
    while (timestamp != backend_payload_timestamp) {
      if (!backend_queue_.pop(backend_payload)) {
        // We had a mesher input but no backend input, something's wrong.
        // We assume mesher runs after backend.
        LOG(ERROR) << "Visualizer's backend payload queue is empty or "
                      "has been shutdown.";
        return nullptr;
      }
      if (backend_payload) {
        backend_payload_timestamp = backend_payload->W_State_Blkf_.timestamp_;
      } else {
        LOG(WARNING) << "Missing backend payload for Module: " << name_id_;
      }
    }
    CHECK(backend_payload);

    // Push the synced messages to the visualizer's input queue
    const StereoFrame& stereo_keyframe = frontend_payload->stereo_frame_lkf_;
    // TODO(TONI): store the payloads' pointers in the visualizer payload
    // so that no copies are done, nor we have dangling references!
    return VIO::make_unique<VisualizerInput>(
        // Pose for trajectory viz.
        backend_payload->W_State_Blkf_.pose_ *
            stereo_keyframe
                .getBPoseCamLRect(),  // This should be pass at ctor level...
        // For visualizeMesh2D and visualizeMesh2DStereo.
        stereo_keyframe,
        // visualizeConvexHull & visualizeMesh3DWithColoredClusters
        std::move(mesher_payload),
        // PointsWithIdMap() should be:
        // visualization_type == VisualizationType::POINTCLOUD
        //    ? vio_backend_module_
        //          ->getMapLmkIdsTo3dPointsInTimeHorizon()  // not thread-safe
        //    : points_with_id_VIO,
        PointsWithIdMap(),
        // visualizeMesh3DWithColoredClusters & visualizePoints3D
        // LmkIdToLmkTypeMap should be lmk_id_to_lmk_type_map,
        LmkIdToLmkTypeMap(),
        // Should be planes_
        std::vector<Plane>(),
        // vio_backend_module_->getFactorsUnsafe(),
        // For plane constraints viz.
        gtsam::NonlinearFactorGraph(),
        backend_payload->state_  // For planes and plane constraints viz.
    );
  }

  virtual OutputPtr spinOnce(const VisualizerInput& input) override {
    return visualizer_->spinOnce(input);
  }

  //! Called when general shutdown of PipelineModule is triggered.
  virtual void shutdownQueues() override {
    LOG(INFO) << "Shutting down queues for: " << name_id_;
    frontend_queue_.shutdown();
    backend_queue_.shutdown();
    mesher_queue_.shutdown();
  };

  //! Checks if the module has work to do (should check input queues are empty)
  virtual bool hasWork() const override {
    LOG_IF(WARNING, mesher_queue_.empty() && !backend_queue_.empty())
        << "Mesher queue is empty, yet backend queue is not!"
           "This should not happen since Mesher runs at Backend pace!";
    // We don't check frontend queue because it runs faster than the other two
    // queues.
    return mesher_queue_.empty();
  };

 private:
  //! Input Queues
  ThreadsafeQueue<VizFrontendInput> frontend_queue_;
  ThreadsafeQueue<VizBackendInput> backend_queue_;
  ThreadsafeQueue<VizMesherInput> mesher_queue_;

  //! Visualizer implementation
  Visualizer3D::UniquePtr visualizer_;
};

}  // namespace VIO
