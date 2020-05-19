/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OpenCvVisualizer3D.h
 * @brief  Build and visualize 3D data: 2D mesh from Frame for example.
 * @author Antoni Rosinol
 */

#pragma once

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <glog/logging.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <opencv2/opencv.hpp>
#include <opencv2/viz/types.hpp>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"
#include "kimera-vio/visualizer/Visualizer3D.h"

namespace VIO {

class OpenCvVisualizer3D : public Visualizer3D {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(OpenCvVisualizer3D);
  KIMERA_POINTER_TYPEDEFS(OpenCvVisualizer3D);
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
  OpenCvVisualizer3D(const VisualizationType& viz_type,
                     const BackendType& backend_type);
  virtual ~OpenCvVisualizer3D();

  /**
   * @brief registerMesh3dVizProperties (Legacy) this was used to paint the
   * mesh with semantic labels if sent by third-party.
   * @param cb callback
   */
  inline void registerMesh3dVizProperties(
      Mesh3dVizPropertiesSetterCallback cb) {
    mesh3d_viz_properties_callback_ = cb;
  }

  /**
   * \brief Returns true if visualization is ready, false otherwise.
   * The actual visualization must be done in the main thread, and as such,
   * it is not done here to separate visualization preparation from display.
   */
  VisualizerOutput::UniquePtr spinOnce(const VisualizerInput& input) override;

  // TODO(marcus): Is there any reason the following two methods must be
  // private?

  //! Visualize 2d mesh.
  static cv::Mat visualizeMesh2DStereo(
      const std::vector<cv::Vec6f>& triangulation_2D,
      const Frame& ref_frame);

  //! Create a 2D mesh from 2D corners in an image, coded as a Frame class
  static cv::Mat visualizeMesh2D(
      const std::vector<cv::Vec6f>& triangulation2D,
      const cv::Mat& img,
      const KeypointsCV& extra_keypoints = KeypointsCV());

 private:
  //! Visualize a 3D point cloud of unique 3D landmarks.
  void visualizePoints3D(const PointsWithIdMap& points_with_id,
                         const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map,
                         WidgetsMap* widgets_map);

  //! Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizePlane(const PlaneId& plane_index,
                      const double& n_x,
                      const double& n_y,
                      const double& n_z,
                      const double& d,
                      WidgetsMap* widgets_map,
                      const bool& visualize_plane_label = true,
                      const int& cluster_id = 1);

  //! Draw a line in opencv.
  void drawLine(const std::string& line_id,
                const double& from_x,
                const double& from_y,
                const double& from_z,
                const double& to_x,
                const double& to_y,
                const double& to_z,
                WidgetsMap* widgets);

  //! Same as above but with different interface
  void drawLine(const std::string& line_id,
                const cv::Point3d& pt1,
                const cv::Point3d& pt2,
                WidgetsMap* widgets);

  //! Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizeMesh3D(const cv::Mat& mapPoints3d,
                       const cv::Mat& polygonsMesh,
                       WidgetsMap* widgets);

  //! Visualize a 3D point cloud of unique 3D landmarks with its connectivity,
  //! and provide color for each polygon.
  void visualizeMesh3D(const cv::Mat& map_points_3d,
                       const cv::Mat& colors,
                       const cv::Mat& polygons_mesh,
                       WidgetsMap* widgets,
                       const cv::Mat& tcoords = cv::Mat(),
                       const cv::Mat& texture = cv::Mat());

  /// Visualize a PLY from filename (absolute path).
  void visualizePlyMesh(const std::string& filename, WidgetsMap* widgets);

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
      const std::vector<Plane>& planes,
      const cv::Mat& map_points_3d,
      const cv::Mat& polygons_mesh,
      WidgetsMap* widgets,
      const bool visualize_mesh_with_colored_polygon_clusters = false,
      const Timestamp& timestamp = 0.0);

  //! Visualize convex hull in 2D for set of points in triangle cluster,
  //! projected along the normal of the cluster.
  void visualizeConvexHull(const TriangleCluster& cluster,
                           const cv::Mat& map_points_3d,
                           const cv::Mat& polygons_mesh,
                           WidgetsMap* widgets);

  //! Visualize trajectory. Adds an image to the frustum if cv::Mat is not empty.
  void visualizeTrajectory3D(const cv::Mat& frustum_image,
                             cv::Affine3d* frustum_pose,
                             WidgetsMap* widgets_map);

  //!!!!!!!! Remove widget. True if successful, false if not.
  bool removeWidget(const std::string& widget_id);

  //! Visualize line widgets from plane to lmks.
  //! Point key is required to avoid duplicated lines!
  void visualizePlaneConstraints(const PlaneId& plane_id,
                                 const gtsam::Point3& normal,
                                 const double& distance,
                                 const LandmarkId& lmk_id,
                                 const gtsam::Point3& point,
                                 WidgetsMap* widgets);

  //! Remove line widgets from plane to lmks, for lines that are not pointing
  //! to any lmk_id in lmk_ids.
  void removeOldLines(const LandmarkIds& lmk_ids);

  //! Remove line widgets from plane to lmks.
  void removePlaneConstraintsViz(const PlaneId& plane_id);

  //! Remove plane widget.
  void removePlane(const PlaneId& plane_index,
                   const bool& remove_plane_label = true);

  //! Add pose to the previous trajectory.
  void addPoseToTrajectory(const gtsam::Pose3& current_pose_gtsam);

  static Mesh3DVizProperties texturizeMesh3D(const Timestamp& image_timestamp,
                                             const cv::Mat& texture_image,
                                             const Mesh2D& mesh_2d,
                                             const Mesh3D& mesh_3d);

 private:
  //! Flags for visualization behaviour.
  const BackendType backend_type_;

  //! Callbacks.
  //! Mesh 3d visualization properties setter callback.
  Mesh3dVizPropertiesSetterCallback mesh3d_viz_properties_callback_;

  std::deque<cv::Affine3d> trajectory_poses_3d_;

  std::map<PlaneId, LineNr> plane_to_line_nr_map_;
  PlaneIdMap plane_id_map_;
  std::map<PlaneId, bool> is_plane_id_in_window_;

  //! Colors
  cv::viz::Color cloud_color_ = cv::viz::Color::white();

  //! Logging instance.
  std::unique_ptr<VisualizerLogger> logger_;

  //! Log mesh to ply file.
  void logMesh(const cv::Mat& map_points_3d,
               const cv::Mat& colors,
               const cv::Mat& polygons_mesh,
               const Timestamp& timestamp,
               bool log_accumulated_mesh = false);

  //! Input the mesh points and triangle clusters, and
  //! output colors matrix for mesh visualizer.
  //! This will color the point with the color of the last plane having it.
  void colorMeshByClusters(const std::vector<Plane>& planes,
                           const cv::Mat& map_points_3d,
                           const cv::Mat& polygons_mesh,
                           cv::Mat* colors) const;

  //! Decide color of the cluster depending on its id.
  void getColorById(const size_t& id, cv::viz::Color* color) const;

  //! Draw a line from lmk to plane center.
  void drawLineFromPlaneToPoint(const std::string& line_id,
                                const double& plane_n_x,
                                const double& plane_n_y,
                                const double& plane_n_z,
                                const double& plane_d,
                                const double& point_x,
                                const double& point_y,
                                const double& point_z,
                                WidgetsMap* widgets);

  //! Update line from lmk to plane center.
  void updateLineFromPlaneToPoint(const std::string& line_id,
                                  const double& plane_n_x,
                                  const double& plane_n_y,
                                  const double& plane_n_z,
                                  const double& plane_d,
                                  const double& point_x,
                                  const double& point_y,
                                  const double& point_z,
                                  WidgetsMap* widgets);
};

}  // namespace VIO
