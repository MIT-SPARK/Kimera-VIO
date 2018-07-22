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

DEFINE_int32(mesh_shading, 0,
             "Mesh shading:\n 0: Flat, 1: Gouraud, 2: Phong");
DEFINE_int32(mesh_representation, 1,
             "Mesh representation:\n 0: Points, 1: Wireframe, 2: Surface");
DEFINE_bool(set_mesh_ambient, false, "Whether to use lighting for the mesh.");
DEFINE_bool(set_mesh_lighting, false, "Whether to use lighting for the mesh.");
DEFINE_bool(log_mesh, false, "Log the mesh at time horizon.");
DEFINE_bool(log_accumulated_mesh, false, "Accumulate the mesh when logging.");

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
    //cv::Affine3f viewer_pose (cv::Vec3f(-1.6432757, 1.6432757, -0.8554352),
    //                          cv::Vec3f(0.0, 0.0, 0.0));
    window_.registerKeyboardCallback(keyboardCallback, &window_);
    //Vec3d cam_pos(-5.0,0.0,6.0);
    //Vec3d cam_focal_point(1.0,0.0,0.0);
    //Vec3d cam_y_dir(-1.0,0.0,0.0);
    //Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    //window_.setViewerPose(cam_pose);
    window_.setWindowPosition(Size(3*1861+1080/2, 2212/2));
    window_.setWindowSize(Size(1861, 2056));
    //window_.setBackgroundColor(background_color_);
    window_.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  }

  typedef size_t LineNr;
  typedef std::uint64_t PlaneId;
  typedef std::map<LandmarkId, size_t> LmkIdToLineIdMap;
  typedef std::map<PlaneId, LmkIdToLineIdMap> PlaneIdMap;

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud using cloud widget from opencv viz.
  void visualizeMap3D(const std::vector<gtsam::Point3>& points) {
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
  }

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizePoints3D(
      const std::unordered_map<LandmarkId, gtsam::Point3>& points_with_id,
      const std::unordered_map<LandmarkId, LandmarkType>& lmk_id_to_lmk_type_map) {
    bool color_the_cloud = false;
    if (lmk_id_to_lmk_type_map.size() != 0) {
      color_the_cloud = true;
      CHECK_EQ(points_with_id.size(), lmk_id_to_lmk_type_map.size());
    }

    // Sanity check dimension.
    if (points_with_id.size() == 0) {
      // No points to visualize.
      return;
    }

    // Populate cloud structure with 3D points.
    cv::Mat point_cloud(1, points_with_id.size(), CV_32FC3);
    cv::Mat point_cloud_color (1, lmk_id_to_lmk_type_map.size(), CV_8UC3,
                               cloud_color_);
    cv::Point3f* data = point_cloud.ptr<cv::Point3f>();
    size_t i = 0;
    for (const std::pair<LandmarkId, gtsam::Point3>& id_point: points_with_id) {
      data[i].x = float(id_point.second.x());
      data[i].y = float(id_point.second.y());
      data[i].z = float(id_point.second.z());
      if (color_the_cloud) {
        CHECK(lmk_id_to_lmk_type_map.find(id_point.first) !=
            lmk_id_to_lmk_type_map.end());
        switch (lmk_id_to_lmk_type_map.at(id_point.first)) {
          case LandmarkType::SMART: {
            point_cloud_color.col(i) = cv::viz::Color::white();
            break;
          }
          case LandmarkType::PROJECTION: {
            point_cloud_color.col(i) = cv::viz::Color::green();
            break;
          }
          default: {
            point_cloud_color.col(i) = cv::viz::Color::white();
            break;
          }
        }
      }
      i++;
    }

    // Create a cloud widget.
    cv::viz::WCloud cloud_widget (point_cloud, cloud_color_);
    if (color_the_cloud) {
      cloud_widget = cv::viz::WCloud(point_cloud, point_cloud_color);
    }
    cloud_widget.setRenderingProperty(cv::viz::POINT_SIZE, 6);
    cloud_widget.setRenderingProperty(cv::viz::IMMEDIATE_RENDERING, 1);

    window_.showWidget("Point cloud.", cloud_widget);
  }

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizePlane(
      const PlaneId& plane_index,
      const double& n_x,
      const double& n_y,
      const double& n_z,
      const double& d,
      const bool& visualize_plane_label = true,
      const int& cluster_id = 1) {
    const std::string& plane_id_for_viz =
        "Plane " + std::to_string(plane_index);
    // Create a plane widget.
    const Vec3d normal (n_x, n_y, n_z);
    const Point3d center (d * n_x, d * n_y, d * n_z);
    static const Vec3d new_yaxis (0, 1, 0);
    static const Size2d size (1.0, 1.0);

    cv::viz::Color plane_color;
    getColorById(cluster_id, &plane_color);
    cv::viz::WPlane plane_widget (center, normal, new_yaxis, size, plane_color);
    plane_widget.setRenderingProperty(cv::viz::IMMEDIATE_RENDERING, 1);

    if (visualize_plane_label) {
      static double increase = 0.0;
      const Point3d text_position (d * n_x, d * n_y,
                                   d * n_z + std::fmod(increase, 1));
      increase += 0.1;
      window_.showWidget(plane_id_for_viz + "_label",
                         cv::viz::WText3D(plane_id_for_viz, text_position,
                                          0.07, true));
    }

    window_.showWidget(plane_id_for_viz, plane_widget);
    is_plane_id_in_window_[plane_index] = true;
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
    line_widget.setRenderingProperty(cv::viz::IMMEDIATE_RENDERING, 1);
    window_.showWidget(line_id, line_widget);
  }

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
  void visualizeMesh3D(const cv::Mat& mapPoints3d, const cv::Mat& polygonsMesh) {
    cv::Mat colors (0, 1, CV_8UC3, cv::viz::Color::gray()); // Do not color mesh.
    visualizeMesh3D(mapPoints3d, colors, polygonsMesh);
  }

  /* ------------------------------------------------------------------------ */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity,
  // and provide color for each polygon.
  void visualizeMesh3D(const cv::Mat& map_points_3d, const cv::Mat& colors,
                       const cv::Mat& polygons_mesh) {
    // Check data
    bool color_mesh = false;
    if (colors.rows != 0) {
      CHECK_EQ(map_points_3d.rows, colors.rows) << "Map points and Colors should "
                                                   "have same number of rows. One"
                                                   " color per map point.";
      LOG(ERROR) << "Coloring mesh!";
      color_mesh = true;
    }

    // No points/mesh to visualize.
    if (map_points_3d.rows == 0 ||
        polygons_mesh.rows == 0) {
      return;
    }

    // Create a mesh widget.
    cv::viz::WMesh mesh (map_points_3d.t(), polygons_mesh);
    if (color_mesh) {
      mesh = cv::viz::WMesh(map_points_3d.t(), polygons_mesh, colors.t());
    }

    // Decide mesh shading style.
    switch (FLAGS_mesh_shading) {
      case 0: {
        mesh.setRenderingProperty(cv::viz::SHADING, cv::viz::SHADING_FLAT);
        break;
      }
      case 1: {
        mesh.setRenderingProperty(cv::viz::SHADING, cv::viz::SHADING_GOURAUD);
        break;
      }
      case 2: {
        mesh.setRenderingProperty(cv::viz::SHADING, cv::viz::SHADING_PHONG);
        break;
      }
      default: {
        break;
      }
    }

    // Decide mesh representation style.
    switch (FLAGS_mesh_representation) {
      case 0: {
        mesh.setRenderingProperty(cv::viz::REPRESENTATION,
                                  cv::viz::REPRESENTATION_POINTS);
        mesh.setRenderingProperty(cv::viz::POINT_SIZE, 8);
        break;
      }
      case 1: {
        mesh.setRenderingProperty(cv::viz::REPRESENTATION,
                                  cv::viz::REPRESENTATION_SURFACE);
        break;
      }
      case 2: {
        mesh.setRenderingProperty(cv::viz::REPRESENTATION,
                                  cv::viz::REPRESENTATION_WIREFRAME);
        break;
      }
      default: {
        break;
      }
    }
    mesh.setRenderingProperty(cv::viz::AMBIENT, FLAGS_set_mesh_ambient);
    mesh.setRenderingProperty(cv::viz::LIGHTING, FLAGS_set_mesh_lighting);
    mesh.setRenderingProperty(cv::viz::IMMEDIATE_RENDERING, true);

    // Plot mesh.
    window_.showWidget("Mesh", mesh);
  }

  /* ------------------------------------------------------------------------ */
  ///Visualize a PLY from filename (absolute path).
  void visualizePlyMesh(const std::string& filename) {
   cv::viz::Widget cloud;
   LOG(INFO) << "Showing ground truth mesh: " << filename;
   cloud.fromPlyFile(filename);

   //// Plot mesh.
   window_.showWidget("Cloud from ply", cloud);
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
  /// [in] color_mesh whether to color the mesh or not
  /// [in] timestamp to store the timestamp of the mesh when logging the mesh.
  void visualizeMesh3DWithColoredClusters(
        const std::vector<Plane>& planes,
        const cv::Mat& map_points_3d,
        const cv::Mat& polygons_mesh,
        const bool& color_mesh = false,
        const Timestamp& timestamp = 0.0) {
    if (color_mesh) {
      // Colour the mesh.
      cv::Mat colors;
      colorMeshByClusters(planes, map_points_3d, polygons_mesh, &colors);
      // Visualize the colored mesh.
      visualizeMesh3D(map_points_3d, colors, polygons_mesh);
      // Log the mesh.
      if (FLAGS_log_mesh) {
        logMesh(map_points_3d, colors, polygons_mesh, timestamp,
                FLAGS_log_accumulated_mesh);
      }
    } else {
      // Visualize the mesh with same colour.
      visualizeMesh3D(map_points_3d, polygons_mesh);
    }
  }

  /* ------------------------------------------------------------------------ */
  // Visualize convex hull in 2D for set of points in triangle cluster,
  // projected along the normal of the cluster.
  void visualizeConvexHull (const TriangleCluster& cluster,
                            const cv::Mat& map_points_3d,
                            const cv::Mat& polygons_mesh) {

    // Create a new coord system, which has as z the normal.
    const cv::Point3f& normal = cluster.cluster_direction_;

    // Find first axis of the coord system.
    // Pick random x and y
    static constexpr float random_x = 0.1;
    static constexpr float random_y = 0.0;
    // Find z, such that dot product with the normal is 0.
    float z = -(normal.x * random_x + normal.y * random_y) / normal.z;
    // Create new first axis:
    cv::Point3f x_axis (random_x, random_y, z);
    // Normalize new first axis:
    x_axis /= cv::norm(x_axis);
    // Create new second axis, by cross product normal to x_axis;
    cv::Point3f y_axis = normal.cross(x_axis);
    // Normalize just in case?
    y_axis /= cv::norm(y_axis);
    // Construct new cartesian coord system:
    cv::Mat new_coordinates (3, 3, CV_32FC1);
    new_coordinates.at<float>(0, 0) = x_axis.x;
    new_coordinates.at<float>(0, 1) = x_axis.y;
    new_coordinates.at<float>(0, 2) = x_axis.z;
    new_coordinates.at<float>(1, 0) = y_axis.x;
    new_coordinates.at<float>(1, 1) = y_axis.y;
    new_coordinates.at<float>(1, 2) = y_axis.z;
    new_coordinates.at<float>(2, 0) = normal.x;
    new_coordinates.at<float>(2, 1) = normal.y;
    new_coordinates.at<float>(2, 2) = normal.z;

    std::vector<cv::Point2f> points_2d;
    std::vector<float> z_s;
    for (const size_t& triangle_id: cluster.triangle_ids_) {
      size_t triangle_idx = std::round(triangle_id * 4);
      if (triangle_idx + 3 >= polygons_mesh.rows) {
        throw std::runtime_error("Visualizer3D: an id in triangle_ids_ is"
                                 " too large.");
      }
      int32_t idx_1 = polygons_mesh.at<int32_t>(triangle_idx + 1);
      int32_t idx_2 = polygons_mesh.at<int32_t>(triangle_idx + 2);
      int32_t idx_3 = polygons_mesh.at<int32_t>(triangle_idx + 3);

      // Project points to new coord system
      cv::Point3f new_map_point_1 =// new_coordinates *
                            // map_points_3d.row(idx_1).t();
                            map_points_3d.at<cv::Point3f>(idx_1);
      cv::Point3f new_map_point_2 =// new_coordinates *
                            // map_points_3d.row(idx_2).t();
                            map_points_3d.at<cv::Point3f>(idx_2);
      cv::Point3f new_map_point_3 =// new_coordinates *
                               // map_points_3d.row(idx_3).t();
                               map_points_3d.at<cv::Point3f>(idx_3);

      // Keep only 1st and 2nd component, aka the projection of the point on the
      // plane.
      points_2d.push_back(cv::Point2f(new_map_point_1.x,
                                      new_map_point_1.y));
      z_s.push_back(new_map_point_1.z);
      points_2d.push_back(cv::Point2f(new_map_point_2.x,
                                      new_map_point_2.y));
      z_s.push_back(new_map_point_2.z);
      points_2d.push_back(cv::Point2f(new_map_point_3.x,
                                      new_map_point_3.y));
      z_s.push_back(new_map_point_3.z);
    }

    // Create convex hull.
    if (points_2d.size() != 0) {
      std::vector<int> hull_idx;
      convexHull(Mat(points_2d), hull_idx, false);

      // Add the z component.
      std::vector<cv::Point3f> hull_3d;
      for (const int& idx: hull_idx) {
        hull_3d.push_back(cv::Point3f(
                            points_2d.at(idx).x,
                            points_2d.at(idx).y,
                            z_s.at(idx)));
      }

      static constexpr bool visualize_hull_as_polyline = false;
      if (visualize_hull_as_polyline) {
        // Close the hull.
        CHECK_NE(hull_idx.size(), 0);
        hull_3d.push_back(cv::Point3f(
                            points_2d.at(hull_idx.at(0)).x,
                            points_2d.at(hull_idx.at(0)).y,
                            z_s.at(hull_idx.at(0))));
        // Visualize convex hull.
        cv::viz::WPolyLine convex_hull (hull_3d);
        window_.showWidget("Convex hull", convex_hull);
      } else {
        // Visualize convex hull as a mesh of one polygon with multiple points.
        if (hull_3d.size() > 2) {
          cv::Mat polygon_hull = cv::Mat(4 * (hull_3d.size() - 2), 1, CV_32SC1);
          size_t i = 1;
          for (size_t k = 0; k + 3 < polygon_hull.rows; k += 4) {
            polygon_hull.row(k)     = 3;
            polygon_hull.row(k + 1) = 0;
            polygon_hull.row(k + 2) = static_cast<int>(i);
            polygon_hull.row(k + 3) = static_cast<int>(i) + 1;
            i++;
          }
          cv::viz::Color mesh_color;
          getColorById(cluster.cluster_id_, &mesh_color);
          cv::Mat normals = cv::Mat(0, 1, CV_32FC3);
          for (size_t k = 0; k < hull_3d.size(); k++) {
            normals.push_back(normal);
          }
          cv::Mat colors (hull_3d.size(), 1, CV_8UC3, mesh_color);
          cv::viz::WMesh mesh (hull_3d, polygon_hull, colors.t(), normals.t());
          window_.showWidget("Convex hull", mesh);
        }
      }
    }
  }

  /* ------------------------------------------------------------------------ */
  // Visualize trajectory.
  void visualizeTrajectory3D(const cv::Mat* frustum_image = nullptr) {
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
    // Option A: This does not work very well.
    // window_.resetCameraViewpoint("Camera Pose with Frustum");
    // Viewer is our viewpoint, camera the pose estimate (frustum).
    static constexpr bool follow_camera = false;
    if (follow_camera) {
      cv::Affine3f camera_in_world_coord = trajectoryPoses3d_.back();
      // Option B: specify viewer wrt camera. Works, but motion is non-smooth.
      //cv::Affine3f viewer_in_camera_coord (Vec3f(
      //                                       -0.3422019, -0.3422019, 1.5435732),
      //                                     Vec3f(3.0, 0.0, -4.5));
      //cv::Affine3f viewer_in_world_coord =
      //    viewer_in_camera_coord.concatenate(camera_in_world_coord);

      // Option C: use "look-at" camera parametrization.
      // Works, but motion is non-smooth as well.
      Vec3d cam_pos (-6.0, 0.0, 6.0);
      Vec3d cam_focal_point (camera_in_world_coord.translation());
      Vec3d cam_y_dir (0.0, 0.0, -1.0);
      Affine3f cam_pose = viz::makeCameraPose(cam_pos,
                                              cam_focal_point,
                                              cam_y_dir);
      window_.setViewerPose(cam_pose);
      //window_.setViewerPose(viewer_in_world_coord);
    }

    // Create a Trajectory widget. (argument can be PATH, FRAMES, BOTH).
    std::vector<cv::Affine3f> trajectory (trajectoryPoses3d_.begin(),
                                          trajectoryPoses3d_.end());
    cv::viz::WTrajectory trajectory_widget (trajectory,
                                            cv::viz::WTrajectory::PATH,
                                            1.0, cv::viz::Color::red());
    window_.showWidget("Trajectory", trajectory_widget);
  }

  /* ------------------------------------------------------------------------ */
  // Remove widget. True if successful, false if not.
  bool removeWidget(const std::string& widget_id) {
    try {
      window_.removeWidget(widget_id);
      return true;
    } catch (const cv::Exception& e) {
      VLOG(20) << e.what();
      LOG(ERROR) << "Widget with id: " << widget_id.c_str()
                 << " is not in window.";
    } catch (...) {
      LOG(ERROR) << "Unrecognized exception when using window_.removeWidget() "
                 << "with widget with id: " << widget_id.c_str();
    }
    return false;
  }

  /* ------------------------------------------------------------------------ */
  // Visualize line widgets from plane to lmks.
  // Point key is required to avoid duplicated lines!
  void visualizePlaneConstraints(const PlaneId& plane_id,
                                 const gtsam::Point3& normal,
                                 const double& distance,
                                 const LandmarkId& lmk_id,
                                 const gtsam::Point3& point) {
    PlaneIdMap::iterator plane_id_it = plane_id_map_.find(plane_id);
    LmkIdToLineIdMap* lmk_id_to_line_id_map_ptr = nullptr;
    LineNr* line_nr_ptr = nullptr;
    if (plane_id_it != plane_id_map_.end()) {
      // We already have this plane id stored.
      lmk_id_to_line_id_map_ptr = &(plane_id_it->second);

      // Ensure we also have the line nr stored.
      const auto& line_nr_it = plane_to_line_nr_map_.find(plane_id);
      CHECK(line_nr_it != plane_to_line_nr_map_.end());
      line_nr_ptr = &(line_nr_it->second);
    } else {
      // We have not this plane id stored.
      // Create it by calling default ctor.
      lmk_id_to_line_id_map_ptr = &(plane_id_map_[plane_id]);
      plane_id_it = plane_id_map_.find(plane_id);
      DCHECK(plane_id_it != plane_id_map_.end());

      // Also start line nr to 0.
      plane_to_line_nr_map_[plane_id] = 0;
      DCHECK(plane_to_line_nr_map_.find(plane_id) !=
          plane_to_line_nr_map_.end());
      line_nr_ptr = &(plane_to_line_nr_map_[plane_id]);
    }
    CHECK_NOTNULL(lmk_id_to_line_id_map_ptr);
    CHECK_NOTNULL(line_nr_ptr);

    // TODO should use map from line_id_to_lmk_id as well,
    // to remove the line_ids which are not having a lmk_id...
    const auto& lmk_id_to_line_id = lmk_id_to_line_id_map_ptr->find(lmk_id);
    if (lmk_id_to_line_id == lmk_id_to_line_id_map_ptr->end()) {
      // We have never drawn this line.
      // Store line nr (as line id).
      (*lmk_id_to_line_id_map_ptr)[lmk_id] = *line_nr_ptr;
      std::string line_id = "Line " +
                            std::to_string((int)plane_id_it->first) +
                            std::to_string((int)(*line_nr_ptr));
      // Draw it.
      drawLineFromPlaneToPoint(line_id,
                               normal.x(), normal.y(), normal.z(), distance,
                               point.x(), point.y(), point.z());
      // Augment line_nr for next line_id.
      (*line_nr_ptr)++;
    } else {
      // We have drawn this line before.
      // Update line.
      std::string line_id = "Line " +
                            std::to_string((int)plane_id_it->first) +
                            std::to_string((int)lmk_id_to_line_id->second);
      updateLineFromPlaneToPoint(
            line_id,
            normal.x(), normal.y(), normal.z(), distance,
            point.x(), point.y(), point.z());
    }
  }

  /* ------------------------------------------------------------------------ */
  // Remove line widgets from plane to lmks, for lines that are not pointing
  // to any lmk_id in lmk_ids.
  void removeOldLines(const LandmarkIds& lmk_ids) {
    for (PlaneIdMap::value_type& plane_id_pair: plane_id_map_) {
      LmkIdToLineIdMap& lmk_id_to_line_id_map = plane_id_pair.second;
      for (LmkIdToLineIdMap::iterator lmk_id_to_line_id_it =
           lmk_id_to_line_id_map.begin();
           lmk_id_to_line_id_it != lmk_id_to_line_id_map.end();) {
        if (std::find(lmk_ids.begin(), lmk_ids.end(),
                      lmk_id_to_line_id_it->first)
            == lmk_ids.end()) {
          // We did not find the lmk_id of the current line in the list
          // of lmk_ids...
          // Delete the corresponding line.
          std::string line_id =
              "Line " + std::to_string((int)plane_id_pair.first) +
              std::to_string((int)lmk_id_to_line_id_it->second);
          removeWidget(line_id);
          // Delete the corresponding entry in the map from lmk id to line id.
          lmk_id_to_line_id_it =
              lmk_id_to_line_id_map.erase(lmk_id_to_line_id_it);
        } else {
          lmk_id_to_line_id_it++;
        }
      }
    }
  }

  /* ------------------------------------------------------------------------ */
  // Remove line widgets from plane to lmks.
  void removePlaneConstraintsViz(const PlaneId& plane_id) {
    PlaneIdMap::iterator plane_id_it = plane_id_map_.find(plane_id);
    if (plane_id_it != plane_id_map_.end()) {
      VLOG(0) << "Removing line constraints for plane with id: "
                   << plane_id;
      for (const auto& lmk_id_to_line_id: plane_id_it->second) {
        std::string line_id =
            "Line " + std::to_string((int)plane_id_it->first) +
            std::to_string((int)lmk_id_to_line_id.second);
        removeWidget(line_id);
      }
      // Delete the corresponding entry in the map for this plane.
      plane_id_map_.erase(plane_id_it);
      // Same for the map holding the line nr.
      auto line_nr_it = plane_to_line_nr_map_.find(plane_id);
      CHECK(line_nr_it != plane_to_line_nr_map_.end());
      plane_to_line_nr_map_.erase(line_nr_it);
    } else{
      // Careful if we did not find, might be because it is a newly segmented
      // plane.
      LOG(WARNING) << "Could not find plane with id: " << plane_id
                   << " from plane_id_map_...";
    }
  }

  /* ------------------------------------------------------------------------ */
  // Remove plane widget.
  void removePlane(const PlaneId& plane_index,
                   const bool& remove_plane_label = true) {
    const std::string& plane_id_for_viz =
        "Plane " + std::to_string(plane_index);
    if (is_plane_id_in_window_.find(plane_index) !=
        is_plane_id_in_window_.end() &&
        is_plane_id_in_window_[plane_index]) {
      if (removeWidget(plane_id_for_viz)) {
        if (remove_plane_label) {
          if (!removeWidget(plane_id_for_viz + "_label")) {
            LOG(WARNING) << "Did you disable labels of planes?, then also"
                            "disable label removal. Otherwise, did you change "
                            "the id of the label? then change it here as well.";
          }
        }
        is_plane_id_in_window_[plane_index] = false;
      } else {
        is_plane_id_in_window_[plane_index] = true;
      }
    }
  }

  /* ------------------------------------------------------------------------ */
  // Add pose to the previous trajectory.
  void addPoseToTrajectory(const gtsam::Pose3& current_pose_gtsam){
    trajectoryPoses3d_.push_back(UtilsOpenCV::Pose2Affine3f(current_pose_gtsam));
    static constexpr size_t trajectory_size = 50;
    while (trajectoryPoses3d_.size() > trajectory_size) {
      trajectoryPoses3d_.pop_front();
    }
  }

  /* ------------------------------------------------------------------------ */
  // Render window with drawn objects/widgets.
  // @param wait_time Amount of time in milliseconds for the event loop to keep running.
  // @param force_redraw If true, window renders.
  void renderWindow(int wait_time = 1, bool force_redraw = true) {
    window_.spinOnce(wait_time, force_redraw);
  }

private:
  cv::viz::Viz3d window_;
  std::deque<cv::Affine3f> trajectoryPoses3d_;
  cv::viz::Color cloud_color_ = cv::viz::Color::white();
  cv::viz::Color background_color_ = cv::viz::Color::black();

  std::map<PlaneId, LineNr> plane_to_line_nr_map_;
  PlaneIdMap plane_id_map_;
  std::map<PlaneId, bool> is_plane_id_in_window_;

  /* ------------------------------------------------------------------------ */
  // Log mesh to ply file.
  void logMesh(const cv::Mat& map_points_3d, const cv::Mat& colors,
               const cv::Mat& polygons_mesh, const Timestamp& timestamp,
               bool log_accumulated_mesh = false) {
    /// Log the mesh in a ply file.
    LoggerMatlab logger;
    static Timestamp last_timestamp = timestamp;
    static const Timestamp first_timestamp = timestamp;
    if ((timestamp - last_timestamp) > 6500000000) { // Log every 6 seconds approx. (a little bit more than time-horizon)
      LOG(WARNING) << "Logging mesh every (ns) = " << timestamp - last_timestamp;
      logger.openLogFiles(10, "\output_mesh_" +
                          std::to_string(timestamp - first_timestamp) + ".ply",
                          log_accumulated_mesh);
      logger.logMesh(map_points_3d, colors, polygons_mesh, timestamp,
                     log_accumulated_mesh);
      logger.closeLogFiles(10);
      last_timestamp = timestamp;
    }
  }

  /* ------------------------------------------------------------------------ */
  // Input the mesh points and triangle clusters, and
  // output colors matrix for mesh visualizer.
  // This will color the point with the color of the last plane having it.
  void colorMeshByClusters(const std::vector<Plane>& planes,
                           const cv::Mat& map_points_3d,
                           const cv::Mat& polygons_mesh,
                           cv::Mat* colors) {
    CHECK_NOTNULL(colors);
    *colors = cv::Mat(map_points_3d.rows, 1, CV_8UC3,
                      cv::viz::Color::gray());
    // The code below assumes triangles as polygons.
    static constexpr bool log_landmarks = false;
    cv::Mat points;
    for (const Plane& plane: planes) {
      const TriangleCluster& cluster = plane.triangle_cluster_;
      // Decide color for cluster.
      cv::viz::Color cluster_color = cv::viz::Color::gray();
      getColorById(cluster.cluster_id_, &cluster_color);

      for (const size_t& triangle_id: cluster.triangle_ids_) {
        size_t triangle_idx = std::round(triangle_id * 4);
        if (triangle_idx + 3 >= polygons_mesh.rows) {
          throw std::runtime_error("Visualizer3D: an id in triangle_ids_ is"
                                   " too large.");
        }
        int32_t idx_1 = polygons_mesh.at<int32_t>(triangle_idx + 1);
        int32_t idx_2 = polygons_mesh.at<int32_t>(triangle_idx + 2);
        int32_t idx_3 = polygons_mesh.at<int32_t>(triangle_idx + 3);
        // Overrides potential previous color.
        colors->row(idx_1) = cluster_color;
        colors->row(idx_2) = cluster_color;
        colors->row(idx_3) = cluster_color;
        // Debug TODO remove: logging triangles perpendicular to z_axis.
        if (cluster.cluster_id_ == 2 && log_landmarks) {
          // Do not use push back, or you'll be having repeated points I guess.
          //points.push_back(map_points_3d.row(idx_1));
          //points.push_back(map_points_3d.row(idx_2));
          //points.push_back(map_points_3d.row(idx_3));
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
  // Decide color of the cluster depending on its id.
  void getColorById(const size_t& id, cv::viz::Color* color) {
    CHECK_NOTNULL(color);
    switch (id) {
      case 0: {
        *color = cv::viz::Color::red();
        break;
      }
      case 1: {
        *color = cv::viz::Color::green();
        break;
      }
      case 2:{
        *color = cv::viz::Color::blue();
        break;
      }
      default :{
        *color = cv::viz::Color::gray();
        break;
      }
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

  /* ------------------------------------------------------------------------ */
  // Keyboard callback.
  static void keyboardCallback(const viz::KeyboardEvent &event, void *t) {
    viz::Viz3d* window = (viz::Viz3d*)t;
    toggleFreezeScreenKeyboardCallback(event.action, event.code, *window);
    getViewerPoseKeyboardCallback(event.action, event.code, *window);
    getCurrentWindowSizeKeyboardCallback(event.action, event.code, *window);
    getScreenshot(event.action, event.code, *window);
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to toggle freezing screen.
  static void toggleFreezeScreenKeyboardCallback(
      const viz::KeyboardEvent::Action& action,
      const uchar code,
      viz::Viz3d& window) {
    if (action == cv::viz::KeyboardEvent::Action::KEY_DOWN) {
      if (code == 't') {
        LOG(WARNING) << "Pressing " << code << " toggles freezing screen.";
        static bool freeze = false;
        freeze = !freeze; // Toggle.
        window.spinOnce(1, true);
        while(!window.wasStopped()) {
          if (freeze) {
            window.spinOnce(1, true);
          } else {
            break;
          }
        }
      }
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to get current viewer pose.
  static void getViewerPoseKeyboardCallback(
      const viz::KeyboardEvent::Action& action,
      const uchar& code,
      viz::Viz3d& window) {
    if (action == cv::viz::KeyboardEvent::Action::KEY_DOWN) {
      if (code == 'v') {
        LOG(INFO) << "Current viewer pose:\n"
                  << "\tRodriguez vector: " << window.getViewerPose().rvec()
                  << "\n\tAffine matrix: " << window.getViewerPose().matrix;
      }
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to get current screen size.
  static void getCurrentWindowSizeKeyboardCallback(
      const viz::KeyboardEvent::Action& action,
      const uchar code,
      viz::Viz3d& window) {
    if (action == cv::viz::KeyboardEvent::Action::KEY_DOWN) {
      if (code == 'w') {
        LOG(WARNING) << "Pressing " << code << " displays current window size:\n"
                     << "\theight: " << window.getWindowSize().height
                     << "\twidth: " << window.getWindowSize().width;
      }
    }
  }

  /* ------------------------------------------------------------------------ */
  // Keyboard callback to get screenshot of current windodw.
  static void getScreenshot(
      const viz::KeyboardEvent::Action& action,
      const uchar code,
      viz::Viz3d& window) {
    if (action == cv::viz::KeyboardEvent::Action::KEY_DOWN) {
      if (code == 's') {
        static int i = 0;
        std::string filename = "screenshot_3d_window" + std::to_string(i);
        LOG(WARNING) << "Pressing " << code << " takes a screenshot of the "
                                               "window, saved in: " + filename;
        window.saveScreenshot(filename);
      }
    }
  }


};
} // namespace VIO
#endif /* Visualizer_H_ */


