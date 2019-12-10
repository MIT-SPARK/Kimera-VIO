/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Visualizer.cpp
 * @brief  Build and visualize 2D mesh from Frame
 * @author Antoni Rosinol, AJ Haeffner, Luca Carlone
 */

#include "kimera-vio/visualizer/Visualizer3D.h"

#include <algorithm>      // for min
#include <memory>         // for shared_ptr<>
#include <string>         // for string
#include <unordered_map>  // for unordered_map<>
#include <utility>        // for pair<>
#include <vector>         // for vector<>

#include <gflags/gflags.h>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/common/FilesystemUtils.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsGTSAM.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

#include "kimera-vio/factors/PointPlaneFactor.h"  // For visualization of constraints.

DEFINE_bool(visualize_mesh, false, "Enable 3D mesh visualization.");

DEFINE_bool(visualize_mesh_2d, false, "Visualize mesh 2D.");

DEFINE_bool(visualize_semantic_mesh, false,
            "Color the 3d mesh according to their semantic labels.");
DEFINE_bool(visualize_mesh_with_colored_polygon_clusters, false,
            "Color the polygon clusters according to their cluster id.");
DEFINE_bool(visualize_point_cloud, true, "Enable point cloud visualization.");
DEFINE_bool(visualize_convex_hull, false, "Enable convex hull visualization.");
DEFINE_bool(visualize_plane_constraints, false,
            "Enable plane constraints"
            " visualization.");
DEFINE_bool(visualize_planes, false, "Enable plane visualization.");
DEFINE_bool(visualize_plane_label, false, "Enable plane label visualization.");
DEFINE_bool(visualize_mesh_in_frustum, false,
            "Enable mesh visualization in "
            "camera frustum.");
DEFINE_string(
    visualize_load_mesh_filename, "",
    "Load a mesh in the visualization, i.e. to visualize ground-truth "
    "point cloud from Euroc's Vicon dataset.");

// 3D Mesh related flags.
DEFINE_int32(mesh_shading, 0, "Mesh shading:\n 0: Flat, 1: Gouraud, 2: Phong");
DEFINE_int32(mesh_representation, 1,
             "Mesh representation:\n 0: Points, 1: Surface, 2: Wireframe");
DEFINE_bool(texturize_3d_mesh, false,
            "Whether you want to add texture to the 3d"
            "mesh. The texture is taken from the image"
            " frame.");
DEFINE_bool(set_mesh_ambient,
            false,
            "Whether to use ambient light for the "
            "mesh.");
DEFINE_bool(set_mesh_lighting, true, "Whether to use lighting for the mesh.");
DEFINE_bool(log_mesh, false, "Log the mesh at time horizon.");
DEFINE_bool(log_accumulated_mesh, false, "Accumulate the mesh when logging.");

DEFINE_int32(displayed_trajectory_length, 50,
             "Set length of plotted trajectory."
             "If -1 then all the trajectory is plotted.");

namespace VIO {

// Contains internal data for Visualizer3D window.
Visualizer3D::WindowData::WindowData()
    : window_(cv::viz::Viz3d("3D Visualizer")),
      cloud_color_(cv::viz::Color::white()),
      background_color_(cv::viz::Color::black()),
      mesh_representation_(FLAGS_mesh_representation),
      mesh_shading_(FLAGS_mesh_shading),
      mesh_ambient_(FLAGS_set_mesh_ambient),
      mesh_lighting_(FLAGS_set_mesh_lighting) {}


/* -------------------------------------------------------------------------- */
Visualizer3D::Visualizer3D(const VisualizationType& viz_type,
                           const BackendType& backend_type)
    : visualization_type_(viz_type),
      backend_type_(backend_type),
      logger_(nullptr) {
  if (FLAGS_log_mesh) {
    logger_ = VIO::make_unique<VisualizerLogger>();
  }

  if (VLOG_IS_ON(2)) {
    window_data_.window_.setGlobalWarnings(true);
  } else {
    window_data_.window_.setGlobalWarnings(false);
  }
  window_data_.window_.registerKeyboardCallback(keyboardCallback,
                                                &window_data_);
  window_data_.window_.setBackgroundColor(window_data_.background_color_);
  window_data_.window_.showWidget("Coordinate Widget",
                                  cv::viz::WCoordinateSystem());
}

/* -------------------------------------------------------------------------- */
// Returns true if visualization is ready, false otherwise.
// TODO(Toni): Put all flags inside spinOnce into Visualizer3DParams!
VisualizerOutput::UniquePtr Visualizer3D::spinOnce(
    const VisualizerInput& input) {
  DCHECK(input.frontend_output_);
  DCHECK(input.mesher_output_);
  DCHECK(input.backend_output_);

  VisualizerOutput::UniquePtr output = VIO::make_unique<VisualizerOutput>();
  output->visualization_type_ = visualization_type_;

  cv::Mat mesh_2d_img;  // Only for visualization.
  const Frame& left_stereo_keyframe =
      input.frontend_output_->stereo_frame_lkf_.getLeftFrame();
  switch (visualization_type_) {
      // Computes and visualizes 3D mesh from 2D triangulation.
      // vertices: all leftframe kps with right-VALID (3D), lmkId != -1 and
      // inside the image triangles: all the ones with edges inside images as
      // produced by cv::subdiv, which have uniform gradient (updateMesh3D also
      // filters out geometrically) Sparsity comes from filtering out triangles
      // corresponding to non planar obstacles which are assumed to have
      // non-uniform gradient.
    case VisualizationType::kMesh2dTo3dSparse: {
      // Visualize 2d mesh.
      if (FLAGS_visualize_mesh_2d || FLAGS_visualize_mesh_in_frustum) {
        const ImageToDisplay& mesh_display = ImageToDisplay(
            "Mesh 2D",
            visualizeMesh2DStereo(input.mesher_output_->mesh_2d_for_viz_,
                                  left_stereo_keyframe));
        if (FLAGS_visualize_mesh_2d) {
          output->images_to_display_.push_back(mesh_display);
        }
        if (FLAGS_visualize_mesh_in_frustum) {
          mesh_2d_img = mesh_display.image_;
        }
      }

      // 3D mesh visualization
      VLOG(10) << "Starting 3D mesh visualization...";

      static std::vector<Plane> planes_prev;
      static PointsWithIdMap points_with_id_VIO_prev;
      static LmkIdToLmkTypeMap lmk_id_to_lmk_type_map_prev;
      static cv::Mat vertices_mesh_prev;
      static cv::Mat polygons_mesh_prev;
      static Mesh3DVizProperties mesh_3d_viz_props_prev;

      if (FLAGS_visualize_mesh) {
        VLOG(10) << "Visualize mesh.";
        if (FLAGS_visualize_semantic_mesh) {
          VLOG(10) << "Visualize Semantic mesh.";
          LOG_IF(WARNING, FLAGS_visualize_mesh_with_colored_polygon_clusters)
              << "Both gflags visualize_semantic_mesh and "
                 "visualize_mesh_with_colored_polygon_cluster are set to True,"
                 " but visualization of the semantic mesh has priority over "
                 "visualization of the polygon clusters.";
          visualizeMesh3D(vertices_mesh_prev,
                          mesh_3d_viz_props_prev.colors_,
                          polygons_mesh_prev,
                          mesh_3d_viz_props_prev.tcoords_,
                          mesh_3d_viz_props_prev.texture_);
        } else {
          VLOG(10) << "Visualize mesh with colored clusters.";
          LOG_IF(ERROR, mesh_3d_viz_props_prev.colors_.rows > 0u)
              << "The 3D mesh is being colored with semantic information, but"
                 " gflag visualize_semantic_mesh is set to false...";
          visualizeMesh3DWithColoredClusters(
              planes_prev,
              vertices_mesh_prev,
              polygons_mesh_prev,
              FLAGS_visualize_mesh_with_colored_polygon_clusters,
              input.timestamp_);
        }
      }

      if (FLAGS_visualize_point_cloud) {
        visualizePoints3D(points_with_id_VIO_prev, lmk_id_to_lmk_type_map_prev);
      }

      if (!FLAGS_visualize_load_mesh_filename.empty()) {
        static bool visualize_ply_mesh_once = true;
        if (visualize_ply_mesh_once) {
          visualizePlyMesh(FLAGS_visualize_load_mesh_filename.c_str());
          visualize_ply_mesh_once = false;
        }
      }

      if (FLAGS_visualize_convex_hull) {
        if (planes_prev.size() != 0) {
          visualizeConvexHull(planes_prev.at(0).triangle_cluster_,
                              vertices_mesh_prev, polygons_mesh_prev);
        }
      }

      if (backend_type_ == BackendType::kStructuralRegularities &&
          FLAGS_visualize_plane_constraints) {
        LandmarkIds lmk_ids_in_current_pp_factors;
        for (const auto& g : input.backend_output_->graph_) {
          const auto& ppf =
              boost::dynamic_pointer_cast<gtsam::PointPlaneFactor>(g);
          if (ppf) {
            // We found a PointPlaneFactor.
            // Get point key.
            Key point_key = ppf->getPointKey();
            LandmarkId lmk_id = gtsam::Symbol(point_key).index();
            lmk_ids_in_current_pp_factors.push_back(lmk_id);
            // Get point estimate.
            gtsam::Point3 point;
            // This call makes visualizer unable to perform this
            // in parallel. But you can just copy the state_
            CHECK(getEstimateOfKey(
                input.backend_output_->state_, point_key, &point));
            // Visualize.
            const Key& ppf_plane_key = ppf->getPlaneKey();
            // not sure, we are having some w planes_prev
            // others with planes...
            for (const Plane& plane : input.mesher_output_->planes_) {
              if (ppf_plane_key == plane.getPlaneSymbol().key()) {
                gtsam::OrientedPlane3 current_plane_estimate;
                CHECK(getEstimateOfKey(  // This call makes visualizer
                                         // unable to perform this in
                                         // parallel.
                    input.backend_output_->state_,
                    ppf_plane_key,
                    &current_plane_estimate));
                // WARNING assumes the backend updates normal and distance
                // of plane and that no one modifies it afterwards...
                visualizePlaneConstraints(
                    plane.getPlaneSymbol().key(),
                    current_plane_estimate.normal().point3(),
                    current_plane_estimate.distance(), lmk_id, point);
                // Stop since there are not multiple planes for one
                // ppf.
                break;
              }
            }
          }
        }

        // Remove lines that are not representing a point plane factor
        // in the current graph.
        removeOldLines(lmk_ids_in_current_pp_factors);
      }

      // Must go after visualize plane constraints.
      if (FLAGS_visualize_planes || FLAGS_visualize_plane_constraints) {
        for (const Plane& plane : input.mesher_output_->planes_) {
          const gtsam::Symbol& plane_symbol = plane.getPlaneSymbol();
          const std::uint64_t& plane_index = plane_symbol.index();
          gtsam::OrientedPlane3 current_plane_estimate;
          if (getEstimateOfKey<gtsam::OrientedPlane3>(
                  input.backend_output_->state_,
                  plane_symbol.key(),
                  &current_plane_estimate)) {
            const cv::Point3d& plane_normal_estimate =
                UtilsOpenCV::unit3ToPoint3d(current_plane_estimate.normal());
            CHECK(plane.normal_ == plane_normal_estimate);
            // We have the plane in the optimization.
            // Visualize plane.
            visualizePlane(plane_index, plane_normal_estimate.x,
                           plane_normal_estimate.y, plane_normal_estimate.z,
                           current_plane_estimate.distance(),
                           FLAGS_visualize_plane_label,
                           plane.triangle_cluster_.cluster_id_);
          } else {
            // We could not find the plane in the optimization...
            // Careful cause we might enter here because there are new
            // segmented planes.
            // Delete the plane.
            LOG(ERROR) << "Remove plane viz for id:" << plane_index;
            if (FLAGS_visualize_plane_constraints) {
              removePlaneConstraintsViz(plane_index);
            }
            removePlane(plane_index);
          }
        }

        // Also remove planes that were deleted by the backend...
        for (const Plane& plane : planes_prev) {
          const gtsam::Symbol& plane_symbol = plane.getPlaneSymbol();
          const std::uint64_t& plane_index = plane_symbol.index();
          gtsam::OrientedPlane3 current_plane_estimate;
          if (!getEstimateOfKey(input.backend_output_->state_,
                                plane_symbol.key(),
                                &current_plane_estimate)) {
            // We could not find the plane in the optimization...
            // Delete the plane.
            if (FLAGS_visualize_plane_constraints) {
              removePlaneConstraintsViz(plane_index);
            }
            removePlane(plane_index);
          }
        }
      }

      planes_prev = input.mesher_output_->planes_;
      vertices_mesh_prev = input.mesher_output_->vertices_mesh_;
      polygons_mesh_prev = input.mesher_output_->polygons_mesh_;
      points_with_id_VIO_prev = input.backend_output_->landmarks_with_id_map_;
      lmk_id_to_lmk_type_map_prev =
          input.backend_output_->lmk_id_to_lmk_type_map_;
      LOG_IF(WARNING, mesh3d_viz_properties_callback_)
          << "Coloring the mesh using semantic segmentation colors.";
      mesh_3d_viz_props_prev =
          // Call semantic mesh segmentation if someone registered a callback.
          mesh3d_viz_properties_callback_
              ? mesh3d_viz_properties_callback_(left_stereo_keyframe.timestamp_,
                                                left_stereo_keyframe.img_,
                                                input.mesher_output_->mesh_2d_,
                                                input.mesher_output_->mesh_3d_)
              : (FLAGS_texturize_3d_mesh ? Visualizer3D::texturizeMesh3D(
                                               left_stereo_keyframe.timestamp_,
                                               left_stereo_keyframe.img_,
                                               input.mesher_output_->mesh_2d_,
                                               input.mesher_output_->mesh_3d_)
                                         : Mesh3DVizProperties()),
      VLOG(10) << "Finished mesh visualization.";

      break;
    }

    // Computes and visualizes a 3D point cloud with VIO points in current time
    // horizon of the optimization.
    case VisualizationType::kPointcloud: {
      // Do not color the cloud, send empty lmk id to lmk type map
      visualizePoints3D(input.backend_output_->landmarks_with_id_map_,
                        input.backend_output_->lmk_id_to_lmk_type_map_);
      break;
    }
    case VisualizationType::kNone: {
      break;
    }
  }

  // Visualize trajectory.
  VLOG(10) << "Starting trajectory visualization...";
  addPoseToTrajectory(input.backend_output_->W_State_Blkf_.pose_.compose(
      input.frontend_output_->stereo_frame_lkf_.getBPoseCamLRect()));
  visualizeTrajectory3D(FLAGS_visualize_mesh_in_frustum
                            ? mesh_2d_img
                            : left_stereo_keyframe.img_);
  VLOG(10) << "Finished trajectory visualization.";

  // TODO avoid copying and use a std::unique_ptr! You need to pass window_data_
  // as a parameter to all the functions and set them as const.
  output->window_ = window_data_.window_;

  return output;
}

/* -------------------------------------------------------------------------- */
// Create a 2D mesh from 2D corners in an image
cv::Mat Visualizer3D::visualizeMesh2D(
    const std::vector<cv::Vec6f>& triangulation2D,
    const cv::Mat& img,
    const KeypointsCV& extra_keypoints) {
  static const cv::Scalar kDelaunayColor(0u, 255u, 0u);
  static const cv::Scalar kPointsColor(255u, 0u, 0u);

  // Duplicate image for annotation and visualization.
  cv::Mat img_clone = img.clone();
  cv::cvtColor(img_clone, img_clone, cv::COLOR_GRAY2BGR);
  cv::Size size = img_clone.size();
  cv::Rect rect(0, 0, size.width, size.height);
  std::vector<cv::Point> pt(3);
  for (size_t i = 0; i < triangulation2D.size(); i++) {
    const cv::Vec6f& t = triangulation2D[i];

    // Visualize mesh vertices.
    pt[0] = cv::Point(cvRound(t[0]), cvRound(t[1]));
    pt[1] = cv::Point(cvRound(t[2]), cvRound(t[3]));
    pt[2] = cv::Point(cvRound(t[4]), cvRound(t[5]));

    // Visualize mesh edges.
    cv::line(img_clone, pt[0], pt[1], kDelaunayColor, 1, CV_AA, 0);
    cv::line(img_clone, pt[1], pt[2], kDelaunayColor, 1, CV_AA, 0);
    cv::line(img_clone, pt[2], pt[0], kDelaunayColor, 1, CV_AA, 0);
  }

  // Visualize extra vertices.
  for (const auto& keypoint : extra_keypoints) {
    cv::circle(img_clone, keypoint, 2, kPointsColor, CV_FILLED, CV_AA, 0);
  }

  return img_clone;
}

/* -------------------------------------------------------------------------- */
// Visualize 2d mesh.
cv::Mat Visualizer3D::visualizeMesh2DStereo(
    const std::vector<cv::Vec6f>& triangulation_2D,
    const Frame& ref_frame) {
  static const cv::Scalar kDelaunayColor(0, 255, 0);          // Green
  static const cv::Scalar kMeshVertexColor(255, 0, 0);        // Blue
  static const cv::Scalar kInvalidKeypointsColor(0, 0, 255);  // Red

  // Sanity check.
  DCHECK(ref_frame.landmarks_.size() == ref_frame.keypoints_.size())
      << "Frame: wrong dimension for the landmarks.";

  // Duplicate image for annotation and visualization.
  cv::Mat img_clone = ref_frame.img_.clone();
  cv::cvtColor(img_clone, img_clone, cv::COLOR_GRAY2BGR);

  // Visualize extra vertices.
  for (size_t i = 0; i < ref_frame.keypoints_.size(); i++) {
    // Only for valid keypoints, but possibly without a right pixel.
    // Kpts that are both valid and have a right pixel are currently the ones
    // passed to the mesh.
    if (ref_frame.landmarks_[i] != -1) {
      cv::circle(img_clone,
                 ref_frame.keypoints_[i],
                 2,
                 kInvalidKeypointsColor,
                 CV_FILLED,
                 CV_AA,
                 0);
    }
  }

  std::vector<cv::Point> pt(3);
  for (const cv::Vec6f& triangle : triangulation_2D) {
    // Visualize mesh vertices.
    pt[0] = cv::Point(cvRound(triangle[0]), cvRound(triangle[1]));
    pt[1] = cv::Point(cvRound(triangle[2]), cvRound(triangle[3]));
    pt[2] = cv::Point(cvRound(triangle[4]), cvRound(triangle[5]));
    cv::circle(img_clone, pt[0], 2, kMeshVertexColor, CV_FILLED, CV_AA, 0);
    cv::circle(img_clone, pt[1], 2, kMeshVertexColor, CV_FILLED, CV_AA, 0);
    cv::circle(img_clone, pt[2], 2, kMeshVertexColor, CV_FILLED, CV_AA, 0);

    // Visualize mesh edges.
    cv::line(img_clone, pt[0], pt[1], kDelaunayColor, 1, CV_AA, 0);
    cv::line(img_clone, pt[1], pt[2], kDelaunayColor, 1, CV_AA, 0);
    cv::line(img_clone, pt[2], pt[0], kDelaunayColor, 1, CV_AA, 0);
  }

  return img_clone;
}

/* -------------------------------------------------------------------------- */
// Visualize a 3D point cloud of unique 3D landmarks.
void Visualizer3D::visualizePoints3D(
    const PointsWithIdMap& points_with_id,
    const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map) {
  bool color_the_cloud = false;
  if (lmk_id_to_lmk_type_map.size() != 0) {
    color_the_cloud = true;
    CHECK_EQ(points_with_id.size(), lmk_id_to_lmk_type_map.size());
  }

  // Sanity check dimension.
  if (points_with_id.size() == 0) {
    // No points to visualize.
    LOG(WARNING) << "No landmark information for Visualizer. "
                    "Not displaying 3D points.";
    return;
  }

  // Populate cloud structure with 3D points.
  cv::Mat point_cloud(1, points_with_id.size(), CV_32FC3);
  cv::Mat point_cloud_color(1, lmk_id_to_lmk_type_map.size(), CV_8UC3,
                            window_data_.cloud_color_);
  cv::Point3f* data = point_cloud.ptr<cv::Point3f>();
  size_t i = 0;
  for (const std::pair<LandmarkId, gtsam::Point3>& id_point : points_with_id) {
    const gtsam::Point3& point_3d = id_point.second;
    data[i].x = static_cast<float>(point_3d.x());
    data[i].y = static_cast<float>(point_3d.y());
    data[i].z = static_cast<float>(point_3d.z());
    if (color_the_cloud) {
      DCHECK(lmk_id_to_lmk_type_map.find(id_point.first) !=
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
  cv::viz::WCloud cloud_widget(point_cloud, window_data_.cloud_color_);
  if (color_the_cloud) {
    cloud_widget = cv::viz::WCloud(point_cloud, point_cloud_color);
  }
  cloud_widget.setRenderingProperty(cv::viz::POINT_SIZE, 6);

  window_data_.window_.showWidget("Point cloud.", cloud_widget);
}

/* -------------------------------------------------------------------------- */
// Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
void Visualizer3D::visualizePlane(const PlaneId& plane_index, const double& n_x,
                                  const double& n_y, const double& n_z,
                                  const double& d,
                                  const bool& visualize_plane_label,
                                  const int& cluster_id) {
  const std::string& plane_id_for_viz = "Plane " + std::to_string(plane_index);
  // Create a plane widget.
  const cv::Vec3d normal(n_x, n_y, n_z);
  const cv::Point3d center(d * n_x, d * n_y, d * n_z);
  static const cv::Vec3d new_yaxis(0, 1, 0);
  static const cv::Size2d size(1.0, 1.0);

  cv::viz::Color plane_color;
  getColorById(cluster_id, &plane_color);
  cv::viz::WPlane plane_widget(center, normal, new_yaxis, size, plane_color);

  if (visualize_plane_label) {
    static double increase = 0.0;
    const cv::Point3d text_position(d * n_x, d * n_y,
                                    d * n_z + std::fmod(increase, 1));
    increase += 0.1;
    window_data_.window_.showWidget(
        plane_id_for_viz + "_label",
        cv::viz::WText3D(plane_id_for_viz, text_position, 0.07, true));
  }

  window_data_.window_.showWidget(plane_id_for_viz, plane_widget);
  is_plane_id_in_window_[plane_index] = true;
}

/* -------------------------------------------------------------------------- */
// Draw a line in opencv.
void Visualizer3D::drawLine(const std::string& line_id, const double& from_x,
                            const double& from_y, const double& from_z,
                            const double& to_x, const double& to_y,
                            const double& to_z) {
  cv::Point3d pt1(from_x, from_y, from_z);
  cv::Point3d pt2(to_x, to_y, to_z);
  drawLine(line_id, pt1, pt2);
}

/* -------------------------------------------------------------------------- */
void Visualizer3D::drawLine(const std::string& line_id, const cv::Point3d& pt1,
                            const cv::Point3d& pt2) {
  cv::viz::WLine line_widget(pt1, pt2);
  window_data_.window_.showWidget(line_id, line_widget);
}

/* -------------------------------------------------------------------------- */
// Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
void Visualizer3D::visualizeMesh3D(const cv::Mat& map_points_3d,
                                   const cv::Mat& polygons_mesh) {
  cv::Mat colors(0, 1, CV_8UC3, cv::viz::Color::gray());  // Do not color mesh.
  visualizeMesh3D(map_points_3d, colors, polygons_mesh);
}

/* -------------------------------------------------------------------------- */
// Visualize a 3D point cloud of unique 3D landmarks with its connectivity,
// and provide color for each polygon.
void Visualizer3D::visualizeMesh3D(const cv::Mat& map_points_3d,
                                   const cv::Mat& colors,
                                   const cv::Mat& polygons_mesh,
                                   const cv::Mat& tcoords,
                                   const cv::Mat& texture) {
  // Check data
  bool color_mesh = false;
  if (colors.rows != 0) {
    CHECK_EQ(map_points_3d.rows, colors.rows)
        << "Map points and Colors should have same number of rows. One"
           " color per map point.";
    LOG(ERROR) << "Coloring mesh!";
    color_mesh = true;
  }

  if (tcoords.rows != 0) {
    CHECK_EQ(map_points_3d.rows, tcoords.rows)
        << "Map points and tcoords should have same number of rows. One"
           "tcoord per map point.";
    CHECK(!texture.empty());
  }

  // No points/mesh to visualize.
  if (map_points_3d.rows == 0 || polygons_mesh.rows == 0) {
    return;
  }

  cv::viz::Mesh cv_mesh;
  cv_mesh.cloud = map_points_3d.t();
  cv_mesh.polygons = polygons_mesh;
  cv_mesh.colors = color_mesh ? colors.t() : cv::Mat();
  cv_mesh.tcoords = tcoords.t();
  cv_mesh.texture = texture;

  // Create a mesh widget.
  cv::viz::WMesh mesh(cv_mesh);

  // Decide mesh shading style.
  switch (window_data_.mesh_shading_) {
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
  switch (window_data_.mesh_representation_) {
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
  mesh.setRenderingProperty(cv::viz::AMBIENT, window_data_.mesh_ambient_);
  mesh.setRenderingProperty(cv::viz::LIGHTING, window_data_.mesh_lighting_);

  // Plot mesh.
  window_data_.window_.showWidget("Mesh", mesh);
}

/* -------------------------------------------------------------------------- */
// Visualize a PLY from filename (absolute path).
void Visualizer3D::visualizePlyMesh(const std::string& filename) {
  LOG(INFO) << "Showing ground truth mesh: " << filename;
  // The ply file must have in the header a "element vertex" and
  // a "element face" primitives, otherwise you'll get a
  // "Cannot read geometry" error.
  cv::viz::Mesh mesh(cv::viz::Mesh::load(filename));
  if (mesh.polygons.size[1] == 0) {
    LOG(WARNING) << "No polygons available for mesh, showing point cloud only.";
    // If there are no polygons, convert to point cloud, otw there will be
    // nothing displayed...
    cv::viz::WCloud cloud(mesh.cloud, cv::viz::Color::lime());
    cloud.setRenderingProperty(cv::viz::REPRESENTATION,
                               cv::viz::REPRESENTATION_POINTS);
    cloud.setRenderingProperty(cv::viz::POINT_SIZE, 2);
    cloud.setRenderingProperty(cv::viz::OPACITY, 0.1);

    // Plot point cloud.
    window_data_.window_.showWidget("Mesh from ply", cloud);
  } else {
    // Plot mesh.
    window_data_.window_.showWidget("Mesh from ply", cv::viz::WMesh(mesh));
  }
}

/* -------------------------------------------------------------------------- */
// Visualize a 3D point cloud of unique 3D landmarks with its connectivity.
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
void Visualizer3D::visualizeMesh3DWithColoredClusters(
    const std::vector<Plane>& planes, const cv::Mat& map_points_3d,
    const cv::Mat& polygons_mesh,
    const bool visualize_mesh_with_colored_polygon_clusters,
    const Timestamp& timestamp) {
  if (visualize_mesh_with_colored_polygon_clusters) {
    // Color the mesh.
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

/* -------------------------------------------------------------------------- */
// Visualize convex hull in 2D for set of points in triangle cluster,
// projected along the normal of the cluster.
void Visualizer3D::visualizeConvexHull(const TriangleCluster& cluster,
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
  cv::Point3f x_axis(random_x, random_y, z);
  // Normalize new first axis:
  x_axis /= cv::norm(x_axis);
  // Create new second axis, by cross product normal to x_axis;
  cv::Point3f y_axis = normal.cross(x_axis);
  // Normalize just in case?
  y_axis /= cv::norm(y_axis);
  // Construct new cartesian coord system:
  cv::Mat new_coordinates(3, 3, CV_32FC1);
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
  for (const size_t& triangle_id : cluster.triangle_ids_) {
    size_t triangle_idx = std::round(triangle_id * 4);
    if (triangle_idx + 3 >= polygons_mesh.rows) {
      throw std::runtime_error(
          "Visualizer3D: an id in triangle_ids_ is"
          " too large.");
    }
    int32_t idx_1 = polygons_mesh.at<int32_t>(triangle_idx + 1);
    int32_t idx_2 = polygons_mesh.at<int32_t>(triangle_idx + 2);
    int32_t idx_3 = polygons_mesh.at<int32_t>(triangle_idx + 3);

    // Project points to new coord system
    cv::Point3f new_map_point_1 =  // new_coordinates *
                                   // map_points_3d.row(idx_1).t();
        map_points_3d.at<cv::Point3f>(idx_1);
    cv::Point3f new_map_point_2 =  // new_coordinates *
                                   // map_points_3d.row(idx_2).t();
        map_points_3d.at<cv::Point3f>(idx_2);
    cv::Point3f new_map_point_3 =  // new_coordinates *
                                   // map_points_3d.row(idx_3).t();
        map_points_3d.at<cv::Point3f>(idx_3);

    // Keep only 1st and 2nd component, aka the projection of the point on the
    // plane.
    points_2d.push_back(cv::Point2f(new_map_point_1.x, new_map_point_1.y));
    z_s.push_back(new_map_point_1.z);
    points_2d.push_back(cv::Point2f(new_map_point_2.x, new_map_point_2.y));
    z_s.push_back(new_map_point_2.z);
    points_2d.push_back(cv::Point2f(new_map_point_3.x, new_map_point_3.y));
    z_s.push_back(new_map_point_3.z);
  }

  // Create convex hull.
  if (points_2d.size() != 0) {
    std::vector<int> hull_idx;
    convexHull(cv::Mat(points_2d), hull_idx, false);

    // Add the z component.
    std::vector<cv::Point3f> hull_3d;
    for (const int& idx : hull_idx) {
      hull_3d.push_back(
          cv::Point3f(points_2d.at(idx).x, points_2d.at(idx).y, z_s.at(idx)));
    }

    static constexpr bool visualize_hull_as_polyline = false;
    if (visualize_hull_as_polyline) {
      // Close the hull.
      CHECK_NE(hull_idx.size(), 0);
      hull_3d.push_back(cv::Point3f(points_2d.at(hull_idx.at(0)).x,
                                    points_2d.at(hull_idx.at(0)).y,
                                    z_s.at(hull_idx.at(0))));
      // Visualize convex hull.
      cv::viz::WPolyLine convex_hull(hull_3d);
      window_data_.window_.showWidget("Convex hull", convex_hull);
    } else {
      // Visualize convex hull as a mesh of one polygon with multiple points.
      if (hull_3d.size() > 2) {
        cv::Mat polygon_hull = cv::Mat(4 * (hull_3d.size() - 2), 1, CV_32SC1);
        size_t i = 1;
        for (size_t k = 0; k + 3 < polygon_hull.rows; k += 4) {
          polygon_hull.row(k) = 3;
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
        cv::Mat colors(hull_3d.size(), 1, CV_8UC3, mesh_color);
        cv::viz::WMesh mesh(hull_3d, polygon_hull, colors.t(), normals.t());
        window_data_.window_.showWidget("Convex hull", mesh);
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
// Visualize trajectory. Adds an image to the frustum if cv::Mat is not empty.
void Visualizer3D::visualizeTrajectory3D(const cv::Mat& frustum_image) {
  if (trajectory_poses_3d_.size() == 0) {  // no points to visualize
    return;
  }

  // Show current camera pose.
  static const cv::Matx33d K(458, 0.0, 360, 0.0, 458, 240, 0.0, 0.0, 1.0);
  cv::viz::WCameraPosition cam_widget_ptr;
  if (frustum_image.empty()) {
    cam_widget_ptr = cv::viz::WCameraPosition(K, 1.0, cv::viz::Color::white());
  } else {
    cam_widget_ptr = cv::viz::WCameraPosition(K, frustum_image, 1.0,
                                              cv::viz::Color::white());
  }
  window_data_.window_.showWidget(
      "Camera Pose with Frustum", cam_widget_ptr, trajectory_poses_3d_.back());
  window_data_.window_.setWidgetPose("Camera Pose with Frustum",
                                     trajectory_poses_3d_.back());

  // Option A: This does not work very well.
  // window_data_.window_.resetCameraViewpoint("Camera Pose with Frustum");
  // Viewer is our viewpoint, camera the pose estimate (frustum).
  static constexpr bool follow_camera = false;
  if (follow_camera) {
    cv::Affine3f camera_in_world_coord = trajectory_poses_3d_.back();
    // Option B: specify viewer wrt camera. Works, but motion is non-smooth.
    // cv::Affine3f viewer_in_camera_coord (Vec3f(
    //                                       -0.3422019, -0.3422019, 1.5435732),
    //                                     Vec3f(3.0, 0.0, -4.5));
    // cv::Affine3f viewer_in_world_coord =
    //    viewer_in_camera_coord.concatenate(camera_in_world_coord);

    // Option C: use "look-at" camera parametrization.
    // Works, but motion is non-smooth as well.
    cv::Vec3d cam_pos(-6.0, 0.0, 6.0);
    cv::Vec3d cam_focal_point(camera_in_world_coord.translation());
    cv::Vec3d cam_y_dir(0.0, 0.0, -1.0);
    cv::Affine3f cam_pose =
        cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    window_data_.window_.setViewerPose(cam_pose);
    // window_data_.window_.setViewerPose(viewer_in_world_coord);
  }

  // Create a Trajectory frustums widget.
  std::vector<cv::Affine3f> trajectory_frustums(
      trajectory_poses_3d_.end() -
          std::min(trajectory_poses_3d_.size(), size_t(10u)),
      trajectory_poses_3d_.end());
  cv::viz::WTrajectoryFrustums trajectory_frustums_widget(
      trajectory_frustums, K, 0.2, cv::viz::Color::red());
  window_data_.window_.showWidget("Trajectory Frustums",
                                  trajectory_frustums_widget);

  // Create a Trajectory widget. (argument can be PATH, FRAMES, BOTH).
  std::vector<cv::Affine3f> trajectory(trajectory_poses_3d_.begin(),
                                       trajectory_poses_3d_.end());
  cv::viz::WTrajectory trajectory_widget(trajectory, cv::viz::WTrajectory::PATH,
                                         1.0, cv::viz::Color::red());
  window_data_.window_.showWidget("Trajectory", trajectory_widget);
}

/* -------------------------------------------------------------------------- */
// Remove widget. True if successful, false if not.
bool Visualizer3D::removeWidget(const std::string& widget_id) {
  try {
    window_data_.window_.removeWidget(widget_id);
    return true;
  } catch (const cv::Exception& e) {
    VLOG(20) << e.what();
    LOG(ERROR) << "Widget with id: " << widget_id.c_str()
               << " is not in window.";
  } catch (...) {
    LOG(ERROR) << "Unrecognized exception when using "
                  "window_data_.window_.removeWidget() "
               << "with widget with id: " << widget_id.c_str();
  }
  return false;
}

/* -------------------------------------------------------------------------- */
// Visualize line widgets from plane to lmks.
// Point key is required to avoid duplicated lines!
void Visualizer3D::visualizePlaneConstraints(const PlaneId& plane_id,
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
    DCHECK(plane_to_line_nr_map_.find(plane_id) != plane_to_line_nr_map_.end());
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
    std::string line_id = "Line " + std::to_string((int)plane_id_it->first) +
                          std::to_string((int)(*line_nr_ptr));
    // Draw it.
    drawLineFromPlaneToPoint(line_id, normal.x(), normal.y(), normal.z(),
                             distance, point.x(), point.y(), point.z());
    // Augment line_nr for next line_id.
    (*line_nr_ptr)++;
  } else {
    // We have drawn this line before.
    // Update line.
    std::string line_id = "Line " + std::to_string((int)plane_id_it->first) +
                          std::to_string((int)lmk_id_to_line_id->second);
    updateLineFromPlaneToPoint(line_id, normal.x(), normal.y(), normal.z(),
                               distance, point.x(), point.y(), point.z());
  }
}

/* -------------------------------------------------------------------------- */
// Remove line widgets from plane to lmks, for lines that are not pointing
// to any lmk_id in lmk_ids.
void Visualizer3D::removeOldLines(const LandmarkIds& lmk_ids) {
  for (PlaneIdMap::value_type& plane_id_pair : plane_id_map_) {
    LmkIdToLineIdMap& lmk_id_to_line_id_map = plane_id_pair.second;
    for (LmkIdToLineIdMap::iterator lmk_id_to_line_id_it =
             lmk_id_to_line_id_map.begin();
         lmk_id_to_line_id_it != lmk_id_to_line_id_map.end();) {
      if (std::find(lmk_ids.begin(), lmk_ids.end(),
                    lmk_id_to_line_id_it->first) == lmk_ids.end()) {
        // We did not find the lmk_id of the current line in the list
        // of lmk_ids...
        // Delete the corresponding line.
        std::string line_id = "Line " +
                              std::to_string((int)plane_id_pair.first) +
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

/* -------------------------------------------------------------------------- */
// Remove line widgets from plane to lmks.
void Visualizer3D::removePlaneConstraintsViz(const PlaneId& plane_id) {
  PlaneIdMap::iterator plane_id_it = plane_id_map_.find(plane_id);
  if (plane_id_it != plane_id_map_.end()) {
    VLOG(0) << "Removing line constraints for plane with id: " << plane_id;
    for (const auto& lmk_id_to_line_id : plane_id_it->second) {
      std::string line_id = "Line " + std::to_string((int)plane_id_it->first) +
                            std::to_string((int)lmk_id_to_line_id.second);
      removeWidget(line_id);
    }
    // Delete the corresponding entry in the map for this plane.
    plane_id_map_.erase(plane_id_it);
    // Same for the map holding the line nr.
    auto line_nr_it = plane_to_line_nr_map_.find(plane_id);
    CHECK(line_nr_it != plane_to_line_nr_map_.end());
    plane_to_line_nr_map_.erase(line_nr_it);
  } else {
    // Careful if we did not find, might be because it is a newly segmented
    // plane.
    LOG(WARNING) << "Could not find plane with id: " << plane_id
                 << " from plane_id_map_...";
  }
}

/* -------------------------------------------------------------------------- */
// Remove plane widget.
void Visualizer3D::removePlane(const PlaneId& plane_index,
                               const bool& remove_plane_label) {
  const std::string& plane_id_for_viz = "Plane " + std::to_string(plane_index);
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

/* -------------------------------------------------------------------------- */
// Add pose to the previous trajectory.
void Visualizer3D::addPoseToTrajectory(const gtsam::Pose3& current_pose_gtsam) {
  trajectory_poses_3d_.push_back(
      UtilsOpenCV::gtsamPose3ToCvAffine3d(current_pose_gtsam));
  if (FLAGS_displayed_trajectory_length > 0) {
    while (trajectory_poses_3d_.size() > FLAGS_displayed_trajectory_length) {
      trajectory_poses_3d_.pop_front();
    }
  }
}

/* -------------------------------------------------------------------------- */
/** Render window with drawn objects/widgets.
 * @param wait_time Amount of time in milliseconds for the event loop to keep
 * running.
 * @param force_redraw If true, window renders.
 */
void Visualizer3D::renderWindow(int wait_time, bool force_redraw) {
  window_data_.window_.spinOnce(wait_time, force_redraw);
}

/* -------------------------------------------------------------------------- */
// Get a screenshot of the window.
void Visualizer3D::getScreenshot(const std::string& filename) {
  LOG(WARNING) << "Taking a screenshot of the window, saved in: " + filename;
  window_data_.window_.saveScreenshot(filename);
}

/* -------------------------------------------------------------------------- */
void Visualizer3D::setOffScreenRendering() {
  window_data_.window_.setOffScreenRendering();
}

/* -------------------------------------------------------------------------- */
// Log mesh to ply file.
void Visualizer3D::logMesh(const cv::Mat& map_points_3d, const cv::Mat& colors,
                           const cv::Mat& polygons_mesh,
                           const Timestamp& timestamp,
                           bool log_accumulated_mesh) {
  /// Log the mesh in a ply file.
  static Timestamp last_timestamp = timestamp;
  static const Timestamp first_timestamp = timestamp;
  if ((timestamp - last_timestamp) >
      6500000000) {  // Log every 6 seconds approx. (a little bit more than
                     // time-horizon)
    LOG(WARNING) << "Logging mesh every (ns) = " << timestamp - last_timestamp;
    CHECK(logger_);
    logger_->logMesh(
        map_points_3d, colors, polygons_mesh, timestamp, log_accumulated_mesh);
    last_timestamp = timestamp;
  }
}

/* -------------------------------------------------------------------------- */
// Input the mesh points and triangle clusters, and
// output colors matrix for mesh visualizer.
// This will color the point with the color of the last plane having it.
void Visualizer3D::colorMeshByClusters(const std::vector<Plane>& planes,
                                       const cv::Mat& map_points_3d,
                                       const cv::Mat& polygons_mesh,
                                       cv::Mat* colors) const {
  CHECK_NOTNULL(colors);
  *colors = cv::Mat(map_points_3d.rows, 1, CV_8UC3, cv::viz::Color::gray());

  // The code below assumes triangles as polygons.
  static constexpr bool log_landmarks = false;
  for (const Plane& plane : planes) {
    const TriangleCluster& cluster = plane.triangle_cluster_;
    // Decide color for cluster.
    cv::viz::Color cluster_color = cv::viz::Color::gray();
    getColorById(cluster.cluster_id_, &cluster_color);

    for (const size_t& triangle_id : cluster.triangle_ids_) {
      size_t triangle_idx = std::round(triangle_id * 4);
      DCHECK_LE(triangle_idx + 3, polygons_mesh.rows)
          << "Visualizer3D: an id in triangle_ids_ is too large.";
      int32_t idx_1 = polygons_mesh.at<int32_t>(triangle_idx + 1);
      int32_t idx_2 = polygons_mesh.at<int32_t>(triangle_idx + 2);
      int32_t idx_3 = polygons_mesh.at<int32_t>(triangle_idx + 3);
      // Overrides potential previous color.
      colors->row(idx_1) = cluster_color;
      colors->row(idx_2) = cluster_color;
      colors->row(idx_3) = cluster_color;
    }
  }
}

/* -------------------------------------------------------------------------- */
// Decide color of the cluster depending on its id.
void Visualizer3D::getColorById(const size_t& id, cv::viz::Color* color) const {
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
    case 2: {
      *color = cv::viz::Color::blue();
      break;
    }
    default: {
      *color = cv::viz::Color::gray();
      break;
    }
  }
}

/* -------------------------------------------------------------------------- */
// Draw a line from lmk to plane center.
void Visualizer3D::drawLineFromPlaneToPoint(
    const std::string& line_id, const double& plane_n_x,
    const double& plane_n_y, const double& plane_n_z, const double& plane_d,
    const double& point_x, const double& point_y, const double& point_z) {
  const cv::Point3d center(plane_d * plane_n_x, plane_d * plane_n_y,
                           plane_d * plane_n_z);
  const cv::Point3d point(point_x, point_y, point_z);
  drawLine(line_id, center, point);
}

/* -------------------------------------------------------------------------- */
// Update line from lmk to plane center.
void Visualizer3D::updateLineFromPlaneToPoint(
    const std::string& line_id, const double& plane_n_x,
    const double& plane_n_y, const double& plane_n_z, const double& plane_d,
    const double& point_x, const double& point_y, const double& point_z) {
  removeWidget(line_id);
  drawLineFromPlaneToPoint(line_id, plane_n_x, plane_n_y, plane_n_z, plane_d,
                           point_x, point_y, point_z);
}

/* -------------------------------------------------------------------------- */
void Visualizer3D::keyboardCallback(const cv::viz::KeyboardEvent& event,
                                    void* t) {
  WindowData* window_data = (Visualizer3D::WindowData*)t;
  if (event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN) {
    toggleFreezeScreenKeyboardCallback(event.code, *window_data);
    setMeshRepresentation(event.code, *window_data);
    setMeshShadingCallback(event.code, *window_data);
    setMeshAmbientCallback(event.code, *window_data);
    setMeshLightingCallback(event.code, *window_data);
    getViewerPoseKeyboardCallback(event.code, *window_data);
    getCurrentWindowSizeKeyboardCallback(event.code, *window_data);
    getScreenshotCallback(event.code, *window_data);
  }
}

/* -------------------------------------------------------------------------- */
void Visualizer3D::recordVideo() {
  static int i = 0u;
  static const std::string dir_path = ".";
  static const std::string dir_name = "3d_viz_video";
  static const std::string dir_full_path =
      common::pathAppend(dir_path, dir_name);
  if (i == 0u) CHECK(common::createDirectory(dir_path, dir_name));
  std::string screenshot_path =
      common::pathAppend(dir_full_path, std::to_string(i));
  i++;
  LOG(WARNING) << "Recording video sequence for 3d Viz, "
               << "current frame saved in: " + screenshot_path;
  window_data_.window_.saveScreenshot(screenshot_path);
  LOG(ERROR) << "WTF";
}

}  // namespace VIO
