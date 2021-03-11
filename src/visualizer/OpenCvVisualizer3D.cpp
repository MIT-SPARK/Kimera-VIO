/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OpenCvVisualizer3D.cpp
 * @brief  Build and visualize 3D data: 2D mesh from frame for example.
 * @author Antoni Rosinol
 */

#include "kimera-vio/visualizer/OpenCvVisualizer3D.h"

#include <algorithm>      // for min
#include <memory>         // for shared_ptr<>
#include <string>         // for string
#include <unordered_map>  // for unordered_map<>
#include <utility>        // for pair<>
#include <vector>         // for vector<>

#include <gflags/gflags.h>

#include <opencv2/viz.hpp>
// To convert from/to eigen
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include <opencv2/core/eigen.hpp>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/factors/PointPlaneFactor.h"  // For visualization of constraints.
#include "kimera-vio/frontend/MonoVisionImuFrontend-definitions.h"
#include "kimera-vio/utils/FilesystemUtils.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsGTSAM.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

// TODO(Toni): remove visualizer gflags! There are far too many, use a
// yaml params class (aka inherit from PipelineParams.
DEFINE_bool(visualize_mesh, false, "Enable 3D mesh visualization.");

DEFINE_bool(visualize_mesh_2d, false, "Visualize mesh 2D.");

DEFINE_bool(visualize_semantic_mesh,
            false,
            "Color the 3d mesh according to their semantic labels.");
DEFINE_bool(visualize_mesh_with_colored_polygon_clusters,
            false,
            "Color the polygon clusters according to their cluster id.");
DEFINE_bool(visualize_point_cloud, true, "Enable point cloud visualization.");
DEFINE_bool(visualize_convex_hull, false, "Enable convex hull visualization.");
DEFINE_bool(visualize_plane_constraints,
            false,
            "Enable plane constraints"
            " visualization.");
DEFINE_bool(visualize_planes, false, "Enable plane visualization.");
DEFINE_bool(visualize_plane_label, false, "Enable plane label visualization.");
DEFINE_bool(visualize_mesh_in_frustum,
            false,
            "Enable mesh visualization in "
            "camera frustum.");
DEFINE_string(
    visualize_load_mesh_filename,
    "",
    "Load a mesh in the visualization, i.e. to visualize ground-truth "
    "point cloud from Euroc's Vicon dataset.");

// 3D Mesh related flags.
DEFINE_bool(texturize_3d_mesh,
            false,
            "Whether you want to add texture to the 3d"
            "mesh. The texture is taken from the image"
            " frame.");
DEFINE_bool(log_mesh, false, "Log the mesh at time horizon.");
DEFINE_bool(log_accumulated_mesh, false, "Accumulate the mesh when logging.");

DEFINE_int32(displayed_trajectory_length,
             50,
             "Set length of plotted trajectory."
             "If -1 then all the trajectory is plotted.");

namespace VIO {

OpenCvVisualizer3D::OpenCvVisualizer3D(const VisualizationType& viz_type,
                                       const BackendType& backend_type)
    : Visualizer3D(viz_type), backend_type_(backend_type), logger_(nullptr) {
  if (FLAGS_log_mesh) {
    logger_ = VIO::make_unique<VisualizerLogger>();
  }
}

OpenCvVisualizer3D::~OpenCvVisualizer3D() {
  LOG(INFO) << "OpenCvVisualizer3D destructor called.";
}

VisualizerOutput::UniquePtr OpenCvVisualizer3D::spinOnce(
    const VisualizerInput& input) {
  CHECK(input.frontend_output_);
  CHECK(input.backend_output_);

  VisualizerOutput::UniquePtr output = VIO::make_unique<VisualizerOutput>();

  // Ensure we have mesher output if the user requested mesh visualization
  // otherwise, switch to pointcloud visualization.
  if (visualization_type_ == VisualizationType::kMesh2dTo3dSparse &&
      !input.mesher_output_) {
    LOG(ERROR) << "Mesh visualization requested, but no "
                  "mesher output available. Switching to Pointcloud"
                  "visualization only.";
    visualization_type_ = VisualizationType::kPointcloud;
  }
  output->visualization_type_ = visualization_type_;

  cv::Mat mesh_2d_img;  // Only for visualization.
  const Frame& left_stereo_keyframe =
      input.frontend_output_->frontend_type_ == FrontendType::kStereoImu
          ? VIO::safeCast<FrontendOutputPacketBase, StereoFrontendOutput>(
                input.frontend_output_)->stereo_frame_lkf_.left_frame_
          : VIO::safeCast<FrontendOutputPacketBase, MonoFrontendOutput> (
                input.frontend_output_)->frame_lkf_;
  switch (visualization_type_) {
    // Computes and visualizes 3D mesh from 2D triangulation.
    // vertices: all leftframe kps with right-VALID (3D), lmkId != -1 and
    // inside the image triangles: all the ones with edges inside images as
    // produced by cv::subdiv, which have uniform gradient (updateMesh3D also
    // filters out geometrically) Sparsity comes from filtering out triangles
    // corresponding to non planar obstacles which are assumed to have
    // non-uniform gradient.
    case VisualizationType::kMesh2dTo3dSparse: {
      CHECK(input.mesher_output_);
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
                          &output->widgets_,
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
              &output->widgets_,
              FLAGS_visualize_mesh_with_colored_polygon_clusters,
              input.timestamp_);
        }
      }

      if (FLAGS_visualize_point_cloud) {
        visualizePoints3D(points_with_id_VIO_prev,
                          lmk_id_to_lmk_type_map_prev,
                          &output->widgets_);
      }

      if (!FLAGS_visualize_load_mesh_filename.empty()) {
        // TODO(Toni): remove static, never use static wo const/constexpr
        static bool visualize_ply_mesh_once = true;
        if (visualize_ply_mesh_once) {
          visualizePlyMesh(FLAGS_visualize_load_mesh_filename.c_str(),
                           &output->widgets_);
          visualize_ply_mesh_once = false;
        }
      }

      if (FLAGS_visualize_convex_hull) {
        if (planes_prev.size() != 0) {
          visualizeConvexHull(planes_prev.at(0).triangle_cluster_,
                              vertices_mesh_prev,
                              polygons_mesh_prev,
                              &output->widgets_);
        }
      }

      if (backend_type_ == BackendType::kStructuralRegularities &&
          FLAGS_visualize_plane_constraints) {
        LandmarkIds lmk_ids_in_current_pp_factors;
        for (const auto& g : input.backend_output_->factor_graph_) {
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
                // WARNING assumes the Backend updates normal and distance
                // of plane and that no one modifies it afterwards...
                visualizePlaneConstraints(
                    plane.getPlaneSymbol().key(),
                    current_plane_estimate.normal().point3(),
                    current_plane_estimate.distance(),
                    lmk_id,
                    point,
                    &output->widgets_);
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
            visualizePlane(plane_index,
                           plane_normal_estimate.x,
                           plane_normal_estimate.y,
                           plane_normal_estimate.z,
                           current_plane_estimate.distance(),
                           &output->widgets_,
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

        // Also remove planes that were deleted by the Backend...
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
              : (FLAGS_texturize_3d_mesh ? OpenCvVisualizer3D::texturizeMesh3D(
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
      // TODO(Toni): don't use the backend's maps, instead, build these maps,
      // in the visualizer using the state and the factor graph.
      visualizePoints3D(input.backend_output_->landmarks_with_id_map_,
                        input.backend_output_->lmk_id_to_lmk_type_map_,
                        &output->widgets_);
      break;
    }
    case VisualizationType::kNone: {
      break;
    }
  }

  // Visualize trajectory.
  // First, add current pose to trajectory
  VLOG(10) << "Starting trajectory visualization...";
  const gtsam::Pose3& b_Pose_cam_Lrect =
      input.frontend_output_->frontend_type_ == FrontendType::kStereoImu
          ? VIO::safeCast<FrontendOutputPacketBase, StereoFrontendOutput>(
                input.frontend_output_)
                ->b_Pose_camL_rect_
          : VIO::safeCast<FrontendOutputPacketBase, MonoFrontendOutput>(
                input.frontend_output_)
                ->b_Pose_cam_rect_;
  addPoseToTrajectory(UtilsOpenCV::gtsamPose3ToCvAffine3d(
      input.backend_output_->W_State_Blkf_.pose_.compose(b_Pose_cam_Lrect)));
  // Generate line through all poses
  visualizeTrajectory3D(&output->widgets_);
  // Generate frustums for the last 10 poses.
  // visualizeTrajectoryWithFrustums(&output->widgets_, 10u);
  // Generate frustum with an image inside it for the current pose.
  visualizePoseWithImgInFrustum(
      FLAGS_visualize_mesh_in_frustum ? mesh_2d_img
      : input.frontend_output_->frontend_type_ == FrontendType::kStereoImu
          ? VIO::safeCast<FrontendOutputPacketBase, StereoFrontendOutput>(
                input.frontend_output_)
                ->feature_tracks_
          : VIO::safeCast<FrontendOutputPacketBase, MonoFrontendOutput>(
                input.frontend_output_)
                ->feature_tracks_,
      trajectory_poses_3d_.back(),
      &output->widgets_);
  VLOG(10) << "Finished trajectory visualization.";
  // Visualize the factor-graph in 3D, this trajectory might be different
  // than the one above!
  visualizeFactorGraph(
      input.backend_output_->state_,
      input.backend_output_->factor_graph_,
      b_Pose_cam_Lrect,
      input.frontend_output_->frontend_type_ == FrontendType::kStereoImu
          ? VIO::safeCast<FrontendOutputPacketBase, StereoFrontendOutput>(
                input.frontend_output_)
                ->b_Pose_camR_rect_
          : VIO::safeCast<FrontendOutputPacketBase, MonoFrontendOutput>(
                input.frontend_output_)
                ->b_Pose_cam_rect_,  // TODO(marcus): should be null
      &output->widgets_);

  // Add widgets to remove
  output->widget_ids_to_remove_ = widget_ids_to_remove_;
  widget_ids_to_remove_.clear();

  return output;
}

void OpenCvVisualizer3D::visualizeFactorGraph(
    const gtsam::Values& state,
    const gtsam::NonlinearFactorGraph& factor_graph,
    const gtsam::Pose3& body_pose_camLrect,
    const gtsam::Pose3& body_pose_camRrect,
    WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);
  // Assert consistency between state and factor_graph

  // First, recolor all previous poses as inactive (white color) by re-drawing
  // them, the active ones will be colored later.
  for (const std::pair<std::string, cv::Affine3d>& widget_id_pose_pair :
       widget_id_to_pose_map_) {
    const auto& widget_id = widget_id_pose_pair.first;
    (*widgets_map)[widget_id] = VIO::make_unique<cv::viz::WCameraPosition>(
        K_, inactive_frustum_scale_, cv::viz::Color::white());
    (*widgets_map)[widget_id]->setPose(widget_id_pose_pair.second);
  }
  widget_id_to_pose_map_.clear();

  for (const std::string& line_id : widget_ids_to_remove_in_next_iter_) {
    // Remove lmk to pose lines
    removeWidget(line_id);
  }
  widget_ids_to_remove_in_next_iter_.clear();

  // Step 1: visualize variables
  // Loop over the state
  for (const gtsam::Values::ConstKeyValuePair& key_value : state) {
    // Regular Landmarks
    const auto& variable_symbol = gtsam::Symbol(key_value.key);
    const auto& variable_type = variable_symbol.chr();
    const auto& variable_index = variable_symbol.index();
    const auto& variable_value = key_value.value;

    static constexpr bool draw_imu_pose = true;
    static constexpr bool draw_left_cam = true;
    static constexpr bool draw_right_cam = false;
    static constexpr bool draw_imu_to_left_cam_arrow = true;
    static constexpr bool draw_velocity = true;
    if (variable_type == kLandmarkSymbolChar) {
      // const LandmarkId& lmk_id = variable_index;  // THIS IS NOT LMK_ID!
      // const gtsam::Point3& lmk_position =
      // variable_value.cast<gtsam::Point3>();
      // DCHECK(points_with_id.find(lmk_id) == points_with_id.end());
      // points_with_id[lmk_id] = lmk_position;
      // lmk_id_to_lmk_type_map[lmk_id] = LandmarkType::PROJECTION;
      continue;
    }

    // Smart landmarks
    // We cannot use the values to get these, since they aren't a variable :(

    // Poses
    if (variable_type == kPoseSymbolChar) {
      const gtsam::Pose3& imu_pose = variable_value.cast<gtsam::Pose3>();
      if (draw_imu_pose) {
        drawImuPose(imu_pose, variable_index, widgets_map);
      }

      // Visualize Camera pose as a frustum
      // Left Cam
      const gtsam::Pose3& world_pose_camLrect =
          imu_pose.compose(body_pose_camLrect);
      if (draw_left_cam) {
        drawLeftCam(world_pose_camLrect, variable_index, widgets_map);
      }

      // Right Cam
      if (draw_right_cam) {
        const gtsam::Pose3& world_pose_camRrect =
            imu_pose.compose(body_pose_camRrect);
        drawRightCam(world_pose_camRrect, variable_index, widgets_map);
      }

      if (draw_imu_to_left_cam_arrow) {
        drawImuToLeftCamArrow(
            imu_pose, world_pose_camLrect, variable_index, widgets_map);
      }
      continue;
    }

    // Velocities
    if (variable_type == kVelocitySymbolChar && draw_velocity) {
      const gtsam::Vector3& imu_velocity =
          variable_value.cast<gtsam::Vector3>();
      drawVelocityArrow(imu_velocity, state, variable_index, widgets_map);
      continue;
    }

    // Imu Biases
    if (variable_type == kImuBiasSymbolChar) {
      // Arrows? might clutter quite a bit...
      continue;
    }
  }

  // Step 2: visualize Factors
  static constexpr bool draw_smart_stereo_factors = false;
  //! typically zero velocity prior
  static constexpr bool draw_vector3_priors = true;
  static constexpr bool draw_pose_priors = true;
  static constexpr bool draw_plane_priors = false;
  static constexpr bool draw_point_plane_factors = false;
  static constexpr bool draw_linear_container_factors = true;
  static constexpr bool draw_preintegrated_imu_factors = true;
  static constexpr bool draw_imu_constant_bias_factors = true;
  static constexpr bool draw_between_factors = true;
  for (const boost::shared_ptr<gtsam::NonlinearFactor>& factor : factor_graph) {
    if (draw_smart_stereo_factors) {
      const SmartStereoFactor::shared_ptr& smart_stereo_factor =
          boost::dynamic_pointer_cast<SmartStereoFactor>(factor);
      if (smart_stereo_factor) {
        drawSmartStereoFactor(
            *smart_stereo_factor, state, body_pose_camLrect, widgets_map);
        continue;
      }
    }

    if (draw_point_plane_factors) {
      const auto& ppf =
          boost::dynamic_pointer_cast<gtsam::PointPlaneFactor>(factor);
      if (ppf) {
        continue;
      }
    }

    if (draw_plane_priors) {
      const auto& ppp = boost::dynamic_pointer_cast<
          gtsam::PriorFactor<gtsam::OrientedPlane3>>(factor);
      if (ppp) {
        continue;
      }
    }

    if (draw_linear_container_factors) {
      const auto& lcf =
          boost::dynamic_pointer_cast<gtsam::LinearContainerFactor>(factor);
      if (lcf) {
        drawLinearContainerFactor(*lcf, state, body_pose_camLrect, widgets_map);
        continue;
      }
    }

    if (draw_vector3_priors) {
      // This is the same as a gtsam::Point3 prior!
      const auto& velocity_prior =
          boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Vector3>>(
              factor);
      if (velocity_prior) {
        drawVelocityPrior(*velocity_prior, state, widgets_map);
        continue;
      }
    }

    if (draw_pose_priors) {
      const auto& pose_prior =
          boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor);
      if (pose_prior) {
        drawPosePrior(*pose_prior, state, body_pose_camLrect, widgets_map);
        continue;
      }
    }

    if (draw_between_factors) {
      const auto& btw_factor =
          boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
              factor);
      if (btw_factor) {
        drawBtwFactor(*btw_factor, state, body_pose_camLrect, widgets_map);
        continue;
      }
    }

    if (draw_preintegrated_imu_factors) {
      const auto& imu_factor =
          boost::dynamic_pointer_cast<gtsam::ImuFactor>(factor);
      if (imu_factor) {
        drawImuFactor(*imu_factor, state, body_pose_camLrect, widgets_map);
        continue;
      }
    }

    if (draw_imu_constant_bias_factors) {
      const auto& imu_constant_bias_prior = boost::dynamic_pointer_cast<
          gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(factor);
      if (imu_constant_bias_prior) {
        continue;
      }
    }
  }
}

void OpenCvVisualizer3D::drawImuPose(const gtsam::Pose3& imu_pose,
                                     const gtsam::Key& variable_index,
                                     WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);
  // Visualize IMU pose as a coordinate frame
  (*widgets_map)["IMU pose " + std::to_string(variable_index)] =
      VIO::make_unique<cv::viz::WCoordinateSystem>(0.1);
  (*widgets_map)["IMU pose " + std::to_string(variable_index)]->setPose(
      UtilsOpenCV::gtsamPose3ToCvAffine3d(imu_pose));
}

void OpenCvVisualizer3D::drawLeftCam(const gtsam::Pose3& world_pose_camLrect,
                                     const gtsam::Key& variable_index,
                                     WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);
  std::string left_cam_id = "Left CAM pose " + std::to_string(variable_index);
  (*widgets_map)[left_cam_id] = VIO::make_unique<cv::viz::WCameraPosition>(
      K_, left_cam_active_frustum_scale_, left_cam_active_frustum_color_);
  const cv::Affine3d& left_cam_pose =
      UtilsOpenCV::gtsamPose3ToCvAffine3d(world_pose_camLrect);
  (*widgets_map)[left_cam_id]->setPose(left_cam_pose);
  widget_id_to_pose_map_[left_cam_id] = left_cam_pose;
}

void OpenCvVisualizer3D::drawRightCam(const gtsam::Pose3& world_pose_camRrect,
                                      const gtsam::Key& variable_index,
                                      WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);
  std::string right_cam_id = "Right CAM pose " + std::to_string(variable_index);
  (*widgets_map)[right_cam_id] = VIO::make_unique<cv::viz::WCameraPosition>(
      K_, right_cam_active_frustum_scale_, right_cam_active_frustum_color_);
  const cv::Affine3d& right_cam_pose =
      UtilsOpenCV::gtsamPose3ToCvAffine3d(world_pose_camRrect);
  (*widgets_map)[right_cam_id]->setPose(
      UtilsOpenCV::gtsamPose3ToCvAffine3d(world_pose_camRrect));
  widget_id_to_pose_map_[right_cam_id] = right_cam_pose;
}

void OpenCvVisualizer3D::drawImuToLeftCamArrow(
    const gtsam::Pose3& imu_pose,
    const gtsam::Pose3& world_pose_camLrect,
    const gtsam::Key& variable_index,
    WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);
  // Visualize arrow from IMU pose to Camera Pose
  const auto& imu_position = imu_pose.translation();
  const auto& left_cam_position = world_pose_camLrect.translation();
  cv::Point3d arrow_start(imu_position.x(), imu_position.y(), imu_position.z());
  cv::Point3d arrow_end(
      left_cam_position.x(), left_cam_position.y(), left_cam_position.z());
  std::string imu_to_left_cam_id =
      "IMU to Left CAM " + std::to_string(variable_index);
  (*widgets_map)[imu_to_left_cam_id] =
      VIO::make_unique<cv::viz::WArrow>(arrow_start,
                                        arrow_end,
                                        imu_to_left_cam_vector_scale_,
                                        imu_to_left_cam_vector_color_);
}

void OpenCvVisualizer3D::drawVelocityArrow(const gtsam::Vector3& imu_velocity,
                                           const gtsam::Values& state,
                                           const gtsam::Key& variable_index,
                                           WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);
  gtsam::Pose3 imu_pose;
  CHECK(getEstimateOfKey(
      state, gtsam::Symbol(kPoseSymbolChar, variable_index), &imu_pose));
  const gtsam::Point3& imu_position = imu_pose.translation();
  gtsam::Point3 end = imu_position + imu_velocity;

  cv::Point3d arrow_start(imu_position.x(), imu_position.y(), imu_position.z());
  cv::Point3d arrow_end(end.x(), end.y(), end.z());

  // Display the velocity as an arrow centered at the IMU widget.
  (*widgets_map)["IMU vel " + std::to_string(variable_index)] =
      VIO::make_unique<cv::viz::WArrow>(
          arrow_start, arrow_end, 0.001, velocity_vector_color_);
}

void OpenCvVisualizer3D::drawSmartStereoFactor(
    const SmartStereoFactor& smart_stereo_factor,
    const gtsam::Values& state,
    const gtsam::Pose3& body_pose_camLrect,
    WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);
  // Get triangulation result from smart factor.
  const gtsam::TriangulationResult& result = smart_stereo_factor.point();
  CHECK(smart_stereo_factor.body_P_sensor().equals(body_pose_camLrect));
  // Check that the boost::optional result is initialized.
  if (result.valid()) {
    // Get lmk
    CHECK(result.is_initialized());
    const gtsam::Point3& lmk = *result;
    // For each left cam in stereo cam, draw line from lmk to optical
    // center.
    const gtsam::CameraSet<gtsam::StereoCamera>& stereo_cams =
        smart_stereo_factor.cameras(state);
    const gtsam::KeyVector& stereo_cam_keys = smart_stereo_factor.keys();
    CHECK_EQ(stereo_cams.size(), stereo_cam_keys.size());
    static LandmarkId lmk_id = 0;
    // Since I don't know the smart factor lmk id, I just invent it...
    lmk_id++;
    for (size_t i = 0u; i < stereo_cams.size(); i++) {
      // Get Camera pose
      const gtsam::StereoCamera& stereo_cam = stereo_cams.at(i);
      const gtsam::Point3& left_cam_pose = stereo_cam.pose().translation();

      std::string cam_to_lmk_line_id = "CAM " +
                                       std::to_string(stereo_cam_keys.at(i)) +
                                       " Lmk " + std::to_string(lmk_id);
      cv::Point3d arrow_start(
          left_cam_pose.x(), left_cam_pose.y(), left_cam_pose.z());
      cv::Point3d arrow_end(lmk.x(), lmk.y(), lmk.z());
      cv::Mat in(1, 1, CV_8UC3);
      in.at<cv::Vec3b>(0, 0) = cv::Vec3b::all(255.0 / std::exp(i * 0.5));
      cv::Mat out(1, 1, CV_8UC3);
      cv::applyColorMap(in, out, cv::COLORMAP_PARULA);
      cv::viz::Color arrow_color(out.at<cv::Vec3b>(0, 0));
      (*widgets_map)[cam_to_lmk_line_id] = VIO::make_unique<cv::viz::WArrow>(
          arrow_start, arrow_end, 0.0005, arrow_color);
      widget_ids_to_remove_in_next_iter_.push_back(cam_to_lmk_line_id);
    }
    // 1. Plot Landmark
    // Check that we have not added this lmk already...
    // We don't know the ids of this lmk unfortunately (only backend
    // knows)
    // CHECK(points_with_id.find(lmk_id) == points_with_id.end());
    // points_with_id[lmk_id] = *result;
    // lmk_id_to_lmk_type_map[lmk_id] = LandmarkType::SMART;

    // 2. Plot factors from Landmark to Poses
    // TODO(Toni): Color-code lmk with its age by using alpha of edges!!
    // smart_stereo_factor.measured().size()
  } else if (result.behindCamera()) {
    CHECK(!result.is_initialized());
    // TODO(TONI): show the measurement rays in RED
  } else if (result.degenerate()) {
    CHECK(!result.is_initialized());
    // TODO(TONI): show the measurement rays in RED
  } else if (result.farPoint()) {
    CHECK(!result.is_initialized());
    // TODO(TONI): show the measurement rays in RED
  } else if (result.outlier()) {
    CHECK(!result.is_initialized());
    // TODO(TONI): show the measurement rays in RED
  } else {
    LOG(FATAL) << "Did someone add a new state to triangulation result?";
  }
}

void OpenCvVisualizer3D::drawLinearContainerFactor(
    const gtsam::LinearContainerFactor& lcf,
    const gtsam::Values& state,
    const gtsam::Pose3& body_pose_camLrect,
    WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);
  for (const gtsam::Key& key : lcf.keys()) {
    // Color-code all poses that are under the LinearContainerFactor
    gtsam::Symbol symbol(key);
    if (symbol.chr() == kPoseSymbolChar) {
      gtsam::Pose3 imu_pose;
      CHECK(getEstimateOfKey(state, key, &imu_pose));
      std::string left_cam_id =
          "Left CAM pose " + std::to_string(symbol.index());
      (*widgets_map)[left_cam_id] = VIO::make_unique<cv::viz::WCameraPosition>(
          K_,
          cam_with_linear_prior_frustum_scale_,
          cam_with_linear_prior_frustum_color_);
      const gtsam::Pose3& world_pose_camLrect =
          imu_pose.compose(body_pose_camLrect);
      const cv::Affine3d& left_cam_with_prior_pose =
          UtilsOpenCV::gtsamPose3ToCvAffine3d(world_pose_camLrect);
      (*widgets_map)[left_cam_id]->setPose(
          UtilsOpenCV::gtsamPose3ToCvAffine3d(world_pose_camLrect));
      widget_id_to_pose_map_[left_cam_id] = left_cam_with_prior_pose;
      // PERHAPS COLOR AGAIN THE LMK TO POSE RAYS IN RED, as in
      // the factor that connects all together!
    } else if (symbol.chr() == kLandmarkSymbolChar) {
      // TODO(Toni): color landmark with prior as such.
      LOG(WARNING) << "Detected Linear Factor on Landmark, but display is"
                      " not implemented...";
    }
  }
}

void OpenCvVisualizer3D::drawVelocityPrior(
    const gtsam::PriorFactor<gtsam::Vector3>& velocity_prior,
    const gtsam::Values& state,
    WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);
  CHECK_EQ(velocity_prior.keys().size(), 1u);
  gtsam::Key velocity_key = velocity_prior.key();
  gtsam::Symbol velocity_symbol(velocity_key);
  if (velocity_symbol.chr() == kVelocitySymbolChar) {
    gtsam::Pose3 imu_pose;
    CHECK(getEstimateOfKey(
        state,
        gtsam::Symbol(kPoseSymbolChar, velocity_symbol.index()),
        &imu_pose));

    // Print text saying that we got a velocity prior.
    const cv::Point3d text_position(imu_pose.x(), imu_pose.y(), imu_pose.z());
    std::string info =
        gtsam::equal(velocity_prior.prior(), gtsam::Vector3::Zero())
            ? "Zero Velocity Prior"
            : "Generic Velocity Prior";
    std::string velocity_prior_text_id =
        info + ", id: " + std::to_string(velocity_symbol.index());
    // By keeping id the same, we overwrite the text, otw too much clutter
    (*widgets_map)["Velocity Prior"] =
        VIO::make_unique<cv::viz::WText3D>(velocity_prior_text_id,
                                           text_position,
                                           0.04,
                                           false,
                                           velocity_prior_color_);

    // Print a red cube around the IMU pose with the velocity prior.
    static constexpr double half_cube_side = 0.02;
    cv::Point3d min_point(imu_pose.x() - half_cube_side,
                          imu_pose.y() - half_cube_side,
                          imu_pose.z() - half_cube_side);
    cv::Point3d max_point(imu_pose.x() + half_cube_side,
                          imu_pose.y() + half_cube_side,
                          imu_pose.z() + half_cube_side);
    std::string velocity_prior_cube_id =
        "Point prior " + std::to_string(velocity_symbol.index());
    (*widgets_map)[velocity_prior_cube_id] = VIO::make_unique<cv::viz::WCube>(
        min_point, max_point, true, velocity_prior_color_);

    // Potentially remove this info on each iteration.
    // widget_ids_to_remove_in_next_iter_.push_back(velocity_prior_id);
  } else {
    LOG(WARNING) << "Prior on gtsam::Vector3 or gtsam::Point3 unrecognized...";
  }
}

void OpenCvVisualizer3D::drawPosePrior(
    const gtsam::PriorFactor<gtsam::Pose3>& pose_prior,
    const gtsam::Values& state,
    const gtsam::Pose3& body_pose_camLrect,
    WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);
  CHECK_EQ(pose_prior.keys().size(), 1u);
  gtsam::Key pose_key = pose_prior.key();
  gtsam::Symbol pose_symbol(pose_key);
  CHECK_EQ(pose_symbol.chr(), kPoseSymbolChar);
  gtsam::Pose3 imu_pose;
  CHECK(getEstimateOfKey(state, pose_key, &imu_pose));
  std::string left_cam_id =
      "Left CAM pose prior " + std::to_string(pose_symbol.index());
  (*widgets_map)[left_cam_id] = VIO::make_unique<cv::viz::WCameraPosition>(
      K_,
      cam_with_pose_prior_frustum_scale_,
      cam_with_pose_prior_frustum_color_);
  const gtsam::Pose3& world_pose_camLrect =
      imu_pose.compose(body_pose_camLrect);
  (*widgets_map)[left_cam_id]->setPose(
      UtilsOpenCV::gtsamPose3ToCvAffine3d(world_pose_camLrect));
  // Let's keep the prior in the visualization window for now
  // const cv::Affine3d& left_cam_with_prior_pose =
  //     UtilsOpenCV::gtsamPose3ToCvAffine3d(world_pose_camLrect);
  // widget_id_to_pose_map_[left_cam_id] = left_cam_with_prior_pose;
}

void OpenCvVisualizer3D::drawBtwFactor(
    const gtsam::BetweenFactor<gtsam::Pose3>& btw_factor,
    const gtsam::Values& state,
    const gtsam::Pose3& body_pose_camLrect,
    WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);
  CHECK_EQ(btw_factor.keys().size(), 2u);
  const gtsam::Key& pose_key_1 = btw_factor.keys().at(0);
  const gtsam::Key& pose_key_2 = btw_factor.keys().at(1);
  gtsam::Symbol pose_symbol_1(pose_key_1);
  gtsam::Symbol pose_symbol_2(pose_key_2);
  CHECK_EQ(pose_symbol_1.chr(), kPoseSymbolChar);
  CHECK_EQ(pose_symbol_2.chr(), kPoseSymbolChar);

  gtsam::Pose3 imu_pose_1;
  CHECK(getEstimateOfKey(state, pose_key_1, &imu_pose_1));
  gtsam::Pose3 pose_2;
  CHECK(getEstimateOfKey(state, pose_key_2, &pose_2));

  cv::Point3d start_point(imu_pose_1.x(), imu_pose_1.y(), imu_pose_1.z());
  cv::Point3d end_point(pose_2.x(), pose_2.y(), pose_2.z());
  cv::Point3d mid_point((pose_2.x() + imu_pose_1.x()) / 2.0,
                        (pose_2.y() + imu_pose_1.y()) / 2.0,
                        (pose_2.z() + imu_pose_1.z()) / 2.0);

  static constexpr double kCylinderRadius = 0.005;
  static constexpr double kSphereRadius = 0.02;
  // Do we have a no-motion prior?
  if (btw_factor.measured().equals(gtsam::Pose3::identity())) {
    // Connect involved poses with a cylinder
    std::string no_motion_prior_id =
        "No Motion prior: " + std::to_string(pose_symbol_1.index());
    (*widgets_map)[no_motion_prior_id + " (edge)"] =
        VIO::make_unique<cv::viz::WCylinder>(start_point,
                                             end_point,
                                             kCylinderRadius,
                                             20,
                                             no_motion_prior_color_);

    // Add a sphere
    (*widgets_map)[no_motion_prior_id + " (factor)"] =
        VIO::make_unique<cv::viz::WSphere>(
            mid_point, kSphereRadius, 10, no_motion_prior_color_);

    // Add text, since the cylinder is likely difficult to visualize
    std::string no_motion_prior_text_id =
        "No Motion Prior, id: " + std::to_string(pose_symbol_1.index());
    // By keeping id the same, we overwrite the text, otw too much clutter
    mid_point.z += 0.3;  //! move text upwards, otw clutters vel prior text
    (*widgets_map)["No Motion Prior Text"] =
        VIO::make_unique<cv::viz::WText3D>(no_motion_prior_text_id,
                                           mid_point,
                                           0.04,
                                           false,
                                           no_motion_prior_color_);
  } else {
    // We have a regular btw factor
    std::string btw_factor_id = "Btw Factor: from " +
                                std::to_string(pose_symbol_1.index()) + " to " +
                                std::to_string(pose_symbol_2.index());

    // Add Edge
    std::string btw_factor_edge_id = btw_factor_id + " (edge)";
    (*widgets_map)[btw_factor_edge_id] = VIO::make_unique<cv::viz::WCylinder>(
        start_point, end_point, kCylinderRadius, 20, btw_factor_color_);
    widget_ids_to_remove_in_next_iter_.push_back(btw_factor_edge_id);

    // Add Sphere
    std::string btw_factor_sphere_id = btw_factor_id + " (factor)";
    (*widgets_map)[btw_factor_sphere_id] = VIO::make_unique<cv::viz::WSphere>(
        mid_point, kSphereRadius, 10, btw_factor_color_);
    widget_ids_to_remove_in_next_iter_.push_back(btw_factor_sphere_id);

    // TODO(Toni): try to color edge according to level of error
    // btw_factor.error(state);

    // Visualize initial guess
    std::string btw_factor_pose_guess_id =
        "Btw factor pose guess" + std::to_string(pose_symbol_2.index());
    (*widgets_map)[btw_factor_pose_guess_id] =
        VIO::make_unique<cv::viz::WCameraPosition>(
            K_,
            btw_factor_pose_guess_active_frustum_scale_,
            btw_factor_pose_guess_active_frustum_color_);
    const gtsam::Pose3& btw_factor_meas_world_pose_body =
        imu_pose_1.compose(btw_factor.measured());
    const cv::Affine3d& left_cam_pose = UtilsOpenCV::gtsamPose3ToCvAffine3d(
        btw_factor_meas_world_pose_body.compose(body_pose_camLrect));
    (*widgets_map)[btw_factor_pose_guess_id]->setPose(left_cam_pose);
    widget_ids_to_remove_in_next_iter_.push_back(btw_factor_pose_guess_id);
    // widget_id_to_pose_map_[btw_factor_imu_pose_guess_id] =
    // left_cam_pose;

    // Draw arrow to associate factor with initial guess
    std::string btw_factor_arrow_id = btw_factor_pose_guess_id + " (arrow)";
    (*widgets_map)[btw_factor_arrow_id] = VIO::make_unique<cv::viz::WArrow>(
        mid_point,
        cv::Point3d(left_cam_pose.translation()),
        btw_factor_to_guess_pose_vector_scale_,
        btw_factor_to_guess_pose_vector_color_);
    widget_ids_to_remove_in_next_iter_.push_back(btw_factor_arrow_id);
  }
}

void OpenCvVisualizer3D::drawImuFactor(const gtsam::ImuFactor& imu_factor,
                                       const gtsam::Values& state,
                                       const gtsam::Pose3& body_pose_camLrect,
                                       WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);
  const gtsam::Symbol& pose_symbol_1(imu_factor.key1());
  const gtsam::Symbol& vel_symbol_1(imu_factor.key2());
  const gtsam::Symbol& pose_symbol_2(imu_factor.key3());
  const gtsam::Symbol& vel_symbol_2(imu_factor.key4());
  const gtsam::Symbol& imu_bias_symbol_1(imu_factor.key5());
  CHECK_EQ(pose_symbol_1.chr(), kPoseSymbolChar);
  CHECK_EQ(vel_symbol_1.chr(), kVelocitySymbolChar);
  CHECK_EQ(pose_symbol_2.chr(), kPoseSymbolChar);
  CHECK_EQ(vel_symbol_2.chr(), kVelocitySymbolChar);
  CHECK_EQ(imu_bias_symbol_1.chr(), kImuBiasSymbolChar);
  gtsam::Pose3 pose_1;
  gtsam::Vector3 vel_1;
  gtsam::Pose3 pose_2;
  gtsam::Vector3 vel_2;
  ImuBias imu_bias_1;
  CHECK(getEstimateOfKey(state, pose_symbol_1.key(), &pose_1));
  CHECK(getEstimateOfKey(state, vel_symbol_1.key(), &vel_1));
  CHECK(getEstimateOfKey(state, pose_symbol_2.key(), &pose_2));
  CHECK(getEstimateOfKey(state, vel_symbol_2.key(), &vel_2));
  CHECK(getEstimateOfKey(state, imu_bias_symbol_1.key(), &imu_bias_1));
  gtsam::NavState navstate_1(pose_1, vel_1);
  const gtsam::NavState& navstate_2 =
      imu_factor.preintegratedMeasurements().predict(navstate_1, imu_bias_1);
  const gtsam::Pose3& meas_pose_2 = navstate_2.pose();
  const gtsam::Vector3& meas_vel_2 = navstate_2.velocity();
  // Draw estimated IMU pose as a frustum for the left cam to be able to
  // compare with the estimate from stereo RANSAC
  std::string imu_factor_pose_guess_id =
      "IMU factor pose guess" + std::to_string(pose_symbol_2.index());
  (*widgets_map)[imu_factor_pose_guess_id] =
      VIO::make_unique<cv::viz::WCameraPosition>(
          K_, imu_factor_to_guess_pose_scale_, imu_factor_to_guess_pose_color_);
  const cv::Affine3d& left_cam_pose_guess = UtilsOpenCV::gtsamPose3ToCvAffine3d(
      meas_pose_2.compose(body_pose_camLrect));
  (*widgets_map)[imu_factor_pose_guess_id]->setPose(left_cam_pose_guess);
  widget_ids_to_remove_in_next_iter_.push_back(imu_factor_pose_guess_id);

  // Draw estimated IMU velocity as an arrow
  gtsam::Vector3 end = pose_2.translation() + meas_vel_2;
  cv::Point3d arrow_start(pose_2.x(), pose_2.y(), pose_2.z());
  cv::Point3d arrow_end(end.x(), end.y(), end.z());
  (*widgets_map)["IMU vel guess " + std::to_string(vel_symbol_2.index())] =
      VIO::make_unique<cv::viz::WArrow>(
          arrow_start, arrow_end, 0.001, imu_factor_guess_velocity_color_);
}

cv::Mat OpenCvVisualizer3D::visualizeMesh2D(
    const std::vector<cv::Vec6f>& triangulation2D,
    const cv::Mat& img,
    const KeypointsCV& extra_keypoints) {
  static const cv::Scalar kDelaunayColor(0u, 255u, 0u);
  static const cv::Scalar kPointsColor(255u, 0u, 0u);

  // Duplicate image for annotation and visualization.
  cv::Mat img_clone = img.clone();
  cv::cvtColor(img_clone, img_clone, cv::COLOR_GRAY2BGR);
  const cv::Size& size = img_clone.size();
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

cv::Mat OpenCvVisualizer3D::visualizeMesh2DStereo(
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
    // Kpts that are both valid and have a right pixel are currently the
    // ones
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

void OpenCvVisualizer3D::visualizePoints3D(
    const PointsWithIdMap& points_with_id,
    const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map,
    WidgetsMap* widgets_map) {
  CHECK(widgets_map);
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
  cv::Mat point_cloud_color(
      1, lmk_id_to_lmk_type_map.size(), CV_8UC3, cloud_color_);
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
  std::unique_ptr<cv::viz::WCloud> cloud_widget =
      VIO::make_unique<cv::viz::WCloud>(point_cloud, cloud_color_);
  if (color_the_cloud) {
    *cloud_widget = cv::viz::WCloud(point_cloud, point_cloud_color);
  }
  cloud_widget->setRenderingProperty(cv::viz::POINT_SIZE, 6);

  (*widgets_map)["Point cloud"] = std::move(cloud_widget);
}

void OpenCvVisualizer3D::visualizePlane(const PlaneId& plane_index,
                                        const double& n_x,
                                        const double& n_y,
                                        const double& n_z,
                                        const double& d,
                                        WidgetsMap* widgets,
                                        const bool& visualize_plane_label,
                                        const int& cluster_id) {
  CHECK_NOTNULL(widgets);
  const std::string& plane_id_for_viz = "Plane " + std::to_string(plane_index);
  // Create a plane widget.
  const cv::Vec3d normal(n_x, n_y, n_z);
  const cv::Point3d center(d * n_x, d * n_y, d * n_z);
  static const cv::Vec3d new_yaxis(0, 1, 0);
  static const cv::Size2d size(1.0, 1.0);

  cv::viz::Color plane_color;
  getColorById(cluster_id, &plane_color);

  if (visualize_plane_label) {
    static double increase = 0.0;
    const cv::Point3d text_position(
        d * n_x, d * n_y, d * n_z + std::fmod(increase, 1));
    increase += 0.1;
    (*widgets)[plane_id_for_viz + "_label"] =
        VIO::make_unique<cv::viz::WText3D>(
            plane_id_for_viz, text_position, 0.07, true);
  }

  (*widgets)[plane_id_for_viz] = VIO::make_unique<cv::viz::WPlane>(
      center, normal, new_yaxis, size, plane_color);
  is_plane_id_in_window_[plane_index] = true;
}

void OpenCvVisualizer3D::drawLine(const std::string& line_id,
                                  const double& from_x,
                                  const double& from_y,
                                  const double& from_z,
                                  const double& to_x,
                                  const double& to_y,
                                  const double& to_z,
                                  WidgetsMap* widgets) {
  cv::Point3d pt1(from_x, from_y, from_z);
  cv::Point3d pt2(to_x, to_y, to_z);
  drawLine("Line " + line_id, pt1, pt2, widgets);
}

void OpenCvVisualizer3D::drawLine(const std::string& line_id,
                                  const cv::Point3d& pt1,
                                  const cv::Point3d& pt2,
                                  WidgetsMap* widgets) {
  CHECK_NOTNULL(widgets);
  (*widgets)[line_id] = VIO::make_unique<cv::viz::WLine>(pt1, pt2);
}

void OpenCvVisualizer3D::visualizeMesh3D(const cv::Mat& map_points_3d,
                                         const cv::Mat& polygons_mesh,
                                         WidgetsMap* widgets) {
  cv::Mat colors(0, 1, CV_8UC3, cv::viz::Color::gray());  // Do not color mesh.
  visualizeMesh3D(map_points_3d, colors, polygons_mesh, widgets);
}

void OpenCvVisualizer3D::visualizeMesh3D(const cv::Mat& map_points_3d,
                                         const cv::Mat& colors,
                                         const cv::Mat& polygons_mesh,
                                         WidgetsMap* widgets,
                                         const cv::Mat& tcoords,
                                         const cv::Mat& texture,
                                         const std::string& mesh_id) {
  CHECK_NOTNULL(widgets);
  // Check data
  bool color_mesh = false;
  if (colors.rows != 0) {
    CHECK_EQ(map_points_3d.rows, colors.rows)
        << "Map points and Colors should have same number of rows. One"
           " color per map point.";
    LOG_IF(ERROR, !tcoords.empty())
        << "Texture provided, but colors as well... Do not provide colors if "
           "you want your mesh to be textured.";
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

  // Plot mesh.
  (*widgets)[mesh_id] = VIO::make_unique<cv::viz::WMesh>(cv_mesh);
}

void OpenCvVisualizer3D::visualizePlyMesh(const std::string& filename,
                                          WidgetsMap* widgets) {
  CHECK_NOTNULL(widgets);
  LOG(INFO) << "Showing ground truth mesh: " << filename;
  // The ply file must have in the header a "element vertex" and
  // a "element face" primitives, otherwise you'll get a
  // "Cannot read geometry" error.
  cv::viz::Mesh mesh(cv::viz::Mesh::load(filename));
  if (mesh.polygons.size[1] == 0) {
    LOG(WARNING) << "No polygons available for mesh, showing point cloud only.";
    // If there are no polygons, convert to point cloud, otw there will be
    // nothing displayed...
    std::unique_ptr<cv::viz::WCloud> cloud =
        VIO::make_unique<cv::viz::WCloud>(mesh.cloud, cv::viz::Color::lime());
    cloud->setRenderingProperty(cv::viz::REPRESENTATION,
                                cv::viz::REPRESENTATION_POINTS);
    cloud->setRenderingProperty(cv::viz::POINT_SIZE, 2);
    cloud->setRenderingProperty(cv::viz::OPACITY, 0.1);

    // Plot point cloud.
    (*widgets)["Mesh from ply"] = std::move(cloud);
  } else {
    // Plot mesh.
    (*widgets)["Mesh from ply"] = VIO::make_unique<cv::viz::WMesh>(mesh);
  }
}

void OpenCvVisualizer3D::visualizePointCloud(const cv::Mat& point_cloud,
                                             WidgetsMap* widgets,
                                             const cv::Affine3d& pose,
                                             const cv::Mat& colors,
                                             const cv::Mat& normals) {
  CHECK_NOTNULL(widgets);
  CHECK(!point_cloud.empty());
  auto pcl_type = point_cloud.type();
  CHECK(pcl_type == CV_32FC3 || pcl_type == CV_32FC4 || pcl_type == CV_64FC3 ||
        pcl_type == CV_64FC4);

  // Create cloud widget.
  std::unique_ptr<cv::viz::WCloud> cloud_widget = nullptr;
  if (!colors.empty() && !normals.empty()) {
    cloud_widget =
        VIO::make_unique<cv::viz::WCloud>(point_cloud, colors, normals);
  } else if (!colors.empty()) {
    cloud_widget = VIO::make_unique<cv::viz::WCloud>(point_cloud, colors);
  } else {
    cloud_widget = VIO::make_unique<cv::viz::WCloud>(point_cloud, cloud_color_);
  }
  CHECK(cloud_widget != nullptr);
  cloud_widget->setPose(pose);
  cloud_widget->setRenderingProperty(cv::viz::POINT_SIZE, 2);

  // Send to maps
  static size_t pcl_id = 0u;
  (*widgets)["3D Point Cloud " + std::to_string(pcl_id)] =
      std::move(cloud_widget);
  pcl_id++;
}

void OpenCvVisualizer3D::visualizeGlobalFrameOfReference(WidgetsMap* widgets,
                                                         double scale) {
  CHECK_NOTNULL(widgets);
  (*widgets)["Global Frame of Reference"] =
      VIO::make_unique<cv::viz::WCoordinateSystem>(scale);
}

void OpenCvVisualizer3D::visualizeMesh3DWithColoredClusters(
    const std::vector<Plane>& planes,
    const cv::Mat& map_points_3d,
    const cv::Mat& polygons_mesh,
    WidgetsMap* widgets,
    const bool visualize_mesh_with_colored_polygon_clusters,
    const Timestamp& timestamp) {
  if (visualize_mesh_with_colored_polygon_clusters) {
    // Color the mesh.
    cv::Mat colors;
    colorMeshByClusters(planes, map_points_3d, polygons_mesh, &colors);
    // Visualize the colored mesh.
    visualizeMesh3D(map_points_3d, colors, polygons_mesh, widgets);
    // Log the mesh.
    if (FLAGS_log_mesh) {
      logMesh(map_points_3d,
              colors,
              polygons_mesh,
              timestamp,
              FLAGS_log_accumulated_mesh);
    }
  } else {
    // Visualize the mesh with same colour.
    visualizeMesh3D(map_points_3d, polygons_mesh, widgets);
  }
}

void OpenCvVisualizer3D::visualizeConvexHull(const TriangleCluster& cluster,
                                             const cv::Mat& map_points_3d,
                                             const cv::Mat& polygons_mesh,
                                             WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);

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
    const size_t& triangle_idx = std::round(triangle_id * 4);
    LOG_IF(FATAL, triangle_idx + 3 >= polygons_mesh.rows)
        << "Visualizer3D: an id in triangle_ids_ is too large.";
    const int32_t& idx_1 = polygons_mesh.at<int32_t>(triangle_idx + 1);
    const int32_t& idx_2 = polygons_mesh.at<int32_t>(triangle_idx + 2);
    const int32_t& idx_3 = polygons_mesh.at<int32_t>(triangle_idx + 3);

    // Project points to new coord system
    const cv::Point3f& new_map_point_1 =  // new_coordinates *
                                          // map_points_3d.row(idx_1).t();
        map_points_3d.at<cv::Point3f>(idx_1);
    const cv::Point3f& new_map_point_2 =  // new_coordinates *
                                          // map_points_3d.row(idx_2).t();
        map_points_3d.at<cv::Point3f>(idx_2);
    const cv::Point3f& new_map_point_3 =  // new_coordinates *
                                          // map_points_3d.row(idx_3).t();
        map_points_3d.at<cv::Point3f>(idx_3);

    // Keep only 1st and 2nd component, aka the projection of the point on
    // the
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
      (*widgets_map)["Convex Hull"] =
          VIO::make_unique<cv::viz::WPolyLine>(hull_3d);
    } else {
      // Visualize convex hull as a mesh of one polygon with multiple
      // points.
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
        (*widgets_map)["Convex Hull"] = VIO::make_unique<cv::viz::WMesh>(
            hull_3d, polygon_hull, colors.t(), normals.t());
      }
    }
  }
}

void OpenCvVisualizer3D::visualizeTrajectory3D(WidgetsMap* widgets_map) {
  CHECK_NOTNULL(widgets_map);

  if (trajectory_poses_3d_.size() == 0) {  // no points to visualize
    LOG(WARNING) << "Trajectory empty, not visualizing.";
    return;
  }

  // Create a Trajectory widget. (argument can be PATH, FRAMES, BOTH).
  std::vector<cv::Affine3f> trajectory;
  trajectory.reserve(trajectory_poses_3d_.size());
  for (const auto& pose : trajectory_poses_3d_) {
    trajectory.push_back(pose);
  }
  (*widgets_map)["Trajectory"] = VIO::make_unique<cv::viz::WTrajectory>(
      trajectory, cv::viz::WTrajectory::PATH, 1.0, cv::viz::Color::red());
}

void OpenCvVisualizer3D::visualizeTrajectoryWithFrustums(
    WidgetsMap* widgets_map,
    const size_t& n_last_frustums) {
  CHECK_NOTNULL(widgets_map);
  std::vector<cv::Affine3f> trajectory_frustums;
  trajectory_frustums.reserve(n_last_frustums);
  for (auto rit = trajectory_poses_3d_.rbegin();
       rit != trajectory_poses_3d_.rend() &&
       trajectory_frustums.size() < n_last_frustums;
       ++rit) {
    trajectory_frustums.push_back(*rit);
  }
  (*widgets_map)["Trajectory Frustums"] =
      VIO::make_unique<cv::viz::WTrajectoryFrustums>(
          trajectory_frustums, K_, 0.2, cv::viz::Color::red());
}

void OpenCvVisualizer3D::visualizePoseWithImgInFrustum(
    const cv::Mat& frustum_image,
    const cv::Affine3d& frustum_pose,
    WidgetsMap* widgets_map,
    const std::string& widget_id,
    const cv::Matx33d K) {
  CHECK_NOTNULL(widgets_map);
  std::unique_ptr<cv::viz::WCameraPosition> cam_widget_ptr = nullptr;
  if (frustum_image.empty()) {
    cam_widget_ptr = VIO::make_unique<cv::viz::WCameraPosition>(
        K_, 1.0, cv::viz::Color::white());
  } else {
    cam_widget_ptr = VIO::make_unique<cv::viz::WCameraPosition>(
        K_, frustum_image, 1.0, cv::viz::Color::white());
  }
  CHECK(cam_widget_ptr);
  cam_widget_ptr->setPose(frustum_pose);
  (*widgets_map)[widget_id] = std::move(cam_widget_ptr);
}

bool OpenCvVisualizer3D::removeWidget(const std::string& widget_id) {
  widget_ids_to_remove_.push_back(widget_id);
  return true;
}

void OpenCvVisualizer3D::visualizePlaneConstraints(const PlaneId& plane_id,
                                                   const gtsam::Point3& normal,
                                                   const double& distance,
                                                   const LandmarkId& lmk_id,
                                                   const gtsam::Point3& point,
                                                   WidgetsMap* widgets_map) {
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
    std::string line_id = "Line " +
                          std::to_string(static_cast<int>(plane_id_it->first)) +
                          std::to_string(static_cast<int>(*line_nr_ptr));
    // Draw it.
    drawLineFromPlaneToPoint(line_id,
                             normal.x(),
                             normal.y(),
                             normal.z(),
                             distance,
                             point.x(),
                             point.y(),
                             point.z(),
                             widgets_map);
    // Augment line_nr for next line_id.
    (*line_nr_ptr)++;
  } else {
    // We have drawn this line before.
    // Update line.
    std::string line_id =
        "Line " + std::to_string(static_cast<int>(plane_id_it->first)) +
        std::to_string(static_cast<int>(lmk_id_to_line_id->second));
    updateLineFromPlaneToPoint(line_id,
                               normal.x(),
                               normal.y(),
                               normal.z(),
                               distance,
                               point.x(),
                               point.y(),
                               point.z(),
                               widgets_map);
  }
}

void OpenCvVisualizer3D::removeOldLines(const LandmarkIds& lmk_ids) {
  for (PlaneIdMap::value_type& plane_id_pair : plane_id_map_) {
    LmkIdToLineIdMap& lmk_id_to_line_id_map = plane_id_pair.second;
    for (LmkIdToLineIdMap::iterator lmk_id_to_line_id_it =
             lmk_id_to_line_id_map.begin();
         lmk_id_to_line_id_it != lmk_id_to_line_id_map.end();) {
      if (std::find(lmk_ids.begin(),
                    lmk_ids.end(),
                    lmk_id_to_line_id_it->first) == lmk_ids.end()) {
        // We did not find the lmk_id of the current line in the list
        // of lmk_ids...
        // Delete the corresponding line.
        std::string line_id =
            "Line " + std::to_string(static_cast<int>(plane_id_pair.first)) +
            std::to_string(static_cast<int>(lmk_id_to_line_id_it->second));
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

void OpenCvVisualizer3D::removePlaneConstraintsViz(const PlaneId& plane_id) {
  PlaneIdMap::iterator plane_id_it = plane_id_map_.find(plane_id);
  if (plane_id_it != plane_id_map_.end()) {
    VLOG(0) << "Removing line constraints for plane with id: " << plane_id;
    for (const auto& lmk_id_to_line_id : plane_id_it->second) {
      std::string line_id =
          "Line " + std::to_string(static_cast<int>(plane_id_it->first)) +
          std::to_string(static_cast<int>(lmk_id_to_line_id.second));
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

void OpenCvVisualizer3D::removePlane(const PlaneId& plane_index,
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

void OpenCvVisualizer3D::addPoseToTrajectory(const cv::Affine3d& pose) {
  trajectory_poses_3d_.push_back(pose);
  if (FLAGS_displayed_trajectory_length > 0) {
    while (trajectory_poses_3d_.size() > FLAGS_displayed_trajectory_length) {
      trajectory_poses_3d_.pop_front();
    }
  }
}

Mesh3DVizProperties OpenCvVisualizer3D::texturizeMesh3D(
    const Timestamp& image_timestamp,
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
  mesh_3d_viz_props.colors_ = cv::Mat(
      mesh_3d.getNumberOfUniqueVertices(), 1, CV_8UC3, cv::viz::Color::red());

  // Add texture to the mesh using the given image.
  // README: tcoords specify the texture coordinates of the 3d mesh wrt 2d
  // image. As a small hack, we not only use the left_image as texture but
  // we
  // also horizontally concatenate a white image so we can set a white
  // texture
  // to those 3d mesh faces which should not have a texture. Below we init
  // all
  // tcoords to 0.99 (1.0) gives a weird texture... Meaning that all faces
  // start with a default white texture, and then we change that texture to
  // the right texture for each 2d triangle that has a corresponding 3d
  // face.
  Mesh2D::Polygon polygon;
  std::vector<cv::Vec2d> tcoords(mesh_3d.getNumberOfUniqueVertices(),
                                 cv::Vec2d(0.9, 0.9));
  for (size_t i = 0; i < mesh_2d.getNumberOfPolygons(); i++) {
    CHECK(mesh_2d.getPolygon(i, &polygon)) << "Could not retrieve 2d polygon.";

    const LandmarkId& lmk0 = polygon.at(0).getLmkId();
    const LandmarkId& lmk1 = polygon.at(1).getLmkId();
    const LandmarkId& lmk2 = polygon.at(2).getLmkId();

    // Returns indices of points in the 3D mesh corresponding to the
    // vertices
    // in the 2D mesh.
    Mesh3D::VertexId p0_id, p1_id, p2_id;
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
  static cv::Mat default_texture(texture_image.rows,
                                 texture_image.cols,
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

void OpenCvVisualizer3D::logMesh(const cv::Mat& map_points_3d,
                                 const cv::Mat& colors,
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

void OpenCvVisualizer3D::colorMeshByClusters(const std::vector<Plane>& planes,
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
      const int32_t& idx_1 = polygons_mesh.at<int32_t>(triangle_idx + 1);
      const int32_t& idx_2 = polygons_mesh.at<int32_t>(triangle_idx + 2);
      const int32_t& idx_3 = polygons_mesh.at<int32_t>(triangle_idx + 3);
      // Overrides potential previous color.
      colors->row(idx_1) = cluster_color;
      colors->row(idx_2) = cluster_color;
      colors->row(idx_3) = cluster_color;
    }
  }
}

void OpenCvVisualizer3D::getColorById(const size_t& id,
                                      cv::viz::Color* color) const {
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

void OpenCvVisualizer3D::drawLineFromPlaneToPoint(const std::string& line_id,
                                                  const double& plane_n_x,
                                                  const double& plane_n_y,
                                                  const double& plane_n_z,
                                                  const double& plane_d,
                                                  const double& point_x,
                                                  const double& point_y,
                                                  const double& point_z,
                                                  WidgetsMap* widgets) {
  const cv::Point3d center(
      plane_d * plane_n_x, plane_d * plane_n_y, plane_d * plane_n_z);
  const cv::Point3d point(point_x, point_y, point_z);
  drawLine(line_id, center, point, widgets);
}

void OpenCvVisualizer3D::updateLineFromPlaneToPoint(const std::string& line_id,
                                                    const double& plane_n_x,
                                                    const double& plane_n_y,
                                                    const double& plane_n_z,
                                                    const double& plane_d,
                                                    const double& point_x,
                                                    const double& point_y,
                                                    const double& point_z,
                                                    WidgetsMap* widgets) {
  removeWidget(line_id);
  drawLineFromPlaneToPoint(line_id,
                           plane_n_x,
                           plane_n_y,
                           plane_n_z,
                           plane_d,
                           point_x,
                           point_y,
                           point_z,
                           widgets);
}

}  // namespace VIO
