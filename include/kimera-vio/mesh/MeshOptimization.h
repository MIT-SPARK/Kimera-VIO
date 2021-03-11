/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MeshOptimization.h
 * @brief  Optimizes vertices of a 3D mesh given depth data on a projective
 * setting (depth map, rgb-d, lidar).
 * @author Antoni Rosinol
 */

#pragma once

#include <map>
#include <vector>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/mesh/Mesh.h"
#include "kimera-vio/mesh/MeshOptimization-definitions.h"
#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/OpenCvVisualizer3D.h"

namespace VIO {

class MeshOptimization {
 public:
  KIMERA_POINTER_TYPEDEFS(MeshOptimization);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MeshOptimization);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef size_t DatapointId;
  typedef size_t TriangleId;
  typedef std::vector<cv::Point3f> Datapoints;
  typedef std::unordered_map<TriangleId, Datapoints> TriangleToDatapoints;
  typedef std::unordered_map<TriangleId, KeypointsCV> TriangleToPixels;

 public:
  MeshOptimization(const MeshOptimizerType& solver_type,
                   const MeshColorType& mesh_color_type,
                   Camera::ConstPtr camera,
                   OpenCvVisualizer3D::Ptr visualizer = nullptr);
  virtual ~MeshOptimization() = default;

  /**
   * @brief MeshOptimization::spinOnce Process only one minimal packet of
   * information.
   * @param input Minimal input for the MeshOptimization to do its work.
   * @return MeshOptimization's output: an optimized mesh.
   */
  virtual MeshOptimizationOutput::UniquePtr spinOnce(
      const MeshOptimizationInput& input);

  static void draw2dMeshOnImg(
      const Mesh2D& mesh_2d,
      cv::Mat* img,
      const cv::viz::Color& color = cv::viz::Color::red(),
      const size_t& thickness = 1u,
      const int line_type = CV_AA);

  //! Visualization functions
  void draw3dMesh(const std::string& id,
                  const Mesh3D& mesh_3d,
                  bool display_as_wireframe = false,
                  const double& opacity = 1.0);

  //! Render the collected visualizations
  void spinDisplay();

 public:
  // In public only for testing... please remove.

  /** Faster version (should be faster using parallelization and vectorization)
   * both in hardware (GPUs) or in software...
   *
   * WARNING assumes the points in mesh_2d are given as u,v not v,u!
   * I.e. if you feed this mesh with the opencv feature detections given
   * as point2f which are (v,u), this will break! Transpose to (u,v) points.
   *
   * @brief collectTriangleDataPointsFast
   * @param noisy_point_cloud
   * @param mesh_2d
   * @param corresp
   * @param pixel_corresp
   * @param number_of_valid_datapoints
   */
  void collectTriangleDataPointsFast(const cv::Mat& noisy_point_cloud,
                                     const Mesh2D& mesh_2d,
                                     TriangleToDatapoints* corresp,
                                     TriangleToPixels* pixel_corresp,
                                     size_t* number_of_valid_datapoints);

  /** THIS IS TOO SLOW, and assumes noisy point cloud is not organized...
   * @brief collectTriangleDataPoints Builds correspondences between 2D
   * triangles and noisy point cloud.
   * @param noisy_point_cloud
   * @param mesh_2d
   * @param corresp
   * @param number_of_valid_datapoints
   */
  void collectTriangleDataPoints(const cv::Mat& noisy_point_cloud,
                                 const Mesh2D& mesh_2d,
                                 TriangleToDatapoints* corresp,
                                 TriangleToPixels* pixel_corresp,
                                 size_t* number_of_valid_datapoints);

  //! Utility functions: put them in MeshUtils
  static cv::Point2f generatePixelFromLandmarkGivenCamera(
      const cv::Point3f& lmk,
      const gtsam::Pose3& extrinsics,
      const gtsam::Cal3_S2& intrinsics);

  // This is taken from stackoverflow:
  // https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
  static float sign(const cv::Point2f& p1,
                    const cv::Point2f& p2,
                    const cv::Point2f& p3);

  static bool pointInTriangle(const cv::Point2f& pt,
                              const cv::Point2f& v1,
                              const cv::Point2f& v2,
                              const cv::Point2f& v3);

 private:
  /**
   * @brief solveOptimalMesh
   * Assumes noisy point cloud is given in camera frame!
   * @param noisy_point_cloud In the frame of reference of the undistorted
   * (and rectified) camera
   * @param mesh_2d In pixel coordinates
   * @return
   */
  MeshOptimizationOutput::UniquePtr solveOptimalMesh(
      const cv::Mat& noisy_point_cloud,
      const cv::Mat& pcl_colors,
      const Mesh2D& mesh_2d);

 private:
  void getBearingVectorFrom2DPixel(const cv::Point2f& pixel,
                                   cv::Point3f* bearing_vector);

  void getBearingVectorFrom3DLmk(const gtsam::Pose3& extrinsics,
                                 const cv::Point3f& lmk,
                                 cv::Point3f* bearing_vector,
                                 float* inverse_depth);

  void drawPixelOnImg(const cv::Point2f& pixel,
                      const cv::Mat& img,
                      const cv::viz::Color& color = cv::viz::Color::red(),
                      const size_t& pixel_size = 5u);

 public:
  /// Image for debug display: gray scale.
  cv::Mat img_ = cv::Mat::zeros(400, 400, CV_8UC1);

 private:
  const MeshOptimizerType mesh_optimizer_type_;

  static constexpr bool kUseSpringEnergies = false;
  static constexpr float kSpringNoiseSigma = 0.5f;
  static constexpr float kDepthMeasNoiseSigma = 0.1f;
  static constexpr float kMissingZ = 10000.0f;
  static constexpr float kMinZ = 0.00001f;
  static constexpr float kMaxZ = 10.0f;

  //! Camera with which the noisy point cloud and the 2d mesh were generated.
  Camera::ConstPtr mono_camera_;
  gtsam::Pose3 body_pose_cam_;

  //! Mesh count: just for visualization to change the ids of the 3d mesh widget
  size_t mesh_count_ = 0u;

  /// 3D plotting
  // TODO(Toni) this should be done by the display module...
  cv::viz::Viz3d window_;
  MeshColorType mesh_color_type_;
  OpenCvVisualizer3D::Ptr visualizer_;
};

}  // namespace VIO
