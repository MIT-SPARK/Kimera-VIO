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

#include <vector>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/mesh/Mesh.h"
#include "kimera-vio/mesh/MeshOptimization-definitions.h"
#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/utils/Macros.h"

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
                   const bool& debug_mode);
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

 private:
  /**
   * @brief collectTriangleDataPoints Builds correspondences between 2D
   * triangles and noisy point cloud.
   * @param noisy_point_cloud
   * @param mesh_2d
   * @param corresp
   * @param number_of_valid_datapoints
   */
  void collectTriangleDataPoints(const cv::Mat& noisy_point_cloud,
                                 const Mesh2D& mesh_2d,
                                 const CameraParams& camera_params,
                                 TriangleToDatapoints* corresp,
                                 TriangleToPixels* pixel_corresp,
                                 size_t* number_of_valid_datapoints);

  /**
   * @brief solveOptimalMesh
   * Assumes noisy point cloud is given in camera frame!
   * @param noisy_point_cloud
   * @param camera_params
   * @param mesh_2d
   * @return
   */
  MeshOptimizationOutput::UniquePtr solveOptimalMesh(
      const cv::Mat& noisy_point_cloud,
      const CameraParams& camera_params,
      const Mesh2D& mesh_2d);

 private:
  //! Utility functions
  cv::Point2f generatePixelFromLandmarkGivenCamera(
      const cv::Point3f& lmk,
      const gtsam::Pose3& extrinsics,
      const gtsam::Cal3_S2& intrinsics);

  void getBearingVectorFrom2DPixel(const gtsam::Cal3_S2& intrinsics,
                                   const cv::Point2f& pixel,
                                   cv::Point3f* bearing_vector);

  void getBearingVectorFrom3DLmk(const gtsam::Pose3& extrinsics,
                                 const cv::Point3f& lmk,
                                 cv::Point3f* bearing_vector,
                                 float* inverse_depth);

  // This is taken from stackoverflow:
  // https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
  float sign(const cv::Point2f& p1,
             const cv::Point2f& p2,
             const cv::Point2f& p3);

  bool pointInTriangle(const cv::Point2f& pt,
                       const cv::Point2f& v1,
                       const cv::Point2f& v2,
                       const cv::Point2f& v3);

  void drawPointCloud(const std::string& id, const cv::Mat& pointcloud);

  /**
   * @brief drawCylinder
   * @param id
   * @param axis_point1 A point1 on the axis of the cylinder.
   * @param axis_point2 A point2 on the axis of the cylinder.
   * @param radius Radius of the cylinder.
   * @param numsides Resolution of the cylinder.
   * @param color Color of the cylinder.
   */
  void drawCylinder(const std::string& id,
                    const cv::Point3d& axis_point1,
                    const cv::Point3d& axis_point2,
                    const double& radius,
                    const int& numsides = 30,
                    const cv::viz::Color& color = cv::viz::Color::red());

  void drawScene(const gtsam::Pose3& extrinsics,
                 const gtsam::Cal3_S2& intrinsics);

  void drawArrow(const cv::Point3f& from,
                 const cv::Point3f& to,
                 const std::string& id,
                 const bool& with_text,
                 const double& arrow_thickness = 0.03,
                 const double& text_thickness = 0.2,
                 const cv::viz::Color& color = cv::viz::Color::blue());

  void drawPixelOnImg(const cv::Point2f& pixel,
                      const cv::Mat& img,
                      const cv::viz::Color& color = cv::viz::Color::red(),
                      const size_t& pixel_size = 5u);

  void spinDisplay();

 public:
  /// Image for debug display
  cv::Mat img_ = cv::Mat::zeros(400, 400, CV_8UC3);

 private:
  /// If debug mode is enabled a 3D viz window will display the problem.
  bool debug_mode_ = true;

  const MeshOptimizerType mesh_optimizer_type_;

  /// 3D plotting
  // TODO(Toni) this should be done by the display module...
  cv::viz::Viz3d window_;
};

}  // namespace VIO
