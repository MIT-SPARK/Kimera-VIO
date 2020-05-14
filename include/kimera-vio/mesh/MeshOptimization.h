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

#include <stdlib.h>
#include <atomic>
#include <limits>   // for numeric_limits<>
#include <utility>  // for move
#include <vector>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>
// To convert from/to eigen
#include <opencv2/core/eigen.hpp>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/mesh/Mesh.h"
#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace VIO {

enum class MeshOptimizerType {
  kConnectedMesh = 0,
  kDisconnectedMesh = 1,
  kClosedForm = 2
};

struct MeshOptimizationInput {
  KIMERA_POINTER_TYPEDEFS(MeshOptimizationInput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MeshOptimizationInput);

  MeshOptimizationInput() = default;
  ~MeshOptimizationInput() = default;

  cv::Mat noisy_point_cloud;
  CameraParams camera_params;
  Mesh2D mesh_2d;
  Mesh3D mesh_3d;
};

struct MeshOptimizationOutput {
  KIMERA_POINTER_TYPEDEFS(MeshOptimizationOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MeshOptimizationOutput);

  MeshOptimizationOutput() = default;
  ~MeshOptimizationOutput() = default;

  Mesh3D optimized_mesh_3d;
};

class MeshOptimization {
 public:
  KIMERA_POINTER_TYPEDEFS(MeshOptimization);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MeshOptimization);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef size_t DatapointId;
  typedef size_t TriangleId;
  typedef std::vector<cv::Point3f> Datapoints;
  typedef std::unordered_map<TriangleId, Datapoints> TriangleToDatapoints;

 public:
  MeshOptimization(const MeshOptimizerType& solver_type,
                            const bool& debug_mode)
      : debug_mode_(debug_mode),
        mesh_optimizer_type_(solver_type),
        window_("Mesh Optimization") {
    window_.setBackgroundColor(cv::viz::Color::white());
    window_.setFullScreen(true);
  }

  virtual ~MeshOptimization() = default;

  /**
   * @brief MeshOptimization::spinOnce Process only one minimal packet of
   * information.
   * @param input Minimal input for the MeshOptimization to do its work.
   * @return MeshOptimization's output: an optimized mesh.
   */
  virtual MeshOptimizationOutput::UniquePtr spinOnce(
      const MeshOptimizationInput& input) {
    return solveOptimalMesh(input.noisy_point_cloud,
                            input.camera_params,
                            input.mesh_2d,
                            input.mesh_3d);
  }

  static void draw2dMeshOnImg(
      const Mesh2D& mesh_2d,
      cv::Mat* img,
      const cv::viz::Color& color = cv::viz::Color::red(),
      const size_t& thickness = 1u,
      const int line_type = 8) {  // Maybe use CV_AA (antialiased).
    CHECK_NOTNULL(img);
    CHECK_EQ(mesh_2d.getMeshPolygonDimension(), 3u);
    CHECK_GT(mesh_2d.getNumberOfPolygons(), 0u);
    // Draw the pixel on the image
    Mesh2D::Polygon polygon;
    for (size_t k = 0u; k < mesh_2d.getNumberOfPolygons(); k++) {
      CHECK(mesh_2d.getPolygon(k, &polygon));
      const Vertex2D& v0 = polygon.at(0).getVertexPosition();
      const Vertex2D& v1 = polygon.at(1).getVertexPosition();
      const Vertex2D& v2 = polygon.at(2).getVertexPosition();
      cv::line(*img, v0, v1, color, thickness, line_type);
      cv::line(*img, v1, v2, color, thickness, line_type);
      cv::line(*img, v2, v0, color, thickness, line_type);
    }
  }

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
                                 size_t* number_of_valid_datapoints) {
    CHECK_NOTNULL(corresp);
    *CHECK_NOTNULL(number_of_valid_datapoints) = 0u;
    for (size_t i = 0; i < noisy_point_cloud.cols; ++i) {
      // 1. Project pointcloud to image (color img with projections)
      // aka get pixel coordinates for all points in pointcloud.
      // TODO(Toni): the projection of all points could be greatly optimized by
      // appending all points into a big matrix and performing dense
      // multiplication.
      const cv::Point3f& lmk = noisy_point_cloud.at<cv::Point3f>(i);
      // TODO(Toni): replace with stereo camera! Or even camera?
      static const gtsam::Cal3_S2 gtsam_intrinsics(
          camera_params.intrinsics_.at(0),
          camera_params.intrinsics_.at(1),
          0,
          camera_params.intrinsics_.at(2),
          camera_params.intrinsics_.at(3));
      const cv::Point2f& pixel = generatePixelFromLandmarkGivenCamera(
          lmk, camera_params.body_Pose_cam_, gtsam_intrinsics);

      if (debug_mode_) {
        drawPixelOnImg(pixel, img_, cv::viz::Color::green(), 1u);
      }

      // 2. Generate correspondences btw points and triangles.
      // For each triangle in 2d Mesh
      // TODO(Toni): this can be greatly optimized by going on a per triangle
      // fashion and using halfplane checks on all points.
      Mesh2D::Polygon polygon;
      for (size_t k = 0; k < mesh_2d.getNumberOfPolygons(); k++) {
        CHECK(mesh_2d.getPolygon(k, &polygon));
        if (pointInTriangle(pixel,
                            polygon.at(0).getVertexPosition(),
                            polygon.at(1).getVertexPosition(),
                            polygon.at(2).getVertexPosition())) {
          // A point should only be in one triangle, once found, we can stop
          // looping over the 2d mesh.
          (*corresp)[k].push_back(lmk);
          ++(*number_of_valid_datapoints);
          break;
        }
      }
    }
  }

  /**
   * @brief solveOptimalMesh
   * Assumes noisy point cloud is given in camera frame!
   * @param noisy_point_cloud
   * @param camera_params
   * @param mesh_2d
   * @param mesh_3d
   * @return
   */
  MeshOptimizationOutput::UniquePtr solveOptimalMesh(
      const cv::Mat& noisy_point_cloud,
      const CameraParams& camera_params,
      const Mesh2D& mesh_2d,
      const Mesh3D& mesh_3d) {
    CHECK_GT(mesh_2d.getNumberOfPolygons(), 0);
    CHECK_EQ(mesh_2d.getNumberOfPolygons(), mesh_3d.getNumberOfPolygons());
    CHECK_GT(mesh_2d.getNumberOfUniqueVertices(), 0);
    CHECK_GT(mesh_3d.getNumberOfUniqueVertices(), 0);
    CHECK_EQ(noisy_point_cloud.rows, 1u);
    CHECK_GT(noisy_point_cloud.cols, 3u);

    static const gtsam::Cal3_S2 gtsam_intrinsics(
        camera_params.intrinsics_.at(0),
        camera_params.intrinsics_.at(1),
        0,
        camera_params.intrinsics_.at(2),
        camera_params.intrinsics_.at(3));

    /// Step 1: Collect all datapoints that fall within triangle
    TriangleToDatapoints corresp;
    size_t number_of_valid_datapoints = 0;
    collectTriangleDataPoints(noisy_point_cloud,
                              mesh_2d,
                              camera_params,
                              &corresp,
                              &number_of_valid_datapoints);

    // Need to visualizeScene again because the image of the camera frustum
    // was updated
    if (debug_mode_) {
      drawPointCloud("Noisy Point Cloud", noisy_point_cloud);
      draw3dMesh("3D Mesh Before Optimization", cv::viz::Color::red(), mesh_3d);
      // draw2dMeshOnImg(img_, mesh_2d);
      drawScene(camera_params.body_Pose_cam_, gtsam_intrinsics);
      spinDisplay();
    }

    CHECK_GT(number_of_valid_datapoints, 3u);
    CHECK_GT(corresp.size(), 0u);
    CHECK_EQ(corresp.size(), mesh_2d.getNumberOfPolygons())
        << "Every triangle should have some data points! (or maybe not really)";

    /// Step 2: Solve for the ys and build the Y matrix incrementally by looping
    /// over all triangles.
    // This matrix has as columns the landmark ids, and as rows the ys of each
    // datapoint, with non-zeros only where a datapoint is associated to a lmk.
    cv::Mat lmk_ids_to_ys = cv::Mat::zeros(number_of_valid_datapoints,
                                           mesh_3d.getNumberOfUniqueVertices(),
                                           CV_32F);
    typedef std::unordered_map<LandmarkId, cv::Point3f> LmkIdToBearingVector;
    LmkIdToBearingVector lmk_ids_to_bearing_vectors;

    // Mesh that will hold the reconstructed one
    Mesh3D reconstructed_mesh;

    // For each triangle in 2d Mesh (as long as non-degenerate).
    Mesh2D::Polygon polygon_2d;
    CHECK_EQ(mesh_2d.getNumberOfPolygons(), corresp.size());
    // VIT this needs to be static or out of the for loop above!
    // otw we overwrite previous rows in Y.
    size_t n_datapoint = 0;
    for (size_t tri_idx = 0; tri_idx < mesh_2d.getNumberOfPolygons();
         tri_idx++) {
      CHECK(mesh_2d.getPolygon(tri_idx, &polygon_2d));
      CHECK_EQ(polygon_2d.size(), 3);

      /// Step 2.1: Build bearing vector matrix of triangle and collect landmark
      /// ids.
      // List of landmark ids associated to the triangle vertices
      LandmarkIds lmk_ids(3);
      // 1 row with 3 columns: each column is a cv::Point3f
      // TODO(Toni): we are recalculating bearing vectors all the time
      // we should be calculating these only once...
      cv::Mat_<Vertex3D> triangle_bearing_vectors(1, 3);
      size_t col = 0;
      for (const auto& vtx : polygon_2d) {
        // Calculate bearing vectors from mesh 2d
        // vtx is pixel, get in bearing vectors?
        const Vertex2D& vtx_pixel = vtx.getVertexPosition();
        cv::Point3f cv_bearing_vector;
        getBearingVectorFrom2DPixel(
            gtsam_intrinsics, vtx_pixel, &cv_bearing_vector);

        if (debug_mode_) {
          drawArrow(cv::Point3d(UtilsOpenCV::gtsamVector3ToCvMat(
                        camera_params.body_Pose_cam_.translation())),
                    cv_bearing_vector,
                    "r" + std::to_string(tri_idx * 3 + col),
                    0.01,
                    0.1,
                    cv::viz::Color::red());
        }
        triangle_bearing_vectors[0][col] = cv_bearing_vector;
        lmk_ids_to_bearing_vectors[vtx.getLmkId()] = cv_bearing_vector;
        col++;
      }

      /// Step 2.2: For each datapoint in current triangle solve for the ys
      /// and build big Y matrix
      // TODO(Toni): check for other degenerate configs such as all points on a
      // line, or same spot or...
      std::vector<cv::Point3f> triangle_datapoints = corresp[tri_idx];
      LOG_IF(FATAL, triangle_datapoints.size() < 3)
          << "Degenerate case optimization problem, we need more than 3 "
             "datapoints: offending triangle idx: "
          << tri_idx;

      /// Step 2.3: Associate Y with its landmark ids:
      cv::Mat Y;
      // TODO(Toni): this is point-wise parallelizable
      for (const cv::Point3f& datapoint : triangle_datapoints) {
        // Create one channel matrices.
        static constexpr int kChannels = 1;
        // Make this have 3 rows
        cv::Mat b = cv::Mat(datapoint).reshape(kChannels, 3);
        // Don't forget the transpose, btw transpose copies!
        cv::Mat A =
            cv::Mat(triangle_bearing_vectors).reshape(kChannels, {3, 3}).t();
        const int& type = A.type();
        CHECK_EQ(A.rows, b.rows);
        CHECK_EQ(type, b.type());
        CHECK(type == CV_32F || type == CV_64F) << "Type is: " << type;
        cv::Mat y;
        cv::solve(A, b, y);
        CHECK_EQ(A.rows, y.rows);

        switch (mesh_optimizer_type_) {
          case MeshOptimizerType::kConnectedMesh: {
            // The row n_datapoint has only non-zeros for those columns
            // associated to the lmk id of the y.
            lmk_ids_to_ys.at<float>(n_datapoint, lmk_ids[0]) = y.at<float>(0);
            lmk_ids_to_ys.at<float>(n_datapoint, lmk_ids[1]) = y.at<float>(1);
            lmk_ids_to_ys.at<float>(n_datapoint, lmk_ids[2]) = y.at<float>(2);
            ++n_datapoint;
            break;
          }
          case MeshOptimizerType::kDisconnectedMesh: {
            if (Y.rows == 0) {
              Y = y.t();
            } else {
              cv::vconcat(Y, y.t(), Y);
            }
            break;
          }
          case MeshOptimizerType::kClosedForm: {
            LOG(ERROR) << "Not implemented yet.";
            break;
          }
          default: {
            LOG(FATAL) << "Unknown Optimal Mesh Solver requestes.";
            break;
          }
        }
      }

      //////////////////////////////////////////////////////////////////////////
      // THAT IS: OPTIMIZE MESH FACE BY FACE AND USE latest estimate
      // for shared vertices....
      // Build y matrix
      if (mesh_optimizer_type_ == MeshOptimizerType::kDisconnectedMesh) {
        const cv::Mat& ones = cv::Mat::ones(Y.rows, 1, CV_32F);
        const int& type = Y.type();
        CHECK_EQ(Y.rows, ones.rows);
        CHECK_EQ(type, ones.type());
        CHECK(type == CV_32F || type == CV_64F) << "Type is: " << type;
        cv::Mat psi;
        // Using QR decomposition (perhaps try SVD)?
        cv::solve(Y, ones, psi, cv::DECOMP_QR);

        // Reconstruct 3D triangle by adding a polygon to 3D mesh.
        Mesh3D::Polygon reconstructed_polygon(3);
        CHECK_EQ(psi.rows, 3);
        CHECK_EQ(triangle_bearing_vectors.cols, 3);
        CHECK_EQ(lmk_ids.size(), 3);
        for (size_t k = 0; k < psi.rows; ++k) {
          const float& depth = 1 / psi.at<float>(k);
          // TODO(Toni): WE ARE OVERRIDING previous optimal vtx position each
          // time! we should be solving for the whole Y system to be optimal.
          const cv::Point3f& lmk = depth * triangle_bearing_vectors[0][k];
          static LandmarkId lmk_id = 0;
          reconstructed_polygon.at(k) = Mesh3D::VertexType(lmk_id, lmk);
          ++lmk_id;
        }
        reconstructed_mesh.addPolygonToMesh(reconstructed_polygon);
        // Display intermediate reconstructed mesh.
        if (debug_mode_) {
          draw3dMesh("Fake Reconstructed Mesh",
                     cv::viz::Color::blue(),
                     reconstructed_mesh,
                     false,
                     0.6);
          spinDisplay();
        }
      }
      //////////////////////////////////////////////////////////////////////////
    }

    /// Recover psis by solving Y
    cv::Mat psi;
    switch (mesh_optimizer_type_) {
      case MeshOptimizerType::kConnectedMesh: {
        // Solve Y matrix
        const cv::Mat& ones = cv::Mat::ones(lmk_ids_to_ys.rows, 1, CV_32F);
        const int& type = lmk_ids_to_ys.type();
        CHECK_EQ(lmk_ids_to_ys.rows, ones.rows);
        CHECK_EQ(type, ones.type());
        CHECK(type == CV_32F || type == CV_64F) << "Type is: " << type;
        // Using QR decomposition (perhaps try SVD)?
        cv::solve(lmk_ids_to_ys, ones, psi, cv::DECOMP_QR);

        /// Reconstruct 3D mesh
        // Clone 3d mesh and update vertices, so we keep same connectivity
        reconstructed_mesh = mesh_3d;
        for (size_t k = 0; k < psi.rows; ++k) {
          const float& depth = 1 / psi.at<float>(k);
          const LandmarkId& lmk_id = k;
          const cv::Point3f& lmk = depth * lmk_ids_to_bearing_vectors[lmk_id];
          CHECK(reconstructed_mesh.setVertexPosition(lmk_id, lmk));
          if (debug_mode_) {
            const cv::Point2f& pixel = generatePixelFromLandmarkGivenCamera(
                lmk, camera_params.body_Pose_cam_, gtsam_intrinsics);
            drawPixelOnImg(pixel, img_, cv::viz::Color::green(), 1u);
          }
        }

        break;
      }
      case MeshOptimizerType::kDisconnectedMesh: {
        // Nothing to do
        break;
      }
      case MeshOptimizerType::kClosedForm: {
        LOG(ERROR) << "Not implemented yet.";
        break;
      }
      default: {
        LOG(FATAL) << "Unknown Optimal Mesh Solver requestes.";
        break;
      }
    }

    // Display reconstructed mesh.
    if (debug_mode_) {
      draw3dMesh("Reconstructed Mesh",
                 cv::viz::Color::yellow(),
                 reconstructed_mesh,
                 false,
                 0.9);
      spinDisplay();
    }
    MeshOptimizationOutput::UniquePtr output =
        VIO::make_unique<MeshOptimizationOutput>();
    output->optimized_mesh_3d = reconstructed_mesh;
    return output;
  }

 private:
  //! Utility functions

  cv::Point2f generatePixelFromLandmarkGivenCamera(
      const cv::Point3f& lmk,
      const gtsam::Pose3& extrinsics,
      const gtsam::Cal3_S2& intrinsics) {
    // Project 3D landmarks to camera image plane and get pixels.
    // (Sub-)Pixel coordinates
    gtsam::Vector3 v(lmk.x, lmk.y, lmk.z);
    gtsam::Point3 pixel = intrinsics.K() * extrinsics.transformTo(v);
    // cv::Point has inverted row/col wrt to cv::Mat!
    return cv::Point2f(pixel.x() / pixel.z(), pixel.y() / pixel.z());
  }

  void getBearingVectorFrom2DPixel(const gtsam::Cal3_S2& intrinsics,
                                   const cv::Point2f& pixel,
                                   cv::Point3f* bearing_vector) {
    CHECK_NOTNULL(bearing_vector);
    gtsam::Vector3 bearing =
        intrinsics.calibrate(gtsam::Vector3(pixel.x, pixel.y, 1.0));
    gtsam::Point3 unit_bearing = gtsam::Point3(bearing[0],
                                               bearing[1],
                                               bearing[2]).normalized();
    *bearing_vector = cv::Point3f(static_cast<float>(unit_bearing.x()),
                                  static_cast<float>(unit_bearing.y()),
                                  static_cast<float>(unit_bearing.z()));
  }

  void getBearingVectorFrom3DLmk(const gtsam::Pose3& extrinsics,
                                 const cv::Point3f& lmk,
                                 cv::Point3f* bearing_vector,
                                 float* inverse_depth) {
    CHECK_NOTNULL(bearing_vector);
    CHECK_NOTNULL(inverse_depth);
    const gtsam::Point3& ray =
        extrinsics.transformTo(gtsam::Vector3(lmk.x, lmk.y, lmk.z));
    const double& norm = ray.norm();
    CHECK_GT(norm, 0.0);
    *inverse_depth = 1 / norm;
    // Divide ray by its length get the normalized bearing vector.
    const gtsam::Point3& bearing = *inverse_depth * ray;
    *bearing_vector = cv::Point3d(bearing.x(), bearing.y(), bearing.z());
  }

  // This is taken from stackoverflow:
  // https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
  float sign(const cv::Point2f& p1,
             const cv::Point2f& p2,
             const cv::Point2f& p3) {
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
  }

  bool pointInTriangle(const cv::Point2f& pt,
                       const cv::Point2f& v1,
                       const cv::Point2f& v2,
                       const cv::Point2f& v3) {
    float d1, d2, d3;
    bool has_neg, has_pos;

    d1 = sign(pt, v1, v2);
    d2 = sign(pt, v2, v3);
    d3 = sign(pt, v3, v1);

    has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);
  }

  //! Visualization functions
  // TODO(Toni): these should be in Visualizer 3D
  void draw3dMesh(const std::string& id,
                  const cv::viz::Color& color,
                  const Mesh3D& mesh_3d,
                  bool display_as_wireframe = false,
                  const double& opacity = 1.0) {
    cv::Mat vertices_mesh;
    cv::Mat polygons_mesh;
    mesh_3d.convertVerticesMeshToMat(&vertices_mesh);
    mesh_3d.convertPolygonsMeshToMat(&polygons_mesh);

    // Build visual mesh
    cv::viz::Mesh cv_mesh;
    cv_mesh.cloud = vertices_mesh.t();
    cv_mesh.polygons = polygons_mesh;
    cv_mesh.colors = cv::Mat(cv_mesh.cloud.size(), CV_8UC3, color);

    // Build widget mesh
    cv::viz::WMesh widget_cv_mesh(cv_mesh);
    widget_cv_mesh.setRenderingProperty(cv::viz::SHADING,
                                        cv::viz::SHADING_FLAT);
    widget_cv_mesh.setRenderingProperty(cv::viz::AMBIENT, 0);
    widget_cv_mesh.setRenderingProperty(cv::viz::LIGHTING, 1);
    widget_cv_mesh.setRenderingProperty(cv::viz::OPACITY, opacity);
    if (display_as_wireframe) {
      widget_cv_mesh.setRenderingProperty(cv::viz::REPRESENTATION,
                                          cv::viz::REPRESENTATION_WIREFRAME);
    }
    window_.showWidget(id.c_str(), widget_cv_mesh);
  }

  void drawPointCloud(const std::string& id, const cv::Mat& pointcloud) {
    cv::viz::WCloud cloud(pointcloud, cv::viz::Color::red());
    cloud.setRenderingProperty(cv::viz::POINT_SIZE, 6);
    window_.showWidget(id, cloud);
  }

  void drawScene(const gtsam::Pose3& extrinsics,
                 const gtsam::Cal3_S2& intrinsics) {
    cv::Mat cv_extrinsics;
    cv::eigen2cv(extrinsics.matrix(), cv_extrinsics);
    cv::Affine3f cam_pose_real;
    cam_pose_real.matrix = cv_extrinsics;

    cv::Matx33d K;
    cv::eigen2cv(intrinsics.K(), K);
    cv::viz::WCameraPosition cpw(0.2);                   // Coordinate axes
    cv::viz::WCameraPosition cpw_frustum(K, img_, 2.0);  // Camera frustum
    window_.showWidget("World Coordinates", cv::viz::WCoordinateSystem(0.5));
    window_.showWidget("Cam Coordinates", cpw, cam_pose_real);
    window_.showWidget("Cam Frustum", cpw_frustum, cam_pose_real);
  }

  void drawArrow(const cv::Point3f& from,
                 const cv::Point3f& to,
                 const std::string& id,
                 const double& arrow_thickness = 0.03,
                 const double& text_thickness = 0.2,
                 const cv::viz::Color& color = cv::viz::Color::blue()) {
    // Display 3D rays from cam origin to lmks.
    window_.showWidget("Arrow Label " + id,
                       cv::viz::WText3D(id, to, text_thickness, true, color));
    window_.showWidget("Arrow " + id,
                       cv::viz::WArrow(from, to, arrow_thickness, color));
  }

  void drawPixelOnImg(const cv::Point2f& pixel,
                      const cv::Mat& img,
                      const cv::viz::Color& color = cv::viz::Color::red(),
                      const size_t& pixel_size = 5u) {
    // Draw the pixel on the image
    cv::circle(img, pixel, pixel_size, color, -1);
  }

  void spinDisplay() {
    // Display 3D window
    window_.spin();
  }

 private:
  /// If debug mode is enabled a 3D viz window will display the problem.
  bool debug_mode_ = true;

  const MeshOptimizerType mesh_optimizer_type_;

  /// Image for debug display
  cv::Mat img_ = cv::Mat::zeros(400, 400, CV_8UC3);

  /// 3D plotting
  // TODO(Toni) this should be done by the display module...
  cv::viz::Viz3d window_;
};

}  // namespace VIO
