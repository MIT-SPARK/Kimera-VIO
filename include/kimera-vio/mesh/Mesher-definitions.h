/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Mesher-definitions.h
 * @brief  Definitions for the mesher.
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <utility>  // For move
#include <vector>

#include <gtsam/geometry/Pose3.h>

#include <opencv2/opencv.hpp>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/mesh/Mesh.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

enum class MesherType {
  //! Generates a per-frame 2D mesh and projects it to 3D.
  PROJECTIVE = 0,
};

/**
 * @brief Structure storing mesh 3d visualization properties.
 */
struct Mesh3DVizProperties {
 public:
  // List of RGB colors, one color (three entries R G B) for each vertex in
  // the Mesh3D. Therefore, colors must have same number of rows than the
  // number of vertices in the 3D mesh and three cols for each RGB entry.
  cv::Mat colors_;
  // Texture coordinates.
  cv::Mat tcoords_;
  // Texture image.
  cv::Mat texture_;
};
/** Given the following:
 * Left image in colors, Mesh in 2D, Mesh in 3D.
 * Returns Colors of the Mesh3D. Each color representing a semantic class.
 */
typedef std::function<Mesh3DVizProperties(const Timestamp& img_left_timestamp,
                                          const cv::Mat& img_left,
                                          const Mesh2D&,
                                          const Mesh3D&)>
    Mesh3dVizPropertiesSetterCallback;

struct MesherParams {
 public:
  KIMERA_POINTER_TYPEDEFS(MesherParams);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MesherParams() = default;
  MesherParams(const gtsam::Pose3& B_Pose_camLrect, const cv::Size& img_size)
      : B_Pose_camLrect_(B_Pose_camLrect), img_size_(img_size) {}
  ~MesherParams() = default;

 public:
  //! B_Pose_camLrect pose of the rectified camera wrt body frame of ref.
  gtsam::Pose3 B_Pose_camLrect_;
  //! img_size size of the camera's images used for 2D triangulation.
  cv::Size img_size_;
};

struct MesherInput : public PipelinePayload {
 public:
  KIMERA_POINTER_TYPEDEFS(MesherInput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MesherInput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Copy the pointers so that we do not need to copy the data, we will
  // reference to it via the copied pointers.
  MesherInput(const Timestamp& timestamp,
              const StereoFrontendOutput::Ptr& frontend_payload,
              const BackendOutput::Ptr& backend_payload)
      : PipelinePayload(timestamp),
        frontend_output_(frontend_payload),
        backend_output_(backend_payload) {
    CHECK(frontend_payload);
    CHECK(backend_payload);
    CHECK_EQ(timestamp, frontend_payload->timestamp_);
    CHECK_EQ(timestamp, backend_payload->timestamp_);
  }
  virtual ~MesherInput() = default;

  // Copy the pointers so that we do not need to copy the data.
  const StereoFrontendOutput::ConstPtr frontend_output_;
  const BackendOutput::ConstPtr backend_output_;
};

struct MesherOutput : public PipelinePayload {
 public:
  KIMERA_POINTER_TYPEDEFS(MesherOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MesherOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit MesherOutput(const Timestamp& timestamp)
      : PipelinePayload(timestamp),
        mesh_2d_(3),
        mesh_3d_(3),
        mesh_2d_for_viz_() {}

  MesherOutput(const Timestamp& timestamp,
               Mesh2D&& mesh_2d,  // Use move semantics for the actual 2d mesh.
               Mesh3D&& mesh_3d,  // Use move semantics for the actual 3d mesh.
               const std::vector<cv::Vec6f>& mesh_2d_for_viz)
      : PipelinePayload(timestamp),
        mesh_2d_(std::move(mesh_2d)),
        mesh_3d_(std::move(mesh_3d)),
        mesh_2d_for_viz_(mesh_2d_for_viz),
        // TODO(Toni): re-implement.
        planes_() {}

  explicit MesherOutput(const MesherOutput::Ptr& in)
      : PipelinePayload(in ? in->timestamp_ : Timestamp()),
        mesh_2d_(3),
        mesh_3d_(3),
        // yet another copy...
        mesh_2d_for_viz_(in ? in->mesh_2d_for_viz_ : std::vector<cv::Vec6f>()),
        // TODO(Toni): re-implement.
        planes_() {}

  virtual ~MesherOutput() = default;

  // Use default move ctor and move assignment operator.
  MesherOutput(MesherOutput&&) = delete;
  MesherOutput& operator=(MesherOutput&&) = delete;

 public:
  Mesh2D mesh_2d_;
  Mesh3D mesh_3d_;

  // TODO(Toni): remove, this info is already in mesh_2d_
  // 2D Mesh visualization.
  std::vector<cv::Vec6f> mesh_2d_for_viz_;

  // 3D Mesh using underlying storage type, aka a list of vertices, together
  // with a list of polygons represented as vertices ids pointing to the list
  // of vertices. (see OpenCV way of storing a Mesh)
  // https://docs.opencv.org/3.4/dc/d4f/classcv_1_1viz_1_1Mesh.html#ac4482e5c832f2bd24bb697c340eaf853
  cv::Mat vertices_mesh_;
  cv::Mat polygons_mesh_;

  //! Planes from Regular VIO Backend
  std::vector<Plane> planes_;
};

}  // namespace VIO
