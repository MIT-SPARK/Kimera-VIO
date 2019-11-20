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

#include "kimera-vio/utils/Macros.h"

#include <vector>

#include <gtsam/geometry/Pose3.h>

#include <opencv2/opencv.hpp>

#include "kimera-vio/mesh/Mesh.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"

namespace VIO {

enum class MesherType {
  //! Generates a per-frame 2D mesh and projects it to 3D.
  PROJECTIVE = 0,
};

struct MesherParams {
  KIMERA_POINTER_TYPEDEFS(MesherParams);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MesherParams(const gtsam::Pose3& B_Pose_camLrect, const cv::Size& img_size)
      : B_Pose_camLrect_(B_Pose_camLrect), img_size_(img_size) {}
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
  MesherInput(
      const Timestamp& timestamp,
      const FrontendOutput::Ptr& frontend_payload,
      const BackendOutput::Ptr& backend_payload,
      const std::unordered_map<LandmarkId, gtsam::Point3>& points_with_id_vio)
      : PipelinePayload(timestamp),
        frontend_payload_(frontend_payload),
        backend_payload_(backend_payload),
        points_with_id_vio_(points_with_id_vio) {
    CHECK_EQ(timestamp, frontend_payload->timestamp_);
    CHECK_EQ(timestamp, backend_payload->timestamp_);
  }
  virtual ~MesherInput() = default;

  // Copy the pointers so that we do not need to copy the data.
  const FrontendOutput::ConstPtr frontend_payload_;
  const BackendOutput::ConstPtr backend_payload_;

  // TODO(Toni): this should be in the backend payload.
  const std::unordered_map<LandmarkId, gtsam::Point3> points_with_id_vio_;
};

struct MesherOutput : public PipelinePayload {
 public:
  KIMERA_POINTER_TYPEDEFS(MesherOutput);
  // TODO(Toni): delete copy constructors
  // KIMERA_DELETE_COPY_CONSTRUCTORS(MesherOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit MesherOutput(const Timestamp& timestamp)
      : PipelinePayload(timestamp),
        mesh_2d_(3),
        mesh_3d_(3),
        mesh_2d_for_viz_(),
        mesh_2d_filtered_for_viz_() {}

  MesherOutput(const Timestamp& timestamp,
               Mesh2D&& mesh_2d,  // Use move semantics for the actual 2d mesh.
               Mesh3D&& mesh_3d,  // Use move semantics for the actual 3d mesh.
               const std::vector<cv::Vec6f>& mesh_2d_for_viz,
               const std::vector<cv::Vec6f>& mesh_2d_filtered_for_viz)
      : PipelinePayload(timestamp),
        mesh_2d_(std::move(mesh_2d)),
        mesh_3d_(std::move(mesh_3d)),
        mesh_2d_for_viz_(mesh_2d_for_viz),
        mesh_2d_filtered_for_viz_(mesh_2d_filtered_for_viz) {}

  explicit MesherOutput(const MesherOutput::Ptr& in)
      : PipelinePayload(in ? in->timestamp_ : Timestamp()),
        mesh_2d_(3),
        mesh_3d_(3),
        mesh_2d_for_viz_(in ? in->mesh_2d_for_viz_
                            : std::vector<cv::Vec6f>()),  // yet another copy...
        mesh_2d_filtered_for_viz_(in ? in->mesh_2d_filtered_for_viz_
                                     : std::vector<cv::Vec6f>()) {}

  virtual ~MesherOutput() = default;

  // Default copy ctor.
  MesherOutput(const MesherOutput& rhs) = default;
  // Default copy assignment operator.
  MesherOutput& operator=(const MesherOutput& rhs) = default;

  // Use default move ctor and move assignment operator.
  MesherOutput(MesherOutput&&) = default;
  MesherOutput& operator=(MesherOutput&&) = default;

 public:
  Mesh2D mesh_2d_;
  Mesh3D mesh_3d_;

  // 2D Mesh visualization.
  std::vector<cv::Vec6f> mesh_2d_for_viz_;
  std::vector<cv::Vec6f> mesh_2d_filtered_for_viz_;

  // 3D Mesh using underlying storage type, aka a list of vertices, together
  // with a list of polygons represented as vertices ids pointing to the list
  // of vertices. (see OpenCV way of storing a Mesh)
  // https://docs.opencv.org/3.4/dc/d4f/classcv_1_1viz_1_1Mesh.html#ac4482e5c832f2bd24bb697c340eaf853
  cv::Mat vertices_mesh_;
  cv::Mat polygons_mesh_;
};

}  // namespace VIO
