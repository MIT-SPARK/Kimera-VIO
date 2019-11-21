/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Visualizer3D-definitions.h
 * @brief  Class definitions for 3D visualizer.
 * @author Antoni Rosinol
 */

#pragma once

#include <vector>

#include <glog/logging.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

enum class VisualizationType {
  MESH2DTo3Dsparse,  // same as MESH2DTo3D but filters out triangles
                     // corresponding to non planar obstacles
  POINTCLOUD,        // visualize 3D VIO points  (no repeated point)
  NONE               // does not visualize map
};

struct VisualizerInput : public PipelinePayload {
  KIMERA_POINTER_TYPEDEFS(VisualizerInput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisualizerInput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VisualizerInput(const Timestamp& timestamp,
                  const gtsam::Pose3& pose,
                  const StereoFrame& stereo_keyframe,
                  const MesherOutput::Ptr& mesher_output_payload,
                  const PointsWithIdMap& points_with_id_VIO,
                  const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map,
                  const std::vector<Plane>& planes,
                  const gtsam::NonlinearFactorGraph& graph,
                  const gtsam::Values& values)
      : PipelinePayload(timestamp),
        pose_(pose),
        stereo_keyframe_(stereo_keyframe),
        mesher_output_payload_(std::move(mesher_output_payload)),
        points_with_id_VIO_(points_with_id_VIO),
        lmk_id_to_lmk_type_map_(lmk_id_to_lmk_type_map),
        planes_(planes),
        graph_(graph),
        values_(values) {}

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
  ImageToDisplay(const std::string& name, const cv::Mat& image)
      : name_(name), image_(image) {}

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

enum class VisualizerType {
  //! OpenCV 3D viz, uses VTK underneath the hood.
  OpenCV = 0u,
};

}  // namespace VIO
