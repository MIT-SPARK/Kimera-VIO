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
  kMesh2dTo3dSparse,  // same as MESH2DTo3D but filters out triangles
                      // corresponding to non planar obstacles
  kPointcloud,        // visualize 3D VIO points  (no repeated point)
  kNone               // does not visualize map
};

struct VisualizerInput : public PipelinePayload {
  KIMERA_POINTER_TYPEDEFS(VisualizerInput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisualizerInput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VisualizerInput(const Timestamp& timestamp,
                  const MesherOutput::Ptr& mesher_output,
                  const BackendOutput::Ptr& backend_output,
                  const FrontendOutput::Ptr& frontend_output)
      : PipelinePayload(timestamp),
        mesher_output_(mesher_output),
        backend_output_(backend_output),
        frontend_output_(frontend_output) {
    CHECK(mesher_output);
    CHECK(backend_output);
    CHECK(frontend_output);
    CHECK_EQ(timestamp, mesher_output->timestamp_);
    CHECK_EQ(timestamp, frontend_output->timestamp_);
    CHECK_EQ(timestamp, backend_output->timestamp_);
  }
  virtual ~VisualizerInput() = default;

  // Copy the pointers so that we do not need to copy the data.
  const MesherOutput::ConstPtr mesher_output_;
  const BackendOutput::ConstPtr backend_output_;
  const FrontendOutput::ConstPtr frontend_output_;
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

  VisualizationType visualization_type_ = VisualizationType::kNone;
  std::vector<ImageToDisplay> images_to_display_;
  cv::viz::Viz3d window_ = cv::viz::Viz3d("3D Visualizer");
};

enum class VisualizerType {
  //! OpenCV 3D viz, uses VTK underneath the hood.
  OpenCV = 0u,
};

}  // namespace VIO
