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

#include <memory>
#include <opencv2/viz/widgets.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/FrontendOutputPacketBase.h"
#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/pipeline/PipelinePayload.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"

namespace VIO {

enum class VisualizerType {
  //! OpenCV 3D viz, uses VTK underneath the hood.
  OpenCV = 0u
};

enum class VisualizationType {
  kMesh2dTo3dSparse = 0,  // same as MESH2DTo3D but filters out triangles
                          // corresponding to non planar obstacles
  kPointcloud = 1,        // visualize 3D VIO points  (no repeated point)
  kNone = 2               // does not visualize map
};

typedef std::unique_ptr<cv::viz::Widget3D> WidgetPtr;
typedef std::map<std::string, WidgetPtr> WidgetsMap;
typedef std::vector<std::string> WidgetIds;

struct ImageToDisplay {
  ImageToDisplay() = default;
  ImageToDisplay(const std::string& name, const cv::Mat& image)
      : name_(name), image_(image) {}

  std::string name_;
  cv::Mat image_;
};

struct DisplayInputBase {
  KIMERA_POINTER_TYPEDEFS(DisplayInputBase);
  KIMERA_DELETE_COPY_CONSTRUCTORS(DisplayInputBase);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DisplayInputBase() = default;
  virtual ~DisplayInputBase() = default;

  Timestamp timestamp_;
  std::vector<ImageToDisplay> images_to_display_;
};
typedef ThreadsafeQueue<DisplayInputBase::UniquePtr> DisplayQueue;

struct VisualizerInput : public PipelinePayload {
  KIMERA_POINTER_TYPEDEFS(VisualizerInput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisualizerInput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VisualizerInput(const Timestamp& timestamp,
                  const MesherOutput::Ptr& mesher_output,
                  const BackendOutput::Ptr& backend_output,
                  const FrontendOutputPacketBase::Ptr& frontend_output)
      : PipelinePayload(timestamp),
        mesher_output_(mesher_output),
        backend_output_(backend_output),
        frontend_output_(frontend_output) {
    if (backend_output) CHECK_EQ(timestamp, backend_output->timestamp_);
    if (frontend_output) CHECK_EQ(timestamp, frontend_output->timestamp_);
    if (mesher_output) CHECK_EQ(timestamp, mesher_output->timestamp_);
  }
  virtual ~VisualizerInput() = default;

  // Copy the pointers so that we do not need to copy the data.
  const MesherOutput::ConstPtr mesher_output_;
  const BackendOutput::ConstPtr backend_output_;
  const FrontendOutputPacketBase::Ptr
      frontend_output_;  // not ConstPtr because polymorphic
};

struct VisualizerOutput : public DisplayInputBase {
  KIMERA_POINTER_TYPEDEFS(VisualizerOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisualizerOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VisualizerOutput()
      : DisplayInputBase(),
        visualization_type_(VisualizationType::kNone),
        widgets_(),
        widget_ids_to_remove_(),
        frustum_pose_(cv::Affine3d::Identity()) {}
  ~VisualizerOutput() = default;

  VisualizationType visualization_type_;
  WidgetsMap widgets_;
  WidgetIds widget_ids_to_remove_;
  cv::Affine3d frustum_pose_;
};

}  // namespace VIO
