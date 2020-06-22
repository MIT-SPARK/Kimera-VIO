/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   PangolinDisplay.h
 * @brief  Class to display visualizer output using Pangolin
 * @author Antoni Rosinol
 */

#pragma once

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include "kimera-vio/pipeline/Pipeline-definitions.h"  // Needed for shutdown cb
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Display-definitions.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

#include "kimera-vio/mesh/Mesh.h"

namespace VIO {

struct PangolinDisplayInput: public DisplayInputBase {
  KIMERA_POINTER_TYPEDEFS(PangolinDisplayInput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(PangolinDisplayInput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PangolinDisplayInput() = default;
  virtual ~PangolinDisplayInput() = default;

  Mesh3D mesh_3d_;
};

class PangolinDisplay : public DisplayBase {
 public:
  KIMERA_POINTER_TYPEDEFS(PangolinDisplay);
  KIMERA_DELETE_COPY_CONSTRUCTORS(PangolinDisplay);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PangolinDisplay(DisplayParams::Ptr display_params,
                  const ShutdownPipelineCallback& shutdown_pipeline_cb);

  // TODO(Toni): consider using `unregisterAllWindows`
  ~PangolinDisplay() override = default;

  /**
   * @brief spinOnce
   * Spins renderers to display data using Pangolin
   * Displaying must be done in the main thread, that it is why it is separated
   * from Visualizer3D (plus it makes everything faster as displaying and
   * building 3D graphics is decoupled).
   * @param viz_output
   */
  void spinOnce(DisplayInputBase::UniquePtr&& viz_input) override;

 private:
  //! Adds 3D widgets to the window, and displays it.
  void spin3dWindow(VisualizerOutput::UniquePtr&& viz_output);

  //! Visualizes 2D data.
  void spin2dWindow(const DisplayInputBase& viz_output);

 private:
  pangolin::OpenGlRenderState s_cam;
  pangolin::Renderable tree;
  std::unique_ptr<pangolin::SceneHandler> handler;
  pangolin::View d_cam;
  //! We use this callback to shutdown the pipeline gracefully if
  //! the visualization window is closed.
  ShutdownPipelineCallback shutdown_pipeline_cb_;
};

}  // namespace VIO
