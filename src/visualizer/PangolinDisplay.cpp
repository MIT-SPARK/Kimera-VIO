/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   PangolinDisplay.cpp
 * @brief  Class to display visualizer output using Pangolin
 * @author Antoni Rosinol
 */

#include "kimera-vio/visualizer/PangolinDisplay.h"

#include <glog/logging.h>

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include "kimera-vio/visualizer/DisplayParams.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"  // Needed for shutdown cb

namespace VIO {

PangolinDisplay::PangolinDisplay(
    DisplayParams::Ptr display_params,
    const ShutdownPipelineCallback& shutdown_pipeline_cb)
    : DisplayBase(display_params->display_type_),
      handler(nullptr),
      shutdown_pipeline_cb_(shutdown_pipeline_cb) {
  CHECK(display_params);
  pangolin::CreateWindowAndBind(display_params->name_, 640, 480);
  glEnable(GL_DEPTH_TEST);

  // Define Projection and initial ModelView matrix
  s_cam = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
      pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));

  tree.Add(std::make_shared<pangolin::Axis>());

  // Create Interactive View in window
  handler = std::make_unique<pangolin::SceneHandler>(tree, s_cam);
  d_cam = pangolin::CreateDisplay()
              .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
              .SetHandler(handler.get());

  d_cam.SetDrawFunction([&](pangolin::View& view) {
    view.Activate(s_cam);
    tree.Render();
  });
}

void PangolinDisplay::spinOnce(
    DisplayInputBase::UniquePtr&& viz_input) {
  CHECK(viz_input);
  if (!pangolin::ShouldQuit()) {
    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);

    // Render OpenGL Cube
    pangolin::glDrawColouredCube();

    // Swap frames and Process Events
    pangolin::FinishFrame();
  } else {
    if (shutdown_pipeline_cb_) shutdown_pipeline_cb_();
  }
}

//! Adds 3D widgets to the window, and displays it.
void PangolinDisplay::spin3dWindow(VisualizerOutput::UniquePtr&& viz_output) {}

//! Visualizes 2D data.
void PangolinDisplay::spin2dWindow(const DisplayInputBase& viz_output) {}

}  // namespace VIO
