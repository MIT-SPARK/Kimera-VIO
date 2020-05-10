/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Display.h
 * @brief  Class to display visualizer output
 * @author Antoni Rosinol
 */

#pragma once

#include <opencv2/opencv.hpp>

#include "kimera-vio/pipeline/Pipeline-definitions.h"  // Needed for shutdown cb
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Display-definitions.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

namespace VIO {

class DisplayBase {
 public:
  KIMERA_POINTER_TYPEDEFS(DisplayBase);
  KIMERA_DELETE_COPY_CONSTRUCTORS(DisplayBase);

  DisplayBase() = default;
  virtual ~DisplayBase() = default;

  /**
   * @brief spinOnce
   * Spins the display once to render the visualizer output.
   * @param viz_output Visualizer output, which is the display input.
   */
  virtual void spinOnce(DisplayInputBase::UniquePtr&& viz_output) = 0;
};

class OpenCv3dDisplay : public DisplayBase {
 public:
  KIMERA_POINTER_TYPEDEFS(OpenCv3dDisplay);
  KIMERA_DELETE_COPY_CONSTRUCTORS(OpenCv3dDisplay);

  OpenCv3dDisplay(const ShutdownPipelineCallback& shutdown_pipeline_cb);

  // TODO(Toni): consider using `unregisterAllWindows`
  ~OpenCv3dDisplay() override = default;

  // Spins renderers to display data using OpenCV imshow and viz3d
  // Displaying must be done in the main thread.
  void spinOnce(DisplayInputBase::UniquePtr&& viz_output) override;

 private:
  VisualizerOutput::UniquePtr safeCast(
      DisplayInputBase::UniquePtr display_input_base) {
    VisualizerOutput::UniquePtr viz_output;
    VisualizerOutput* tmp = nullptr;
    try {
      tmp = dynamic_cast<VisualizerOutput*>(display_input_base.get());
    } catch (const std::bad_cast& e) {
      LOG(ERROR) << "Seems that you are casting DisplayInputBase to "
                    "VisualizerOutput, but this object is not "
                    "a VisualizerOutput!";
      LOG(FATAL) << e.what();
    } catch (...) {
      LOG(FATAL) << "Exception caught when casting to VisualizerOutput.";
    }
    CHECK_NOTNULL(tmp);
    display_input_base.release();
    viz_output.reset(tmp);
    return viz_output;
  }

  // Adds 3D widgets to the window, and displays it.
  void spin3dWindow(VisualizerOutput::UniquePtr&& viz_output);

  void spin2dWindow(const VisualizerOutput& viz_output);

  //! Sets the visualization properties of the 3D mesh.
  void setMeshProperties(WidgetsMap* widgets);

  //! Sets a 3D Widget Pose, because Widget3D::setPose() doesn't work;
  void setFrustumPose(const cv::Affine3d& frustum_pose);

  // Keyboard callback.
  static void keyboardCallback(const cv::viz::KeyboardEvent& event, void* t);

  // Keyboard callback to toggle freezing screen.
  static void toggleFreezeScreenKeyboardCallback(const uchar& code,
                                                 WindowData* window_data);

  // Keyboard callback to set mesh representation.
  static void setMeshRepresentation(const uchar& code, WindowData* window_data);

  // Keyboard callback to set mesh shading.
  static void setMeshShadingCallback(const uchar& code,
                                     WindowData* window_data);

  // Keyboard callback to set mesh ambient.
  static void setMeshAmbientCallback(const uchar& code,
                                     WindowData* window_data);

  // Keyboard callback to set mesh lighting.
  static void setMeshLightingCallback(const uchar& code,
                                      WindowData* window_data);

  // Keyboard callback to get current viewer pose.
  static void getViewerPoseKeyboardCallback(const uchar& code,
                                            WindowData* window_data);

  // Keyboard callback to get current screen size.
  static void getCurrentWindowSizeKeyboardCallback(const uchar& code,
                                                   WindowData* window_data);

  // Keyboard callback to get screenshot of current windodw.
  static void getScreenshotCallback(const uchar& code, WindowData* window_data);

  // Record video sequence at a hardcoded directory relative to executable.
  void recordVideo();

  // Useful for when testing on servers without display screen.
  void setOffScreenRendering();

 private:
  WindowData window_data_;

  //! We use this callback to shutdown the pipeline gracefully if
  //! the visualization window is closed.
  ShutdownPipelineCallback shutdown_pipeline_cb_;
};

}  // namespace VIO
