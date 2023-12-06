/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OpenCvDisplay.h
 * @brief  Class to display visualizer output using OpenCV
 * @author Antoni Rosinol
 */

#pragma once

#include <opencv2/opencv.hpp>

#include "kimera-vio/pipeline/Pipeline-definitions.h"  // Needed for shutdown cb
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Display-definitions.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"
#include "kimera-vio/visualizer/OpenCvDisplayParams.h"

namespace VIO {

class OpenCv3dDisplay : public DisplayBase {
 public:
  KIMERA_POINTER_TYPEDEFS(OpenCv3dDisplay);
  KIMERA_DELETE_COPY_CONSTRUCTORS(OpenCv3dDisplay);

  OpenCv3dDisplay(DisplayParams::Ptr display_params,
                  const ShutdownPipelineCallback& shutdown_pipeline_cb);

  // TODO(Toni): consider using `unregisterAllWindows`
  ~OpenCv3dDisplay() override = default;

  /**
   * @brief spinOnce
   * Spins renderers to display data using OpenCV imshow and viz3d
   * Displaying must be done in the main thread, that it is why it is separated
   * from Visualizer3D (plus it makes everything faster as displaying and
   * building 3D graphics is decoupled).
   * @param viz_output
   */
  void spinOnce(DisplayInputBase::UniquePtr&& viz_output) override;

 private:
  //! Adds 3D widgets to the window, and displays it.
  void spin3dWindow(VisualizerOutput::UniquePtr&& viz_output);

  //! Visualizes 2D data.
  void spin2dWindow(const DisplayInputBase& viz_output);

  //! Set the pose of a 3D widget
  void setWidgetPose(const std::string& widget_id,
                     const cv::Affine3d& widget_pose);

  //! Sets the visualization properties of the 3D mesh.
  void setMeshProperties(WidgetsMap* widgets);

  //! Sets a 3D Widget Pose, because Widget3D::setPose() doesn't work;
  void setFrustumPose(const cv::Affine3d& frustum_pose);

  //! Remove widget. True if successful, false if not.
  bool removeWidget(const std::string& widget_id);

  //! Keyboard callback.
  static void keyboardCallback(const cv::viz::KeyboardEvent& event, void* t);

  //! Keyboard callback to toggle freezing screen.
  static void toggleFreezeScreenKeyboardCallback(const uchar& code,
                                                 WindowData* window_data);

  //! Keyboard callback to set mesh representation.
  static void setMeshRepresentation(const uchar& code, WindowData* window_data);

  //! Keyboard callback to set mesh shading.
  static void setMeshShadingCallback(const uchar& code,
                                     WindowData* window_data);

  //! Keyboard callback to set mesh ambient.
  static void setMeshAmbientCallback(const uchar& code,
                                     WindowData* window_data);

  //! Keyboard callback to set mesh lighting.
  static void setMeshLightingCallback(const uchar& code,
                                      WindowData* window_data);

  //! Keyboard callback to get current viewer pose.
  static void getViewerPoseKeyboardCallback(const uchar& code,
                                            WindowData* window_data);

  //! Keyboard callback to get current screen size.
  static void getCurrentWindowSizeKeyboardCallback(const uchar& code,
                                                   WindowData* window_data);

  //! Keyboard callback to get screenshot of current windodw.
  static void getScreenshotCallback(const uchar& code, WindowData* window_data);

  //! Record video sequence at a hardcoded directory relative to executable.
  void recordVideo();

  //! Useful for when testing on servers without display screen.
  void setOffScreenRendering();

 private:
  //! Data to visualize (in 3D)
  WindowData window_data_;

  //! We use this callback to shutdown the pipeline gracefully if
  //! the visualization window is closed.
  ShutdownPipelineCallback shutdown_pipeline_cb_;

  OpenCv3dDisplayParams params_;
};

}  // namespace VIO
