/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DisplayModule.cpp
 * @brief  Pipeline module to render/display 2D/3D visualization.
 * @author Antoni Rosinol
 */

#pragma once

#include <vector>

#include <glog/logging.h>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"
#include "kimera-vio/visualizer/Visualizer3D.h"

#include <opencv2/highgui/highgui_c.h>

namespace VIO {

// Contains internal data for Visualizer3D window.
struct WindowData {
 public:
  WindowData();
  ~WindowData();

 public:
  //! 3D visualization
  cv::viz::Viz3d window_;
  WidgetsMap widgets_;

  //! Colors
  cv::viz::Color background_color_;

  //! Stores the user set mesh representation.
  //! These objects are further modified by callbacks in the display module
  //! And are as well read by the visualizer module.
  int mesh_representation_;
  int mesh_shading_;
  bool mesh_ambient_;
  bool mesh_lighting_;
};

class DisplayBase {
 public:
  KIMERA_POINTER_TYPEDEFS(DisplayBase);
  KIMERA_DELETE_COPY_CONSTRUCTORS(DisplayBase);

  DisplayBase() = default;
  virtual ~DisplayBase() = default;

  // Spins the display once to render the visualizer output.
  virtual void spinOnce(const VisualizerOutput& viz_output) = 0;
};

class OpenCv3dDisplay : public DisplayBase {
 public:
  KIMERA_POINTER_TYPEDEFS(OpenCv3dDisplay);
  KIMERA_DELETE_COPY_CONSTRUCTORS(OpenCv3dDisplay);

  OpenCv3dDisplay();
  virtual ~OpenCv3dDisplay() override = default;

  // Spins renderers to display data using OpenCV imshow and viz3d
  // Displaying must be done in the main thread.
  void spinOnce(const VisualizerOutput& viz_output) override {
    // Display 3D window.
    spin3dWindow(viz_output);
    // Display 2D images.
    spin2dWindow(viz_output);
  }

  // Adds 3D widgets to the window, and displays it.
  void spin3dWindow(const VisualizerOutput& viz_output) {
    if (viz_output.visualization_type_ != VisualizationType::kNone) {
      CHECK(!window_data_.window_.wasStopped());
      // viz_output.window_->spinOnce(1, true);
      for (const auto& widget : viz_output.widgets_) {
        window_data_.window_.showWidget(widget.first, *(widget.second));
      }
      window_data_.window_.spinOnce(1, true);
    }
  }

  void spin2dWindow(const VisualizerOutput& viz_output) {
    // TODO(Toni): consider creating named window!
    for (const ImageToDisplay& img_to_display : viz_output.images_to_display_) {
      cv::imshow(img_to_display.name_, img_to_display.image_);
    }
    VLOG(10) << "Spin Visualize 2D output.";
    cv::waitKey(1);
  }

 private:
  // Keyboard callback.
  static void keyboardCallback(const cv::viz::KeyboardEvent& event, void* t);

  // Keyboard callback to toggle freezing screen.
  static void toggleFreezeScreenKeyboardCallback(const uchar code,
                                                 WindowData& window_data);

  // Keyboard callback to set mesh representation.
  static void setMeshRepresentation(const uchar code, WindowData& window_data);

  // Keyboard callback to set mesh shading.
  static void setMeshShadingCallback(const uchar code, WindowData& window_data);

  // Keyboard callback to set mesh ambient.
  static void setMeshAmbientCallback(const uchar code, WindowData& window_data);

  // Keyboard callback to set mesh lighting.
  static void setMeshLightingCallback(const uchar code,
                                      WindowData& window_data);

  // Keyboard callback to get current viewer pose.
  static void getViewerPoseKeyboardCallback(const uchar& code,
                                            WindowData& window_data);

  // Keyboard callback to get current screen size.
  static void getCurrentWindowSizeKeyboardCallback(const uchar code,
                                                   WindowData& window_data);

  // Keyboard callback to get screenshot of current windodw.
  static void getScreenshotCallback(const uchar code, WindowData& window_data);

  // Record video sequence at a hardcoded directory relative to executable.
  void recordVideo();

  // Useful for when testing on servers without display screen.
  void setOffScreenRendering();

 private:
  WindowData window_data_;
};

enum class DisplayType {
  kOpenCV = 0,
};

class DisplayFactory {
 public:
  KIMERA_POINTER_TYPEDEFS(DisplayFactory);
  KIMERA_DELETE_COPY_CONSTRUCTORS(DisplayFactory);

  DisplayFactory() = default;
  ~DisplayFactory() = default;

  static DisplayBase::UniquePtr makeDisplay(const DisplayType& display_type) {
    switch (display_type) {
      case DisplayType::kOpenCV: {
        return VIO::make_unique<OpenCv3dDisplay>();
      }
      default: {
        LOG(FATAL) << "Requested display type is not supported.\n"
                   << "Currently supported display types:\n"
                   << "0: OpenCV 3D viz\n 1: Pangolin (not supported yet)\n"
                   << " but requested display: "
                   << static_cast<int>(display_type);
      }
    }
  }
};

/**
 * @brief The DisplayModule class receives a VisualizerOutput payload which
 * contains images to be displayed and uses a display to render it.
 * Since displaying must be done in the main thread, the
 * DisplayModule should spin in the main thread to avoid errors.
 */
class DisplayModule
    : public SISOPipelineModule<VisualizerOutput, NullPipelinePayload> {
 public:
  KIMERA_POINTER_TYPEDEFS(DisplayModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(DisplayModule);

  using SISO = SISOPipelineModule<VisualizerOutput, NullPipelinePayload>;
  using InputQueue = ThreadsafeQueue<typename PIO::InputUniquePtr>;

  DisplayModule(InputQueue* input_queue,
                OutputQueue* output_queue,
                bool parallel_run,
                DisplayBase::UniquePtr&& display)
      : SISO(input_queue, output_queue, "Display", parallel_run),
        display_(std::move(display)) {}
  virtual ~DisplayModule() = default;

  virtual OutputUniquePtr spinOnce(VisualizerOutput::UniquePtr input) {
    CHECK(input);
    display_->spinOnce(*input);
    return VIO::make_unique<NullPipelinePayload>();
  }

 private:
  // The renderer used to display the visualizer output.
  DisplayBase::UniquePtr display_;
};

}  // namespace VIO
