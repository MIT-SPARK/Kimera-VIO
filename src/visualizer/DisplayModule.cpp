/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DisplayModule.cpp
 * @brief  Pipeline module to render/display 2D/3D data.
 * @author Antoni Rosinol
 */

#include "kimera-vio/visualizer/DisplayModule.h"

#include <gflags/gflags.h>

#include "kimera-vio/common/FilesystemUtils.h"

DEFINE_int32(mesh_shading, 0, "Mesh shading:\n 0: Flat, 1: Gouraud, 2: Phong");
DEFINE_int32(mesh_representation,
             1,
             "Mesh representation:\n 0: Points, 1: Surface, 2: Wireframe");
DEFINE_bool(set_mesh_ambient,
            false,
            "Whether to use ambient light for the "
            "mesh.");
DEFINE_bool(set_mesh_lighting, true, "Whether to use lighting for the mesh.");

namespace VIO {

// Contains internal data for Visualizer3D window.
WindowData::WindowData()
    : window_(cv::viz::Viz3d("3D Visualizer")),
      background_color_(cv::viz::Color::black()),
      mesh_representation_(FLAGS_mesh_representation),
      mesh_shading_(FLAGS_mesh_shading),
      mesh_ambient_(FLAGS_set_mesh_ambient),
      mesh_lighting_(FLAGS_set_mesh_lighting) {}

WindowData::~WindowData() {
  // OpenCV 3d Viz has an issue that I can't resolve, it throws a segfault
  // at the end of the program. Probably because of memory not released.
  // See issues in opencv git:
  // https://github.com/opencv/opencv/issues/11219 and many more...
  window_.close();
}

OpenCv3dDisplay::OpenCv3dDisplay() : DisplayBase() {
  if (VLOG_IS_ON(2)) {
    window_data_.window_.setGlobalWarnings(true);
  } else {
    window_data_.window_.setGlobalWarnings(false);
  }
  window_data_.window_.registerKeyboardCallback(keyboardCallback,
                                                &window_data_);
  window_data_.window_.setBackgroundColor(window_data_.background_color_);
  window_data_.window_.showWidget("Coordinate Widget",
                                  cv::viz::WCoordinateSystem());
}

void OpenCv3dDisplay::keyboardCallback(const cv::viz::KeyboardEvent& event,
                                       void* t) {
  WindowData* window_data = static_cast<WindowData*>(t);
  if (event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN) {
    toggleFreezeScreenKeyboardCallback(event.code, *window_data);
    setMeshRepresentation(event.code, *window_data);
    setMeshShadingCallback(event.code, *window_data);
    setMeshAmbientCallback(event.code, *window_data);
    setMeshLightingCallback(event.code, *window_data);
    getViewerPoseKeyboardCallback(event.code, *window_data);
    getCurrentWindowSizeKeyboardCallback(event.code, *window_data);
    getScreenshotCallback(event.code, *window_data);
  }
}

// Keyboard callback to toggle freezing screen.
void OpenCv3dDisplay::toggleFreezeScreenKeyboardCallback(
    const uchar code,
    WindowData& window_data) {
  if (code == 't') {
    LOG(WARNING) << "Pressing " << code << " toggles freezing screen.";
    bool freeze = false;
    freeze = !freeze;  // Toggle.
    window_data.window_.spinOnce(1, true);
    while (!window_data.window_.wasStopped()) {
      if (freeze) {
        window_data.window_.spinOnce(1, true);
      } else {
        break;
      }
    }
  }
}

// Keyboard callback to set mesh representation.
void OpenCv3dDisplay::setMeshRepresentation(const uchar code,
                                            WindowData& window_data) {
  if (code == '0') {
    LOG(WARNING) << "Pressing " << code
                 << " sets mesh representation to "
                    "a point cloud.";
    window_data.mesh_representation_ = 0u;
  } else if (code == '1') {
    LOG(WARNING) << "Pressing " << code
                 << " sets mesh representation to "
                    "a mesh.";
    window_data.mesh_representation_ = 1u;
  } else if (code == '2') {
    LOG(WARNING) << "Pressing " << code
                 << " sets mesh representation to "
                    "a wireframe.";
    window_data.mesh_representation_ = 2u;
  }
}

// Keyboard callback to set mesh shading.
void OpenCv3dDisplay::setMeshShadingCallback(const uchar code,
                                             WindowData& window_data) {
  if (code == '4') {
    LOG(WARNING) << "Pressing " << code
                 << " sets mesh shading to "
                    "flat.";
    window_data.mesh_shading_ = 0u;
  } else if (code == '5') {
    LOG(WARNING) << "Pressing " << code
                 << " sets mesh shading to "
                    "Gouraud.";
    window_data.mesh_shading_ = 1u;
  } else if (code == '6') {
    LOG(WARNING) << "Pressing " << code
                 << " sets mesh shading to "
                    "Phong.";
    window_data.mesh_shading_ = 2u;
  }
}

// Keyboard callback to set mesh ambient.
void OpenCv3dDisplay::setMeshAmbientCallback(const uchar code,
                                             WindowData& window_data) {
  if (code == 'a') {
    window_data.mesh_ambient_ = !window_data.mesh_ambient_;
    LOG(WARNING) << "Pressing " << code << " toggles mesh ambient."
                 << " Now set to " << window_data.mesh_ambient_;
  }
}

// Keyboard callback to set mesh lighting.
void OpenCv3dDisplay::setMeshLightingCallback(const uchar code,
                                              WindowData& window_data) {
  if (code == 'l') {
    window_data.mesh_lighting_ = !window_data.mesh_lighting_;
    LOG(WARNING) << "Pressing " << code << " toggles mesh lighting."
                 << " Now set to " << window_data.mesh_lighting_;
  }
}

// Keyboard callback to get current viewer pose.
void OpenCv3dDisplay::getViewerPoseKeyboardCallback(const uchar& code,
                                                    WindowData& window_data) {
  if (code == 'v') {
    LOG(INFO) << "Current viewer pose:\n"
              << "\tRodriguez vector: "
              << window_data.window_.getViewerPose().rvec()
              << "\n\tAffine matrix: "
              << window_data.window_.getViewerPose().matrix;
  }
}

// Keyboard callback to get current screen size.
void OpenCv3dDisplay::getCurrentWindowSizeKeyboardCallback(
    const uchar code,
    WindowData& window_data) {
  if (code == 'w') {
    LOG(WARNING) << "Pressing " << code << " displays current window size:\n"
                 << "\theight: " << window_data.window_.getWindowSize().height
                 << "\twidth: " << window_data.window_.getWindowSize().width;
  }
}

// Keyboard callback to get screenshot of current windodw.
void OpenCv3dDisplay::getScreenshotCallback(const uchar code,
                                            WindowData& window_data) {
  if (code == 's') {
    int i = 0;
    std::string filename = "screenshot_3d_window" + std::to_string(i);
    LOG(WARNING) << "Pressing " << code
                 << " takes a screenshot of the "
                    "window, saved in: " +
                        filename;
    window_data.window_.saveScreenshot(filename);
  }
}

// Record video sequence at a hardcoded directory relative to executable.
void OpenCv3dDisplay::recordVideo() {
  int i = 0u;
  const std::string dir_path = ".";
  const std::string dir_name = "3d_viz_video";
  const std::string dir_full_path = common::pathAppend(dir_path, dir_name);
  if (i == 0u) CHECK(common::createDirectory(dir_path, dir_name));
  std::string screenshot_path =
      common::pathAppend(dir_full_path, std::to_string(i));
  i++;
  LOG(WARNING) << "Recording video sequence for 3d Viz, "
               << "current frame saved in: " + screenshot_path;
  // window_data_.window_.saveScreenshot(screenshot_path);
  LOG(ERROR) << "WTF";
}

void OpenCv3dDisplay::setOffScreenRendering() {
  window_data_.window_.setOffScreenRendering();
}

}  // namespace VIO
