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

#include <opencv2/opencv.hpp>

namespace VIO {

/**
 * @brief The DisplayType enum: enumerates the types of supported renderers.
 */
enum class DisplayType {
  kOpenCV = 0,
};

/**
 * @brief The WindowData struct: Contains internal data for Visualizer3D window.
 */
struct WindowData {
 public:
  WindowData();
  ~WindowData();

 public:
  //! 3D visualization
  cv::viz::Viz3d window_;

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

}  // namespace VIO
