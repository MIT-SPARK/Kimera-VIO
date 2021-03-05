/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OpenCvDisplayParams.h
 * @brief  Params for OpenCV display
 * @author Antoni Rosinol
 */

#pragma once

#include <opencv2/opencv.hpp>

#include "kimera-vio/pipeline/Pipeline-definitions.h"  // Needed for shutdown cb
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Display-definitions.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

namespace VIO {

class OpenCv3dDisplayParams : public DisplayParams {
 public:
  KIMERA_POINTER_TYPEDEFS(OpenCv3dDisplayParams);
  OpenCv3dDisplayParams();
  ~OpenCv3dDisplayParams() override = default;

  // Parse YAML file describing camera parameters.
  bool parseYAML(const std::string& filepath) override;

  // Display all params.
  void print() const override;

  // Assert equality up to a tolerance.
  bool equals(const OpenCv3dDisplayParams& cam_par,
              const double& tol = 1e-9) const;

 protected:
  inline bool equals(const DisplayParams& rhs,
                     const double& tol = 1e-9) const override {
    return equals(static_cast<const OpenCv3dDisplayParams&>(rhs), tol);
  }

 public:
  //! Spins the 3D window or 2D image display indefinitely, until user closes
  //! the window.
  bool hold_3d_display_ = false;
  bool hold_2d_display_ = false;
  cv::viz::Color background_color_ = cv::viz::Color::black();
};

}  // namespace VIO
