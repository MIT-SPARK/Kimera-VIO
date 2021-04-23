/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OpenCvDisplayParams.cpp
 * @brief  Params for the opencv display
 * @author Antoni Rosinol
 */

#include "kimera-vio/visualizer/OpenCvDisplayParams.h"

#include <glog/logging.h>

#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

OpenCv3dDisplayParams::OpenCv3dDisplayParams()
    : DisplayParams(DisplayType::kOpenCV) {
  CHECK(display_type_ == DisplayType::kOpenCV);
}

bool OpenCv3dDisplayParams::parseYAML(const std::string& filepath) {
  bool parent = DisplayParams::parseYAML(filepath);
  YamlParser yaml_parser(filepath);
  yaml_parser.getYamlParam("hold_3d_display", &hold_3d_display_);
  yaml_parser.getYamlParam("hold_2d_display", &hold_2d_display_);
  return true && parent;
}

void OpenCv3dDisplayParams::print() const {
  std::stringstream out;
  PipelineParams::print(out,
                        "Display Type ",
                        VIO::to_underlying(display_type_),
                        "Hold 2D Display ",
                        hold_2d_display_,
                        "Hold 3D Display ",
                        hold_3d_display_);
}

bool OpenCv3dDisplayParams::equals(const OpenCv3dDisplayParams& cam_par,
                                   const double& tol) const {
  return display_type_ == cam_par.display_type_ &&
         hold_2d_display_ == cam_par.hold_2d_display_ &&
         hold_3d_display_ == cam_par.hold_3d_display_;
}

}  // namespace VIO
