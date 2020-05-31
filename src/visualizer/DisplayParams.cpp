/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DisplayParams.cpp
 * @brief  Parameters describing a display.
 * @author Antoni Rosinol
 */

#include "kimera-vio/visualizer/DisplayParams.h"

#include <fstream>
#include <iostream>

#include "kimera-vio/utils/YamlParser.h"
#include "kimera-vio/visualizer/Display-definitions.h"

namespace VIO {

DisplayParams::DisplayParams()
    : PipelineParams("Display Parameters"),
      display_type_(DisplayType::kOpenCV) {}

// Parse YAML file describing camera parameters.
bool DisplayParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);

  int display_type;
  yaml_parser.getYamlParam("display_type", &display_type);
  display_type_ = static_cast<DisplayType>(display_type);

  return true;
}

// Display all params.
void DisplayParams::print() const {
  std::stringstream out;
  PipelineParams::print(
      out, "Display Type ", VIO::to_underlying(display_type_));
}

// Assert equality up to a tolerance.
bool DisplayParams::equals(const DisplayParams& cam_par,
                           const double& tol) const {
  return display_type_ == cam_par.display_type_;
}

}  // namespace VIO
