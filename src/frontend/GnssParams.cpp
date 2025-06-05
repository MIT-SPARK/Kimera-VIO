/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GnssParams.cpp
 * @brief  Params for GnssStereoVisionImuFrontend.
 * @author Elizaveta Karaseva
 */

#include "kimera-vio/frontend/GnssParams.h"

#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>

#include <string>
#include <vector>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/utils/UtilsOpenCV.h"
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

GnssParams::GnssParams() : PipelineParams("GNSS params") {}

bool GnssParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);
  int n_rows = 0;
  yaml_parser.getNestedYamlParam("T_BS", "rows", &n_rows);
  CHECK_EQ(n_rows, 4u);
  int n_cols = 0;
  yaml_parser.getNestedYamlParam("T_BS", "cols", &n_cols);
  CHECK_EQ(n_cols, 4u);
  std::vector<double> vector_pose;
  yaml_parser.getNestedYamlParam("T_BS", "data", &vector_pose);
  b_pose_gnss_ = UtilsOpenCV::poseVectorToGtsamPose3(vector_pose);
  yaml_parser.getYamlParam("timestamp", &timestamp_);
  return true;
}

void GnssParams::print() const {
  std::stringstream out;
  PipelineParams::print(out, "b_pose_: ", b_pose_gnss_);
  LOG(INFO) << out.str();
}

bool GnssParams::equals(const PipelineParams& obj) const {
  const auto& rhs = static_cast<const GnssParams&>(obj);
  // clang-format off
  return b_pose_gnss_.equals(rhs.b_pose_gnss_) &&
      timestamp_ == rhs.timestamp_;
}

}  // namespace VIO
