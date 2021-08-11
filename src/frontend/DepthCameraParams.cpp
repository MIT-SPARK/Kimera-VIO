/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DepthCameraParams.cpp
 * @brief  Parameters describing depth measurements from a depth camera
 * @author Nathan Hughes
 */

#include "kimera-vio/frontend/DepthCameraParams.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace VIO {

bool DepthCameraParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);

  yaml_parser.getYamlParam("virtual_baseline", &virtual_baseline_);
  CHECK_GT(virtual_baseline_, 0.0) << "Baseline must be positive";
  yaml_parser.getYamlParam("depth_to_meters", &depth_to_meters_);
  yaml_parser.getYamlParam("min_depth", &min_depth_);
  yaml_parser.getYamlParam("max_depth", &max_depth_);
  yaml_parser.getYamlParam("is_registered", &is_registered_);
  if (!is_registered_) {
    std::vector<double> pose_elements;
    yaml_parser.getNestedYamlParam("T_color_depth", "data", &pose_elements);
    T_color_depth_ = UtilsOpenCV::poseVectorToGtsamPose3(pose_elements);

    std::vector<double> intrinsics;
    yaml_parser.getYamlParam("intrinsics", &intrinsics);
    depth_camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    depth_camera_matrix_.at<double>(0, 0) = intrinsics[0];
    depth_camera_matrix_.at<double>(1, 1) = intrinsics[1];
    depth_camera_matrix_.at<double>(0, 2) = intrinsics[2];
    depth_camera_matrix_.at<double>(1, 2) = intrinsics[3];
  }
  return true;
}

void DepthCameraParams::print() const {
  std::stringstream out;
  PipelineParams::print(out,
                        "virtual_baseline",
                        virtual_baseline_,
                        "depth_to_meters",
                        depth_to_meters_,
                        "min_depth",
                        min_depth_,
                        "max_depth",
                        max_depth_);
  LOG(INFO) << out.str();
}

bool DepthCameraParams::equals(const PipelineParams& base_params) const {
  try {
    const DepthCameraParams& params =
        dynamic_cast<const DepthCameraParams&>(base_params);
    return virtual_baseline_ == params.virtual_baseline_ &&
           depth_to_meters_ == params.depth_to_meters_ &&
           min_depth_ == params.min_depth_ && max_depth_ == params.max_depth_;
  } catch (const std::bad_cast&) {
    return false;
  }
}

}  // namespace VIO
