/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DepthCameraParams.h
 * @brief  Parameters describing depth measurements from a depth camera
 * @author Nathan Hughes
 */

#pragma once
#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/YamlParser.h"

#include <gtsam/geometry/Pose3.h>

namespace VIO {

/**
 * @brief Necessary information turning depth measurements into a hallucinated
 * "right image"
 */
class DepthCameraParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(DepthCameraParams);

  DepthCameraParams() : PipelineParams("Depth Parameters") {}

  virtual ~DepthCameraParams() = default;

  /**
   * @brief parseYAML
   * Parse YAML file describing camera parameters.
   * @param filepath Path to the yaml file with the params
   * @return
   */
  bool parseYAML(const std::string& filepath) override;

  //! Display all params.
  void print() const override;

 protected:
  bool equals(const PipelineParams& rhs) const override;

 public:
  //! Virtual depth baseline: smaller means less disparity (i.e. more of the
  //! image can be used)
  float virtual_baseline_ = 1.0e-2f;

  //! Conversion factor between raw depth measurements and meters
  float depth_to_meters_ = 1.0f;

  //! Minimum depth to convert (out-of-frame features are rejected regardless)
  float min_depth_ = 0.0f;

  //! Maximum depth (for visualization only)
  float max_depth_ = 10.0f;

  bool is_registered_ = true;

  cv::Mat depth_camera_matrix_;

  gtsam::Pose3 T_color_depth_;
};

}  // namespace VIO
