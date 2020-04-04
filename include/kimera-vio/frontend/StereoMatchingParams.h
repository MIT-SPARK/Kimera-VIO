/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoMatchingParams.h
 * @brief  Parameters for stereo matching.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/StereoFrame-definitions.h"
#include "kimera-vio/pipeline/PipelineParams.h"

namespace VIO {

class StereoMatchingParams : public PipelineParams {
 public:
  StereoMatchingParams();
  virtual ~StereoMatchingParams() = default;

  void print() const;
  bool parseYAML(const std::string& filepath);
  bool equals(const StereoMatchingParams& tp2, double tol = 1e-10) const;

 protected:
  // Parameters of the pipeline must specify how to be compared.
  virtual bool equals(const PipelineParams& obj) const {
    const auto& rhs = static_cast<const StereoMatchingParams&>(obj);
    return equals(rhs);
  }

 private:
  void checkParams();

 public:
  double tolerance_template_matching_ = 0.15;
  double nominal_baseline_ = 0.11;
  int templ_cols_ = 101;       // must be odd
  int templ_rows_ = 11;        // must be odd
  int stripe_extra_rows_ = 0;  // must be even
  // stereo points triangulated below this distance are discarded.
  double min_point_dist_ = 0.1;
  // stereo points triangulated beyond this distance are discarded.
  double max_point_dist_ = 15.0;
  // check best match left->right and right->left
  bool bidirectional_matching_ = false;
  // refine stereo matches with subpixel accuracy
  bool subpixel_refinement_ = false;
  // do equalize image before processing options to use RGB-D vs. stereo.
  bool equalize_image_ = false;
  VisionSensorType vision_sensor_type_ = VisionSensorType::STEREO;
  double min_depth_factor_ = 0.3;    // min-depth to be used with RGB-D
  double map_depth_factor_ = 0.001;  // depth-map to be used with RGB-D
};

}  // namespace VIO
