/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoMatchingParams.cpp
 * @brief  Parameters for stereo matching.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/StereoMatchingParams.h"

#include <glog/logging.h>

#include "kimera-vio/frontend/StereoFrame-definitions.h"
#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

StereoMatchingParams::StereoMatchingParams()
    : PipelineParams("Stereo Matching Parameters") {
  checkParams();
}

void StereoMatchingParams::checkParams() {
  CHECK(!(templ_cols_ % 2 != 1 ||
          templ_rows_ % 2 != 1))  // check that they are odd
      << "StereoMatchingParams: template size must be odd!";
  CHECK(!(stripe_extra_rows_ % 2 != 0))  // check that they are even
      << "StereoMatchingParams: stripe_extra_rows size must be even!";
}

bool StereoMatchingParams::equals(const StereoMatchingParams& tp2,
                                  double tol) const {
  return (fabs(nominal_baseline_ - tp2.nominal_baseline_) <= tol) &&
         (equalize_image_ == tp2.equalize_image_) &&
         (fabs(tolerance_template_matching_ -
               tp2.tolerance_template_matching_) <= tol) &&
         (templ_cols_ == tp2.templ_cols_) && (templ_rows_ == tp2.templ_rows_) &&
         (stripe_extra_rows_ == tp2.stripe_extra_rows_) &&
         (fabs(min_point_dist_ - tp2.min_point_dist_) <= tol) &&
         (fabs(max_point_dist_ - tp2.max_point_dist_) <= tol) &&
         (bidirectional_matching_ == tp2.bidirectional_matching_) &&
         (subpixel_refinement_ == tp2.subpixel_refinement_) &&
         (vision_sensor_type_ == tp2.vision_sensor_type_);
}

void StereoMatchingParams::print() const {
  std::stringstream out;
  PipelineParams::print(out,
                        "equalize_image_: ",
                        equalize_image_,
                        "nominalBaseline_: ",
                        nominal_baseline_,
                        "toleranceTemplateMatching_: ",
                        tolerance_template_matching_,
                        "templ_cols_: ",
                        templ_cols_,
                        "templ_rows_: ",
                        templ_rows_,
                        "stripe_extra_rows_: ",
                        stripe_extra_rows_,
                        "minPointDist_: ",
                        min_point_dist_,
                        "maxPointDist_: ",
                        max_point_dist_,
                        "bidirectionalMatching_: ",
                        bidirectional_matching_,
                        "subpixelRefinementStereo_: ",
                        subpixel_refinement_);
  LOG(INFO) << out.str();

  LOG(INFO) << "Using vision_sensor_type_: "
            << to_underlying(vision_sensor_type_);
  if (vision_sensor_type_ == VisionSensorType::RGBD) {
    LOG(INFO) << "minDepthFactor_: " << min_depth_factor_ << '\n'
              << "mapDepthFactor_: " << map_depth_factor_;
  }
}

bool StereoMatchingParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);
  yaml_parser.getYamlParam("equalizeImage", &equalize_image_);
  yaml_parser.getYamlParam("nominalBaseline", &nominal_baseline_);
  yaml_parser.getYamlParam("toleranceTemplateMatching",
                           &tolerance_template_matching_);
  yaml_parser.getYamlParam("templ_cols", &templ_cols_);
  yaml_parser.getYamlParam("templ_rows", &templ_rows_);
  yaml_parser.getYamlParam("stripe_extra_rows", &stripe_extra_rows_);
  yaml_parser.getYamlParam("minPointDist", &min_point_dist_);
  yaml_parser.getYamlParam("maxPointDist", &max_point_dist_);
  yaml_parser.getYamlParam("bidirectionalMatching", &bidirectional_matching_);
  yaml_parser.getYamlParam("subpixelRefinementStereo", &subpixel_refinement_);
  return true;
}

}  // namespace VIO
