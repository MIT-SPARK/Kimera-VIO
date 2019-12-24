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

#include <glog/logging.h>

#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

class StereoMatchingParams {
 public:
  StereoMatchingParams() { checkParams(); }
  StereoMatchingParams(const double& tol_template_matching,
                       const int& templ_cols,
                       const int& templ_rows,
                       const int& stripe_extra_rows,
                       const double& min_point_dist,
                       const double& max_point_dist,
                       const bool& bidirectional_matching,
                       // NOTE that this is hard coded (for EuRoC)
                       const double& nominal_baseline,
                       const bool& subpixel_refinement,
                       const bool& equalize_image,
                       const VisionSensorType& vision_sensor_type,
                       // NOTE that this is hard coded (for RealSense)
                       const double& min_depth_factor,
                       const double& map_depth_factor)
      :  // NOTE that this is hard coded (for RealSense)
        tolerance_template_matching_(tol_template_matching),
        nominal_baseline_(nominal_baseline),
        templ_cols_(templ_cols),
        templ_rows_(templ_rows),
        stripe_extra_rows_(stripe_extra_rows),
        min_point_dist_(std::max(min_point_dist, 1e-3)),
        max_point_dist_(max_point_dist),
        bidirectional_matching_(bidirectional_matching),
        subpixel_refinement_(subpixel_refinement),
        equalize_image_(equalize_image),
        vision_sensor_type_(vision_sensor_type),
        min_depth_factor_(min_depth_factor),
        map_depth_factor_(map_depth_factor) {
    checkParams();
  }

  void checkParams() {
    CHECK(!(templ_cols_ % 2 != 1 ||
            templ_rows_ % 2 != 1))  // check that they are odd
        << "StereoMatchingParams: template size must be odd!";
    CHECK(!(stripe_extra_rows_ % 2 != 0))  // check that they are even
        << "StereoMatchingParams: stripe_extra_rows size must be even!";
  }

  /* ------------------------------------------------------------------------ */
  bool equals(const StereoMatchingParams& tp2, double tol = 1e-10) const {
    return (fabs(nominal_baseline_ - tp2.nominal_baseline_) <= tol) &&
           (equalize_image_ == tp2.equalize_image_) &&
           (fabs(tolerance_template_matching_ -
                 tp2.tolerance_template_matching_) <= tol) &&
           (templ_cols_ == tp2.templ_cols_) &&
           (templ_rows_ == tp2.templ_rows_) &&
           (stripe_extra_rows_ == tp2.stripe_extra_rows_) &&
           (fabs(min_point_dist_ - tp2.min_point_dist_) <= tol) &&
           (fabs(max_point_dist_ - tp2.max_point_dist_) <= tol) &&
           (bidirectional_matching_ == tp2.bidirectional_matching_) &&
           (subpixel_refinement_ == tp2.subpixel_refinement_) &&
           (vision_sensor_type_ == tp2.vision_sensor_type_);
  }

  void print() const {
    LOG(INFO) << "Using vision_sensor_type_: "
              << to_underlying(vision_sensor_type_);
    LOG(INFO) << "** Sparse Stereo Matching parameters **\n"
              << "equalize_image_: " << equalize_image_ << '\n'
              << "nominalBaseline_: " << nominal_baseline_ << '\n'
              << "toleranceTemplateMatching_: " << tolerance_template_matching_
              << '\n'
              << "templ_cols_: " << templ_cols_ << '\n'
              << "templ_rows_: " << templ_rows_ << '\n'
              << "stripe_extra_rows_: " << stripe_extra_rows_ << '\n'
              << "minPointDist_: " << min_point_dist_ << '\n'
              << "maxPointDist_: " << max_point_dist_ << '\n'
              << "bidirectionalMatching_: " << bidirectional_matching_ << '\n'
              << "subpixelRefinementStereo_: " << subpixel_refinement_;
    if (vision_sensor_type_ == VisionSensorType::RGBD) {
      LOG(INFO) << "minDepthFactor_: " << min_depth_factor_ << '\n'
                << "mapDepthFactor_: " << map_depth_factor_;
    }
  }

  bool parseYAML(const std::string& filepath) {
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
