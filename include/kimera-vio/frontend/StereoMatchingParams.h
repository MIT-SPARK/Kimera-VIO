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

namespace VIO {

class StereoMatchingParams {
 public:
  double tolerance_template_matching_;
  double nominal_baseline_;
  int templ_cols_;         // must be odd
  int templ_rows_;         // must be odd
  int stripe_extra_rows_;  // must be even
  double min_point_dist_;  // stereo points triangulated below this distance are
                           // discarded
  double max_point_dist_;  // stereo points triangulated beyond this distance
                           // are discarded=
  bool bidirectional_matching_;  // check best match left->right and right->left
  bool subpixel_refinement_;     // refine stereo matches with subpixel accuracy
  bool equalize_image_;          // do equalize image before processing
  VisionSensorType vision_sensor_type_;  // options to use RGB-D vs. stereo
  double min_depth_factor_;      // min-depth to be used with RGB-D
  double map_depth_factor_;      // depth-map to be used with RGB-D

 public:
  StereoMatchingParams(
      double tol_template_matching = 0.15,
      int templ_cols = 101,
      int templ_rows = 11,
      int stripe_extra_rows = 0,
      double min_point_dist = 0.1,
      double max_point_dist = 15.0,
      bool bidirectional_matching = false,
      double nominal_baseline =
          0.11,  // NOTE that this is hard coded (for EuRoC)
      bool subpixel_refinement = false,
      bool equalize_image = false,
      VisionSensorType vision_sensor_type = VisionSensorType::STEREO,
      double min_depth_factor =
          0.3,  // NOTE that this is hard coded (for RealSense)
      double map_depth_factor = 0.001)
      :  // NOTE that this is hard coded (for RealSense)
        tolerance_template_matching_(std::move(tol_template_matching)),
        nominal_baseline_(std::move(nominal_baseline)),
        templ_cols_(std::move(templ_cols)),
        templ_rows_(std::move(templ_rows)),
        stripe_extra_rows_(std::move(stripe_extra_rows)),
        min_point_dist_(std::max(min_point_dist, 1e-3)),
        max_point_dist_(std::move(max_point_dist)),
        bidirectional_matching_(std::move(bidirectional_matching)),
        subpixel_refinement_(std::move(subpixel_refinement)),
        equalize_image_(std::move(equalize_image)),
        vision_sensor_type_(std::move(vision_sensor_type)),
        min_depth_factor_(std::move(min_depth_factor)),
        map_depth_factor_(std::move(map_depth_factor)) {
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
};

}  // namespace VIO
