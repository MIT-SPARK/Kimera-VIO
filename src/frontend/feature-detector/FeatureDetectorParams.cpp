/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FeatureDetector-definitions.cpp
 * @brief  Parameters for feature detector
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/feature-detector/FeatureDetectorParams.h"
#include "kimera-vio/frontend/VisionFrontEndParams.h"
#include "kimera-vio/frontend/feature-detector/NonMaximumSuppression.h"
#include "kimera-vio/frontend/feature-detector/anms/anms.h"  // REMOVE

namespace VIO {

FeatureDetectorParams::FeatureDetectorParams()
    : PipelineParams("FeatureDetector Parameters") {
  print();
}

void FeatureDetectorParams::print() const {
  std::stringstream out;
  PipelineParams::print(out,
                        "Max features per frame",
                        max_features_per_frame_,
                        "Enable Subpixel corner refinement",
                        enable_subpixel_corner_refinement_,
                        "Enable Non-maximum suppression",
                        enable_non_max_suppression,
                        "Non-maximum suppression type",
                        VIO::to_underlying(non_max_suppression_type),
                        "Min dist btw tracked/detected features",
                        min_distance_btw_tracked_and_detected_features_);
  LOG(INFO) << out.str();
  if (enable_subpixel_corner_refinement_) {
    subpixel_corner_finder_params_.print();
  }
}
bool FeatureDetectorParams::parseYAML(const std::string& filepath) {}
bool FeatureDetectorParams::equals(const FeatureDetectorParams& tp2,
                                   double tol) const {}

}  // namespace VIO
