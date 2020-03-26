/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FeatureDetectorParams.h
 * @brief  Parameters for feature detection.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/VisionFrontEndParams.h"
#include "kimera-vio/frontend/feature-detector/NonMaximumSuppression.h"

namespace VIO {

struct FeatureDetectorParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(FeatureDetectorParams);
  FeatureDetectorParams();
  virtual ~FeatureDetectorParams() = default;

 public:
  void print() const;
  bool parseYAML(const std::string& filepath);
  bool equals(const FeatureDetectorParams& tp2, double tol = 1e-10) const;

 protected:
  // Parameters of the pipeline must specify how to be compared.
  virtual bool equals(const PipelineParams& obj) const {
    const auto& rhs = static_cast<const FeatureDetectorParams&>(obj);
    return equals(rhs);
  }

 public:
  //! Maximum amount of features to be detected per frame.
  int max_features_per_frame_ = 400;
  //! Whether to enable subpixel feature detection.
  bool enable_subpixel_corner_refinement_ = true;
  //! Parameters for subpixel refinement in case it is enabled.
  SubPixelCornerFinderParams subpixel_corner_finder_params_;
  //! Whether to enable non maximum suppression after feature detection.
  bool enable_non_max_suppression = true;
  AnmsAlgorithmType non_max_suppression_type = AnmsAlgorithmType::RangeTree;
  //! Minimum distance between the already tracked features and the new
  //! features to be detected. This avoids detections near tracked features.
  int min_distance_btw_tracked_and_detected_features_ = 10;
};

}  // namespace VIO
