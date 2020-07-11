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

#include "kimera-vio/frontend/feature-detector/FeatureDetector-definitions.h"
#include "kimera-vio/frontend/feature-detector/NonMaximumSuppression.h"
#include "kimera-vio/pipeline/PipelineParams.h"

namespace VIO {

struct SubPixelCornerFinderParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(SubPixelCornerFinderParams);
  SubPixelCornerFinderParams();
  virtual ~SubPixelCornerFinderParams() = default;

 public:
  void print() const;
  bool parseYAML(const std::string& filepath);
  bool equals(const SubPixelCornerFinderParams& tp2, double tol = 1e-10) const;

 protected:
  // Parameters of the pipeline must specify how to be compared.
  virtual bool equals(const PipelineParams& obj) const {
    const auto& rhs = static_cast<const SubPixelCornerFinderParams&>(obj);
    return equals(rhs);
  }

 public:
  /// Termination criteria defined in terms of change in error and maximum
  /// number of iterations
  cv::TermCriteria term_criteria_ =
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                       10,
                       0.01);
  cv::Size window_size_ = cv::Size(10, 10);
  cv::Size zero_zone_ = cv::Size(-1, -1);
};


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
  FeatureDetectorType feature_detector_type_ = FeatureDetectorType::GFTT;
  //! Maximum amount of features to be detected per frame.
  int max_features_per_frame_ = 400;
  //! Whether to enable subpixel feature detection.
  bool enable_subpixel_corner_refinement_ = true;
  //! Parameters for subpixel refinement in case it is enabled.
  SubPixelCornerFinderParams subpixel_corner_finder_params_ =
      SubPixelCornerFinderParams();
  //! Whether to enable non maximum suppression after feature detection.
  bool enable_non_max_suppression_ = true;
  AnmsAlgorithmType non_max_suppression_type_ = AnmsAlgorithmType::RangeTree;
  //! Minimum distance between the already tracked features and the new
  //! features to be detected. This avoids detections near tracked features.
  int min_distance_btw_tracked_and_detected_features_ = 10;

  // GFTT specific parameters
  // TODO(Toni): add comments on each parameter
  double quality_level_ = 0.001;
  int block_size_ = 3;
  bool use_harris_corner_detector_ = false;
  double k_ = 0.04;

  // FAST specific params
  int fast_thresh_ = 10;
};

}  // namespace VIO
