/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FeatureDetector.h
 * @brief  Base class for feature detector interface
 * @author Antoni Rosinol
 */

#pragma once

#include <Eigen/Eigen>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/VisionFrontEndParams.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector-definitions.h"
#include "kimera-vio/frontend/feature-detector/NonMaximumSuppression.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

// TODO(Toni): put in feature detector params!
struct FeatureDetectorParams {
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

class FeatureDetector {
 public:
  KIMERA_POINTER_TYPEDEFS(FeatureDetector);
  KIMERA_DELETE_COPY_CONSTRUCTORS(FeatureDetector);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FeatureDetector(const FeatureDetectorType& feature_detector_type,
                  const FeatureDetectorParams& feature_detector_params);
  virtual ~FeatureDetector() = default;

 public:
  void featureDetection(Frame* cur_frame);

 private:
  // Returns landmark_count (updated from the new keypoints),
  // and nr or extracted corners.
  KeypointsCV featureDetection(const Frame& cur_frame,
                               const int& need_n_corners);

  // Parameters.
  const FeatureDetectorType feature_detector_type_;
  const FeatureDetectorParams feature_detector_params_;

  // TODO(TOni): should be debug feature detector info...
  // Debug info.
  // DebugTrackerInfo debug_info_;

  // NonMaximum Suppresion Algorithm to have homogeneous feature distributions
  NonMaximumSuppression::UniquePtr non_max_suppression_;

  // Actual feature detector implementation.
  cv::Ptr<cv::Feature2D> feature_detector_;
};

}  // namespace VIO
