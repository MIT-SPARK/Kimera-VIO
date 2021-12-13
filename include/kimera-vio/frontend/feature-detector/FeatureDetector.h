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
#include <opencv2/features2d.hpp>
#include <vector>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector-definitions.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetectorParams.h"
#include "kimera-vio/frontend/feature-detector/NonMaximumSuppression.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {
class FeatureDetector {
 public:
  KIMERA_POINTER_TYPEDEFS(FeatureDetector);
  KIMERA_DELETE_COPY_CONSTRUCTORS(FeatureDetector);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FeatureDetector(const FeatureDetectorParams& feature_detector_params);
  virtual ~FeatureDetector() = default;

 public:
  void featureDetection(Frame* cur_frame,
                        boost::optional<cv::Mat> R = boost::none);

  /**
   * @brief rawFeatureDetection Raw feature detection: in image, out keypoints
   * @param img
   * @return keypoints
   */
  std::vector<cv::KeyPoint> rawFeatureDetection(
      const cv::Mat& img,
      const cv::Mat& mask = cv::Mat());

 private:
  // Returns landmark_count (updated from the new keypoints),
  // and nr or extracted corners.
  KeypointsCV featureDetection(const Frame& cur_frame,
                               const int& need_n_corners);

  // Parameters.
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
