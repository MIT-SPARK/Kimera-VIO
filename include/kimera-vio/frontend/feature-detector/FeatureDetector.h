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
// TODO(Toni): remove, use instead FeatureDetectorParams
#include "kimera-vio/frontend/VisionFrontEndParams.h"
#include "kimera-vio/frontend/feature-detector/NonMaximumSuppression.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class FeatureDetector {
 public:
  KIMERA_POINTER_TYPEDEFS(FeatureDetector);
  KIMERA_DELETE_COPY_CONSTRUCTORS(FeatureDetector);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FeatureDetector(const VisionFrontEndParams& tracker_params);
  virtual ~FeatureDetector() = default;

 public:
  void featureDetection(Frame* cur_frame);

 private:
  // Returns landmark_count (updated from the new keypoints),
  // and nr or extracted corners.
  KeypointsCV featureDetection(const Frame& cur_frame,
                               const int& need_n_corners);

  // TODO(TONI): should be detector params...
  // Tracker parameters.
  const VisionFrontEndParams tracker_params_;
  // TODO(TOni): should be debug feature detector info...
  // Debug info.
  DebugTrackerInfo debug_info_;

  // NonMaximum Suppresion Algorithm to have homogeneous feature distributions
  NonMaximumSuppression::UniquePtr non_max_suppression_;

  // Actual feature detector
  cv::Ptr<cv::Feature2D> feature_detector_;
};

}  // namespace VIO
