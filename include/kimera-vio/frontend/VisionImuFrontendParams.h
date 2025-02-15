/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionImuFrontendParams.h
 * @brief  Class to parse, print, and store the parameters of the frontend.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/frontend/VisionImuTrackerParams.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector-definitions.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetectorParams.h"
#include "kimera-vio/pipeline/PipelineParams.h"

namespace VIO {

struct FrontendParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(FrontendParams);
  FrontendParams();

 public:
  void print() const;

  bool parseYAML(const std::string& filepath);

  bool equals(const FrontendParams& tp2, double tol = 1e-10) const;

 protected:
  virtual bool equals(const PipelineParams& obj) const {
    const auto& rhs = static_cast<const FrontendParams&>(obj);
    return equals(rhs);
  }

 public:
  FeatureDetectorParams feature_detector_params_ = FeatureDetectorParams();
  TrackerParams tracker_params_ = TrackerParams();
  StereoMatchingParams stereo_matching_params_ = StereoMatchingParams();

  double min_intra_keyframe_time_ns_ = 0.2 * 10e6;
  double max_intra_keyframe_time_ns_ = 10.0 * 10e6;
  size_t min_number_features_ = 0u;

  //! If set to false, pipeline reduces to monocular tracking.
  bool use_stereo_tracking_ = true;
  double max_disparity_since_lkf_ = 200.0;
  //! Outlier rejection method choices
  bool useRANSAC_ = true;
  bool use_2d2d_tracking_ = true;
  bool use_3d3d_tracking_ = true;
  bool use_pnp_tracking_ = true;

  // These flags are parsed through gflags...
  //! Display feature tracks.
  bool visualize_feature_tracks_ = true;
  //! Display images in Frontend logger for debugging (only use
  //! if in sequential mode, otherwise expect segfaults).
  bool visualize_frontend_images_ = false;
  //! Save images in Frontend logger to disk for debugging (only use
  //! if in sequential mode, otherwise expect segfaults).
  bool save_frontend_images_ = false;
  //! Display/Save feature tracks images.
  bool log_feature_tracks_ = false;
  //! Display/Save mono tracking.
  bool log_mono_tracking_images_ = false;
  //! Display/Save stereo tracking rectified and unrectified images.
  bool log_stereo_matching_images_ = false;
};

}  // namespace VIO
