/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionImuTrackerParams.h
 * @brief  Class to parse, print, and store the parameters of the tracker.
 * @author Marcus Abate
 */

#pragma once

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/frontend/optical-flow/OpticalFlowPredictor-definitions.h"
#include "kimera-vio/pipeline/PipelineParams.h"

namespace VIO {

struct TrackerParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(TrackerParams);
  TrackerParams();

 public:
  void print() const;
  bool parseYAML(const std::string& filepath);
  bool equals(const TrackerParams& tp2, double tol = 1e-10) const;

 protected:
  virtual bool equals(const PipelineParams& obj) const {
    const auto& rhs = static_cast<const TrackerParams&>(obj);
    return equals(rhs);
  }

 public:
  // Tracking (Optical flow) params
  int klt_win_size_ = 24;  // size of the window
  int klt_max_iter_ = 30;  // max iterations
  int klt_max_level_ = 3;

  //! The desired accuracy for KLT
  double klt_eps_ = 0.01;

  //! We cut feature tracks longer than that
  size_t max_feature_track_age_ = 25;

  //! RANSAC parameters
  int minNrMonoInliers_ = 10;
  int minNrStereoInliers_ = 5;  // TODO should be size_t
  int min_pnp_inliers_ = 10;
  double ransac_threshold_mono_ = 1.0e-6;
  double ransac_threshold_stereo_ = 1.0;
  double ransac_threshold_pnp_ = 1.0;  //! Max reprojection error for inliers
  int ransac_max_iterations_ = 100;    // TODO (minor) : should we split this in
                                       // mono and stereo?
  double ransac_probability_ = 0.995;  // TODO (minor) : should we split this in
                                       // mono and stereo?
  bool ransac_randomize_ = true;
  bool ransac_use_1point_stereo_ = true;
  bool ransac_use_2point_mono_ = true;
  
  //! Use 2D-2D tracking to remove outliers
  Pose2d2dAlgorithm pose_2d2d_algorithm_ = Pose2d2dAlgorithm::NISTER;
  bool optimize_2d2d_pose_from_inliers_ = false;

  //! Use 3D-3D tracking to remove outliers
  bool optimize_3d3d_pose_from_inliers_ = false;

  //! PnP tracking parameters
  Pose3d2dAlgorithm pnp_algorithm_ = Pose3d2dAlgorithm::EPNP;
  bool optimize_2d3d_pose_from_inliers_ = false;

  //! Optical flow
  OpticalFlowPredictorType optical_flow_predictor_type_ =
      OpticalFlowPredictorType::kNoPrediction;

  //! Others:
  // max disparity under which we consider the vehicle steady
  double disparityThreshold_ = 0.5;
};

}  // namespace VIO
