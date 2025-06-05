/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GnssStereoVisionImuFrontend-definitions.h
 * @brief  Definitions for GnssStereoVisionImuFrontend
 * @author Elizaveta Karaseva
 */

#pragma once

#include <optional>
#include <vector>  // for vector<>

#include "kimera-vio/frontend/Gnss.h"
#include "kimera-vio/frontend/StereoVisionImuFrontend-definitions.h"

namespace VIO {

struct GnssStereoFrontendOutput : public VIO::StereoFrontendOutput {
 public:
  KIMERA_POINTER_TYPEDEFS(GnssStereoFrontendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(GnssStereoFrontendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GnssStereoFrontendOutput(
      const bool is_keyframe,
      const StatusStereoMeasurementsPtr& status_stereo_measurements,
      const gtsam::Pose3& b_Pose_camL_rect,
      const gtsam::Pose3& b_Pose_camR_rect,
      const StereoFrame& stereo_frame_lkf,
      const ImuFrontend::PimPtr& pim,
      const ImuAccGyrS& imu_acc_gyrs,
      const cv::Mat& feature_tracks,
      const DebugTrackerInfo& debug_tracker_info,
      std::optional<gtsam::Pose3> lkf_body_Pose_kf_body = std::nullopt,
      std::optional<gtsam::Velocity3> body_world_Vel_body = std::nullopt,
      std::optional<GnssPoint> gnss_point = std::nullopt)
      : StereoFrontendOutput(is_keyframe,
                             status_stereo_measurements,
                             b_Pose_camL_rect,
                             b_Pose_camR_rect,
                             stereo_frame_lkf,
                             pim,
                             imu_acc_gyrs,
                             feature_tracks,
                             debug_tracker_info),
        gnss_point_(gnss_point) {}

  virtual ~GnssStereoFrontendOutput() = default;

 public:
  std::optional<GnssPoint> gnss_point_;
};

}  // namespace VIO
