/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVisionImuFrontend-definitions.h
 * @brief  Definitions for StereoVisionImuFrontend
 * @author Antoni Rosinol
 */

#pragma once

#include <optional>

#include "kimera-vio/frontend/FrontendOutputPacketBase.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/VisionImuFrontend-definitions.h"

namespace VIO {

struct StereoFrontendOutput : public FrontendOutputPacketBase {
 public:
  KIMERA_POINTER_TYPEDEFS(StereoFrontendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoFrontendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StereoFrontendOutput(
      const bool is_keyframe,
      const StatusStereoMeasurementsPtr& status_stereo_measurements,
      const gtsam::Pose3& b_Pose_camL_rect,
      const gtsam::Pose3& b_Pose_camR_rect,
      const StereoFrame& stereo_frame_lkf,
      // Use rvalue reference: FrontendOutput owns pim now.
      const ImuFrontend::PimPtr& pim,
      const ImuAccGyrS& imu_acc_gyrs,
      const cv::Mat& feature_tracks,
      const DebugTrackerInfo& debug_tracker_info,
      std::optional<gtsam::Pose3> lkf_body_Pose_kf_body = std::nullopt,
      std::optional<gtsam::Velocity3> body_world_Vel_body = std::nullopt)
      : FrontendOutputPacketBase(stereo_frame_lkf.timestamp_,
                                 is_keyframe,
                                 FrontendType::kStereoImu,
                                 pim,
                                 imu_acc_gyrs,
                                 debug_tracker_info,
                                 lkf_body_Pose_kf_body,
                                 body_world_Vel_body),
        status_stereo_measurements_(status_stereo_measurements),
        b_Pose_camL_rect_(b_Pose_camL_rect),
        b_Pose_camR_rect_(b_Pose_camR_rect),
        stereo_frame_lkf_(stereo_frame_lkf),
        feature_tracks_(feature_tracks) {}

  virtual ~StereoFrontendOutput() = default;

  virtual const Frame* getTrackingFrame() const override {
    return &stereo_frame_lkf_.left_frame_;
  }

  virtual const cv::Mat* getTrackingImage() const override {
    return &feature_tracks_;
  }

  virtual const gtsam::Pose3* getBodyPoseCam() const override {
    return &b_Pose_camL_rect_;
  }

  virtual const gtsam::Pose3* getBodyPoseCamRight() const override {
    return &b_Pose_camR_rect_;
  }

  virtual const TrackerStatusSummary* getTrackerStatus() const override {
    return status_stereo_measurements_ ? &(status_stereo_measurements_->first)
                                       : nullptr;
  }

 public:
  const StatusStereoMeasurementsPtr status_stereo_measurements_;
  const gtsam::Pose3 b_Pose_camL_rect_;
  const gtsam::Pose3 b_Pose_camR_rect_;
  // TODO(nathan) make this name consistent
  /**
   * This member is not necessarily a key-frame and can be one of two things:
   * - The last frame processed (is_keyframe_ = false)
   * - The newest keyframe (is_keyframe_ = true)
   */
  const StereoFrame stereo_frame_lkf_;
  const cv::Mat feature_tracks_;
};

}  // namespace VIO
