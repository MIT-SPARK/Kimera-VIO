/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdVisionImuFrontend-definitions.h
 * @brief  Definitions for RgbdVisionImuFrontend
 * @author Nathan Hughes
 */

#pragma once

#include "kimera-vio/frontend/FrontendOutputPacketBase.h"
#include "kimera-vio/frontend/RgbdFrame.h"
#include "kimera-vio/frontend/VisionImuFrontend-definitions.h"

#include <optional>

namespace VIO {

struct RgbdFrontendOutput : public FrontendOutputPacketBase {
 public:
  KIMERA_POINTER_TYPEDEFS(RgbdFrontendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RgbdFrontendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RgbdFrontendOutput(
      const bool is_keyframe,
      const StatusStereoMeasurementsPtr& status_stereo_measurements,
      const gtsam::Pose3& b_Pose_cam_rect,
      const StereoFrame& frame_lkf,
      const RgbdFrame& rgbd_frame_lkf,
      const ImuFrontend::PimPtr& pim,
      const ImuAccGyrS& imu_acc_gyrs,
      const cv::Mat& feature_tracks,
      const DebugTrackerInfo& debug_tracker_info,
      std::optional<gtsam::Pose3> lkf_body_Pose_kf_body = std::nullopt,
      std::optional<gtsam::Velocity3> body_world_Vel_body = std::nullopt)
      : FrontendOutputPacketBase(frame_lkf.timestamp_,
                                 is_keyframe,
                                 FrontendType::kRgbdImu,
                                 pim,
                                 imu_acc_gyrs,
                                 debug_tracker_info,
                                 lkf_body_Pose_kf_body,
                                 body_world_Vel_body),
        status_stereo_measurements_(status_stereo_measurements),
        b_Pose_cam_rect_(b_Pose_cam_rect),
        frame_lkf_(frame_lkf),
        rgbd_frame_lkf_(rgbd_frame_lkf),
        feature_tracks_(feature_tracks) {}

  virtual ~RgbdFrontendOutput() = default;

  virtual const Frame* getTrackingFrame() const override {
    return &frame_lkf_.left_frame_;
  }

  virtual const cv::Mat* getTrackingImage() const override {
    return &feature_tracks_;
  }

  virtual const gtsam::Pose3* getBodyPoseCam() const override {
    return &b_Pose_cam_rect_;
  }

  virtual const TrackerStatusSummary* getTrackerStatus() const override {
    return status_stereo_measurements_ ? &(status_stereo_measurements_->first)
                                       : nullptr;
  }

 public:
  const StatusStereoMeasurementsPtr status_stereo_measurements_;
  const gtsam::Pose3 b_Pose_cam_rect_;
  // TODO(nathan) make this name consistent (see stereo)
  const StereoFrame frame_lkf_;
  const RgbdFrame rgbd_frame_lkf_;
  const cv::Mat feature_tracks_;
};

}  // namespace VIO
