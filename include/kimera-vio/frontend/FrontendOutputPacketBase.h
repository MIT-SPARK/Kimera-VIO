/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FrontendOutputPacketBase.h
 * @brief  Class describing the minimum output of the Frontend.
 * @author Marcus Abate
 */

#pragma once

#include <optional>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/frontend/VisionImuFrontend-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontend.h"
#include "kimera-vio/pipeline/PipelinePayload.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class FrontendOutputPacketBase : public PipelinePayload {
 public:
  KIMERA_POINTER_TYPEDEFS(FrontendOutputPacketBase);
  KIMERA_DELETE_COPY_CONSTRUCTORS(FrontendOutputPacketBase);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrontendOutputPacketBase(
      const Timestamp& timestamp,
      const bool& is_keyframe,
      const FrontendType& frontend_type,
      const ImuFrontend::PimPtr& pim,
      const ImuAccGyrS& imu_acc_gyrs,
      const DebugTrackerInfo& debug_tracker_info,
      std::optional<gtsam::Pose3> body_lkf_OdomPose_body_kf = std::nullopt,
      std::optional<gtsam::Velocity3> body_kf_world_OdomVel_body_kf =
          std::nullopt)
      : PipelinePayload(timestamp),
        is_keyframe_(is_keyframe),
        frontend_type_(frontend_type),
        pim_(pim),
        imu_acc_gyrs_(imu_acc_gyrs),
        debug_tracker_info_(debug_tracker_info),
        body_lkf_OdomPose_body_kf_(body_lkf_OdomPose_body_kf),
        body_kf_world_OdomVel_body_kf_(body_kf_world_OdomVel_body_kf) {}

  virtual ~FrontendOutputPacketBase() = default;

  virtual const Frame* getTrackingFrame() const { return nullptr; }

  virtual const cv::Mat* getTrackingImage() const { return nullptr; }

  virtual const gtsam::Pose3* getBodyPoseCam() const { return nullptr; }

  virtual const gtsam::Pose3* getBodyPoseCamRight() const { return nullptr; }

  virtual const TrackerStatusSummary* getTrackerStatus() const {
    return nullptr;
  }

 public:
  const bool is_keyframe_;
  const FrontendType frontend_type_;
  const ImuFrontend::PimPtr pim_;
  const ImuAccGyrS imu_acc_gyrs_;
  const DebugTrackerInfo debug_tracker_info_;
  // between pose of the body from the current to the last keyframe as estimated
  // by an external odometry source
  std::optional<gtsam::Pose3> body_lkf_OdomPose_body_kf_;
  // velocity of the current body frame w.r.t. world frame in the current body
  // frame from odometry
  std::optional<gtsam::Velocity3> body_kf_world_OdomVel_body_kf_;

  inline DebugTrackerInfo getTrackerInfo() const { return debug_tracker_info_; }
};

}  // namespace VIO
