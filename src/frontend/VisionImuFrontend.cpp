/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionImuFrontend.cpp
 * @brief  Class describing an abstract VIO Frontend
 * @author Marcus Abate
 */

#include "kimera-vio/frontend/VisionImuFrontend.h"
#include "kimera-vio/initial/CrossCorrTimeAligner.h"

namespace VIO {

VisionImuFrontend::VisionImuFrontend(
    const ImuParams& imu_params,
    const ImuBias& imu_initial_bias,
    DisplayQueue* display_queue,
    bool log_output,
    boost::optional<OdometryParams> odom_params)
    : frontend_state_(FrontendState::Bootstrap),
      frame_count_(0),
      keyframe_count_(0),
      last_keyframe_timestamp_(0),
      imu_frontend_(nullptr),
      tracker_(nullptr),
      tracker_status_summary_(),
      display_queue_(display_queue),
      logger_(nullptr),
      odom_params_(odom_params) {
  imu_frontend_ = VIO::make_unique<ImuFrontend>(imu_params, imu_initial_bias);
  if (log_output) {
    logger_ = VIO::make_unique<FrontendLogger>();
  }
  time_aligner_ = VIO::make_unique<CrossCorrTimeAligner>(imu_params);
}

VisionImuFrontend::~VisionImuFrontend() {
  LOG(INFO) << "VisionImuFrontend destructor called.";
}

FrontendOutputPacketBase::UniquePtr VisionImuFrontend::spinOnce(
    FrontendInputPacketBase::UniquePtr&& input) {
  const FrontendState& frontend_state = frontend_state_;
  switch (frontend_state) {
    case FrontendState::Bootstrap:
      return bootstrapSpin(std::move(input));
    case FrontendState::InitialTimeAlignment:
      return timeAlignmentSpin(std::move(input));
    case FrontendState::Nominal:
      return nominalSpin(std::move(input));
    default:
      LOG(FATAL) << "Unrecognized Frontend state.";
      break;
  }
}

FrontendOutputPacketBase::UniquePtr VisionImuFrontend::timeAlignmentSpin(
    FrontendInputPacketBase::UniquePtr&& input) {
  CHECK(time_aligner_);

  ImuStampS imu_stamps = input->imu_stamps_;
  ImuAccGyrS imu_accgyrs = input->imu_accgyrs_;

  FrontendOutputPacketBase::UniquePtr nominal_output =
      nominalSpin(std::move(input));
  CHECK(nominal_output);

  TimeAlignerBase::Result result = time_aligner_->estimateTimeAlignment(
      *tracker_, *nominal_output, imu_stamps, imu_accgyrs, logger_.get());
  if (result.valid) {
    CHECK(imu_time_shift_update_callback_);
    imu_time_shift_update_callback_(result.imu_time_shift);
    frontend_state_ = FrontendState::Nominal;
  }

  // We always return an invalid input to ensure other modules don't
  // start until after time alignment is finished
  return nullptr;
}

void VisionImuFrontend::outlierRejectionMono(
    const gtsam::Rot3& keyframe_R_cur_frame,
    Frame* frame_lkf,
    Frame* frame_k,
    TrackingStatusPose* status_pose_mono) {
  CHECK_NOTNULL(status_pose_mono);
  if (tracker_->tracker_params_.ransac_use_2point_mono_ &&
      !keyframe_R_cur_frame.equals(gtsam::Rot3()) &&
      frontend_state_ != FrontendState::InitialTimeAlignment) {
    // 2-point RANSAC.
    // TODO(marcus): move things from tracker here, only ransac in tracker.cpp
    gtsam::Pose3 keyframe_Pose_cur_frame(keyframe_R_cur_frame,
                                         gtsam::Point3::Identity());
    *status_pose_mono = tracker_->geometricOutlierRejection2d2d(
        frame_lkf, frame_k, keyframe_Pose_cur_frame);
  } else {
    // 5-point RANSAC.
    *status_pose_mono =
        tracker_->geometricOutlierRejection2d2d(frame_lkf, frame_k);
  }
}

void VisionImuFrontend::printTrackingStatus(const TrackingStatus& status,
                                            const std::string& type) {
  LOG(INFO) << "Status " << type << ": "
            << TrackerStatusSummary::asString(status);
}

}  // namespace VIO
