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
#include "kimera-vio/utils/UtilsNumerical.h"

namespace VIO {

VisionImuFrontend::VisionImuFrontend(const FrontendParams& frontend_params,
                                     const ImuParams& imu_params,
                                     const ImuBias& imu_initial_bias,
                                     DisplayQueue* display_queue,
                                     bool log_output,
                                     std::optional<OdometryParams> odom_params)
    : frontend_params_(frontend_params),
      frontend_state_(FrontendState::Bootstrap),
      frame_count_(0),
      keyframe_count_(0),
      last_keyframe_timestamp_(0),
      imu_frontend_(nullptr),
      tracker_(nullptr),
      tracker_status_summary_(),
      display_queue_(display_queue),
      logger_(nullptr),
      odom_params_(odom_params) {
  imu_frontend_ = std::make_unique<ImuFrontend>(imu_params, imu_initial_bias);
  if (log_output) {
    logger_ = std::make_unique<FrontendLogger>();
  }
  time_aligner_ = std::make_unique<CrossCorrTimeAligner>(imu_params);
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
    TrackingStatusPose* status_pose_mono) const {
  CHECK_NOTNULL(status_pose_mono);

  const bool given_rot = !keyframe_R_cur_frame.equals(gtsam::Rot3());
  const bool time_aligned =
      frontend_state_ != FrontendState::InitialTimeAlignment;
  const bool imu_ok = given_rot && time_aligned;

  if (tracker_->tracker_params_.ransac_use_2point_mono_ && imu_ok) {
    // 2-point RANSAC.
    // TODO(marcus): move things from tracker here, only ransac in tracker.cpp
    gtsam::Pose3 keyframe_Pose_cur_frame(keyframe_R_cur_frame, gtsam::Point3());
    *status_pose_mono = tracker_->geometricOutlierRejection2d2d(
        frame_lkf, frame_k, keyframe_Pose_cur_frame);
  } else {
    // 5-point RANSAC.
    *status_pose_mono =
        tracker_->geometricOutlierRejection2d2d(frame_lkf, frame_k);
  }
}

void VisionImuFrontend::outlierRejectionStereo(
    const gtsam::StereoCamera& stereo_camera,
    const gtsam::Rot3& keyframe_R_cur_frame,
    StereoFrame* frame_lkf,
    StereoFrame* frame_k,
    TrackingStatusPose* status_pose_stereo,
    gtsam::Matrix3* translation_info_matrix) const {
  CHECK(frame_lkf);
  CHECK(frame_k);
  CHECK(tracker_);
  CHECK_NOTNULL(status_pose_stereo);
  CHECK_NOTNULL(translation_info_matrix);

  const bool given_rot = !keyframe_R_cur_frame.equals(gtsam::Rot3());
  const bool time_aligned =
      frontend_state_ != FrontendState::InitialTimeAlignment;
  const bool imu_ok = given_rot && time_aligned;

  if (tracker_->tracker_params_.ransac_use_1point_stereo_ && imu_ok) {
    // 1-point RANSAC.
    std::tie(*status_pose_stereo, *translation_info_matrix) =
        tracker_->geometricOutlierRejection3d3dGivenRotation(
            *frame_lkf, *frame_k, stereo_camera, keyframe_R_cur_frame);
  } else {
    // 3-point RANSAC.
    *status_pose_stereo =
        tracker_->geometricOutlierRejection3d3d(frame_lkf, frame_k);
    *translation_info_matrix = gtsam::Matrix3::Zero();
  }
}

void VisionImuFrontend::outlierRejectionPnP(
    const StereoFrame& frame,
    TrackingStatusPose* status_pnp) const {
  CHECK_NOTNULL(status_pnp);

  gtsam::Pose3 best_absolute_pose;
  std::vector<int> inliers;
  const auto valid = tracker_->pnp(frame, &status_pnp->second, &inliers);
  const auto num_points = frame.keypoints_3d_.size();
  const auto num_outliers = num_points - inliers.size();

  const auto min_inliers =
      static_cast<size_t>(tracker_->tracker_params_.min_pnp_inliers_);
  if (valid && inliers.size() > min_inliers) {
    status_pnp->first = TrackingStatus::VALID;
    VLOG(5) << "PnP tracking success:" << std::endl
            << "- # inliers: " << inliers.size() << std::endl
            << "- # outliers: " << num_outliers << std::endl
            << "Total: " << num_points;
    // TODO(Toni): remove outliers from the tracking?
  } else {
    status_pnp->first = TrackingStatus::FEW_MATCHES;
    VLOG(5) << "PnP tracking failed..." << std::endl
            << "- # inliers: " << inliers.size() << std::endl
            << "- # outliers: " << num_outliers << std::endl
            << "Total: " << num_points;
  }
}

bool VisionImuFrontend::shouldBeKeyframe(const Frame& frame,
                                         const Frame& frame_lkf) const {
  const Timestamp kf_diff_ns = frame.timestamp_ - frame_lkf.timestamp_;
  const size_t nr_valid_features = frame.getNrValidKeypoints();

  const bool min_time_elapsed =
      kf_diff_ns >= frontend_params_.min_intra_keyframe_time_ns_;
  const bool max_time_elapsed =
      kf_diff_ns >= frontend_params_.max_intra_keyframe_time_ns_;
  const bool nr_features_low =
      nr_valid_features <= frontend_params_.min_number_features_;

  KeypointMatches matches_ref_cur;
  tracker_->findMatchingKeypoints(frame_lkf, frame, &matches_ref_cur);

  // check for large enough disparity
  double disparity;
  tracker_->computeMedianDisparity(
      frame_lkf.keypoints_, frame.keypoints_, matches_ref_cur, &disparity);

  const bool is_disparity_low =
      disparity < tracker_->tracker_params_.disparityThreshold_;
  const bool disparity_low_first_time =
      is_disparity_low && !(tracker_status_summary_.kfTrackingStatus_mono_ ==
                            TrackingStatus::LOW_DISPARITY);
  const bool enough_disparity = !is_disparity_low;

  const bool max_disparity_reached =
      disparity > frontend_params_.max_disparity_since_lkf_;
  const bool disparity_flipped =
      ((enough_disparity || disparity_low_first_time) && min_time_elapsed);

  const bool need_new_keyframe = max_time_elapsed || max_disparity_reached ||
                                 disparity_flipped || nr_features_low ||
                                 frame.isKeyframe_;

  if (!need_new_keyframe) {
    return false;  // no keyframe conditions are met
  }

  VLOG(2) << "Keyframe after [s]: " << UtilsNumerical::NsecToSec(kf_diff_ns);

  // log why a keyframe was detected
  VLOG_IF(2, max_time_elapsed) << "Keyframe reason: max time elapsed.";
  VLOG_IF(2, max_disparity_reached)
      << "Keyframe reason: max disparity reached.";
  VLOG_IF(2, (enough_disparity && min_time_elapsed))
      << "Keyframe reason: enough disparity and min time elapsed).";
  VLOG_IF(2, disparity_low_first_time)
      << "Keyframe reason: disparity low first time.";
  VLOG_IF(2, nr_features_low)
      << "Keyframe reason: low nr of features (" << nr_valid_features << " < "
      << frontend_params_.min_number_features_ << ").";
  LOG_IF(WARNING, frame.isKeyframe_)
      << "Keyframe reason: user enforced keyframe!";

  return true;
}

void VisionImuFrontend::printTrackingStatus(const TrackingStatus& status,
                                            const std::string& type) {
  LOG(INFO) << "Status " << type << ": "
            << TrackerStatusSummary::asString(status);
}

void VisionImuFrontend::cacheExternalOdometry(FrontendInputPacketBase* input) {
  if (input->world_NavState_ext_odom_) {
    VLOG(2) << "Caching first odom measurement in boostrapSpin";
    const gtsam::Pose3 ext_odom_Pose_body =
        (*odom_params_).body_Pose_ext_odom_.inverse();
    world_OdomPose_body_lkf_ =
        (*input->world_NavState_ext_odom_).pose().compose(ext_odom_Pose_body);
  }
}

// can't be const (needs to cache keyframe odom if possible)
std::optional<gtsam::Pose3>
VisionImuFrontend::getExternalOdometryRelativeBodyPose(
    FrontendInputPacketBase* input) {
  if (!odom_params_) {
    return std::nullopt;
  }

  // Past this point we are using external odometry
  CHECK(input);
  if (!input->world_NavState_ext_odom_) {
    LOG(WARNING)
        << "Input packet did not contain valid external odometry measurement";
    return std::nullopt;
  }

  // First time getting a odometry measurement
  const gtsam::Pose3& ext_odom_Pose_body =
      (*odom_params_).body_Pose_ext_odom_.inverse();
  if (!world_OdomPose_body_lkf_) {
    world_OdomPose_body_lkf_ =
        (*input->world_NavState_ext_odom_).pose().compose(ext_odom_Pose_body);
    return std::nullopt;
  }

  gtsam::Pose3 world_Pose_body_kf =
      (*input->world_NavState_ext_odom_).pose().compose(ext_odom_Pose_body);
  gtsam::Pose3 body_lkf_Pose_body_kf =
      (*world_OdomPose_body_lkf_).between(world_Pose_body_kf);

  // We cache the current keyframe odometry for the next keyframe
  world_OdomPose_body_lkf_ = world_Pose_body_kf;
  return body_lkf_Pose_body_kf;
}

std::optional<gtsam::Velocity3>
VisionImuFrontend::getExternalOdometryWorldVelocity(
    FrontendInputPacketBase* input) const {
  if (!odom_params_) {
    return std::nullopt;
  }

  CHECK(input);
  if (!input->world_NavState_ext_odom_) {
    // we could log here too, but RelativePose handles it...
    return std::nullopt;
  }

  // Pass the sensor velocity in the world frame if available
  // NOTE: typical odometry is not suitable for this since the vel estimate
  // in the world frame will not have a bounded error.
  return (*input->world_NavState_ext_odom_).velocity();
}

}  // namespace VIO
