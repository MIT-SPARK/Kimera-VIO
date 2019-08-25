/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackEnd-definitions.h
 * @brief  Definitions for VioBackEnd.
 * @author Antoni Rosinol
 */

#pragma once

#include <vector>

#include <boost/optional.hpp>

#include <glog/logging.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include "StereoVisionFrontEnd-definitions.h"
#include "Tracker-definitions.h"
#include "UtilsOpenCV.h"
#include "common/vio_types.h"
#include "imu-frontend/ImuFrontEnd-definitions.h"
#include "imu-frontend/ImuFrontEnd.h"

namespace VIO {

// Gtsam types. // TODO remove these!!
using gtsam::Cal3_S2;
using gtsam::Key;
using gtsam::Point2;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::StereoPoint2;

#define INCREMENTAL_SMOOTHER
//#define USE_COMBINED_IMU_FACTOR

#ifdef INCREMENTAL_SMOOTHER
typedef gtsam::IncrementalFixedLagSmoother Smoother;
#else
typedef gtsam::BatchFixedLagSmoother Smoother;
#endif

// Backend types
using SmartStereoFactor = gtsam::SmartStereoProjectionPoseFactor;
using SmartFactorParams = gtsam::SmartStereoProjectionParams;
using LandmarkIdSmartFactorMap =
    std::unordered_map<LandmarkId, SmartStereoFactor::shared_ptr>;
using Slot = long int;
using SmartFactorMap =
    gtsam::FastMap<LandmarkId, std::pair<SmartStereoFactor::shared_ptr, Slot>>;

using PointWithId = std::pair<LandmarkId, gtsam::Point3>;
using PointsWithId = std::vector<PointWithId>;
using PointsWithIdMap = std::unordered_map<LandmarkId, gtsam::Point3>;
using LmkIdToLmkTypeMap = std::unordered_map<LandmarkId, LandmarkType>;

////////////////////////////////////////////////////////////////////////////////
// FeatureTrack
class FeatureTrack {
 public:
  //! Observation: {FrameId, Px-Measurement}
  std::vector<std::pair<FrameId, StereoPoint2>> obs_;

  // Is the lmk in the graph?
  bool in_ba_graph_ = false;

  FeatureTrack(FrameId frame_id, const StereoPoint2& px) {
    obs_.push_back(std::make_pair(frame_id, px));
  }

  void print() const {
    std::cout << "feature track with cameras: ";
    for (size_t i = 0; i < obs_.size(); i++) {
      std::cout << " " << obs_[i].first << " ";
    }
    std::cout << std::endl;
  }
};

// Landmark id to measurements.
// Key is the lmk_id and feature track the collection of pairs of
// frame id and pixel location.
using FeatureTracks = std::unordered_map<Key, FeatureTrack>;

////////////////////////////////////////////////////////////////////////////////
class DebugVioInfo {
 public:
  int numSF_;
  int numValid_;
  int numDegenerate_;
  int numFarPoints_;
  int numOutliers_;
  int numCheirality_;
  int numNonInitialized_;

  gtsam::Rot3 imuR_lkf_kf = gtsam::Rot3();

  gtsam::Values stateBeforeOpt;
  gtsam::NonlinearFactorGraph graphBeforeOpt;
  gtsam::NonlinearFactorGraph graphToBeDeleted;

  double factorsAndSlotsTime_;
  double preUpdateTime_;
  double updateTime_;
  double updateSlotTime_;
  double extraIterationsTime_;

  double meanPixelError_;
  double maxPixelError_;
  double meanTrackLength_;
  size_t maxTrackLength_;

  int numAddedSmartF_;
  int numAddedImuF_;
  int numAddedNoMotionF_;
  int numAddedConstantVelF_;
  int numAddedBetweenStereoF_;

  int nrElementsInMatrix_;
  int nrZeroElementsInMatrix_;

  gtsam::NavState navstate_k_;

  double linearizeTime_;
  double linearSolveTime_;
  double retractTime_;
  double linearizeMarginalizeTime_;
  double marginalizeTime_;

  /* ------------------------------------------------------------------------ */
  void resetSmartFactorsStatistics() {
    numSF_ = 0;
    numValid_ = 0;
    numDegenerate_ = 0;
    numFarPoints_ = 0;
    numOutliers_ = 0;
    numCheirality_ = 0;
    numNonInitialized_ = 0;

    meanPixelError_ = 0;
    maxPixelError_ = 0;
    meanTrackLength_ = 0;
    maxTrackLength_ = 0;
  }

  /* ------------------------------------------------------------------------ */
  void resetTimes() {
    factorsAndSlotsTime_ = 0;
    preUpdateTime_ = 0;
    updateTime_ = 0;
    updateSlotTime_ = 0;
    extraIterationsTime_ = 0;
    linearizeTime_ = 0;
    linearSolveTime_ = 0;
    retractTime_ = 0;
    linearizeMarginalizeTime_ = 0;
    marginalizeTime_ = 0;
  }

  /* ------------------------------------------------------------------------ */
  void resetAddedFactorsStatistics() {
    numAddedSmartF_ = 0;
    numAddedImuF_ = 0;
    numAddedNoMotionF_ = 0;
    numAddedConstantVelF_ = 0;
    numAddedBetweenStereoF_ = 0;
  }

  /* ------------------------------------------------------------------------ */
  double sumAllTimes() const {
    return factorsAndSlotsTime_ + preUpdateTime_ + updateTime_ +
           updateSlotTime_ + extraIterationsTime_;
  }

  /* ------------------------------------------------------------------------ */
  void printTimes() const {
    LOG(INFO) << "-------- VIO Backend Timing --------\n"
              << "Find delete time      :" << factorsAndSlotsTime_ << '\n'
              << "preUpdate time        :" << preUpdateTime_ << '\n'
              << "Update Time time      :" << updateTime_ << '\n'
              << "Update slot time      :" << updateSlotTime_ << '\n'
              << "Extra iterations time :" << extraIterationsTime_;
  }

  /* ------------------------------------------------------------------------ */
  void print() const {
    LOG(INFO) << "-------- DebugVioInfo: --------\n"
              << " numSF: " << numSF_ << '\n'
              << " numValid: " << numValid_ << '\n'
              << " numDegenerate: " << numDegenerate_ << '\n'
              << " numOutliers: " << numOutliers_ << '\n'
              << " numFarPoints: " << numFarPoints_ << '\n'
              << " numCheirality: " << numCheirality_ << '\n'
              << " numNonInitialized: " << numNonInitialized_ << '\n'
              << " meanPixelError: " << meanPixelError_ << '\n'
              << " maxPixelError: " << maxPixelError_ << '\n'
              << " meanTrackLength: " << meanTrackLength_ << '\n'
              << " maxTrackLength: " << maxTrackLength_;
  }
};

////////////////////////////////////////////////////////////////////////////////
struct VioBackEndInputPayload {
  VioBackEndInputPayload(
      const Timestamp& timestamp_kf_nsec,
      const StatusSmartStereoMeasurements& status_smart_stereo_measurements_kf,
      const TrackingStatus&
          stereo_tracking_status,  // stereo_vision_frontend_->trackerStatusSummary_.kfTrackingStatus_stereo_;
      const ImuFrontEnd::PreintegratedImuMeasurements& pim,
      boost::optional<gtsam::Pose3> stereo_ransac_body_pose = boost::none,
      std::vector<Plane>* planes = nullptr)
      : timestamp_kf_nsec_(timestamp_kf_nsec),
        status_smart_stereo_measurements_kf_(
            status_smart_stereo_measurements_kf),
        stereo_tracking_status_(stereo_tracking_status),
        pim_(pim),
        planes_(planes),
        stereo_ransac_body_pose_(stereo_ransac_body_pose) {}
  const Timestamp timestamp_kf_nsec_;
  const StatusSmartStereoMeasurements status_smart_stereo_measurements_kf_;
  const TrackingStatus
      stereo_tracking_status_;  // stereo_vision_frontend_->trackerStatusSummary_.kfTrackingStatus_stereo_;
  // I believe we do not need EIGEN_MAKE_ALIGNED_OPERATOR_NEW for these members
  // as they are dynamic eigen, not "fixed-size vectorizable matrices and
  // vectors."
  const gtsam::PreintegratedImuMeasurements pim_;
  std::vector<Plane>* planes_;
  boost::optional<gtsam::Pose3> stereo_ransac_body_pose_;

 public:
  void print() const {
    LOG(INFO) << "VioBackEnd Input Payload print:\n"
              << "Timestamp: " << timestamp_kf_nsec_ << '\n'
              << "Status smart stereo measurements: "
              << "\n\t Meas size: "
              << status_smart_stereo_measurements_kf_.second.size();

    LOG(INFO) << "Mono Tracking Status: "
              << TrackerStatusSummary::asString(
                     status_smart_stereo_measurements_kf_.first
                         .kfTrackingStatus_mono_);
    status_smart_stereo_measurements_kf_.first.lkf_T_k_mono_.print(
        "\n\t Tracker Pose (mono): ");
    LOG(INFO) << "Stereo Tracking Status: "
              << TrackerStatusSummary::asString(
                     status_smart_stereo_measurements_kf_.first
                         .kfTrackingStatus_stereo_);
    status_smart_stereo_measurements_kf_.first.lkf_T_k_stereo_.print(
        "\n\t Tracker Pose (stereo): ");

    LOG(INFO) << "Stereo Tracking Status: "
              << TrackerStatusSummary::asString(stereo_tracking_status_);
    pim_.print("PIM : ");
    LOG_IF(INFO, planes_ != nullptr) << "Number of planes: " << planes_->size();
    LOG_IF(INFO, stereo_ransac_body_pose_)
        << "Stereo Ransac Body Pose: " << *stereo_ransac_body_pose_;
  }
};

////////////////////////////////////////////////////////////////////////////////
struct VioBackEndOutputPayload {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VioBackEndOutputPayload(
      const Timestamp& timestamp_kf, const gtsam::Values state,
      const gtsam::Pose3& W_Pose_Blkf, const Vector3& W_Vel_Blkf,
      const gtsam::Pose3& B_Pose_leftCam, const ImuBias& imu_bias_lkf,
      const gtsam::Matrix& state_covariance_lkf, const int& cur_kf_id,
      const int& landmark_count, const DebugVioInfo& debug_info)
      : timestamp_kf_(timestamp_kf),
        state_(state),
        W_Pose_Blkf_(W_Pose_Blkf),
        W_Vel_Blkf_(W_Vel_Blkf),
        B_Pose_leftCam_(B_Pose_leftCam),
        imu_bias_lkf_(imu_bias_lkf),
        state_covariance_lkf_(state_covariance_lkf),
        cur_kf_id_(cur_kf_id),
        landmark_count_(landmark_count),
        debug_info_(debug_info) {}

  const Timestamp timestamp_kf_;
  const gtsam::Values state_;
  const gtsam::Pose3 W_Pose_Blkf_;
  const Vector3 W_Vel_Blkf_;
  const gtsam::Pose3 B_Pose_leftCam_;
  const ImuBias imu_bias_lkf_;
  const gtsam::Matrix state_covariance_lkf_;
  const int cur_kf_id_;
  const int landmark_count_;
  const DebugVioInfo debug_info_;
};

}  // namespace VIO
