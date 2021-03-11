/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackend-definitions.h
 * @brief  Definitions for VioBackend.
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

#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/StereoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontend.h"
#include "kimera-vio/pipeline/PipelinePayload.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace VIO {

// Gtsam types. // TODO remove these!!
using gtsam::Cal3_S2;
using gtsam::Key;
using gtsam::Point2;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::StereoPoint2;
using StereoCalibPtr = gtsam::Cal3_S2Stereo::shared_ptr;

using SymbolChar = unsigned char;
static constexpr SymbolChar kPoseSymbolChar = 'x';
static constexpr SymbolChar kVelocitySymbolChar = 'v';
static constexpr SymbolChar kImuBiasSymbolChar = 'b';
static constexpr SymbolChar kLandmarkSymbolChar = 'l';

#define INCREMENTAL_SMOOTHER
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

using Landmark = gtsam::Point3;
using Landmarks = std::vector<Landmark>;
using PointWithId = std::pair<LandmarkId, Landmark>;
using PointsWithId = std::vector<PointWithId>;
using PointsWithIdMap = std::unordered_map<LandmarkId, Landmark>;
using LmkIdToLmkTypeMap = std::unordered_map<LandmarkId, LandmarkType>;

////////////////////////////////////////////////////////////////////////////////
// FeatureTrack
// TODO(Toni): what is this doing here... should be in Frontend at worst.
class FeatureTrack {
  // TODO(Toni): a feature track should have a landmark id...
  // TODO(Toni): a feature track should contain a pixel measurement per frame
  // but allow for multi-frame measurements at a time.
  // TODO(Toni): add getters for feature track length
 public:
  //! Observation: {FrameId, Px-Measurement}
  std::vector<std::pair<FrameId, StereoPoint2>> obs_;

  // Is the lmk in the graph?
  bool in_ba_graph_ = false;

  FeatureTrack(FrameId frame_id, const StereoPoint2& px) {
    obs_.push_back(std::make_pair(frame_id, px));
  }

  void print() const {
    LOG(INFO) << "Feature track with cameras: ";
    for (size_t i = 0u; i < obs_.size(); i++) {
      std::cout << " " << obs_[i].first << " ";
    }
    std::cout << std::endl;
  }
};

// Landmark id to measurements.
// Key is the lmk_id and feature track the collection of pairs of
// frame id and pixel location.
// TODO(Toni): what is this doing here... should be in Frontend at worst.
using FeatureTracks = std::unordered_map<LandmarkId, FeatureTrack>;

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
struct BackendInput : public PipelinePayload {
 public:
  KIMERA_POINTER_TYPEDEFS(BackendInput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(BackendInput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BackendInput(
      const Timestamp& timestamp_kf_nsec,
      const StatusStereoMeasurementsPtr& status_stereo_measurements_kf,
      const TrackingStatus& stereo_tracking_status,
      const ImuFrontend::PimPtr& pim,
      //! Raw imu msgs for Backend init only
      const ImuAccGyrS& imu_acc_gyrs,
      boost::optional<gtsam::Pose3> stereo_ransac_body_pose = boost::none)
      : PipelinePayload(timestamp_kf_nsec),
        status_stereo_measurements_kf_(status_stereo_measurements_kf),
        stereo_tracking_status_(stereo_tracking_status),
        pim_(pim),
        imu_acc_gyrs_(imu_acc_gyrs),
        stereo_ransac_body_pose_(stereo_ransac_body_pose) {}

 public:
  const StatusStereoMeasurementsPtr status_stereo_measurements_kf_;
  // stereo_vision_frontend_->trackerStatusSummary_.kfTrackingStatus_stereo_;
  const TrackingStatus stereo_tracking_status_;
  ImuFrontend::PimPtr pim_;
  ImuAccGyrS imu_acc_gyrs_;
  boost::optional<gtsam::Pose3> stereo_ransac_body_pose_;

 public:
  void print() const {
    LOG(INFO) << "VioBackend Input Payload print:\n"
              << "Timestamp: " << timestamp_;
    if (status_stereo_measurements_kf_) {
      LOG(INFO) << "Status smart stereo measurements: "
                << "\n\t Meas size: "
                << status_stereo_measurements_kf_->second.size();

      LOG(INFO)
          << "Mono Tracking Status: "
          << TrackerStatusSummary::asString(
                 status_stereo_measurements_kf_->first.kfTrackingStatus_mono_);
      status_stereo_measurements_kf_->first.lkf_T_k_mono_.print(
          "\n\t Tracker Pose (mono): ");
      LOG(INFO) << "Stereo Tracking Status: "
                << TrackerStatusSummary::asString(
                       status_stereo_measurements_kf_->first
                           .kfTrackingStatus_stereo_);
      status_stereo_measurements_kf_->first.lkf_T_k_stereo_.print(
          "\n\t Tracker Pose (stereo): ");

      LOG(INFO) << "Stereo Tracking Status: "
                << TrackerStatusSummary::asString(stereo_tracking_status_);
    }
    if (pim_) pim_->print("PIM : ");
    LOG_IF(INFO, stereo_ransac_body_pose_) << "Stereo Ransac Body Pose: "
                                           << *stereo_ransac_body_pose_;
  }
};

////////////////////////////////////////////////////////////////////////////////
struct BackendOutput : public PipelinePayload {
  KIMERA_POINTER_TYPEDEFS(BackendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(BackendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BackendOutput(const Timestamp& timestamp_kf,
                const gtsam::Values& state,
                const gtsam::NonlinearFactorGraph& factor_graph,
                const gtsam::Pose3& W_Pose_Blkf,
                const Vector3& W_Vel_Blkf,
                const ImuBias& imu_bias_lkf,
                const gtsam::Matrix& state_covariance_lkf,
                const FrameId& cur_kf_id,
                const int& landmark_count,
                const DebugVioInfo& debug_info,
                const PointsWithIdMap& landmarks_with_id_map,
                const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map)
      : PipelinePayload(timestamp_kf),
        W_State_Blkf_(timestamp_kf, W_Pose_Blkf, W_Vel_Blkf, imu_bias_lkf),
        state_(state),
        state_covariance_lkf_(state_covariance_lkf),
        factor_graph_(factor_graph),
        cur_kf_id_(cur_kf_id),
        landmark_count_(landmark_count),
        debug_info_(debug_info),
        landmarks_with_id_map_(landmarks_with_id_map),
        lmk_id_to_lmk_type_map_(lmk_id_to_lmk_type_map) {}

  BackendOutput(const VioNavStateTimestamped& vio_navstate_timestamped,
                const gtsam::Values& state,
                const gtsam::NonlinearFactorGraph& factor_graph,
                const gtsam::Matrix& state_covariance_lkf,
                const FrameId& cur_kf_id,
                const int& landmark_count,
                const DebugVioInfo& debug_info,
                const PointsWithIdMap& landmarks_with_id_map,
                const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map)
      : PipelinePayload(vio_navstate_timestamped.timestamp_),
        W_State_Blkf_(vio_navstate_timestamped),
        state_(state),
        state_covariance_lkf_(state_covariance_lkf),
        factor_graph_(factor_graph),
        cur_kf_id_(cur_kf_id),
        landmark_count_(landmark_count),
        debug_info_(debug_info),
        landmarks_with_id_map_(landmarks_with_id_map),
        lmk_id_to_lmk_type_map_(lmk_id_to_lmk_type_map) {}

  const VioNavStateTimestamped W_State_Blkf_;
  const gtsam::Values state_;
  const gtsam::Matrix state_covariance_lkf_;
  const gtsam::NonlinearFactorGraph factor_graph_;
  const FrameId cur_kf_id_;
  const int landmark_count_;
  const DebugVioInfo debug_info_;
  const PointsWithIdMap landmarks_with_id_map_;
  const LmkIdToLmkTypeMap lmk_id_to_lmk_type_map_;
};

////////////////////////////////////////////////////////////////////////////////
/** \brief The BackendType enum
 *  - kStereoImu: vanilla Backend type using Stereo and IMU
 *  - kStructuralRegularities: the `regular VIO` Backend, using structural
 * regularities derived from the 3D Mesh.
 */
enum class BackendType { kStereoImu = 0, kStructuralRegularities = 1 };

}  // namespace VIO
