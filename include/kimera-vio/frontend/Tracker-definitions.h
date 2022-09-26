/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Tracker-definitions.h
 * @brief  Definitions for Tracker
 * @author Antoni Rosinol
 */

#pragma once

#include <string>

#include <glog/logging.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>

// OpenGV for Ransac
// TODO clean this includes!! some are only needed in cpp of tracker.
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/triangulation/methods.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/point_cloud/methods.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>

#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

#include "kimera-vio/common/vio_types.h"

namespace VIO {

// Mono (2d2d)
// 5-point ransac
using Problem2d2d =
    opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;

// Mono (2d2d, with given rotation) MonoTranslationOnly:
// TranslationOnlySacProblem 2-point ransac
using Problem2d2dGivenRot =
    opengv::sac_problems::relative_pose::TranslationOnlySacProblem;
using Adapter2d2d = opengv::relative_pose::CentralRelativeAdapter;

// PnP (2d3d)
using ProblemPnP = opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem;
using AdapterPnp = opengv::absolute_pose::CentralAbsoluteAdapter;

// Stereo (3d3d)
// Arun's problem (3-point ransac)
using Problem3d3d = opengv::sac_problems::point_cloud::PointCloudSacProblem;
using Adapter3d3d = opengv::point_cloud::PointCloudAdapter;

using Pose2d2dAlgorithm = opengv::sac_problems::relative_pose::
    CentralRelativePoseSacProblem::Algorithm;

enum class Pose3d2dAlgorithm {
  KneipP2P = 0,
  KneipP3P = 1,
  GaoP3P = 2,
  EPNP = 3,
  UPNP = 4,
  UP3P = 5,
  NonlinearOptimization = 6,
  MLPNP = 7  //! Requires OpenGV fork.
};

////////////////////////////////////////////////////////////////////////////////
class DebugTrackerInfo {
public:
  // Info about feature detection, tracking and ransac.
  size_t nrDetectedFeatures_ = 0, nrTrackerFeatures_ = 0, nrMonoInliers_ = 0;
  size_t nrMonoPutatives_ = 0, nrStereoInliers_ = 0, nrStereoPutatives_ = 0;
  size_t monoRansacIters_ = 0, stereoRansacIters_ = 0;

  // Info about performance of sparse stereo matching (and ransac):
  // RPK = right keypoints
  size_t nrValidRKP_ = 0, nrNoLeftRectRKP_ = 0, nrNoRightRectRKP_ = 0;
  size_t nrNoDepthRKP_ = 0, nrFailedArunRKP_ = 0;

  // Info about timing.
  double featureDetectionTime_ = 0, featureTrackingTime_ = 0;
  double monoRansacTime_ = 0, stereoRansacTime_ = 0;

  // Info about feature selector.
  double featureSelectionTime_ = 0;
  size_t extracted_corners_ = 0, need_n_corners_ = 0;

  void printTimes() const {
    LOG(INFO) << "featureDetectionTime_: " << featureDetectionTime_ << " s\n"
              << "featureSelectionTime_: " << featureSelectionTime_ << " s\n"
              << "featureTrackingTime_: " << featureTrackingTime_ << " s\n"
              << "monoRansacTime_: " << monoRansacTime_ << " s\n"
              << "stereoRansacTime_: " << stereoRansacTime_ << " s";
  }

  void print() const {
    LOG(INFO) << "nrDetectedFeatures_: " << nrDetectedFeatures_ << "\n"
              << "nrTrackerFeatures_: " << nrTrackerFeatures_ << "\n"
              << "nrMonoInliers_: " << nrMonoInliers_ << "\n"
              << "nrMonoPutatives_: " << nrMonoPutatives_ << "\n"
              << "nrStereoInliers_: " << nrStereoInliers_ << "\n"
              << "nrStereoPutatives_: " << nrStereoPutatives_ << "\n"
              << "monoRansacIters_: " << monoRansacIters_ << "\n"
              << "stereoRansacIters_: " << stereoRansacIters_ << "\n"
              << "nrValidRKP_: " << nrValidRKP_ << "\n"
              << "nrNoLeftRectRKP_: " << nrNoLeftRectRKP_ << "\n"
              << "nrNoRightRectRKP_: " << nrNoRightRectRKP_ << "\n"
              << "nrNoDepthRKP_: " << nrNoDepthRKP_ << "\n"
              << "nrFailedArunRKP_: " << nrFailedArunRKP_;
  }

};

enum class TrackingStatus {
  VALID,
  LOW_DISPARITY,
  FEW_MATCHES,
  INVALID,
  DISABLED
};
typedef std::pair<TrackingStatus, gtsam::Pose3> TrackingStatusPose;

////////////////////////////////////////////////////////////////////////////////
class TrackerStatusSummary {
public:
 TrackerStatusSummary()
     : kfTrackingStatus_mono_(TrackingStatus::INVALID),
       kfTrackingStatus_stereo_(TrackingStatus::INVALID),
       kfTracking_status_pnp_(TrackingStatus::INVALID),
       lkf_T_k_mono_(gtsam::Pose3()),
       lkf_T_k_stereo_(gtsam::Pose3()),
       W_T_k_pnp_(gtsam::Pose3()),
       infoMatStereoTranslation_(gtsam::Matrix3::Zero()) {}

 /* ------------------------------------------------------------------------ */
 // Returns the tracking status as a string for debugging
 static std::string asString(const TrackingStatus& status) {
   std::string status_str = "";
   switch (status) {
     case TrackingStatus::VALID: {
       status_str = "VALID";
       break;
     }
     case TrackingStatus::INVALID: {
       status_str = "INVALID";
       break;
     }
     case TrackingStatus::DISABLED: {
       status_str = "DISABLED";
       break;
     }
     case TrackingStatus::FEW_MATCHES: {
       status_str = "FEW_MATCHES";
       break;
     }
     case TrackingStatus::LOW_DISPARITY: {
       status_str = "LOW_DISPARITY";
       break;
     }
   }
   return status_str;
  }

public:
  TrackingStatus kfTrackingStatus_mono_;
  TrackingStatus kfTrackingStatus_stereo_;
  TrackingStatus kfTracking_status_pnp_;
  gtsam::Pose3 lkf_T_k_mono_;
  gtsam::Pose3 lkf_T_k_stereo_;
  gtsam::Pose3 W_T_k_pnp_;
  gtsam::Matrix3 infoMatStereoTranslation_;
};

typedef double KeypointScore;
typedef std::vector<double> KeypointScores;
typedef std::pair<KeypointsCV, KeypointScores> KeypointsWithScores;
typedef std::pair<size_t, size_t> KeypointMatch;
typedef std::vector<std::pair<size_t, size_t>> KeypointMatches;

} // End of VIO namespace.
