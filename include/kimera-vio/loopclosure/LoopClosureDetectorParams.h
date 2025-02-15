/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoopClosureDetectorParams.h
 * @brief  Class collecting the parameters used for loop closure detection
 * @author Marcus Abate
 */

#pragma once

#include <glog/logging.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "kimera-vio/frontend/VisionImuTrackerParams.h"
#include "kimera-vio/loopclosure/FrameCache.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"
#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

enum class PoseRecoveryType {
  k3d3d = 0,
  kPnP = 1,
  k5ptRotOnly = 2,
};

class LoopClosureDetectorParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(LoopClosureDetectorParams);
  LoopClosureDetectorParams();
  virtual ~LoopClosureDetectorParams() = default;

  // NOTE: we cannot parse width, height principe pt and focal length from here.
  // Those are done at initialization of the LoopClosureDetector.
  void print() const override;
  bool parseYAML(const std::string& filepath) override;
  bool equals(const LoopClosureDetectorParams& lp2, double tol = 1e-10) const;

 protected:
  bool equals(const PipelineParams& obj) const override {
    const auto& rhs = static_cast<const LoopClosureDetectorParams&>(obj);
    return equals(rhs);
  }

 public:
  TrackerParams tracker_params_;

  //////////////////////////// Loop Detection Params ///////////////////////////
  bool use_nss_ = true;  // If true, normalize score wrt score achieved across
                         // consecutive queries
  float alpha_ = 0.1;    // Alpha similarity threshold for matches
  int min_temporal_matches_ = 3;  // Min consistent matches to pass temporal
                                  // check; if set to 1, no temporal check
  int recent_frames_window_ =
      20;  // Number of recent frames we don't consider for loop closures
  int max_db_results_ =
      50;  // Max number of results from db queries to consider
  float min_nss_factor_ =
      0.005;  // Min acceptable value of normalization factor
  int min_matches_per_island_ = 1;  // Min number of close matches in an island
  int max_intraisland_gap_ =
      3;  // Max separation between matches of the same island
  int max_nrFrames_between_islands_ = 3;  // Max separation between groups
  int max_nrFrames_between_queries_ =
      2;  // Max separation between two queries s.t. they count towards
          // min_temporal_matches_
  //////////////////////////////////////////////////////////////////////////////

  /////////////////////////// 3D Pose Recovery Params //////////////////////////
  bool refine_pose_ = true;
  PoseRecoveryType pose_recovery_type_ = PoseRecoveryType::k3d3d;
  static constexpr double max_pose_recovery_translation_ = 1e3;
  //////////////////////////////////////////////////////////////////////////////

  ///////////////////////// ORB feature matching params ////////////////////////
  double lowe_ratio_ = 0.7;
#if CV_VERSION_MAJOR == 3
  int matcher_type_ = 3;
#else
  cv::DescriptorMatcher::MatcherType matcher_type_ =
      cv::DescriptorMatcher::MatcherType::BRUTEFORCE_L1;
#endif
  //////////////////////////////////////////////////////////////////////////////

  ///////////////////////// ORB feature detector params ////////////////////////
  int nfeatures_ = 500;
  float scale_factor_ = 1.2f;
  int nlevels_ = 8;
  int edge_threshold_ = 31;
  int first_level_ = 0;
  int WTA_K_ = 2;
#if CV_VERSION_MAJOR == 3
  int score_type_ = cv::ORB::HARRIS_SCORE;
#else
  cv::ORB::ScoreType score_type_ = cv::ORB::ScoreType::HARRIS_SCORE;
#endif
  int patch_sze_ = 31;
  int fast_threshold_ = 20;
  //////////////////////////////////////////////////////////////////////////////

  double betweenRotationPrecision_ = 1 / (0.1 * 0.1);
  double betweenTranslationPrecision_ = 1 / (0.1 * 0.1);

  ////////////////////////////// PGO solver params /////////////////////////////
  double odom_rot_threshold_ = 0.01;
  double odom_trans_threshold_ = 0.1;
  double pcm_rot_threshold_ = -1;
  double pcm_trans_threshold_ = -1;
  double gnc_alpha_ = 0.9;
  //////////////////////////////////////////////////////////////////////////////

  int max_lc_cached_before_optimize_ = 10;

  FrameCacheConfig frame_cache;
};

}  // namespace VIO
