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

#include <memory>
#include <string>

#include <glog/logging.h>

#include <opencv2/opencv.hpp>

#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"
#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

class LoopClosureDetectorParams : public PipelineParams {
 public:
  LoopClosureDetectorParams(
      int image_width = 752,
      int image_height = 480,
      double focal_length = 1.0,
      cv::Point2d principle_point = cv::Point2d(0.0, 0.0),

      bool use_nss = true,
      float alpha = 0.1,
      int min_temporal_matches = 3,
      int recent_frames_window = 20,
      int max_db_results = 50,
      float min_nss_factor = 0.005,
      int min_matches_per_island = 1,
      int max_intraisland_gap = 3,
      int max_nrFrames_between_islands = 3,
      int max_nrFrames_between_queries = 2,

      GeomVerifOption geom_check = GeomVerifOption::NISTER,
      int min_correspondences = 12,
      int max_ransac_iterations_mono = 500,
      double ransac_probability_mono = 0.99,
      double ransac_threshold_mono = 1e-6,
      bool ransac_randomize_mono = false,
      double ransac_inlier_threshold_mono = 0.5,

      PoseRecoveryOption pose_recovery_option = PoseRecoveryOption::GIVEN_ROT,
      int max_ransac_iterations_stereo = 500,
      double ransac_probability_stereo = 0.995,
      double ransac_threshold_stereo = 0.15,
      bool ransac_randomize_stereo = false,
      double ransac_inlier_threshold_stereo = 0.5,
      bool use_mono_rot = true,
      bool refine_pose = true,
      double lowe_ratio = 0.7,
#if CV_VERSION_MAJOR == 3
      int matcher_type = cv::DescriptorMatcher::BRUTEFORCE_HAMMING,
#else
      cv::DescriptorMatcher::MatcherType matcher_type =
          cv::DescriptorMatcher::MatcherType::BRUTEFORCE_HAMMING,
#endif

      int nfeatures = 500,
      float scale_factor = 1.2f,
      int nlevels = 8,
      int edge_threshold = 31,
      int first_level = 0,
      int WTA_K = 2,
#if CV_VERSION_MAJOR == 3
      int score_type = cv::ORB::HARRIS_SCORE,
#else
      cv::ORB::ScoreType score_type = cv::ORB::ScoreType::HARRIS_SCORE,
#endif
      int patch_sze = 31,
      int fast_threshold = 20,

      double betweenRotationPrecision = 1/ (0.01 * 0.01),
      double betweenTranslationPrecision = 1 / (0.1 * 0.1),

      double pgo_rot_threshold = 0.01,
      double pgo_trans_threshold = 0.1);

 public:
  virtual ~LoopClosureDetectorParams() = default;

  // NOTE: we cannot parse width, height principe pt and focal length from here.
  // Those are done at initialization of the LoopClosureDetector.
  bool parseYAML(const std::string& filepath) override;

  void print() const override;

  bool equals(const PipelineParams& obj) const override {
    const auto& rhs = static_cast<const LoopClosureDetectorParams&>(obj);
    return
      image_width_ == rhs.image_width_ &&
      image_height_ == rhs.image_height_ &&
      focal_length_ == rhs.focal_length_ &&
      principle_point_ == rhs.principle_point_ &&

      use_nss_ == rhs.use_nss_ &&
      alpha_== rhs.alpha_ &&
      min_temporal_matches_== rhs.min_temporal_matches_ &&
      recent_frames_window_== rhs.recent_frames_window_ &&
      max_db_results_== rhs.max_db_results_ &&
      min_nss_factor_== rhs.min_nss_factor_ &&
      min_matches_per_island_== rhs.min_matches_per_island_ &&
      max_intraisland_gap_== rhs.max_intraisland_gap_ &&
      max_nrFrames_between_islands_== rhs.max_nrFrames_between_islands_ &&
      max_nrFrames_between_queries_== rhs.max_nrFrames_between_queries_ &&

      geom_check_== rhs.geom_check_ &&
      min_correspondences_== rhs.min_correspondences_ &&
      max_ransac_iterations_mono_== rhs.max_ransac_iterations_mono_ &&
      ransac_probability_mono_== rhs.ransac_probability_mono_ &&
      ransac_threshold_mono_== rhs.ransac_threshold_mono_ &&
      ransac_randomize_mono_== rhs.ransac_randomize_mono_ &&
      ransac_inlier_threshold_mono_== rhs.ransac_inlier_threshold_mono_ &&

      pose_recovery_option_== rhs.pose_recovery_option_ &&
      max_ransac_iterations_stereo_== rhs.max_ransac_iterations_stereo_ &&
      ransac_probability_stereo_== rhs.ransac_probability_stereo_ &&
      ransac_threshold_stereo_== rhs.ransac_threshold_stereo_ &&
      ransac_randomize_stereo_== rhs.ransac_randomize_stereo_ &&
      ransac_inlier_threshold_stereo_== rhs.ransac_inlier_threshold_stereo_ &&
      use_mono_rot_== rhs.use_mono_rot_ &&
      refine_pose_ == rhs.refine_pose_ &&
      lowe_ratio_== rhs.lowe_ratio_ &&
      matcher_type_== rhs.matcher_type_ &&

      nfeatures_== rhs.nfeatures_ &&
      scale_factor_== rhs.scale_factor_ &&
      nlevels_== rhs.nlevels_ &&
      edge_threshold_== rhs.edge_threshold_ &&
      first_level_== rhs.first_level_ &&
      WTA_K_== rhs.WTA_K_ &&
      score_type_== rhs.score_type_ &&
      patch_sze_== rhs.patch_sze_ &&
      fast_threshold_== rhs.fast_threshold_ &&

      betweenRotationPrecision_ == rhs.betweenRotationPrecision_ &&
      betweenTranslationPrecision_ == rhs.betweenTranslationPrecision_ &&

      pgo_rot_threshold_== rhs.pgo_rot_threshold_ &&
      pgo_trans_threshold_== rhs.pgo_trans_threshold_;
  }

 public:
  /////////////////////////// Camera intrinsic Params //////////////////////////
  int image_width_;
  int image_height_;
  double focal_length_;          // Focal length of camera
  cv::Point2d principle_point_;  // Principle point of the camera
  //////////////////////////////////////////////////////////////////////////////

  //////////////////////////// Loop Detection Params ///////////////////////////
  bool use_nss_;              // If true, normalize score wrt score achieved across consecutive queries
  float alpha_;               // Alpha similarity threshold for matches
  int min_temporal_matches_;  // Min consistent matches to pass temporal check; if set to 1, no temporal check
  int recent_frames_window_;  // Number of recent frames we don't consider for loop closures
  int max_db_results_;    // Max number of results from db queries to consider
  float min_nss_factor_;  // Min acceptable value of normalization factor
  int min_matches_per_island_;  // Min number of close matches in an island
  int max_intraisland_gap_;     // Max separation between matches of the same island
  int max_nrFrames_between_islands_;   // Max separation between groups
  int max_nrFrames_between_queries_;  // Max separation between two queries s.t. they count towards min_temporal_matches_
  //////////////////////////////////////////////////////////////////////////////

  /////////////////////// Geometrical Verification Params //////////////////////
  GeomVerifOption geom_check_;  // Geometrical check
  int min_correspondences_;     // Min number of inliers when computing a pose
  int max_ransac_iterations_mono_;       // Max number of iterations of RANSAC
  double ransac_probability_mono_;       // Success probability of RANSAC
  double ransac_threshold_mono_;         // Threshold for 5-pt algorithm
  bool ransac_randomize_mono_;           // Randomize seed for ransac
  double ransac_inlier_threshold_mono_;  // Threshold for ransac inliers
  //////////////////////////////////////////////////////////////////////////////

  /////////////////////////// 3D Pose Recovery Params //////////////////////////
  PoseRecoveryOption pose_recovery_option_;
  int max_ransac_iterations_stereo_;
  double ransac_probability_stereo_;
  double ransac_threshold_stereo_;
  bool ransac_randomize_stereo_;
  double ransac_inlier_threshold_stereo_;
  bool use_mono_rot_;
  bool refine_pose_;
  //////////////////////////////////////////////////////////////////////////////

  double betweenRotationPrecision_;
  double betweenTranslationPrecision_;

  ///////////////////////// ORB feature matching params ////////////////////////
  double lowe_ratio_;
#if CV_VERSION_MAJOR == 3
  int matcher_type_;
#else
  cv::DescriptorMatcher::MatcherType matcher_type_;
#endif
  //////////////////////////////////////////////////////////////////////////////

  ///////////////////////// ORB feature detector params ////////////////////////
  int nfeatures_;
  float scale_factor_;
  int nlevels_;
  int edge_threshold_;
  int first_level_;
  int WTA_K_;
#if CV_VERSION_MAJOR == 3
  int score_type_;
#else
  cv::ORB::ScoreType score_type_;
#endif
  int patch_sze_;
  int fast_threshold_;
  //////////////////////////////////////////////////////////////////////////////

  ////////////////////////////// PGO solver params /////////////////////////////
  double pgo_rot_threshold_;
  double pgo_trans_threshold_;
  //////////////////////////////////////////////////////////////////////////////
};

}  // namespace VIO
