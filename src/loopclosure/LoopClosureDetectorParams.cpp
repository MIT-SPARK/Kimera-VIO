/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoopClosureDetectorParams.cpp
 * @brief
 * @author Antoni Rosinol
 */

#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"

namespace VIO {

LoopClosureDetectorParams::LoopClosureDetectorParams()
    : PipelineParams("Loop Closure Parameters") {
  // Trivial sanity checks:
  CHECK(alpha_ > 0);
  CHECK(nfeatures_ >= 100);  // TODO(marcus): add more checks, change this one
}

bool LoopClosureDetectorParams::parseYAML(const std::string& filepath) {
  // Use defaults and override the outlier-rejection params required
  // TODO(marcus + toni): after fix/frontend is merged, just use the
  // OutlierRejection class as a member and parse params here.
  // tracker_params_.parseYAML(filepath);

  YamlParser yaml_parser(filepath);

  yaml_parser.getYamlParam("use_nss", &use_nss_);
  yaml_parser.getYamlParam("alpha", &alpha_);
  yaml_parser.getYamlParam("min_temporal_matches", &min_temporal_matches_);
  yaml_parser.getYamlParam("recent_frames_window", &recent_frames_window_);
  yaml_parser.getYamlParam("max_db_results", &max_db_results_);
  yaml_parser.getYamlParam("min_nss_factor", &min_nss_factor_);
  yaml_parser.getYamlParam("min_matches_per_island", &min_matches_per_island_);
  yaml_parser.getYamlParam("max_intraisland_gap", &max_intraisland_gap_);
  yaml_parser.getYamlParam("max_nrFrames_between_islands",
                          &max_nrFrames_between_islands_);
  yaml_parser.getYamlParam("max_nrFrames_between_queries",
                          &max_nrFrames_between_queries_);
  yaml_parser.getYamlParam("refine_pose", &refine_pose_);
  yaml_parser.getYamlParam("use_pnp_pose_recovery", &use_pnp_pose_recovery_);
  yaml_parser.getYamlParam("lowe_ratio", &lowe_ratio_);

  int matcher_type_id;
  yaml_parser.getYamlParam("matcher_type", &matcher_type_id);
#if CV_VERSION_MAJOR == 3
    matcher_type_ = matcher_type_id;
#else
    matcher_type_ = 
      static_cast<cv::DescriptorMatcher::MatcherType>(matcher_type_id);
#endif
  
  yaml_parser.getYamlParam("nfeatures", &nfeatures_);
  yaml_parser.getYamlParam("scale_factor", &scale_factor_);
  yaml_parser.getYamlParam("nlevels", &nlevels_);
  yaml_parser.getYamlParam("edge_threshold", &edge_threshold_);
  yaml_parser.getYamlParam("first_level", &first_level_);
  yaml_parser.getYamlParam("WTA_K", &WTA_K_);

  int score_type_id;
  yaml_parser.getYamlParam("score_type_id", &score_type_id);
  switch (score_type_id) {
    case 0:
#if CV_VERSION_MAJOR == 3
      score_type_ = cv::ORB::HARRIS_SCORE;
#else
      score_type_ = cv::ORB::ScoreType::HARRIS_SCORE;
#endif
      break;
    // TODO(marcus): add the rest of the options here
    default:
      throw std::runtime_error("LCDparams parseYAML: wrong score_type_id");
      break;
  }
  yaml_parser.getYamlParam("patch_sze", &patch_sze_);
  yaml_parser.getYamlParam("fast_threshold", &fast_threshold_);
  yaml_parser.getYamlParam("betweenRotationPrecision",
                          &betweenRotationPrecision_);
  yaml_parser.getYamlParam("betweenTranslationPrecision",
                          &betweenTranslationPrecision_);
  yaml_parser.getYamlParam("odom_rot_threshold", &odom_rot_threshold_);
  yaml_parser.getYamlParam("odom_trans_threshold", &odom_trans_threshold_);
  yaml_parser.getYamlParam("pcm_rot_threshold", &pcm_rot_threshold_);
  yaml_parser.getYamlParam("pcm_trans_threshold", &pcm_trans_threshold_);
  yaml_parser.getYamlParam("gnc_alpha", &gnc_alpha_);

  yaml_parser.getYamlParam("max_lc_cached_before_optimize",
                           &max_lc_cached_before_optimize_);

  // Now manually change required parameters in tracker
  yaml_parser.getYamlParam("disparity_threshold",
                           &tracker_params_.disparityThreshold_);

  yaml_parser.getYamlParam("min_nr_2d2d_inliers",
                           &tracker_params_.minNrMonoInliers_);
  yaml_parser.getYamlParam("min_nr_3d3d_inliers",
                           &tracker_params_.minNrStereoInliers_);
  yaml_parser.getYamlParam("min_nr_2d3d_inliers",
                           &tracker_params_.min_pnp_inliers_);

  yaml_parser.getYamlParam("ransac_threshold_2d2d",
                           &tracker_params_.ransac_threshold_mono_);
  yaml_parser.getYamlParam("ransac_threshold_3d3d",
                           &tracker_params_.ransac_threshold_stereo_);
  yaml_parser.getYamlParam("ransac_threshold_2d3d",
                           &tracker_params_.ransac_threshold_pnp_);
  yaml_parser.getYamlParam("ransac_max_iterations",
                           &tracker_params_.ransac_max_iterations_);
  yaml_parser.getYamlParam("ransac_probability",
                           &tracker_params_.ransac_probability_);
  yaml_parser.getYamlParam("ransac_randomize",
                           &tracker_params_.ransac_randomize_);

  yaml_parser.getYamlParam("ransac_use_1point_3d3d",
                           &tracker_params_.ransac_use_1point_stereo_);
  yaml_parser.getYamlParam("ransac_use_2point_2d2d",
                           &tracker_params_.ransac_use_2point_mono_);

  int pose_2d2d_algorithm;
  yaml_parser.getYamlParam("ransac_2d2d_algorithm", &pose_2d2d_algorithm);
  tracker_params_.pose_2d2d_algorithm_ =
      static_cast<Pose2d2dAlgorithm>(pose_2d2d_algorithm);

  int pnp_algorithm;
  yaml_parser.getYamlParam("ransac_2d3d_algorithm", &pnp_algorithm);
  tracker_params_.pnp_algorithm_ =
      static_cast<Pose3d2dAlgorithm>(pnp_algorithm);

  yaml_parser.getYamlParam("optimize_2d2d_pose_from_inliers",
                           &tracker_params_.optimize_2d2d_pose_from_inliers_);
  yaml_parser.getYamlParam("optimize_3d3d_pose_from_inliers",
                           &tracker_params_.optimize_3d3d_pose_from_inliers_);

  return true;
}

void LoopClosureDetectorParams::print() const {
  tracker_params_.print();

  std::stringstream out;
  PipelineParams::print(out,
                        "use_nss_: ",
                        use_nss_,
                        "alpha_: ",
                        alpha_,
                        "min_temporal_matches_: ",
                        min_temporal_matches_,
                        "recent_frames_window_: ",
                        recent_frames_window_,
                        "max_db_results_: ",
                        max_db_results_,
                        "max_db_results_: ",
                        max_db_results_,
                        "min_nss_factor_: ",
                        min_nss_factor_,
                        "min_matches_per_island_: ",
                        min_matches_per_island_,
                        "max_intraisland_gap_: ",
                        max_intraisland_gap_,
                        "max_nrFrames_between_islands_: ",
                        max_nrFrames_between_islands_,
                        "max_nrFrames_between_queries_: ",
                        max_nrFrames_between_queries_,

                        "refine_pose_:",
                        refine_pose_,
                        "use_pnp_pose_recovery",
                        use_pnp_pose_recovery_,
                        "lowe_ratio_: ",
                        lowe_ratio_,
                        "matcher_type_:",
                        static_cast<unsigned int>(matcher_type_),

                        "nfeatures_: ",
                        nfeatures_,
                        "scale_factor_: ",
                        scale_factor_,
                        "nlevels_: ",
                        nlevels_,
                        "edge_threshold_: ",
                        edge_threshold_,
                        "first_level_: ",
                        first_level_,
                        "WTA_K_: ",
                        WTA_K_,
                        "score_type_: ",
                        static_cast<unsigned int>(score_type_),
                        "patch_sze_: ",
                        patch_sze_,
                        "fast_threshold_: ",
                        fast_threshold_,

                        "betweenRotationPrecision_: ",
                        betweenRotationPrecision_,
                        "betweenTranslationPrecision_: ",
                        betweenTranslationPrecision_,

                        "odom_rot_threshold_: ",
                        odom_rot_threshold_,
                        "odom_trans_threshold_: ",
                        odom_trans_threshold_,
                        "pcm_rot_threshold_: ",
                        pcm_rot_threshold_,
                        "pcm_trans_threshold_: ",
                        pcm_trans_threshold_,
                        "gnc_alpha_",
                        gnc_alpha_,
                        "max_lc_cached_before_optimize_",
                        max_lc_cached_before_optimize_);
  LOG(INFO) << out.str();
}

bool LoopClosureDetectorParams::equals(const LoopClosureDetectorParams& lp2, double tol) const {
  return tracker_params_.equals(lp2.tracker_params_, tol) &&
         (use_nss_ == lp2.use_nss_) && (fabs(alpha_ - lp2.alpha_) <= tol) &&
         (min_temporal_matches_ == lp2.min_temporal_matches_) &&
         (recent_frames_window_ == lp2.recent_frames_window_) &&
         (max_db_results_ == lp2.max_db_results_) &&
         (fabs(min_nss_factor_ - lp2.min_nss_factor_) <= tol) &&
         (min_matches_per_island_ == lp2.min_matches_per_island_) &&
         (max_intraisland_gap_ == lp2.max_intraisland_gap_) &&
         (max_nrFrames_between_islands_ == lp2.max_nrFrames_between_islands_) &&
         (max_nrFrames_between_queries_ == lp2.max_nrFrames_between_queries_) &&

         (refine_pose_ == lp2.refine_pose_) &&
         (use_pnp_pose_recovery_ == lp2.use_pnp_pose_recovery_) &&
         (fabs(lowe_ratio_ - lp2.lowe_ratio_) <= tol) &&
         (matcher_type_ == lp2.matcher_type_) &&

         (nfeatures_ == lp2.nfeatures_) &&
         (fabs(scale_factor_ - lp2.scale_factor_) <= tol) &&
         (nlevels_ == lp2.nlevels_) &&
         (edge_threshold_ == lp2.edge_threshold_) &&
         (first_level_ == lp2.first_level_) && (WTA_K_ == lp2.WTA_K_) &&
         (score_type_ == lp2.score_type_) && (patch_sze_ == lp2.patch_sze_) &&
         (fast_threshold_ == lp2.fast_threshold_) &&

         (fabs(betweenRotationPrecision_ - lp2.betweenRotationPrecision_) <=
          tol) &&
         (fabs(betweenTranslationPrecision_ -
               lp2.betweenTranslationPrecision_) <= tol) &&

         (fabs(odom_rot_threshold_ - lp2.odom_rot_threshold_) <= tol) &&
         (fabs(odom_trans_threshold_ - lp2.odom_trans_threshold_) <= tol) &&
         (fabs(pcm_rot_threshold_ - lp2.pcm_rot_threshold_) <= tol) &&
         (fabs(pcm_trans_threshold_ - lp2.pcm_trans_threshold_) <= tol) &&
         (fabs(gnc_alpha_ - lp2.gnc_alpha_) <= tol) &&
         (max_lc_cached_before_optimize_ == lp2.max_lc_cached_before_optimize_);
}

}  // namespace VIO
