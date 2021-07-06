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
  tracker_params_.parseYAML(filepath);

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
  yaml_parser.getYamlParam("matcher_type", &matcher_type_);
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
  yaml_parser.getYamlParam("pgo_rot_threshold", &pgo_rot_threshold_);
  yaml_parser.getYamlParam("pgo_trans_threshold", &pgo_trans_threshold_);

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

                        "pgo_rot_threshold_: ",
                        pgo_rot_threshold_,
                        "pgo_trans_threshold_: ",
                        pgo_trans_threshold_);
  LOG(INFO) << out.str();
}
}  // namespace VIO
