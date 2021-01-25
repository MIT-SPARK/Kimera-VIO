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

LoopClosureDetectorParams::LoopClosureDetectorParams(
    int image_width,
    int image_height,
    double focal_length,
    cv::Point2d principle_point,

    bool use_nss,
    float alpha,
    int min_temporal_matches,
    int recent_frames_window,
    int max_db_results,
    float min_nss_factor,
    int min_matches_per_island,
    int max_intraisland_gap,
    int max_nrFrames_between_islands,
    int max_nrFrames_between_queries,

    GeomVerifOption geom_check,
    int min_correspondences,
    int max_ransac_iterations_mono,
    double ransac_probability_mono,
    double ransac_threshold_mono,
    bool ransac_randomize_mono,
    double ransac_inlier_threshold_mono,

    PoseRecoveryOption pose_recovery_option,
    int max_ransac_iterations_stereo,
    double ransac_probability_stereo,
    double ransac_threshold_stereo,
    bool ransac_randomize_stereo,
    double ransac_inlier_threshold_stereo,
    bool use_mono_rot,
    bool refine_pose,
    double lowe_ratio,
#if CV_VERSION_MAJOR == 3
    int matcher_type,
#else
    cv::DescriptorMatcher::MatcherType matcher_type,
#endif

    int nfeatures,
    float scale_factor,
    int nlevels,
    int edge_threshold,
    int first_level,
    int WTA_K,
#if CV_VERSION_MAJOR == 3
    int score_type,
#else
    cv::ORB::ScoreType score_type,
#endif
    int patch_sze,
    int fast_threshold,

    double betweenRotationPrecision,
    double betweenTranslationPrecision,

    double pgo_rot_threshold,
    double pgo_trans_threshold)
    : PipelineParams("Loop Closure Parameters"),
      image_width_(image_width),
      image_height_(image_height),
      focal_length_(focal_length),
      principle_point_(principle_point),

      use_nss_(use_nss),
      alpha_(alpha),
      min_temporal_matches_(min_temporal_matches),
      recent_frames_window_(recent_frames_window),
      max_db_results_(max_db_results),
      min_nss_factor_(min_nss_factor),
      min_matches_per_island_(min_matches_per_island),
      max_intraisland_gap_(max_intraisland_gap),
      max_nrFrames_between_islands_(max_nrFrames_between_islands),
      max_nrFrames_between_queries_(max_nrFrames_between_queries),

      geom_check_(geom_check),
      min_correspondences_(min_correspondences),
      max_ransac_iterations_mono_(max_ransac_iterations_mono),
      ransac_probability_mono_(ransac_probability_mono),
      ransac_threshold_mono_(ransac_threshold_mono),
      ransac_randomize_mono_(ransac_randomize_mono),
      ransac_inlier_threshold_mono_(ransac_inlier_threshold_mono),

      pose_recovery_option_(pose_recovery_option),
      max_ransac_iterations_stereo_(max_ransac_iterations_stereo),
      ransac_probability_stereo_(ransac_probability_stereo),
      ransac_threshold_stereo_(ransac_threshold_stereo),
      ransac_randomize_stereo_(ransac_randomize_stereo),
      ransac_inlier_threshold_stereo_(ransac_inlier_threshold_stereo),
      use_mono_rot_(use_mono_rot),
      refine_pose_(refine_pose),

      lowe_ratio_(lowe_ratio),
      matcher_type_(matcher_type),

      nfeatures_(nfeatures),
      scale_factor_(scale_factor),
      nlevels_(nlevels),
      edge_threshold_(edge_threshold),
      first_level_(first_level),
      WTA_K_(WTA_K),
      score_type_(score_type),
      patch_sze_(patch_sze),
      fast_threshold_(fast_threshold),

      betweenRotationPrecision_(betweenRotationPrecision),
      betweenTranslationPrecision_(betweenTranslationPrecision),

      pgo_rot_threshold_(pgo_rot_threshold),
      pgo_trans_threshold_(pgo_trans_threshold) {
  // Trivial sanity checks:
  CHECK(alpha_ > 0);
  CHECK(nfeatures_ >= 100);  // TODO(marcus): add more checks, change this one
}

bool LoopClosureDetectorParams::parseYAML(const std::string& filepath) {
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

  int geom_check_id;
  yaml_parser.getYamlParam("geom_check_id", &geom_check_id);
  switch (geom_check_id) {
    case static_cast<unsigned int>(GeomVerifOption::NISTER):
      geom_check_ = GeomVerifOption::NISTER;
      break;
    case static_cast<unsigned int>(GeomVerifOption::NONE):
      geom_check_ = GeomVerifOption::NONE;
      break;
    default:
      throw std::runtime_error("LCDparams parseYAML: wrong geom_check_id");
      break;
  }
  yaml_parser.getYamlParam("min_correspondences", &min_correspondences_);
  yaml_parser.getYamlParam("max_ransac_iterations_mono",
                           &max_ransac_iterations_mono_);
  yaml_parser.getYamlParam("ransac_probability_mono",
                           &ransac_probability_mono_);
  yaml_parser.getYamlParam("ransac_threshold_mono", &ransac_threshold_mono_);
  yaml_parser.getYamlParam("ransac_randomize_mono", &ransac_randomize_mono_);
  yaml_parser.getYamlParam("ransac_inlier_threshold_mono",
                           &ransac_inlier_threshold_mono_);

  int pose_recovery_option_id;
  yaml_parser.getYamlParam("pose_recovery_option_id", &pose_recovery_option_id);
  switch (pose_recovery_option_id) {
    case static_cast<unsigned int>(PoseRecoveryOption::RANSAC_ARUN):
      pose_recovery_option_ = PoseRecoveryOption::RANSAC_ARUN;
      break;
    case static_cast<unsigned int>(PoseRecoveryOption::GIVEN_ROT):
      pose_recovery_option_ = PoseRecoveryOption::GIVEN_ROT;
      break;
    default:
      throw std::runtime_error(
          "LCDparams parseYAML: wrong pose_recovery_option_id");
      break;
  }
  yaml_parser.getYamlParam("max_ransac_iterations_stereo",
                           &max_ransac_iterations_stereo_);
  yaml_parser.getYamlParam("ransac_probability_stereo",
                           &ransac_probability_stereo_);
  yaml_parser.getYamlParam("ransac_threshold_stereo",
                           &ransac_threshold_stereo_);
  yaml_parser.getYamlParam("ransac_randomize_stereo",
                           &ransac_randomize_stereo_);
  yaml_parser.getYamlParam("ransac_inlier_threshold_stereo",
                           &ransac_inlier_threshold_stereo_);
  yaml_parser.getYamlParam("use_mono_rot", &use_mono_rot_);
  yaml_parser.getYamlParam("refine_pose", &refine_pose_);
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
  yaml_parser.getYamlParam("betweenRotationPrecision", &betweenRotationPrecision_);
  yaml_parser.getYamlParam("betweenTranslationPrecision", &betweenTranslationPrecision_);
  yaml_parser.getYamlParam("pgo_rot_threshold", &pgo_rot_threshold_);
  yaml_parser.getYamlParam("pgo_trans_threshold", &pgo_trans_threshold_);

  return true;
}

void LoopClosureDetectorParams::print() const {
  // TODO(marcus): print all params
  std::stringstream out;
  PipelineParams::print(out,
                        "image_width_: ",
                        image_width_,
                        "image_height_: ",
                        image_height_,
                        "focal_length_: ",
                        focal_length_,
                        "principle_point_: ",
                        principle_point_,

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

                        "geom_check_: ",
                        static_cast<unsigned int>(geom_check_),
                        "min_correspondences_: ",
                        min_correspondences_,
                        "max_ransac_iterations_mono_: ",
                        max_ransac_iterations_mono_,

                        "ransac_probability_mono_: ",
                        ransac_probability_mono_,
                        "ransac_threshold_mono_: ",
                        ransac_threshold_mono_,
                        "ransac_randomize_mono_: ",
                        ransac_randomize_mono_,
                        "ransac_inlier_threshold_mono_: ",
                        ransac_inlier_threshold_mono_,

                        "pose_recovery_option_: ",
                        static_cast<unsigned int>(pose_recovery_option_),
                        "max_ransac_iterations_stereo_: ",
                        max_ransac_iterations_stereo_,

                        "ransac_probability_stereo_: ",
                        ransac_probability_stereo_,
                        "ransac_threshold_stereo_: ",
                        ransac_threshold_stereo_,
                        "ransac_randomize_stereo_: ",
                        ransac_randomize_stereo_,
                        "ransac_inlier_threshold_stereo_: ",
                        ransac_inlier_threshold_stereo_,
                        "use_mono_rot_:",
                        use_mono_rot_,
                        "refine_pose_:",
                        refine_pose_,
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
