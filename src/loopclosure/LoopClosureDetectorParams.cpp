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

bool LoopClosureDetectorParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);

  yaml_parser.getYamlParam("use_nss", &use_nss_);
  yaml_parser.getYamlParam("alpha", &alpha_);
  yaml_parser.getYamlParam("min_temporal_matches", &min_temporal_matches_);
  yaml_parser.getYamlParam("dist_local", &dist_local_);
  yaml_parser.getYamlParam("max_db_results", &max_db_results_);
  yaml_parser.getYamlParam("min_nss_factor", &min_nss_factor_);
  yaml_parser.getYamlParam("min_matches_per_group", &min_matches_per_group_);
  yaml_parser.getYamlParam("max_intragroup_gap", &max_intragroup_gap_);
  yaml_parser.getYamlParam("max_distance_between_groups",
                           &max_distance_between_groups_);
  yaml_parser.getYamlParam("max_distance_between_queries",
                           &max_distance_between_queries_);

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
      score_type_ = cv::ORB::HARRIS_SCORE;
      break;
    // TODO(marcus): add the rest of the options here
    default:
      throw std::runtime_error("LCDparams parseYAML: wrong score_type_id");
      break;
  }
  yaml_parser.getYamlParam("patch_sze", &patch_sze_);
  yaml_parser.getYamlParam("fast_threshold", &fast_threshold_);
  yaml_parser.getYamlParam("pgo_rot_threshold", &pgo_rot_threshold_);
  yaml_parser.getYamlParam("pgo_trans_threshold", &pgo_trans_threshold_);

  return true;
}

void LoopClosureDetectorParams::print() const {
  // TODO(marcus): print all params
  LOG(INFO)
      << "$$$$$$$$$$$$$$$$$$$$$ LCD PARAMETERS $$$$$$$$$$$$$$$$$$$$$\n"
      << "image_width_: " << image_width_ << '\n'
      << "image_height_: " << image_height_ << '\n'
      << "focal_length_: " << focal_length_ << '\n'
      << "principle_point_: " << principle_point_ << '\n'

      << "use_nss_: " << use_nss_ << '\n'
      << "alpha_: " << alpha_ << '\n'
      << "min_temporal_matches_: " << min_temporal_matches_ << '\n'
      << "dist_local_: " << dist_local_ << '\n'
      << "max_db_results_: " << max_db_results_ << '\n'
      << "max_db_results_: " << max_db_results_ << '\n'
      << "min_nss_factor_: " << min_nss_factor_ << '\n'
      << "min_matches_per_group_: " << min_matches_per_group_ << '\n'
      << "max_intragroup_gap_: " << max_intragroup_gap_ << '\n'
      << "max_distance_between_groups_: " << max_distance_between_groups_
      << '\n'
      << "max_distance_between_queries_: " << max_distance_between_queries_
      << '\n'

      << "geom_check_: " << static_cast<unsigned int>(geom_check_) << '\n'
      << "min_correspondences_: " << min_correspondences_ << '\n'
      << "max_ransac_iterations_mono_: " << max_ransac_iterations_mono_ << '\n'
      << "ransac_probability_mono_: " << ransac_probability_mono_ << '\n'
      << "ransac_threshold_mono_: " << ransac_threshold_mono_ << '\n'
      << "ransac_randomize_mono_: " << ransac_randomize_mono_ << '\n'
      << "ransac_inlier_threshold_mono_: " << ransac_inlier_threshold_mono_
      << '\n'

      << "pose_recovery_option_: "
      << static_cast<unsigned int>(pose_recovery_option_) << '\n'
      << "max_ransac_iterations_stereo_: " << max_ransac_iterations_stereo_
      << '\n'
      << "ransac_probability_stereo_: " << ransac_probability_stereo_ << '\n'
      << "ransac_threshold_stereo_: " << ransac_threshold_stereo_ << '\n'
      << "ransac_randomize_stereo_: " << ransac_randomize_stereo_ << '\n'
      << "ransac_inlier_threshold_stereo_: " << ransac_inlier_threshold_stereo_
      << '\n'
      << "use_mono_rot_:" << use_mono_rot_ << '\n'

      << "lowe_ratio_: " << lowe_ratio_ << '\n'
      << "matcher_type_:" << static_cast<unsigned int>(matcher_type_) << '\n'

      << "nfeatures_: " << nfeatures_ << '\n'
      << "scale_factor_: " << scale_factor_ << '\n'
      << "nlevels_: " << nlevels_ << '\n'
      << "edge_threshold_: " << edge_threshold_ << '\n'
      << "first_level_: " << first_level_ << '\n'
      << "WTA_K_: " << WTA_K_ << '\n'
      << "score_type_: " << score_type_ << '\n'
      << "patch_sze_: " << patch_sze_ << '\n'
      << "fast_threshold_: " << fast_threshold_ << '\n'

      << "pgo_rot_threshold_: " << pgo_rot_threshold_ << '\n'
      << "pgo_trans_threshold_: " << pgo_trans_threshold_;
}
}  // namespace VIO
