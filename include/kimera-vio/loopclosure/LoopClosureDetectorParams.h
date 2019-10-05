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

#include <opencv/cv.hpp>

#include "kimera-vio/YamlParser.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"

namespace VIO {

class LoopClosureDetectorParams {
 public:
  LoopClosureDetectorParams(
      int image_width = 752,
      int image_height = 480,
      double focal_length = 1.0,
      cv::Point2d principle_point = cv::Point2d(0.0, 0.0),

      bool use_nss = true,
      float alpha = 0.1,
      int min_temporal_matches = 3,
      int dist_local = 20,
      int max_db_results = 50,
      float min_nss_factor = 0.005,
      int min_matches_per_group = 1,
      int max_intragroup_gap = 3,
      int max_distance_between_groups = 3,
      int max_distance_between_queries = 2,

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

      double lowe_ratio = 0.7,
      int matcher_type = 4,

      int nfeatures = 500,
      float scale_factor = 1.2f,
      int nlevels = 8,
      int edge_threshold = 31,
      int first_level = 0,
      int WTA_K = 2,
      int score_type = cv::ORB::HARRIS_SCORE,
      int patch_sze = 31,
      int fast_threshold = 20,

      double pgo_rot_threshold = 0.01,
      double pgo_trans_threshold = 0.1)
      : image_width_(image_width),
        image_height_(image_height),
        focal_length_(focal_length),
        principle_point_(principle_point),

        use_nss_(use_nss),
        alpha_(alpha),
        min_temporal_matches_(min_temporal_matches),
        dist_local_(dist_local),
        max_db_results_(max_db_results),
        min_nss_factor_(min_nss_factor),
        min_matches_per_group_(min_matches_per_group),
        max_intragroup_gap_(max_intragroup_gap),
        max_distance_between_groups_(max_distance_between_groups),
        max_distance_between_queries_(max_distance_between_queries),

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

        pgo_rot_threshold_(pgo_rot_threshold),
        pgo_trans_threshold_(pgo_trans_threshold) {
    // Trivial sanity checks:
    CHECK(alpha_ > 0);
    CHECK(nfeatures_ >= 100);  // TODO(marcus): add more checks, change this one
  }

 public:
  /////////////////////////// Camera intrinsic Params //////////////////////////
  int image_width_;
  int image_height_;
  double focal_length_;          // Focal length of camera
  cv::Point2d principle_point_;  // Principle point of the camera
  //////////////////////////////////////////////////////////////////////////////

  //////////////////////////// Loop Detection Params ///////////////////////////
  bool use_nss_;              // Use normalized similarity score?
  float alpha_;               // Alpha threshold for matches
  int min_temporal_matches_;  // Min consistent matches to pass temporal check
  int dist_local_;            // Distance between entries to be consider a match
  int max_db_results_;    // Max number of results from db queries to consider
  float min_nss_factor_;  // Min raw score between entries to consider a match
  int min_matches_per_group_;  // Min number of close matches in a group
  int max_intragroup_gap_;     // Max separation btwn matches of the same group
  int max_distance_between_groups_;   // Max separation between groups
  int max_distance_between_queries_;  // Max separation between two queries
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
  //////////////////////////////////////////////////////////////////////////////

  ///////////////////////// ORB feature matching params ////////////////////////
  double lowe_ratio_;
  int matcher_type_;
  //////////////////////////////////////////////////////////////////////////////

  ///////////////////////// ORB feature detector params ////////////////////////
  int nfeatures_;
  float scale_factor_;
  int nlevels_;
  int edge_threshold_;
  int first_level_;
  int WTA_K_;
  int score_type_;
  int patch_sze_;
  int fast_threshold_;
  //////////////////////////////////////////////////////////////////////////////

  ////////////////////////////// PGO solver params /////////////////////////////
  double pgo_rot_threshold_;
  double pgo_trans_threshold_;
  //////////////////////////////////////////////////////////////////////////////

  virtual ~LoopClosureDetectorParams() = default;

  // NOTE: we cannot parse width, height principe pt and focal length from here.
  // Those are done via setIntrinsics() in real time in the first StereoFrame.
  bool parseYAML(const std::string& filepath) {
    yaml_parser_ = std::make_shared<YamlParser>(filepath);

    yaml_parser_->getYamlParam("use_nss", &use_nss_);
    yaml_parser_->getYamlParam("alpha", &alpha_);
    yaml_parser_->getYamlParam("min_temporal_matches", &min_temporal_matches_);
    yaml_parser_->getYamlParam("dist_local", &dist_local_);
    yaml_parser_->getYamlParam("max_db_results", &max_db_results_);
    yaml_parser_->getYamlParam("min_nss_factor", &min_nss_factor_);
    yaml_parser_->getYamlParam("min_matches_per_group",
                               &min_matches_per_group_);
    yaml_parser_->getYamlParam("max_intragroup_gap", &max_intragroup_gap_);
    yaml_parser_->getYamlParam("max_distance_between_groups",
                               &max_distance_between_groups_);
    yaml_parser_->getYamlParam("max_distance_between_queries",
                               &max_distance_between_queries_);

    int geom_check_id;
    yaml_parser_->getYamlParam("geom_check_id", &geom_check_id);
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
    yaml_parser_->getYamlParam("min_correspondences", &min_correspondences_);
    yaml_parser_->getYamlParam("max_ransac_iterations_mono",
                               &max_ransac_iterations_mono_);
    yaml_parser_->getYamlParam("ransac_probability_mono",
                               &ransac_probability_mono_);
    yaml_parser_->getYamlParam("ransac_threshold_mono",
                               &ransac_threshold_mono_);
    yaml_parser_->getYamlParam("ransac_randomize_mono",
                               &ransac_randomize_mono_);
    yaml_parser_->getYamlParam("ransac_inlier_threshold_mono",
                               &ransac_inlier_threshold_mono_);

    int pose_recovery_option_id;
    yaml_parser_->getYamlParam("pose_recovery_option_id",
                               &pose_recovery_option_id);
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
    yaml_parser_->getYamlParam("max_ransac_iterations_stereo",
                               &max_ransac_iterations_stereo_);
    yaml_parser_->getYamlParam("ransac_probability_stereo",
                               &ransac_probability_stereo_);
    yaml_parser_->getYamlParam("ransac_threshold_stereo",
                               &ransac_threshold_stereo_);
    yaml_parser_->getYamlParam("ransac_randomize_stereo",
                               &ransac_randomize_stereo_);
    yaml_parser_->getYamlParam("ransac_inlier_threshold_stereo",
                               &ransac_inlier_threshold_stereo_);
    yaml_parser_->getYamlParam("use_mono_rot", &use_mono_rot_);
    yaml_parser_->getYamlParam("lowe_ratio", &lowe_ratio_);
    yaml_parser_->getYamlParam("matcher_type", &matcher_type_);
    yaml_parser_->getYamlParam("nfeatures", &nfeatures_);
    yaml_parser_->getYamlParam("scale_factor", &scale_factor_);
    yaml_parser_->getYamlParam("nlevels", &nlevels_);
    yaml_parser_->getYamlParam("edge_threshold", &edge_threshold_);
    yaml_parser_->getYamlParam("first_level", &first_level_);
    yaml_parser_->getYamlParam("WTA_K", &WTA_K_);

    int score_type_id;
    yaml_parser_->getYamlParam("score_type_id", &score_type_id);
    switch (score_type_id) {
      case 0:
        score_type_ = cv::ORB::HARRIS_SCORE;
        break;
      // TODO(marcus): add the rest of the options here
      default:
        throw std::runtime_error("LCDparams parseYAML: wrong score_type_id");
        break;
    }
    yaml_parser_->getYamlParam("patch_sze", &patch_sze_);
    yaml_parser_->getYamlParam("fast_threshold", &fast_threshold_);
    yaml_parser_->getYamlParam("pgo_rot_threshold", &pgo_rot_threshold_);
    yaml_parser_->getYamlParam("pgo_trans_threshold", &pgo_trans_threshold_);

    return true;
  }

  void print() const {
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
        << "max_ransac_iterations_mono_: " << max_ransac_iterations_mono_
        << '\n'
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
        << "ransac_inlier_threshold_stereo_: "
        << ransac_inlier_threshold_stereo_ << '\n'
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

 private:
  // TODO(Toni) Needs to be shared because we are copying params around, if
  // parsing was separated from actual params struct we would not have this
  // issue.
  std::shared_ptr<YamlParser> yaml_parser_;
};  // class LoopClosureDetectorParams

}  // namespace VIO
