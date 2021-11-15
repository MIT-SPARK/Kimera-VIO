/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionImuTrackerParams.cpp
 * @brief  Class to parse, print, and store the parameters of the tracker.
 * @author Marcus Abate
 */

#include "kimera-vio/frontend/VisionImuTrackerParams.h"

#include <string>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

TrackerParams::TrackerParams() : PipelineParams("Tracker Parameters") {}

void TrackerParams::print() const {
  std::stringstream out;
  PipelineParams::print(out,
                        // Tracker params
                        "klt_win_size_: ",
                        klt_win_size_,
                        "klt_max_iter_: ",
                        klt_max_iter_,
                        "klt_max_level_: ",
                        klt_max_level_,
                        "klt_eps_: ",
                        klt_eps_,
                        "max_feature_track_age_: ",
                        max_feature_track_age_,
                        "Optical Flow Predictor Type",
                        VIO::to_underlying(optical_flow_predictor_type_),
                        // RANSAC params
                        "minNrMonoInliers_: ",
                        minNrMonoInliers_,
                        "minNrStereoInliers_: ",
                        minNrStereoInliers_,
                        "ransac_threshold_mono_: ",
                        ransac_threshold_mono_,
                        "ransac_threshold_stereo_: ",
                        ransac_threshold_stereo_,
                        "ransac_use_1point_stereo_: ",
                        ransac_use_1point_stereo_,
                        "ransac_use_2point_mono_: ",
                        ransac_use_2point_mono_,
                        "ransac_max_iterations_: ",
                        ransac_max_iterations_,
                        "ransac_probability_: ",
                        ransac_probability_,
                        "ransac_randomize_: ",
                        ransac_randomize_,
                        "2D2D Algorithm",
                        VIO::to_underlying(pose_2d2d_algorithm_),
                        "Optimize 2D2D Pose",
                        optimize_2d2d_pose_from_inliers_,
                        "Optimize 3D3D Pose",
                        optimize_3d3d_pose_from_inliers_,
                        "PnP Algorithm",
                        VIO::to_underlying(pnp_algorithm_),
                        "Min PnP inliers count: ",
                        min_pnp_inliers_,
                        "Ransac threshold PnP: ",
                        ransac_threshold_pnp_,
                        "Optimize 2D3D Pose",
                        optimize_2d3d_pose_from_inliers_,
                        // OTHER parameters
                        "disparityThreshold_: ",
                        disparityThreshold_);
  LOG(INFO) << out.str();
}

bool TrackerParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);
  yaml_parser.getYamlParam("klt_win_size", &klt_win_size_);
  CHECK_GT(klt_win_size_, 0);
  yaml_parser.getYamlParam("klt_max_iter", &klt_max_iter_);
  CHECK_GT(klt_max_iter_, 0);
  yaml_parser.getYamlParam("klt_max_level", &klt_max_level_);
  CHECK_GE(klt_max_level_, 0);
  yaml_parser.getYamlParam("klt_eps", &klt_eps_);
  CHECK_GT(klt_eps_, 0);

  int max_feature_track_age;
  yaml_parser.getYamlParam("maxFeatureAge", &max_feature_track_age);
  CHECK_GT(max_feature_track_age, 2);
  max_feature_track_age_ = static_cast<size_t>(max_feature_track_age);

  yaml_parser.getYamlParam("minNrMonoInliers", &minNrMonoInliers_);
  yaml_parser.getYamlParam("minNrStereoInliers", &minNrStereoInliers_);
  yaml_parser.getYamlParam("min_pnp_inliers", &min_pnp_inliers_);
  yaml_parser.getYamlParam("ransac_threshold_mono", &ransac_threshold_mono_);
  yaml_parser.getYamlParam("ransac_threshold_stereo",
                           &ransac_threshold_stereo_);
  yaml_parser.getYamlParam("ransac_threshold_pnp", &ransac_threshold_pnp_);
  yaml_parser.getYamlParam("ransac_max_iterations", &ransac_max_iterations_);
  yaml_parser.getYamlParam("ransac_probability", &ransac_probability_);
  yaml_parser.getYamlParam("ransac_randomize", &ransac_randomize_);
  yaml_parser.getYamlParam("ransac_use_1point_stereo",
                           &ransac_use_1point_stereo_);
  yaml_parser.getYamlParam("ransac_use_2point_mono", &ransac_use_2point_mono_);

  int pose_2d2d_algorithm;
  yaml_parser.getYamlParam("2d2d_algorithm", &pose_2d2d_algorithm);
  pose_2d2d_algorithm_ = static_cast<Pose2d2dAlgorithm>(pose_2d2d_algorithm);

  yaml_parser.getYamlParam("optimize_2d2d_pose_from_inliers",
                           &optimize_2d2d_pose_from_inliers_);
  yaml_parser.getYamlParam("optimize_3d3d_pose_from_inliers",
                           &optimize_3d3d_pose_from_inliers_);

  int pnp_algorithm;
  yaml_parser.getYamlParam("pnp_algorithm", &pnp_algorithm);
  pnp_algorithm_ = static_cast<Pose3d2dAlgorithm>(pnp_algorithm);

  int optical_flow_predictor_type;
  yaml_parser.getYamlParam("optical_flow_predictor_type", &optical_flow_predictor_type);
  optical_flow_predictor_type_ =
      static_cast<OpticalFlowPredictorType>(optical_flow_predictor_type);

  yaml_parser.getYamlParam("disparityThreshold", &disparityThreshold_);

  yaml_parser.getYamlParam("optimize_2d3d_pose_from_inliers",
                           &optimize_2d3d_pose_from_inliers_);

  return true;
}

bool TrackerParams::equals(const TrackerParams& tp2, double tol) const {
  return (klt_win_size_ == tp2.klt_win_size_) &&
         (klt_max_iter_ == tp2.klt_max_iter_) &&
         (klt_max_level_ == tp2.klt_max_level_) &&
         (fabs(klt_eps_ - tp2.klt_eps_) <= tol) &&
         (max_feature_track_age_ == tp2.max_feature_track_age_) &&
         // RANSAC parameters
         (minNrMonoInliers_ == tp2.minNrMonoInliers_) &&
         (minNrStereoInliers_ == tp2.minNrStereoInliers_) &&
         (fabs(ransac_threshold_mono_ - tp2.ransac_threshold_mono_) <= tol) &&
         (fabs(ransac_threshold_stereo_ - tp2.ransac_threshold_stereo_) <=
          tol) &&
         (ransac_use_1point_stereo_ == tp2.ransac_use_1point_stereo_) &&
         (ransac_use_2point_mono_ == tp2.ransac_use_2point_mono_) &&
         (ransac_max_iterations_ == tp2.ransac_max_iterations_) &&
         (fabs(ransac_probability_ - tp2.ransac_probability_) <= tol) &&
         (ransac_randomize_ == tp2.ransac_randomize_) &&
         // others:
         (optical_flow_predictor_type_ == tp2.optical_flow_predictor_type_) &&
         (pose_2d2d_algorithm_ == tp2.pose_2d2d_algorithm_) &&
         (pnp_algorithm_ == tp2.pnp_algorithm_) &&
         (min_pnp_inliers_ == tp2.min_pnp_inliers_) &&
         (ransac_threshold_pnp_ == tp2.ransac_threshold_pnp_) &&
         (fabs(disparityThreshold_ - tp2.disparityThreshold_) <= tol);
}

}  // namespace VIO
