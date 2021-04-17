/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file VisionImuFrontendParams.cpp
 * @brief Parameters for the visual Frontend of the pipeline
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/VisionImuFrontendParams.h"

#include <string>
#include <utility>

#include <glog/logging.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/pipeline/PipelineParams.h"

namespace VIO {

FrontendParams::FrontendParams() : PipelineParams("Frontend Parameters") {}

void FrontendParams::print() const {
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
                        "maxFeatureAge_: ",
                        maxFeatureAge_,
                        "Optical Flow Predictor Type",
                        VIO::to_underlying(optical_flow_predictor_type_),
                        // RANSAC params
                        "useRANSAC_: ",
                        useRANSAC_,
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
                        // "** STEREO tracker parameters **\n"
                        "intra_keyframe_time_: ",
                        intra_keyframe_time_ns_,
                        "minNumberFeatures_: ",
                        min_number_features_,
                        "useStereoTracking_: ",
                        useStereoTracking_,
                        // OTHER parameters
                        "disparityThreshold_: ",
                        disparityThreshold_);
  LOG(INFO) << out.str();

  feature_detector_params_.print();

  if (useStereoTracking_) {
    stereo_matching_params_.print();
  }
}

bool FrontendParams::parseYAML(const std::string& filepath) {
  stereo_matching_params_.parseYAML(filepath);
  feature_detector_params_.parseYAML(filepath);

  YamlParser yaml_parser(filepath);
  yaml_parser.getYamlParam("klt_win_size", &klt_win_size_);
  yaml_parser.getYamlParam("klt_max_iter", &klt_max_iter_);
  yaml_parser.getYamlParam("klt_max_level", &klt_max_level_);
  yaml_parser.getYamlParam("klt_eps", &klt_eps_);
  yaml_parser.getYamlParam("maxFeatureAge", &maxFeatureAge_);

  yaml_parser.getYamlParam("useRANSAC", &useRANSAC_);
  yaml_parser.getYamlParam("minNrMonoInliers", &minNrMonoInliers_);
  yaml_parser.getYamlParam("minNrStereoInliers", &minNrStereoInliers_);
  yaml_parser.getYamlParam("ransac_threshold_mono", &ransac_threshold_mono_);
  yaml_parser.getYamlParam("ransac_threshold_stereo",
                           &ransac_threshold_stereo_);
  yaml_parser.getYamlParam("ransac_use_1point_stereo",
                           &ransac_use_1point_stereo_);
  yaml_parser.getYamlParam("ransac_use_2point_mono", &ransac_use_2point_mono_);
  yaml_parser.getYamlParam("ransac_max_iterations", &ransac_max_iterations_);
  yaml_parser.getYamlParam("ransac_probability", &ransac_probability_);
  yaml_parser.getYamlParam("ransac_randomize", &ransac_randomize_);

  // Given in seconds, needs to be converted to nanoseconds.
  double intra_keyframe_time_seconds;
  yaml_parser.getYamlParam("intra_keyframe_time", &intra_keyframe_time_seconds);
  intra_keyframe_time_ns_ =
      UtilsNumerical::SecToNsec(intra_keyframe_time_seconds);

  int min_number_features;
  yaml_parser.getYamlParam("minNumberFeatures", &min_number_features);
  min_number_features_ = static_cast<size_t>(min_number_features);
  yaml_parser.getYamlParam("useStereoTracking", &useStereoTracking_);
  yaml_parser.getYamlParam("disparityThreshold", &disparityThreshold_);

  int optical_flow_predictor_type;
  yaml_parser.getYamlParam("optical_flow_predictor_type",
                           &optical_flow_predictor_type);
  switch (optical_flow_predictor_type) {
    case VIO::to_underlying(OpticalFlowPredictorType::kNoPrediction): {
      optical_flow_predictor_type_ = OpticalFlowPredictorType::kNoPrediction;
      break;
    }
    case VIO::to_underlying(OpticalFlowPredictorType::kRotational): {
      optical_flow_predictor_type_ = OpticalFlowPredictorType::kRotational;
      break;
    }
    default: {
      LOG(FATAL) << "Unknown Optical Flow Predictor Type: "
                 << optical_flow_predictor_type;
    }
  }

  return true;
}

bool FrontendParams::equals(const FrontendParams& tp2, double tol) const {
  return (klt_win_size_ == tp2.klt_win_size_) &&
         (klt_max_iter_ == tp2.klt_max_iter_) &&
         (klt_max_level_ == tp2.klt_max_level_) &&
         (fabs(klt_eps_ - tp2.klt_eps_) <= tol) &&
         (maxFeatureAge_ == tp2.maxFeatureAge_) &&
         // stereo matching
         stereo_matching_params_.equals(tp2.stereo_matching_params_, tol) &&
         // RANSAC parameters
         (useRANSAC_ == tp2.useRANSAC_) &&
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
         // STEREO parameters:
         (fabs(intra_keyframe_time_ns_ - tp2.intra_keyframe_time_ns_) <= tol) &&
         (min_number_features_ == tp2.min_number_features_) &&
         (useStereoTracking_ == tp2.useStereoTracking_) &&
         // others:
         (optical_flow_predictor_type_ == tp2.optical_flow_predictor_type_) &&
         (fabs(disparityThreshold_ - tp2.disparityThreshold_) <= tol);
}

}  // namespace VIO
