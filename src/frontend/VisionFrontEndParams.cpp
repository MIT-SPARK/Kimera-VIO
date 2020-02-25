/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file VisionFrontEndParams.cpp
 * @brief Parameters for the visual frontend of the pipeline
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/VisionFrontEndParams.h"

#include <string>
#include <utility>

#include <glog/logging.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/pipeline/PipelineParams.h"

namespace VIO {

SubPixelCornerFinderParams::SubPixelCornerFinderParams()
    : PipelineParams("SubPixelCornerFinder Parameters") {}

void SubPixelCornerFinderParams::print() const {
  LOG(INFO) << "************* SubPixel Corner Finder Params *****************\n"
            << "Termination criteria type: " << term_criteria_.type << '\n'
            << "max_iters_: " << term_criteria_.maxCount << '\n'
            << "epsilon_error_: " << term_criteria_.epsilon << '\n'
            << "window_size_: " << window_size_ << '\n'
            << "zero_zone_: " << zero_zone_ << '\n';
}

bool SubPixelCornerFinderParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);
  term_criteria_.type = CV_TERMCRIT_EPS + CV_TERMCRIT_ITER;
  yaml_parser.getYamlParam("max_iters", &term_criteria_.maxCount);
  yaml_parser.getYamlParam("epsilon_error", &term_criteria_.epsilon);
  int window_size = 0;
  yaml_parser.getYamlParam("window_size", &window_size);
  window_size_ = cv::Size(window_size, window_size);
  int zero_zone = -1;
  yaml_parser.getYamlParam("zero_zone", &zero_zone);
  zero_zone_ = cv::Size(zero_zone, zero_zone);
  return true;
}

bool SubPixelCornerFinderParams::equals(const SubPixelCornerFinderParams& tp2,
                                        double tol) const {
  return (term_criteria_.type == tp2.term_criteria_.type) &&
         (term_criteria_.maxCount == tp2.term_criteria_.maxCount) &&
         (term_criteria_.epsilon == tp2.term_criteria_.epsilon) &&
         (window_size_ == tp2.window_size_) && (zero_zone_ == tp2.zero_zone_);
}

VisionFrontEndParams::VisionFrontEndParams()
    : PipelineParams("Frontend Parameters") {}

void VisionFrontEndParams::print() const {
  LOG(INFO) << "****************** Feature Tracker Params *******************\n"
            << "** Feature tracking parameters **\n"
            << "klt_win_size_: " << klt_win_size_ << '\n'
            << "klt_max_iter_: " << klt_max_iter_ << '\n'
            << "klt_max_level_: " << klt_max_level_ << '\n'
            << "klt_eps_: " << klt_eps_ << '\n'
            << "maxFeatureAge_: " << maxFeatureAge_ << '\n'
            << "enable_subpixel_corner_finder: "
            << enable_subpixel_corner_finder_;

  if (enable_subpixel_corner_finder_) {
    subpixel_corner_finder_params_.print();
  }

  LOG(INFO) << "** Feature detection parameters **\n"
            << "maxFeaturesPerFrame_: " << maxFeaturesPerFrame_ << '\n'
            << "quality_level_: " << quality_level_ << '\n'
            << "min_distance_: " << min_distance_ << '\n'
            << "block_size_: " << block_size_ << '\n'
            << "use_harris_detector_: " << use_harris_detector_ << '\n'
            << "k_: " << k_;

  stereo_matching_params_.print();

  LOG(INFO)
      << "** RANSAC parameters **\n"
      << "useRANSAC_: " << useRANSAC_ << '\n'
      << "minNrMonoInliers_: " << minNrMonoInliers_ << '\n'
      << "minNrStereoInliers_: " << minNrStereoInliers_ << '\n'
      << "ransac_threshold_mono_: " << ransac_threshold_mono_ << '\n'
      << "ransac_threshold_stereo_: " << ransac_threshold_stereo_ << '\n'
      << "ransac_use_1point_stereo_: " << ransac_use_1point_stereo_ << '\n'
      << "ransac_use_2point_mono_: " << ransac_use_2point_mono_ << '\n'
      << "ransac_max_iterations_: " << ransac_max_iterations_ << '\n'
      << "ransac_probability_: " << ransac_probability_ << '\n'
      << "ransac_randomize_: " << ransac_randomize_ << '\n'

      << "** STEREO tracker parameters **\n"
      << "intra_keyframe_time_: " << intra_keyframe_time_ns_ << '\n'
      << "minNumberFeatures_: " << min_number_features_ << '\n'
      << "useStereoTracking_: " << useStereoTracking_ << '\n'

      << "** OTHER parameters **" << '\n'
      << "disparityThreshold_: " << disparityThreshold_ << '\n'
      << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&";
}

bool VisionFrontEndParams::parseYAML(const std::string& filepath) {
  stereo_matching_params_.parseYAML(filepath);

  YamlParser yaml_parser(filepath);
  yaml_parser.getYamlParam("klt_win_size", &klt_win_size_);
  yaml_parser.getYamlParam("klt_max_iter", &klt_max_iter_);
  yaml_parser.getYamlParam("klt_max_level", &klt_max_level_);
  yaml_parser.getYamlParam("klt_eps", &klt_eps_);
  yaml_parser.getYamlParam("maxFeatureAge", &maxFeatureAge_);
  yaml_parser.getYamlParam("enable_subpixel_corner_finder",
                           &enable_subpixel_corner_finder_);

  if (enable_subpixel_corner_finder_) {
    subpixel_corner_finder_params_.parseYAML(filepath);
  }

  int feature_detector_type;
  yaml_parser.getYamlParam("feature_detector_type", &feature_detector_type);
  switch (feature_detector_type) {
    case VIO::to_underlying(FeatureDetectorType::FAST): {
      feature_detector_type_ = FeatureDetectorType::FAST;
      break;
    }
    case VIO::to_underlying(FeatureDetectorType::ORB): {
      feature_detector_type_ = FeatureDetectorType::ORB;
      break;
    }
    case VIO::to_underlying(FeatureDetectorType::AGAST): {
      feature_detector_type_ = FeatureDetectorType::AGAST;
      break;
    }
    case VIO::to_underlying(FeatureDetectorType::GFTT): {
      feature_detector_type_ = FeatureDetectorType::GFTT;
      break;
    }
    default: {
      LOG(FATAL) << "Unknown Feature Detector Type: " << feature_detector_type;
    }
  }

  yaml_parser.getYamlParam("maxFeaturesPerFrame", &maxFeaturesPerFrame_);
  yaml_parser.getYamlParam("quality_level", &quality_level_);
  yaml_parser.getYamlParam("min_distance", &min_distance_);
  yaml_parser.getYamlParam("block_size", &block_size_);
  yaml_parser.getYamlParam("use_harris_detector", &use_harris_detector_);
  yaml_parser.getYamlParam("k", &k_);

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
    case VIO::to_underlying(OpticalFlowPredictorType::kStatic): {
      optical_flow_predictor_type_ = OpticalFlowPredictorType::kStatic;
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

bool VisionFrontEndParams::equals(const VisionFrontEndParams& tp2,
                                  double tol) const {
  return (klt_win_size_ == tp2.klt_win_size_) &&
         (klt_max_iter_ == tp2.klt_max_iter_) &&
         (klt_max_level_ == tp2.klt_max_level_) &&
         (fabs(klt_eps_ - tp2.klt_eps_) <= tol) &&
         (maxFeatureAge_ == tp2.maxFeatureAge_) &&
         (enable_subpixel_corner_finder_ == tp2.enable_subpixel_corner_finder_) &&
         (subpixel_corner_finder_params_.equals(
             tp2.subpixel_corner_finder_params_)) &&
         // detection parameters
         (maxFeaturesPerFrame_ == tp2.maxFeaturesPerFrame_) &&
         (fabs(quality_level_ - tp2.quality_level_) <= tol) &&
         (fabs(min_distance_ - tp2.min_distance_) <= tol) &&
         (block_size_ == tp2.block_size_) &&
         (use_harris_detector_ == tp2.use_harris_detector_) &&
         (fabs(k_ - tp2.k_) <= tol) &&
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
