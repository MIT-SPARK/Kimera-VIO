/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FeatureDetector-definitions.cpp
 * @brief  Parameters for feature detector
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/feature-detector/FeatureDetectorParams.h"
#include "kimera-vio/frontend/VisionImuFrontendParams.h"
#include "kimera-vio/frontend/feature-detector/NonMaximumSuppression.h"
#include "kimera-vio/frontend/feature-detector/anms/anms.h"  // REMOVE
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

SubPixelCornerFinderParams::SubPixelCornerFinderParams()
    : PipelineParams("SubPixelCornerFinder Parameters") {}

void SubPixelCornerFinderParams::print() const {
  std::stringstream out;
  PipelineParams::print(out,
                        "Termination criteria type",
                        term_criteria_.type,
                        "Termination criteria maximum iters",
                        term_criteria_.maxCount,
                        "Termination criteria epsilon",
                        term_criteria_.epsilon,
                        "Window size",
                        window_size_,
                        "Zero zone",
                        zero_zone_);
  LOG(INFO) << out.str();
}

bool SubPixelCornerFinderParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);
  term_criteria_.type = cv::TermCriteria::EPS + cv::TermCriteria::COUNT;
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

FeatureDetectorParams::FeatureDetectorParams()
    : PipelineParams("FeatureDetector Parameters") {}

void FeatureDetectorParams::print() const {
  std::stringstream out;
  PipelineParams::print(out,
                        "Feature Detector Type: ",
                        VIO::to_underlying(feature_detector_type_),
                        "Max features per frame",
                        max_features_per_frame_,
                        "Enable Subpixel corner refinement",
                        enable_subpixel_corner_refinement_,
                        "Enable Non-maximum suppression",
                        enable_non_max_suppression_,
                        "Non-maximum suppression type",
                        VIO::to_underlying(non_max_suppression_type_),
                        "Min dist btw tracked/detected features",
                        min_distance_btw_tracked_and_detected_features_,
                        "quality_level_: ",
                        quality_level_,
                        "block_size_: ",
                        block_size_,
                        "use_harris_corner_detector",
                        use_harris_corner_detector_,
                        "k_: ",
                        k_,
                        "Fast Threshold",
                        fast_thresh_);
  LOG(INFO) << out.str();
  if (enable_subpixel_corner_refinement_) {
    subpixel_corner_finder_params_.print();
  }
}

bool FeatureDetectorParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);
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

  yaml_parser.getYamlParam("enable_subpixel_corner_finder",
                           &enable_subpixel_corner_refinement_);

  if (enable_subpixel_corner_refinement_) {
    subpixel_corner_finder_params_.parseYAML(filepath);
  }

  yaml_parser.getYamlParam("enable_non_max_suppression",
                           &enable_non_max_suppression_);
  int non_max_suppression_type;
  yaml_parser.getYamlParam("non_max_suppression_type",
                           &non_max_suppression_type);
  switch (non_max_suppression_type) {
    case VIO::to_underlying(AnmsAlgorithmType::TopN): {
      non_max_suppression_type_ = AnmsAlgorithmType::TopN;
      break;
    }
    case VIO::to_underlying(AnmsAlgorithmType::BrownANMS): {
      non_max_suppression_type_ = AnmsAlgorithmType::BrownANMS;
      break;
    }
    case VIO::to_underlying(AnmsAlgorithmType::SDC): {
      non_max_suppression_type_ = AnmsAlgorithmType::SDC;
      break;
    }
    case VIO::to_underlying(AnmsAlgorithmType::KdTree): {
      non_max_suppression_type_ = AnmsAlgorithmType::KdTree;
      break;
    }
    case VIO::to_underlying(AnmsAlgorithmType::RangeTree): {
      non_max_suppression_type_ = AnmsAlgorithmType::RangeTree;
      break;
    }
    case VIO::to_underlying(AnmsAlgorithmType::Ssc): {
      non_max_suppression_type_ = AnmsAlgorithmType::Ssc;
      break;
    }
    default: {
      LOG(FATAL) << "Unknown Non Maximum Suppresion Type: "
                 << non_max_suppression_type;
    }
  }

  yaml_parser.getYamlParam("maxFeaturesPerFrame", &max_features_per_frame_);

  // GFTT specific parameters
  yaml_parser.getYamlParam("quality_level", &quality_level_);
  yaml_parser.getYamlParam("min_distance",
                           &min_distance_btw_tracked_and_detected_features_);
  yaml_parser.getYamlParam("block_size", &block_size_);
  yaml_parser.getYamlParam("use_harris_detector", &use_harris_corner_detector_);
  yaml_parser.getYamlParam("k", &k_);

  // FAST specific params
  yaml_parser.getYamlParam("fast_thresh", &fast_thresh_);

  return true;
}

bool FeatureDetectorParams::equals(const FeatureDetectorParams& tp2,
                                   double tol) const {
  return (feature_detector_type_ == tp2.feature_detector_type_) &&
         (max_features_per_frame_ == tp2.max_features_per_frame_) &&
         (enable_subpixel_corner_refinement_ ==
          tp2.enable_subpixel_corner_refinement_) &&
         (enable_non_max_suppression_ == tp2.enable_non_max_suppression_) &&
         (non_max_suppression_type_ == tp2.non_max_suppression_type_) &&
         (subpixel_corner_finder_params_.equals(
             tp2.subpixel_corner_finder_params_)) &&
         (fabs(min_distance_btw_tracked_and_detected_features_ -
               tp2.min_distance_btw_tracked_and_detected_features_) <= tol) &&
         (fabs(quality_level_ - tp2.quality_level_) <= tol) &&
         (block_size_ == tp2.block_size_) &&
         (use_harris_corner_detector_ == tp2.use_harris_corner_detector_) &&
         (fabs(k_ - tp2.k_) <= tol) && (fast_thresh_ == tp2.fast_thresh_);
}

}  // namespace VIO
