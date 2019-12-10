/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioFrontEndParams.h
 * @brief  Class collecting the parameters used for stereo feature tracking
 * @author Antoni Rosinol, Luca Carlone
 */

#pragma once

#include <time.h>
#include <boost/shared_ptr.hpp>  // used for opengv

#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/UtilsOpenCV.h"
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

class FeatureSelectorParams {
 public:
  // TODO make an enum class.
  enum FeatureSelectionCriterion { QUALITY, MIN_EIG, LOGDET, RANDOM };

  FeatureSelectorParams()
      : featureSelectionCriterion_(FeatureSelectionCriterion::QUALITY),
        featureSelectionHorizon_(3),  // in seconds
        featureSelectionNrCornersToSelect_(
            1000),  // detect larger number of keypoints, and then select
        // maxFeaturesPerFrame_
        featureSelectionImuRate_(0.005),     // for feature selector
        featureSelectionDefaultDepth_(5.0),  // for feature selector
        featureSelectionCosineNeighborhood_(
            cos((10 * M_PI) / (180.0))),  // 10 degrees
        featureSelectionUseLazyEvaluation_(true) {}

  void print() const {
    LOG(INFO) << "** Feature selection parameters **\n"
              << "featureSelectionCriterion_: " << featureSelectionCriterion_
              << '\n'
              << "featureSelectionHorizon_: " << featureSelectionHorizon_
              << '\n'
              << "featureSelectionNrCornersToSelect_: "
              << featureSelectionNrCornersToSelect_ << '\n'
              << "featureSelectionImuRate_: " << featureSelectionImuRate_
              << '\n'
              << "featureSelectionDefaultDepth_: "
              << featureSelectionDefaultDepth_ << '\n'
              << "featureSelectionCosineNeighborhood_: "
              << featureSelectionCosineNeighborhood_ << '\n'
              << "featureSelectionUseLazyEvaluation_: "
              << featureSelectionUseLazyEvaluation_ << '\n'
              << "useSuccessProbabilities_: " << useSuccessProbabilities_;
  }

 public:
  // Selection params
  int featureSelectionNrCornersToSelect_;
  FeatureSelectionCriterion featureSelectionCriterion_;
  double featureSelectionHorizon_, featureSelectionImuRate_;
  double featureSelectionDefaultDepth_, featureSelectionCosineNeighborhood_;
  bool featureSelectionUseLazyEvaluation_;
  bool useSuccessProbabilities_;

  /* ------------------------------------------------------------------------ */
  static std::string FeatureSelectionCriterionStr(const int i) {
    std::string featSelCriterionStr;
    switch (i) {
    case 0:
      featSelCriterionStr = "QUALITY";
      break;
    case 1:
      featSelCriterionStr = "MIN_EIG";
      break;
    case 2:
      featSelCriterionStr = "LOGDET";
      break;
    case 3:
      featSelCriterionStr = "RANDOM";
      break;
    default:
      LOG(FATAL) << "FeatureSelectionCriterionStr: invalid feature selection "
                    "criterion";
    }
    return featSelCriterionStr;
  }

  bool parseYAML(const std::string& filepath) {
    YamlParser yaml_parser(filepath);

    int featureSelectionCriterionNr;
    yaml_parser.getYamlParam("featureSelectionCriterion",
                             &featureSelectionCriterionNr);
    switch (featureSelectionCriterionNr) {
      case 0:
        featureSelectionCriterion_ = FeatureSelectionCriterion::QUALITY;
        break;
      case 1:
        featureSelectionCriterion_ = FeatureSelectionCriterion::MIN_EIG;
        break;
      case 2:
        featureSelectionCriterion_ = FeatureSelectionCriterion::LOGDET;
        break;
      case 3:
        featureSelectionCriterion_ = FeatureSelectionCriterion::RANDOM;
        break;
      default:
        LOG(FATAL) << "Wrong choice of featureSelectionCriterion parameter.";
        break;
    }

    yaml_parser.getYamlParam("featureSelectionHorizon",
                             &featureSelectionHorizon_);
    yaml_parser.getYamlParam("featureSelectionNrCornersToSelect",
                             &featureSelectionNrCornersToSelect_);
    yaml_parser.getYamlParam("featureSelectionImuRate",
                             &featureSelectionImuRate_);
    yaml_parser.getYamlParam("featureSelectionDefaultDepth",
                             &featureSelectionDefaultDepth_);
    yaml_parser.getYamlParam("featureSelectionCosineNeighborhood",
                             &featureSelectionCosineNeighborhood_);
    yaml_parser.getYamlParam("featureSelectionUseLazyEvaluation",
                             &featureSelectionUseLazyEvaluation_);

    yaml_parser.getYamlParam("useSuccessProbabilities",
                             &useSuccessProbabilities_);
    return true;
  }
};

class VioFrontEndParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(VioFrontEndParams);
  VioFrontEndParams() : PipelineParams("Frontend Parameters") {}

 public:
  bool equals(const VioFrontEndParams& tp2, double tol = 1e-10) const {
    return (klt_win_size_ == tp2.klt_win_size_) &&
           (klt_max_iter_ == tp2.klt_max_iter_) &&
           (klt_max_level_ == tp2.klt_max_level_) &&
           (fabs(klt_eps_ - tp2.klt_eps_) <= tol) &&
           (maxFeatureAge_ == tp2.maxFeatureAge_) &&
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
           (fabs(intra_keyframe_time_ - tp2.intra_keyframe_time_) <= tol) &&
           (min_number_features_ == tp2.min_number_features_) &&
           (useStereoTracking_ == tp2.useStereoTracking_) &&
           // others:
           (fabs(disparityThreshold_ - tp2.disparityThreshold_) <= tol);
  }

  void print() const {
    LOG(INFO) << "&&&&&&&&&&&&&&&&&&&& TRACKER PARAMETERS "
                 "&&&&&&&&&&&&&&&&&&&&&&\n"
              << "** Feature tracking parameters **\n"
              << "klt_win_size_: " << klt_win_size_ << '\n'
              << "klt_max_iter_: " << klt_max_iter_ << '\n'
              << "klt_max_level_: " << klt_max_level_ << '\n'
              << "klt_eps_: " << klt_eps_ << '\n'
              << "maxFeatureAge_: " << maxFeatureAge_ << '\n'

              << "** Feature detection parameters **\n"
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
        << "intra_keyframe_time_: " << intra_keyframe_time_ << '\n'
        << "minNumberFeatures_: " << min_number_features_ << '\n'
        << "useStereoTracking_: " << useStereoTracking_ << '\n'

        << "** OTHER parameters **" << '\n'
        << "disparityThreshold_: " << disparityThreshold_ << '\n'
        << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&";
  }

  bool parseYAML(const std::string &filepath) {
    stereo_matching_params_.parseYAML(filepath);

    YamlParser yaml_parser(filepath);
    yaml_parser.getYamlParam("klt_win_size", &klt_win_size_);
    yaml_parser.getYamlParam("klt_max_iter", &klt_max_iter_);
    yaml_parser.getYamlParam("klt_max_level", &klt_max_level_);
    yaml_parser.getYamlParam("klt_eps", &klt_eps_);
    yaml_parser.getYamlParam("maxFeatureAge", &maxFeatureAge_);

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
    yaml_parser.getYamlParam("ransac_use_2point_mono",
                             &ransac_use_2point_mono_);
    yaml_parser.getYamlParam("ransac_max_iterations", &ransac_max_iterations_);
    yaml_parser.getYamlParam("ransac_probability", &ransac_probability_);
    yaml_parser.getYamlParam("ransac_randomize", &ransac_randomize_);

    yaml_parser.getYamlParam("intra_keyframe_time", &intra_keyframe_time_);
    int min_number_features;
    yaml_parser.getYamlParam("minNumberFeatures", &min_number_features);
    min_number_features_ = static_cast<size_t>(min_number_features);
    yaml_parser.getYamlParam("useStereoTracking", &useStereoTracking_);
    yaml_parser.getYamlParam("disparityThreshold", &disparityThreshold_);
    return true;
  }

 public:
  // tracking (Optical flow) params
  int klt_win_size_ = 24;  // size of the window
  int klt_max_iter_ = 30;  // max iterations
  int klt_max_level_ = 4;
  double klt_eps_ = 0.1;    // @TODO: add comments on each parameter
  int maxFeatureAge_ = 25;  // we cut feature tracks longer than that

  // Detection parameters
  int maxFeaturesPerFrame_ = 1000;
  double quality_level_ = 0.001;  // @TODO: add comments on each parameter
  double min_distance_ = 10.0;    // min distance to create mask around old
                                // keypoints for detector
  int block_size_ = 3;
  bool use_harris_detector_ = false;
  double k_ = 0.04;

  // Encapsulate StereoMatchingParams.
  StereoMatchingParams stereo_matching_params_ = StereoMatchingParams();

  // RANSAC parameters
  bool useRANSAC_ = true;
  int minNrMonoInliers_ = 10;
  int minNrStereoInliers_ = 5;  // TODO should be size_t
  double ransac_threshold_mono_ = 1.0e-6;
  double ransac_threshold_stereo_ = 1.0;
  int ransac_max_iterations_ = 100;  // TODO (minor) : should we split this in
                                     // mono and stereo?
  double ransac_probability_ = 0.995;  // TODO (minor) : should we split this in
                                       // mono and stereo?
  bool ransac_randomize_ = true;
  bool ransac_use_1point_stereo_ = true;
  bool ransac_use_2point_mono_ = true;

  // STEREO parameters:
  double intra_keyframe_time_ = 0.2;
  size_t min_number_features_ = 0u;
  // if set to false pipeline reduces to monocular tracking
  bool useStereoTracking_ = true;

  // others:
  // max disparity under which we consider the vehicle steady
  double disparityThreshold_ = 0.5;
};

} // namespace VIO
