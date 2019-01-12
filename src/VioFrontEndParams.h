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
 * @author Luca Carlone
 */

#ifndef VioFrontEndParams_H_
#define VioFrontEndParams_H_

#include <boost/shared_ptr.hpp>     // used for opengv
#include <time.h>

#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "UtilsOpenCV.h"
#include "StereoFrame.h"

namespace VIO {

///////////////////////////////////////////////////////////////////////////////////////
class VioFrontEndParams{
public:
  enum FeatureSelectionCriterion {
    QUALITY, MIN_EIG, LOGDET, RANDOM
  };

  VioFrontEndParams():
    // tracking params
    klt_win_size_(24),
    klt_max_iter_(30),
    klt_max_level_(4),
    klt_eps_(0.1), // Before tuning: 0.001
    maxFeatureAge_(25), // upper bounded by horizon / min intra_keyframe_time_
    // detection params
    maxFeaturesPerFrame_(1000), // Max nr of features to track per frame
    quality_level_(0.001), // Quality of feature from 0-1 (mono: 0.995) // before tuning: 0.5
    min_distance_(10.0), // Minimum allowable distance (in pixels) between feature detections // Before tuning: 20
    block_size_(3),
    use_harris_detector_(false),
    k_(0.04),
    equalizeImage_(false),
    // stereo matching
    nominalBaseline_(0.11),
    toleranceTemplateMatching_(0.15),
    templ_cols_(101), // must be odd
    templ_rows_(11),
    stripe_extra_rows_(0),
    minPointDist_(0.1), // stereo points triangulated below this distance are discarded
    maxPointDist_(15), // stereo points triangulated beyond this distance are discarded
    bidirectionalMatching_(false),
    subpixelRefinementStereo_(false),
    // selector params
    featureSelectionCriterion_(FeatureSelectionCriterion::QUALITY),
    featureSelectionHorizon_(3), // in seconds
    featureSelectionNrCornersToSelect_(1000), // detect larger number of keypoints, and then select maxFeaturesPerFrame_
    featureSelectionImuRate_(0.005), // for feature selector
    featureSelectionDefaultDepth_(5.0), // for feature selector
    featureSelectionCosineNeighborhood_(cos( (10*M_PI)/(180.0) )), // 10 degrees
    featureSelectionUseLazyEvaluation_(true),
    useSuccessProbabilities_(true),
    // RANSAC params:
    useRANSAC_(true), // if false RANSAC is completely disabled
    minNrMonoInliers_(10),
    minNrStereoInliers_(5),
    ransac_threshold_mono_(1e-6), // threshold Some threshold value for classifying samples as an inlier or an outlier
    ransac_threshold_stereo_(1), // 0.3 for 3point method, 3-7 for 1-point
    ransac_use_1point_stereo_(true),
    ransac_use_2point_mono_(true),
    ransac_max_iterations_(100),
    ransac_probability_(0.995), // The probability of being able to draw at least one sample that is free of outliers
    ransac_randomize_(true),
    // StereoTracker params (kept here for simplicity)
    intra_keyframe_time_(0.2),  // in seconds
    minNumberFeatures_(0),
    useStereoTracking_(true),
    // other params
    display_time_(100),
    disparityThreshold_(0.5) // in pixels
  {}

  // tracking (Optical flow) params
  int klt_win_size_; // size of the window
  int klt_max_iter_; // max iterations
  int klt_max_level_;
  double klt_eps_; // @TODO: add comments on each parameter
  int maxFeatureAge_; // we cut feature tracks longer than that

  // Detection parameters
  int maxFeaturesPerFrame_;
  double quality_level_; // @TODO: add comments on each parameter
  double min_distance_; // min distance to create mask around old keypoints for detector
  int block_size_;
  bool use_harris_detector_;
  double k_;
  bool equalizeImage_;

  // stereo matching
  double nominalBaseline_;
  double toleranceTemplateMatching_;
  int templ_cols_;
  int templ_rows_;
  int stripe_extra_rows_;
  double minPointDist_;
  double maxPointDist_;
  bool bidirectionalMatching_;
  bool subpixelRefinementStereo_;

  // Selection params
  int featureSelectionNrCornersToSelect_;
  FeatureSelectionCriterion featureSelectionCriterion_;
  double featureSelectionHorizon_, featureSelectionImuRate_;
  double featureSelectionDefaultDepth_, featureSelectionCosineNeighborhood_;
  bool featureSelectionUseLazyEvaluation_;
  bool useSuccessProbabilities_;

  // RANSAC parameters
  bool useRANSAC_;
  int minNrMonoInliers_, minNrStereoInliers_;
  double ransac_threshold_mono_, ransac_threshold_stereo_;
  int    ransac_max_iterations_; // TODO (minor) : should we split this in mono and stereo?
  double ransac_probability_; // TODO (minor) : should we split this in mono and stereo?
  bool ransac_randomize_;
  bool ransac_use_1point_stereo_, ransac_use_2point_mono_;

  // STEREO parameters:
  double intra_keyframe_time_;
  int minNumberFeatures_;
  bool useStereoTracking_; // if set to false pipeline reduces to monocular tracking

  // others:
  double disparityThreshold_; // max disparity under which we consider the vehicle steady
  double display_time_; // time for imshow

  /* -------------------------------------------------------------------------- */
  static std::string FeatureSelectionCriterionStr(const int i){
    std::string featSelCriterionStr;
    switch(i){
    case 0 : featSelCriterionStr = "QUALITY"; break;
    case 1 : featSelCriterionStr = "MIN_EIG"; break;
    case 2 : featSelCriterionStr = "LOGDET"; break;
    case 3 : featSelCriterionStr = "RANDOM"; break;
    default :
      throw std::runtime_error("FeatureSelectionCriterionStr: invalid feature selection criterion");
      break;
    }
    return featSelCriterionStr;
  }
  /* -------------------------------------------------------------------------- */
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
        (equalizeImage_ == tp2.equalizeImage_) &&
        // stereo matching
        (fabs(nominalBaseline_ - tp2.nominalBaseline_) <= tol) &&
        (fabs(toleranceTemplateMatching_ - tp2.toleranceTemplateMatching_) <= tol) &&
        (templ_cols_ == tp2.templ_cols_) &&
        (templ_rows_ == tp2.templ_rows_) &&
        (stripe_extra_rows_ == tp2.stripe_extra_rows_) &&
        (fabs(minPointDist_ - tp2.minPointDist_) <= tol) &&
        (fabs(maxPointDist_ - tp2.maxPointDist_) <= tol) &&
        (bidirectionalMatching_ == tp2.bidirectionalMatching_) &&
        (subpixelRefinementStereo_ == tp2.subpixelRefinementStereo_) &&
        // Selection params
        (featureSelectionNrCornersToSelect_ == tp2.featureSelectionNrCornersToSelect_) &&
        (featureSelectionCriterion_ == tp2.featureSelectionCriterion_) &&
        (fabs(featureSelectionHorizon_ - tp2.featureSelectionHorizon_) <= tol) &&
        (fabs(featureSelectionImuRate_ - tp2.featureSelectionImuRate_) <= tol) &&
        (fabs(featureSelectionDefaultDepth_ - tp2.featureSelectionDefaultDepth_) <= tol) &&
        (fabs(featureSelectionCosineNeighborhood_ - tp2.featureSelectionCosineNeighborhood_) <= tol) &&
        (featureSelectionUseLazyEvaluation_ == tp2.featureSelectionUseLazyEvaluation_) &&
        (useSuccessProbabilities_ == tp2.useSuccessProbabilities_) &&
        // RANSAC parameters
        (useRANSAC_ == tp2.useRANSAC_) &&
        (minNrMonoInliers_ == tp2.minNrMonoInliers_) &&
        (minNrStereoInliers_ == tp2.minNrStereoInliers_) &&
        (fabs(ransac_threshold_mono_ - tp2.ransac_threshold_mono_) <= tol) &&
        (fabs(ransac_threshold_stereo_ - tp2.ransac_threshold_stereo_) <= tol) &&
        (ransac_use_1point_stereo_ == tp2.ransac_use_1point_stereo_) &&
        (ransac_use_2point_mono_ == tp2.ransac_use_2point_mono_) &&
        (ransac_max_iterations_ == tp2.ransac_max_iterations_) &&
        (fabs(ransac_probability_ - tp2.ransac_probability_) <= tol) &&
        (ransac_randomize_ == tp2.ransac_randomize_) &&
        // STEREO parameters:
        (fabs(intra_keyframe_time_ - tp2.intra_keyframe_time_) <= tol) &&
        (minNumberFeatures_ == tp2.minNumberFeatures_) &&
        (useStereoTracking_ == tp2.useStereoTracking_) &&
        // others:
        (fabs(disparityThreshold_ - tp2.disparityThreshold_) <= tol) &&
        (fabs(display_time_ - tp2.display_time_) <= tol);
  }

  /* -------------------------------------------------------------------------- */
  StereoMatchingParams getStereoMatchingParams(){
    return StereoMatchingParams(toleranceTemplateMatching_, templ_cols_,templ_rows_,stripe_extra_rows_,minPointDist_,
        maxPointDist_,bidirectionalMatching_,nominalBaseline_,subpixelRefinementStereo_,equalizeImage_);
  }

  /* -------------------------------------------------------------------------- */
  void print() const {
    LOG(INFO) << "&&&&&&&&&&&&&&&&&&&& TRACKER PARAMETERS &&&&&&&&&&&&&&&&&&&&&&&&&&&\n"
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
     << "k_: " << k_ << '\n'
     << "equalizeImage_: " << equalizeImage_ << '\n'

     << "** Sparse Stereo Matching parameters **\n"
     << "nominalBaseline_: " << nominalBaseline_ << '\n'
     << "toleranceTemplateMatching_: " << toleranceTemplateMatching_ << '\n'
     << "templ_cols_: " << templ_cols_ << '\n'
     << "templ_rows_: " << templ_rows_ << '\n'
     << "stripe_extra_rows_: " << stripe_extra_rows_ << '\n'
     << "minPointDist_: " << minPointDist_ << '\n'
     << "maxPointDist_: " << maxPointDist_ << '\n'
     << "bidirectionalMatching_: " << bidirectionalMatching_ << '\n'
     << "subpixelRefinementStereo_: " << subpixelRefinementStereo_ << '\n'

     << "** Feature selection parameters **\n"
     << "featureSelectionCriterion_: " << featureSelectionCriterion_ << '\n'
     << "featureSelectionHorizon_: " << featureSelectionHorizon_ << '\n'
     << "featureSelectionNrCornersToSelect_: " << featureSelectionNrCornersToSelect_ << '\n'
     << "featureSelectionImuRate_: " << featureSelectionImuRate_ << '\n'
     << "featureSelectionDefaultDepth_: " << featureSelectionDefaultDepth_ << '\n'
     << "featureSelectionCosineNeighborhood_: " << featureSelectionCosineNeighborhood_ << '\n'
     << "featureSelectionUseLazyEvaluation_: " << featureSelectionUseLazyEvaluation_ << '\n'
     << "useSuccessProbabilities_: " << useSuccessProbabilities_ << '\n'

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
     << "minNumberFeatures_: " << minNumberFeatures_ << '\n'
     << "useStereoTracking_: " << useStereoTracking_ << '\n'

     << "** OTHER parameters **" << '\n'
     << "disparityThreshold_: " << disparityThreshold_ << '\n'
     << "display_time_: " << display_time_ << '\n'
     << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&";
  }
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  bool parseYAML(std::string filepath){
    // make sure that each YAML file has %YAML:1.0 as first line
    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cout << "Cannot open file in parseYAML: " << filepath << std::endl;
      throw std::runtime_error("parseYAML (Tracker): cannot open file (remember first line: %YAML:1.0)");
    }

    fs["klt_win_size"] >> klt_win_size_;
    fs["klt_max_iter"] >> klt_max_iter_;
    fs["klt_max_level"] >> klt_max_level_;
    fs["klt_eps"] >> klt_eps_;
    fs["maxFeatureAge"] >> maxFeatureAge_;

    fs["maxFeaturesPerFrame"] >> maxFeaturesPerFrame_;
    fs["quality_level"] >> quality_level_;
    fs["min_distance"] >> min_distance_;
    fs["block_size"] >> block_size_;
    fs["use_harris_detector"] >> use_harris_detector_;
    fs["k"] >> k_;
    fs["equalizeImage"] >> equalizeImage_;

    fs["nominalBaseline"] >> nominalBaseline_;
    fs["toleranceTemplateMatching"] >> toleranceTemplateMatching_;
    fs["templ_cols"] >> templ_cols_;
    fs["templ_rows"] >> templ_rows_;
    fs["stripe_extra_rows"] >> stripe_extra_rows_;
    fs["minPointDist"] >> minPointDist_;
    fs["maxPointDist"] >> maxPointDist_;
    fs["bidirectionalMatching"] >> bidirectionalMatching_;
    fs["subpixelRefinementStereo"] >> subpixelRefinementStereo_;

    int featureSelectionCriterionNr;
    fs["featureSelectionCriterion"] >> featureSelectionCriterionNr;
    switch(featureSelectionCriterionNr){
    case 0 : featureSelectionCriterion_ = FeatureSelectionCriterion::QUALITY; break;
    case 1 : featureSelectionCriterion_ = FeatureSelectionCriterion::MIN_EIG; break;
    case 2 : featureSelectionCriterion_ = FeatureSelectionCriterion::LOGDET; break;
    case 3 : featureSelectionCriterion_ = FeatureSelectionCriterion::RANDOM; break;
    default: throw std::runtime_error("parseYAML: wrong choice of featureSelectionCriterion"); break;
    }
    fs["featureSelectionHorizon"] >> featureSelectionHorizon_;
    fs["featureSelectionNrCornersToSelect"] >> featureSelectionNrCornersToSelect_;
    fs["featureSelectionImuRate"] >> featureSelectionImuRate_;
    fs["featureSelectionDefaultDepth"] >> featureSelectionDefaultDepth_;
    fs["featureSelectionCosineNeighborhood"] >> featureSelectionCosineNeighborhood_;
    fs["featureSelectionUseLazyEvaluation"] >> featureSelectionUseLazyEvaluation_;
    fs["useSuccessProbabilities"] >> useSuccessProbabilities_;

    fs["useRANSAC"] >> useRANSAC_;
    fs["minNrMonoInliers"] >> minNrMonoInliers_;
    fs["minNrStereoInliers"] >> minNrStereoInliers_;
    fs["ransac_threshold_mono"] >> ransac_threshold_mono_;
    fs["ransac_threshold_stereo"] >> ransac_threshold_stereo_;
    fs["ransac_use_1point_stereo"] >> ransac_use_1point_stereo_;
    fs["ransac_use_2point_mono"] >> ransac_use_2point_mono_;

    fs["ransac_max_iterations"] >> ransac_max_iterations_;
    fs["ransac_probability"] >> ransac_probability_;
    fs["ransac_randomize"] >> ransac_randomize_;

    fs["intra_keyframe_time"] >> intra_keyframe_time_;
    fs["minNumberFeatures"] >> minNumberFeatures_;
    fs["useStereoTracking"] >> useStereoTracking_;

    fs["display_time"] >> display_time_;
    fs["disparityThreshold"] >> disparityThreshold_;

    fs.release();
    return true;
  }
};

} // namespace VIO
#endif /* VioFrontEndParams_H_ */
