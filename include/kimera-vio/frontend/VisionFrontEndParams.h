/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionFrontEndParams.h
 * @brief  Class collecting the parameters used for stereo feature tracking
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#pragma once

#include <time.h>
#include <boost/shared_ptr.hpp>  // used for opengv

#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "kimera-vio/frontend/OpticalFlowPredictor-definitions.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/UtilsNumerical.h"
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

struct SubPixelCornerFinderParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(SubPixelCornerFinderParams);
  SubPixelCornerFinderParams();

 public:
  void print() const;
  bool parseYAML(const std::string& filepath);
  bool equals(const SubPixelCornerFinderParams& tp2, double tol = 1e-10) const;

 public:
  /// Termination criteria defined in terms of change in error and maximum
  /// number of iterations
  cv::TermCriteria term_criteria_ =
      cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.01);
  cv::Size window_size_ = cv::Size(10, 10);
  cv::Size zero_zone_ = cv::Size(-1, -1);
};

struct VisionFrontEndParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(VisionFrontEndParams);
  VisionFrontEndParams();

 public:
  void print() const;
  bool parseYAML(const std::string& filepath);
  bool equals(const VisionFrontEndParams& tp2, double tol = 1e-10) const;

 public:
  // tracking (Optical flow) params
  int klt_win_size_ = 24;  // size of the window
  int klt_max_iter_ = 30;  // max iterations
  int klt_max_level_ = 4;
  double klt_eps_ = 0.1;    // @TODO: add comments on each parameter
  int maxFeatureAge_ = 25;  // we cut feature tracks longer than that

  /// Find subpixel corners when doing monocular feature tracking
  /// by solving iterative optimization problem.
  bool enable_subpixel_corner_finder_ = false;
  SubPixelCornerFinderParams subpixel_corner_finder_params_ =
      SubPixelCornerFinderParams();

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
  int ransac_max_iterations_ = 100;    // TODO (minor) : should we split this in
                                       // mono and stereo?
  double ransac_probability_ = 0.995;  // TODO (minor) : should we split this in
                                       // mono and stereo?
  bool ransac_randomize_ = true;
  bool ransac_use_1point_stereo_ = true;
  bool ransac_use_2point_mono_ = true;

  // STEREO parameters:
  double intra_keyframe_time_ns_ = 0.2 * 10e6;
  size_t min_number_features_ = 0u;
  // if set to false pipeline reduces to monocular tracking
  bool useStereoTracking_ = true;

  // others:
  // max disparity under which we consider the vehicle steady
  double disparityThreshold_ = 0.5;

  OpticalFlowPredictorType optical_flow_predictor_type_ =
      OpticalFlowPredictorType::kStatic;
};

}  // namespace VIO
