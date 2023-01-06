/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoMatchingParams.h
 * @brief  Parameters for stereo matching.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/StereoFrame-definitions.h"
#include "kimera-vio/pipeline/PipelineParams.h"

#include <opencv2/calib3d.hpp> // Only for StereoBM (put in another file).

namespace VIO {

class StereoMatchingParams : public PipelineParams {
 public:
  StereoMatchingParams();
  virtual ~StereoMatchingParams() = default;

  void print() const;
  bool parseYAML(const std::string& filepath);
  bool equals(const StereoMatchingParams& tp2, double tol = 1e-10) const;

 protected:
  // Parameters of the pipeline must specify how to be compared.
  virtual bool equals(const PipelineParams& obj) const {
    const auto& rhs = static_cast<const StereoMatchingParams&>(obj);
    return equals(rhs);
  }

 private:
  void checkParams();

 public:
  double tolerance_template_matching_ = 0.15;
  double nominal_baseline_ = 0.11;
  int templ_cols_ = 101;       // must be odd
  int templ_rows_ = 11;        // must be odd
  int stripe_extra_rows_ = 0;  // must be even
  // stereo points triangulated below this distance are discarded.
  double min_point_dist_ = 0.1;
  // stereo points triangulated beyond this distance are discarded.
  double max_point_dist_ = 15.0;
  // check best match left->right and right->left
  bool bidirectional_matching_ = false;
  // refine stereo matches with subpixel accuracy
  bool subpixel_refinement_ = false;
  // do equalize image before processing options to use RGB-D vs. stereo.
  bool equalize_image_ = false;
};

// TODO(Toni) make it a pipeline params and parseable.
struct DenseStereoParams {
  bool use_sgbm_ = true;
  bool post_filter_disparity_ = false;
  bool median_blur_disparity_ = false;
  int pre_filter_cap_ = 31;
  int sad_window_size_ = 11;
  int min_disparity_ = 1;
  int num_disparities_ = 64;
  int uniqueness_ratio_ = 0;
  int speckle_range_ = 3;
  int speckle_window_size_ = 500;
  // bm parameters
  int texture_threshold_ = 0;
  int pre_filter_type_ = cv::StereoBM::PREFILTER_XSOBEL;
  int pre_filter_size_ = 9;
  // sgbm parameters
  int p1_ = 120;
  int p2_ = 240;
  int disp_12_max_diff_ = -1;
  bool use_mode_HH_ = true;
};

}  // namespace VIO
