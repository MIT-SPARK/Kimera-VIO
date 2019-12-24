/* -----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testVioFrontEndParams.h
 * @brief  test VioFrontEndParams
 * @author Antoni Rosinol, Luca Carlone
 */

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/VioFrontEndParams.h"

DECLARE_string(test_data_path);

using namespace gtsam;
using namespace std;
using namespace VIO;

static const double tol = 1e-7;

/* ************************************************************************* */
TEST(testVioFrontEndParams, TrackerParamParseYAML) {
  // check that YAML is parsed correctly

  // Test parseYAML
  VioFrontEndParams tp;
  tp.parseYAML(FLAGS_test_data_path + "/ForTracker/trackerParameters.yaml");

  // Compare results!
  EXPECT_EQ(tp.klt_win_size_, 24);
  EXPECT_EQ(tp.klt_max_iter_, 30);
  EXPECT_EQ(tp.klt_max_level_, 2);
  EXPECT_EQ(tp.klt_eps_, 0.001);
  EXPECT_EQ(tp.maxFeatureAge_, 10);

  EXPECT_EQ(tp.maxFeaturesPerFrame_, 200);
  EXPECT_EQ(tp.quality_level_, 0.5);
  EXPECT_EQ(tp.min_distance_, 20);
  EXPECT_EQ(tp.block_size_, 3);
  EXPECT_EQ(tp.use_harris_detector_, 0);
  EXPECT_EQ(tp.k_, 0.04);

  EXPECT_EQ(tp.stereo_matching_params_.equalize_image_, true);
  EXPECT_EQ(tp.stereo_matching_params_.nominal_baseline_, 110);
  EXPECT_EQ(tp.stereo_matching_params_.tolerance_template_matching_, 0.17);
  EXPECT_EQ(tp.stereo_matching_params_.templ_cols_, 103);
  EXPECT_EQ(tp.stereo_matching_params_.templ_rows_, 5);
  EXPECT_EQ(tp.stereo_matching_params_.stripe_extra_rows_, 2);
  EXPECT_EQ(tp.stereo_matching_params_.min_point_dist_, 0.1);
  EXPECT_EQ(tp.stereo_matching_params_.max_point_dist_, 150);
  EXPECT_EQ(tp.stereo_matching_params_.bidirectional_matching_, true);
  EXPECT_EQ(tp.stereo_matching_params_.subpixel_refinement_, true);

  EXPECT_EQ(tp.useRANSAC_, false);
  EXPECT_EQ(tp.minNrMonoInliers_, 2000);
  EXPECT_EQ(tp.minNrStereoInliers_, 1000);
  EXPECT_EQ(tp.ransac_threshold_mono_, 1e-06);
  EXPECT_EQ(tp.ransac_threshold_stereo_, 0.3);
  EXPECT_EQ(tp.ransac_use_1point_stereo_, false);
  EXPECT_EQ(tp.ransac_use_2point_mono_, true);
  EXPECT_EQ(tp.ransac_max_iterations_, 100);
  EXPECT_EQ(tp.ransac_probability_, 0.995);
  EXPECT_EQ(tp.ransac_randomize_, false);

  EXPECT_EQ(tp.intra_keyframe_time_, 0.5);
  EXPECT_EQ(tp.min_number_features_, 100);
  EXPECT_EQ(tp.useStereoTracking_, 1);
  EXPECT_EQ(tp.disparityThreshold_, 1);
}

/* ************************************************************************** */
TEST(testVioFrontEndParams, equals) {
  VioFrontEndParams tp = VioFrontEndParams();
  EXPECT_TRUE(tp.equals(tp));
}
