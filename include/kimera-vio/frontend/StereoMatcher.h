/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoMatcher.h
 * @brief  Class describing a stereo matching algorithms.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class StereoMatcher {
 public:
  KIMERA_POINTER_TYPEDEFS(StereoMatcher);
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoMatcher);

  /**
   * @brief StereoMatcher definition of what a Stereo Matcher is. Computes
   * stereo matches between left/right images of a stereo camera.
   * @param stereo_camera
   * @param stereo_matching_params
   */
  StereoMatcher(const StereoCamera& stereo_camera,
                const StereoMatchingParams& stereo_matching_params);

  virtual ~StereoMatcher() = default;

 public:
  /**
   * @brief stereoDisparityReconstruction
   * Given left and right images reconstructs a dense disparity image.
   * @param[in] left_img Undistorted rectified left image
   * @param[in] right_img Undistorted rectified right image
   * @param[out] disparity_img Disparity image
   */
  void denseStereoReconstruction(const cv::Mat& left_img_rectified,
                                 const cv::Mat& right_img_rectified,
                                 cv::Mat* disparity_img);

  /**
   * @brief spaseStereoReconstruction
   * Given left and right images together with sparse left keypoints:
   * finds the right keypoint in the right image.
   * @param[in] left_img Undistorted rectified left image
   * @param[in] right_img Undistorted rectified right image
   * @param[in] left_keypoints Left keypoints
   * @param[out] right_keypoints Corresponding right keypoints wrt left
   */
  void sparseStereoReconstruction(
      const cv::Mat& left_img_rectified,
      const cv::Mat& right_img_rectified,
      const StatusKeypointsCV& left_keypoints_rectified,
      StatusKeypointsCV* right_keypoints_rectified);

 protected:
  /**
   * @brief getRightKeypointsRectified
   * @param left_img_rectified
   * @param right_img_rectified
   * @param left_keypoints_rectified
   * @param fx
   * @param baseline
   * @param right_keypoints_rectified
   */
  void getRightKeypointsRectified(
      const cv::Mat& left_img_rectified,
      const cv::Mat& right_img_rectified,
      const StatusKeypointsCV& left_keypoints_rectified,
      const double& fx,
      const double& baseline,
      StatusKeypointsCV* right_keypoints_rectified) const;

 protected:
  /**
   * @brief searchRightKeypointEpipolar
   * @param left_img_rectified
   * @param left_keypoint_rectified
   * @param right_rectified
   * @param stripe_cols
   * @param stripe_rows
   * @param stereo_matching_params
   * @param right_keypoint_rectified
   * @param score
   */
  void searchRightKeypointEpipolar(
      const cv::Mat& left_img_rectified,
      const KeypointCV& left_keypoint_rectified,
      const cv::Mat& right_rectified,
      const int& stripe_cols,
      const int& stripe_rows,
      const StereoMatchingParams& stereo_matching_params,
      StatusKeypointCV* right_keypoint_rectified,
      double* score) const;

 protected:
  //! Stereo camera
  StereoCamera stereo_camera_;

  //! Parameters for dense stereo matching
  StereoMatchingParams stereo_matching_params_;

  // TODO(Toni): put on its own struct, dense stereo depth reconstruction
  //! Dense Stereo Reconstruction params
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
