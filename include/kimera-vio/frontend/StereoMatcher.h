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
 * @author Marcus Abate
 */

#pragma once

#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class StereoMatcher {
 public:
  KIMERA_POINTER_TYPEDEFS(StereoMatcher);
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoMatcher);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief StereoMatcher definition of what a Stereo Matcher is. Computes
   * stereo matches between left/right images of a stereo camera.
   * @param stereo_camera
   * @param stereo_matching_params
   */
  StereoMatcher(const StereoCamera::ConstPtr& stereo_camera,
                const StereoMatchingParams& stereo_matching_params);

  virtual ~StereoMatcher() = default;

 public:
  /**
   * @brief denseStereoReconstruction
   * Given left and right images reconstructs a dense disparity image.
   * @param[in] left_img Undistorted rectified left image
   * @param[in] right_img Undistorted rectified right image
   * @param[out] disparity_img Disparity image
   */
  void denseStereoReconstruction(const cv::Mat& left_img_rectified,
                                 const cv::Mat& right_img_rectified,
                                 cv::Mat* disparity_img);


  /**
   * @brief sparseStereoReconstruction
   * Given a stereo frame finds the right keypoints in the right image for
   * the left keypoints in the left image. The left keypoints in the stereo
   * frame should be already computed and the stereo frame is rectified.
   * Since StereoMatcher is a friend of StereoFrame, we can access its inner
   * members.
   * @param[in/out] stereo_frame In the keypoints from left image and out the
   * right keypoints.
   */
  void sparseStereoReconstruction(StereoFrame* stereo_frame);

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

  void getRightKeypointsRectified(
      const cv::Mat& left_img_rectified,
      const cv::Mat& right_img_rectified,
      const StatusKeypointsCV& left_keypoints_rectified,
      const double& fx,
      const double& baseline,
      StatusKeypointsCV* right_keypoints_rectified) const;

  void getDepthFromRectifiedMatches(
      StatusKeypointsCV& left_keypoints_rectified,
      StatusKeypointsCV& right_keypoints_rectified,
      Depths* keypoints_depth) const;

 protected:
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
  //! Stereo camera shared that might be shared across modules
  StereoCamera::ConstPtr stereo_camera_;

  //! Parameters for sparse stereo matching
  StereoMatchingParams stereo_matching_params_;

  //! Parameters for dense stereo matching
  DenseStereoParams dense_stereo_params_;
};

}  // namespace VIO
