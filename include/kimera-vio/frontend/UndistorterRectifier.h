/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   UndistorterRectifier.h
 * @brief  Class to undistort (and rectify) images.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#pragma once

#include <gtsam/geometry/Point3.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

/**
 * @brief The UndistorterRectifier class Computes undistortion maps and
 * undistorts on a per-image basis. Optionally, one can also apply image
 * rectification by providing a corresponding rotation matrix R.
 */
class UndistorterRectifier {
 public:
  KIMERA_POINTER_TYPEDEFS(UndistorterRectifier);
  KIMERA_DELETE_COPY_CONSTRUCTORS(UndistorterRectifier);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief UndistorterRectifier
   * @param P new projection matrix after stereo rectification (identity if no
   * stereo rectification needed).
   * P is normally set to P1 or P2 computed by cv::stereoRectify.
   * @param cam_params Camera Parameters.
   * @param R optional rotation matrix if you want to rectify image (aka apply
   * a rotation matrix) typically computed by cv::stereoRectify. If you have
   * a mono camera, you typically don't set this matrix.
   */
  UndistorterRectifier(const cv::Mat& P,
                       const CameraParams& cam_params,
                       const cv::Mat& R = cv::Mat());
  virtual ~UndistorterRectifier() = default;

 public:
  /**
   * @brief undistortRectifyKeypoints undistorts and rectifies
   */
  static void UndistortRectifyKeypoints(
      const KeypointsCV& keypoints,
      KeypointsCV* undistorted_keypoints,
      const CameraParams& cam_param,
      boost::optional<cv::Mat> R = boost::none,
      boost::optional<cv::Mat> P = boost::none);

  /**
   * @brief GetBearingVector undistort a single pixel,
   * and return the corresponding versor.
   * (unit norm vector corresponding to bearing).
   * @param cv_px keypoint
   * @param cam_param CameraParams instance
   */
  static gtsam::Vector3 GetBearingVector(
      const KeypointCV& keypoint,
      const CameraParams& cam_param,
      boost::optional<cv::Mat> R = boost::none);

  /**
   * @brief undistortRectifyImage Given distorted (and optionally non-rectified)
   * image, returns a distortion-free rectified one.
   * @param img Distorted non-rectified input image
   * @param undistorted_img Undistorted Rectified output image
   */
  void undistortRectifyImage(const cv::Mat& img,
                             cv::Mat* undistorted_img) const;

  /**
   * @brief undistortRectifyKeypoints Undistorts and rectifies a sparse set of
   * keypoints (instead of a whole image), using OpenCV undistortPoints.
   *
   * OpenCV undistortPoints (this does not use the remap function, but an
   * iterative approach to find the undistorted keypoints...).
   * Check OpenCV documentation for details about this function.
   *
   * It uses the internal camera parameters, and the optional R_ and P_ matrices
   * as provided at construction (by cv::stereoRectify). If R_/P_ are identity
   * no rectification is performed and the resulting keypoints are in normalized
   * coordinates.
   * @param keypoints Distorted and unrectified keypoints
   * @param undistorted_keypoints Undistorted and rectified keypoints.
   */
  void undistortRectifyKeypoints(const KeypointsCV& keypoints,
                                 KeypointsCV* undistorted_keypoints) const;

  void checkUndistortedRectifiedLeftKeypoints(
      const KeypointsCV& distorted_kpts,
      const KeypointsCV& undistorted_kpts,
      StatusKeypointsCV* status_kpts,
      // This tolerance is huge...
      const float& pixel_tolerance = 2.0f) const;

  void distortUnrectifyKeypoints(const StatusKeypointsCV& keypoints_rectified,
                                 KeypointsCV* keypoints_unrectified) const;

 protected:
  /**
   * @brief initUndistortRectifyMaps Initialize pixel to pixel maps for
   * undistortion and rectification. If rectification is not needed, as is the
   * case with a monocular camera, the identity matrix should be passed as R.
   *
   * The function computes the joint undistortion and rectification
   * transformation and represents the
   * result in the form of maps for remap. The undistorted image looks like
   * original, as if it is
   * captured with a camera using the camera matrix = P and zero
   * distortion. In case of a
   * monocular camera, P is usually equal to cameraMatrix, or it
   * can be computed by
   * cv::getOptimalNewCameraMatrix for a better control over scaling. In case of
   * a stereo camera, it is the output of stereo rectification
   * (cv::stereoRectify)
   *
   * NOTE: for stereo cameras, cam_params.P_ should be already computed using
   * cv::stereoRectify().
   *
   * @param cam_params Camera Parameters.
   * @param R optional rotation matrix if you want to rectify image (aka apply
   * a rotation matrix typically computed by cv::stereoRectify).
   * @param P new projection matrix after stereo rectification (identity if no
   * stereo rectification needed).
   * P is normally set to P1 or P2 computed by cv::stereoRectify.
   */
  void initUndistortRectifyMaps(const CameraParams& cam_params,
                                const cv::Mat& R,
                                const cv::Mat& P,
                                cv::Mat* map_x,
                                cv::Mat* map_y);

 protected:
  cv::Mat map_x_;
  cv::Mat map_y_;

  cv::Mat P_;
  cv::Mat R_;

  CameraParams cam_params_;

  // Replicate instead of constant is more efficient for GPUs to calculate.
  bool remap_use_constant_border_type_ = false;
  int remap_interpolation_type_ = cv::INTER_LINEAR;
};

}  // namespace VIO
