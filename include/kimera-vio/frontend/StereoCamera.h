/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoCamera.h
 * @brief  Class describing a StereoCamera.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#pragma once

#include <Eigen/Core>

#include <opencv2/core.hpp>

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>

#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/frontend/UndistorterRectifier.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class StereoFrame;

class StereoCamera {
 public:
  KIMERA_POINTER_TYPEDEFS(StereoCamera);
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoCamera);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Baseline = double;

  /**
   * @brief StereoCamera definition of what a Stereo Camera is. Computes
   * rectification and undistortion parameters that can be readily applied
   * to stereo images.
   * @param left_cam_params
   * @param right_cam_params
   */
  StereoCamera(const CameraParams& left_cam_params,
               const CameraParams& right_cam_params);

  StereoCamera(Camera::ConstPtr left_camera,
               Camera::ConstPtr right_camera);

  virtual ~StereoCamera() = default;

 public:
  /** NOT TESTED
   * @brief project 3D Lmks into images as 2D pixels, doesn't do any check...
   * @param lmks Given in World coordinates or rather in whatever frame of
   * reference the pose of the left camera is given (usually the body frame!)
   * @param left_kpts
   * @param right_kpts
   */
  void project(const LandmarksCV& lmks,
               KeypointsCV* left_kpts,
               KeypointsCV* right_kpts) const;

  /**
   * @brief project One lmk into left/right images, doesn't do any check...
   * @param[in] lmks
   * @param[out] left_kpts
   * @param[out] right_kpts
   */
  void project(const LandmarkCV& lmk,
               KeypointCV* left_kpt,
               KeypointCV* right_kpt) const;

  /**
   * @brief backProjectDisparity Back project a 2D keypoint in 3D given a
   * disparity value. The 3D vector
   * is expressed in body coordinates (or whatever frame of reference the
   * stereo camera's pose is).
   * @param kp 2D keypoint in pixel coordinates
   * @param disparity of the 3D landmark
   * @param lmk 3D landmark resulting from the backProjection of the 2D keypoint
   */
  void backProjectDisparity(const KeypointCV& kp,
                            const Depth& disparity,
                            LandmarkCV* lmk) const;

  /**
   * @brief backProject A 2D keypoint in 3D given a depth value. The 3D vector
   * is expressed in body coordinates (or whatever frame of reference the
   * stereo camera's pose is).
   * @param kp 2D keypoint in pixel coordinates
   * @param depth Depth of the landmark
   * @param lmk 3D landmark resulting from the backProjection of the 2D keypoint
   */
  void backProjectDepth(const KeypointCV& kp,
                        const Depth& depth,
                        LandmarkCV* lmk) const;

  /** // NOT TESTED and probably wrong!
   * @brief backProject keypoints given disparity image
   * @param kps
   * @param disparity_img
   */
  void backProject(const KeypointsCV& kps,
                   const cv::Mat& disparity_img,
                   LandmarksCV* lmks) const;

  /**
   * @brief backProjectDisparityTo3D Given a disparity image, it
   * @param disparity_img
   * Input single-channel 8-bit unsigned, 16-bit signed, 32-bit signed or 32-bit
   * floating-point disparity image. If 16-bit signed format is used, the values
   * are assumed to have no
   * fractional bits.
   * @param depth
   * Output 3-channel floating-point image of the same size as disparity . Each
   * element of _3dImage(u, v) contains 3D coordinates of the keypoint (u, v)
   * computed from the disparity map.
   * WARNING The output 3D landmarks (x, y, z) are given in camera coordinates!!
   * rather than body coordinates!
   */
  void backProjectDisparityTo3D(const cv::Mat& disparity_img,
                                cv::Mat* depth) const;
  void backProjectDisparityTo3DManual(const cv::Mat& disparity_img,
                                      cv::Mat* depth) const;

  inline const Camera::ConstPtr& getOriginalLeftCamera() const {
    return original_left_camera_;
  }

  inline const Camera::ConstPtr& getOriginalRightCamera() const {
    return original_right_camera_;
  }

  inline gtsam::StereoCamera getUndistortedRectifiedStereoCamera() const {
    return undistorted_rectified_stereo_camera_impl_;
  }

  /**
   * @brief getBodyPoseLeftCamRect Get left camera pose after rectification with
   * respect to the body frame.
   * @return
   */
  inline gtsam::Pose3 getBodyPoseLeftCamRect() const {
    return B_Pose_camLrect_;
  }
  /**
   * @brief getBodyPoseRightCamRect Get right camera pose after rectification
   * with respect to the body frame.
   * @return
   */
  inline gtsam::Pose3 getBodyPoseRightCamRect() const {
    return B_Pose_camRrect_;
  }

  // Ideally this would return a const shared pointer or a copy, but GTSAM's
  // idiosyncrasies require shared ptrs all over the place.
  /**
   * @brief getStereoCalib
   * @return stereo camera calibration after undistortion and rectification.
   */
  inline gtsam::Cal3_S2Stereo::shared_ptr getStereoCalib() const {
    CHECK(stereo_calibration_);
    return stereo_calibration_;
  }

  /**
   * @brief getImageSize
   * @return image size of left/right frames
   */
  inline cv::Size getImageSize() const {
    CHECK_EQ(ROI1_, ROI2_);
    return cv::Size(ROI1_.x, ROI1_.y);
  }
  inline cv::Rect getROI1() const { return ROI1_; }
  inline cv::Rect getROI2() const { return ROI2_; }

  inline CameraParams getLeftCamParams() const {
    return original_left_camera_->getCamParams();
  }

  inline CameraParams getRightCamParams() const {
    return original_right_camera_->getCamParams();
  }

  inline cv::Mat getP1() const { return P1_; }

  inline cv::Mat getP2() const { return P2_; }

  inline cv::Mat getR1() const { return R1_; }

  inline cv::Mat getR2() const { return R2_; }

  inline cv::Mat getQ() const { return Q_; }

  inline Baseline getBaseline() const { return stereo_baseline_; }

  /**
   * @brief rectifyUndistortStereoFrame
   * @param stereo_frame
   */
  void undistortRectifyStereoFrame(StereoFrame* stereo_frame) const;

  /**
   * @brief undistortRectifyLeftKeypoints Undistorts and rectifies left keypoints
   * using the left camera distortion and rectification parameters.
   * Further provides a KeypointStatus on the keypoints so that out of image
   * and/or badly undistorted-rectified keypoints can be discarded.
   * @param keypoints
   * @param status_keypoints
   */
  void undistortRectifyLeftKeypoints(const KeypointsCV& keypoints,
                                     StatusKeypointsCV* status_keypoints) const;

  void distortUnrectifyRightKeypoints(
      const StatusKeypointsCV& status_keypoints,
      KeypointsCV* keypoints) const;

  /**
   * @brief computeRectificationParameters
   *
   * Outputs new rotation matrices R1,R2
   * so that the image planes of the stereo camera are parallel.
   * It also outputs new projection matrices P1, P2, and a disparity to depth
   * matrix for stereo pointcloud reconstruction.
   *
   * @param left_cam_params Left camera parameters
   * @param right_cam_params Right camera parameters
   *
   * @param R1 Output 3x3 rectification transform (rotation matrix) for the
   * first camera.
   * @param R2 Output 3x3 rectification transform (rotation matrix) for the
   * second camera.
   * @param P1 Output 3x4 projection matrix in the new (rectified) coordinate
   * systems for the first camera.
   * @param P2 Output 3x4 projection matrix in the new (rectified) coordinate
   * systems for the second camera.
   * @param Q Output \f$4 \times 4\f$ disparity-to-depth mapping matrix (see
   * reprojectImageTo3D ).
   * @param ROI1 Region of interest in image 1.
   * @param ROI2 Region of interest in image 2.
   */
  static void computeRectificationParameters(
      const CameraParams& left_cam_params,
      const CameraParams& right_cam_params,
      cv::Mat* R1,
      cv::Mat* R2,
      cv::Mat* P1,
      cv::Mat* P2,
      cv::Mat* Q,
      cv::Rect* ROI1,
      cv::Rect* ROI2);

 private:
  //! Left and right camera objects.
  //! These are neither undistorted nor rectified
  VIO::Camera::ConstPtr original_left_camera_;
  VIO::Camera::ConstPtr original_right_camera_;

  //! Stereo camera implementation
  gtsam::StereoCamera undistorted_rectified_stereo_camera_impl_;

  //! Stereo camera calibration
  gtsam::Cal3_S2Stereo::shared_ptr stereo_calibration_;

  //! Pose from Body to Left/Right Camera after rectification
  gtsam::Pose3 B_Pose_camLrect_;
  gtsam::Pose3 B_Pose_camRrect_;

  //! Undistortion rectification pre-computed maps for cv::remap
  UndistorterRectifier::UniquePtr left_cam_undistort_rectifier_;
  UndistorterRectifier::UniquePtr right_cam_undistort_rectifier_;

  // TODO(Toni): perhaps wrap these params in a struct instead.
  /// Projection matrices after rectification
  /// P1,P2 Output 3x4 projection matrix in the new (rectified) coordinate
  /// systems for the left and right camera (see cv::stereoRectify).
  cv::Mat P1_, P2_;

  /// R1,R2 Output 3x3 rectification transform (rotation matrix) for the left
  /// and for the right camera.
  cv::Mat R1_, R2_;

  /// Q Output 4x4 disparity-to-depth mapping matrix (see
  /// cv::reprojectImageTo3D or cv::stereoRectify).
  cv::Mat Q_;

  //! Regions of interest in the left/right image.
  cv::Rect ROI1_, ROI2_;

  //! Stereo baseline
  Baseline stereo_baseline_;
};

}  // namespace VIO
