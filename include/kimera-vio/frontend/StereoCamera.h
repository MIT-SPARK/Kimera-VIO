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
 */

#pragma once

#include <Eigen/Core>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>

#include "kimera-vio/frontend/UndistorterRectifier.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class StereoFrame;

class StereoCamera {
 public:
  KIMERA_POINTER_TYPEDEFS(StereoCamera);
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoCamera);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief StereoCamera definition of what a Stereo Camera is. Computes
   * rectification and undistortion parameters that can be readily applied
   * to stereo images.
   * @param left_cam_params
   * @param right_cam_params
   * @param stereo_matching_params
   */
  StereoCamera(const CameraParams& left_cam_params,
               const CameraParams& right_cam_params,
               const StereoMatchingParams& stereo_matching_params);

  virtual ~StereoCamera() = default;

 public:
  /** NOT TESTED
   * @brief project Lmks into images, doesn't do any check...
   * @param lmks
   * @param left_kpts
   * @param right_kpts
   */
  void project(const LandmarksCV& lmks,
               KeypointsCV* left_kpts,
               KeypointsCV* right_kpts) const;

  /**
   * @brief backProject keypoints given disparity image
   * @param kps
   * @param disparity_img
   */
  void backProject(const KeypointsCV& kps,
                   const cv::Mat& disparity_img,
                   LandmarksCV* lmks) const;

  /**
   * @brief stereoDisparityReconstruction
   * Given left and right images reconstructs a dense disparity image.
   * @param left_img
   * @param right_img
   * @param disparity_img
   */
  void stereoDisparityReconstruction(const cv::Mat& left_img,
                                     const cv::Mat& right_img,
                                     cv::Mat* disparity_img);

  /**
   * @brief backProjectDisparityTo3D Given a disparity image, it
   * @param disparity_img
   * Input single-channel 8-bit unsigned, 16-bit signed, 32-bit signed or 32-bit
   * floating-point disparity image. If 16-bit signed format is used, the values
   * are assumed to have no
   * fractional bits.
   * @param depth
   * Output 3-channel floating-point image of the same size as disparity . Each
   * element of _3dImage(x,y) contains 3D coordinates of the point (x,y)
   * computed from the disparity
   * map.
   */
  void backProjectDisparityTo3D(const cv::Mat& disparity_img, cv::Mat* depth);

  /**
   * @brief getLeftCamRectPose Get left camera pose after rectification with
   * respect to the body frame.
   * @return
   */
  inline gtsam::Pose3 getLeftCamRectPose() const { return B_Pose_camLrect_; }

  // Ideally this would return a const shared pointer or a copy, but GTSAM's
  // idiosyncrasies require shared ptrs all over the place.
  /**
   * @brief getStereoCalib
   * @return stereo camera calibration after undistortion and rectification.
   */
  inline gtsam::Cal3_S2Stereo::shared_ptr getStereoCalib() const {
    return stereo_calibration_;
  }

  /**
   * @brief rectifyUndistortStereoFrame
   * @param stereo_frame
   */
  void undistortRectifyStereoFrame(StereoFrame* stereo_frame);

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

 protected:
  //! Stereo camera implementation
  gtsam::StereoCamera stereo_camera_impl_;

  //! Stereo camera calibration
  gtsam::Cal3_S2Stereo::shared_ptr stereo_calibration_;

  //! Pose from Body to Left Camera after rectification
  gtsam::Pose3 B_Pose_camLrect_;

  //! Non-rectified parameters
  CameraParams left_cam_params_;
  CameraParams right_cam_params_;

  //! Parameters for dense stereo matching
  StereoMatchingParams stereo_matching_params_;
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

  cv::Rect ROI1_, ROI2_;

  //! Stereo baseline
  double baseline_;

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
