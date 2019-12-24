/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Camera.h
 * @brief  Class describing a camera and derivatives: MonoCamera, StereoCamera.
 * @author Antoni Rosinol
 */

#pragma once

#include <Eigen/Core>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/utility.hpp>  // for tie

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>

#include <glog/logging.h>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

// class Camera {
//
//
//};
//
// class MultiCamera : public Camera {
//
//};

// TODO inherits from a multi camera class
class StereoCamera {
 public:
  KIMERA_POINTER_TYPEDEFS(StereoCamera);
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoCamera);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief StereoCamera definition of what a Stereo Camera is. Computes
   * rectification and undistortion parameters that can be readily applied
   * to stereo images.
   * @param camL_Pose_camR
   * @param left_cam_params
   * @param right_cam_params
   * @param stereo_matching_params
   */
  StereoCamera(const CameraParams& left_cam_params,
               const CameraParams& right_cam_params,
               const StereoMatchingParams& stereo_matching_params)
      : B_Pose_camLrect_(),
        stereo_camera_impl_(),
        stereo_calibration_(nullptr),
        baseline_(0.0),
        left_cam_params_(left_cam_params),
        right_cam_params_(right_cam_params),
        left_cam_rectified_params_(),
        right_cam_rectified_params_(),
        stereo_matching_params_(stereo_matching_params) {
    computeRectificationParameters(left_cam_params,
                                   right_cam_params,
                                   &left_cam_rectified_params_,
                                   &right_cam_rectified_params_,
                                   &B_Pose_camLrect_,
                                   &baseline_);
    gtsam::Cal3_S2 left_undist_rect_cam_mat =
        UtilsOpenCV::Cvmat2Cal3_S2(left_cam_rectified_params_.P_);

    //! Create stereo camera calibration after rectification and undistortion.
    stereo_calibration_ = boost::make_shared<gtsam::Cal3_S2Stereo>(
        left_undist_rect_cam_mat.fx(),
        left_undist_rect_cam_mat.fy(),
        left_undist_rect_cam_mat.skew(),
        left_undist_rect_cam_mat.px(),
        left_undist_rect_cam_mat.py(),
        baseline_);

    //! Create stereo camera implementation
    stereo_camera_impl_ =
        gtsam::StereoCamera(B_Pose_camLrect_, stereo_calibration_);
    CHECK_GT(baseline_, 0);
  }
  virtual ~StereoCamera() = default;

  /**
   * @brief getStereoCalib
   * @return stereo camera calibration after undistortion and rectification.
   */
  // Ideally this would return a const shared pointer or a copy, but GTSAM's
  // idiosyncrasies require shared ptrs all over the place.
  gtsam::Cal3_S2Stereo::shared_ptr getStereoCalib() const {
    return stereo_calibration_;
  }

  /**
   * @brief getLeftCamPose
   * @return left cam pose after rectification wrt body frame of reference.
   */
  gtsam::Pose3 getLeftCamPose() const { return B_Pose_camLrect_; }

  void rectifyUndistortStereoFrame(
      StereoFrame* stereo_frame,
      const cv::InterpolationFlags& interpolation_type = cv::INTER_LINEAR) {
    CHECK_NOTNULL(stereo_frame);
    //! Rectify and undistort images using precomputed maps.
    cv::remap(stereo_frame->getLeftFrameMutable()->img_,
              stereo_frame->left_img_rectified_,
              left_cam_params_.undistRect_map_x_,
              left_cam_params_.undistRect_map_y_,
              interpolation_type);
    cv::remap(stereo_frame->getRightFrameMutable()->img_,
              stereo_frame->right_img_rectified_,
              right_cam_params_.undistRect_map_x_,
              right_cam_params_.undistRect_map_y_,
              interpolation_type);
  }

 protected:
  //! Body pose of left cam after rectification
  gtsam::Pose3 B_Pose_camLrect_;

  //! Stereo camera implementation
  gtsam::StereoCamera stereo_camera_impl_;

  //! Stereo camera calibration
  gtsam::Cal3_S2Stereo::shared_ptr stereo_calibration_;

  //! Stereo baseline
  double baseline_;

  //! Non-rectified parameters.
  CameraParams left_cam_params_;
  CameraParams right_cam_params_;

  //! Rectified parameters.
  CameraParams left_cam_rectified_params_;
  CameraParams right_cam_rectified_params_;

  //! Parameters for dense stereo matching
  StereoMatchingParams stereo_matching_params_;

  /**
   * @brief computeRectificationParameters
   * @param[in,out] left_cam_params IN Non-rectified camera parameters, OUT
   * camera parameters after rectification
   * @param[in,out] right_cam_params IN non-rectified, OUT rectified.
   * @param[out] B_Pose_camLrect pose from Body to rectified left camera
   * @param[out] baseline New baseline after rectification
   */
  static void computeRectificationParameters(
      const CameraParams& left_cam_params,
      const CameraParams& right_cam_params,
      CameraParams* left_cam_rectified_params,
      CameraParams* right_cam_rectified_params,
      gtsam::Pose3* B_Pose_camLrect,
      double* baseline) {
    CHECK_NOTNULL(left_cam_rectified_params);
    CHECK_NOTNULL(right_cam_rectified_params);
    CHECK_NOTNULL(B_Pose_camLrect);
    CHECK_NOTNULL(baseline);

    // Get extrinsics in open CV format.
    cv::Mat L_Rot_R, L_Tran_R;

    //! Extrinsics of the stereo (not rectified) relative pose between cameras
    gtsam::Pose3 camL_Pose_camR = (left_cam_params.body_Pose_cam_)
                                      .between(right_cam_params.body_Pose_cam_);
    // NOTE: openCV pose convention is the opposite, that's why we have to
    // invert
    boost::tie(L_Rot_R, L_Tran_R) =
        UtilsOpenCV::Pose2cvmats(camL_Pose_camR.inverse());

    //////////////////////////////////////////////////////////////////////////////
    // get rectification matrices
    *left_cam_rectified_params = left_cam_params;
    *right_cam_rectified_params = right_cam_params;

    // P1 and P2 are the new camera matrices, but with an extra 0 0 0 column
    cv::Mat P1, P2, Q;

    if (left_cam_params.distortion_model_ == "radtan" ||
        left_cam_params.distortion_model_ == "radial-tangential") {
      // Get stereo rectification
      VLOG(10) << "Stereo camera distortion for rectification: radtan";
      cv::stereoRectify(
          // Input
          left_cam_params.camera_matrix_,
          left_cam_params.distortion_coeff_,
          right_cam_params.camera_matrix_,
          right_cam_params.distortion_coeff_,
          left_cam_params.image_size_,
          L_Rot_R,
          L_Tran_R,
          // Output
          left_cam_rectified_params->R_rectify_,
          right_cam_rectified_params->R_rectify_,
          P1,
          P2,
          Q);
    } else if (left_cam_params.distortion_model_ == "equidistant") {
      // Get stereo rectification
      VLOG(10) << "Stereo camera distortion for rectification: equidistant";
      cv::fisheye::stereoRectify(left_cam_params.camera_matrix_,
                                 left_cam_params.distortion_coeff_,
                                 right_cam_params.camera_matrix_,
                                 right_cam_params.distortion_coeff_,
                                 left_cam_params.image_size_,
                                 L_Rot_R,
                                 L_Tran_R,
                                 // following are output
                                 left_cam_rectified_params->R_rectify_,
                                 right_cam_rectified_params->R_rectify_,
                                 P1,
                                 P2,
                                 Q,
                                 // TODO: Flag to maximise area???
                                 cv::CALIB_ZERO_DISPARITY);
    } else {
      LOG(ERROR) << "Stereo camera distortion model not found for stereo "
                    "rectification!";
    }

    VLOG(10) << "RESULTS OF RECTIFICATION: \n"
             << "left_camera_info.R_rectify_\n"
             << left_cam_rectified_params->R_rectify_ << '\n'
             << "right_camera_info.R_rectify_\n"
             << right_cam_rectified_params->R_rectify_;

    // Left camera pose after rectification
    gtsam::Rot3 camL_Rot_camLrect =
        UtilsOpenCV::cvMatToGtsamRot3(left_cam_rectified_params->R_rectify_)
            .inverse();
    //! Fix camL position to camLrect.
    //! aka use gtsam::Point3()
    gtsam::Pose3 camL_Pose_camLrect(camL_Rot_camLrect, gtsam::Point3::Zero());
    *B_Pose_camLrect =
        left_cam_params.body_Pose_cam_.compose(camL_Pose_camLrect);

    // right camera pose after rectification
    gtsam::Rot3 camR_Rot_camRrect =
        UtilsOpenCV::cvMatToGtsamRot3(right_cam_rectified_params->R_rectify_)
            .inverse();
    gtsam::Pose3 camR_Pose_camRrect =
        gtsam::Pose3(camR_Rot_camRrect, gtsam::Point3());
    gtsam::Pose3 B_Pose_camRrect =
        (right_cam_params.body_Pose_cam_).compose(camR_Pose_camRrect);

    // Relative pose after rectification
    gtsam::Pose3 camLrect_Pose_calRrect =
        B_Pose_camLrect->between(B_Pose_camRrect);
    // get baseline
    *baseline = camLrect_Pose_calRrect.translation().x();

    // Sanity check.
    LOG_IF(FATAL,
           gtsam::Rot3::Logmap(camLrect_Pose_calRrect.rotation()).norm() > 1e-5)
        << "camL_Pose_camR log: "
        << gtsam::Rot3::Logmap(camL_Pose_camR.rotation()).norm() << '\n'
        << "camLrect_Pose_calRrect log: "
        << gtsam::Rot3::Logmap(camLrect_Pose_calRrect.rotation()).norm() << '\n'
        << "Vio constructor: camera poses do not seem to be rectified (rot)";
    LOG_IF(FATAL,
           fabs(camLrect_Pose_calRrect.translation().y()) > 1e-3 ||
               fabs(camLrect_Pose_calRrect.translation().z()) > 1e-3)
        << "Vio constructor: camera poses do not seem to be rectified (tran) \n"
        << "camLrect_Poe_calRrect: " << camLrect_Pose_calRrect;

    //////////////////////////////////////////////////////////////////////////////
    // TODO: Unit tests for this sections!!!!!

    // Left camera
    if (left_cam_params.distortion_model_ == "radtan" ||
        left_cam_params.distortion_model_ == "radial-tangential") {
      // Get rectification & undistortion maps. (radtan dist. model)
      VLOG(10) << "Left camera distortion: radtan";
      cv::initUndistortRectifyMap(left_cam_params.camera_matrix_,
                                  left_cam_params.distortion_coeff_,
                                  left_cam_params.R_rectify_,
                                  P1,
                                  left_cam_params.image_size_,
                                  CV_32FC1,
                                  // Output:
                                  left_cam_rectified_params->undistRect_map_x_,
                                  left_cam_rectified_params->undistRect_map_y_);
    } else if (left_cam_params.distortion_model_ == "equidistant") {
      // Get rectification & undistortion maps. (equi dist. model)
      VLOG(10) << "Left camera distortion: equidistant";
      cv::fisheye::initUndistortRectifyMap(
          left_cam_params.camera_matrix_,
          left_cam_params.distortion_coeff_,
          left_cam_params.R_rectify_,
          P1,
          left_cam_params.image_size_,
          CV_32F,
          // Output:
          left_cam_rectified_params->undistRect_map_x_,
          left_cam_rectified_params->undistRect_map_y_);
    } else {
      LOG(ERROR) << "Camera distortion model not found for left camera!";
    }

    // Right camera
    if (right_cam_params.distortion_model_ == "radtan" ||
        right_cam_params.distortion_model_ == "radial-tangential") {
      // Get rectification & undistortion maps. (radtan dist. model)
      VLOG(10) << "Right camera distortion: radtan";
      cv::initUndistortRectifyMap(
          right_cam_params.camera_matrix_,
          right_cam_params.distortion_coeff_,
          right_cam_params.R_rectify_,
          P2,
          right_cam_params.image_size_,
          CV_32FC1,
          // Output:
          right_cam_rectified_params->undistRect_map_x_,
          right_cam_rectified_params->undistRect_map_y_);
    } else if (right_cam_params.distortion_model_ == "equidistant") {
      // Get rectification & undistortion maps. (equi dist. model)
      VLOG(10) << "Right camera distortion: equidistant";
      cv::fisheye::initUndistortRectifyMap(
          right_cam_params.camera_matrix_,
          right_cam_params.distortion_coeff_,
          right_cam_params.R_rectify_,
          P2,
          right_cam_params.image_size_,
          CV_32F,
          // Output:
          right_cam_rectified_params->undistRect_map_x_,
          right_cam_rectified_params->undistRect_map_y_);
    } else {
      LOG(ERROR) << "Camera distortion model not found for right camera!";
    }

    // Store intermediate results from rectification.
    // contains an extra column to project in homogeneous coordinates
    left_cam_rectified_params->P_ = P1;
    // contains an extra column to project in homogeneous coordinates
    right_cam_rectified_params->P_ = P2;
  }

 private:
};

}  // namespace VIO
