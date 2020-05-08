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
//#include <opencv2/ximgproc.hpp>

#include <boost/utility.hpp>  // for tie

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class StereoFrame;

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
                       const cv::Mat& R = cv::Mat()) {
    initUndistortRectifyMaps(cam_params, R, P, &map_x_, &map_y_);
  }

  virtual ~UndistorterRectifier() = default;

 public:
  void undistortRectifyImage(const cv::Mat& img, cv::Mat* undistorted_img) {
    CHECK_NOTNULL(undistorted_img);
    CHECK_EQ(map_x_.size, img.size);
    CHECK_EQ(map_y_.size, img.size);
    cv::remap(img,
              *undistorted_img,
              map_x_,
              map_y_,
              remap_interpolation_type_,
              remap_use_constant_border_type_ ? cv::BORDER_CONSTANT
                                              : cv::BORDER_REPLICATE);
  }

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
                                cv::Mat* map_y) {
    CHECK_NOTNULL(map_x);
    CHECK_NOTNULL(map_y);
    static constexpr int kImageType = CV_32FC1;
    cv::Mat map_x_float, map_y_float;
    switch (cam_params.distortion_model_) {
      case DistortionModel::NONE: {
        map_x_float.create(cam_params.image_size_, kImageType);
        map_y_float.create(cam_params.image_size_, kImageType);
      } break;
      case DistortionModel::RADTAN: {
        cv::initUndistortRectifyMap(
            // Input
            cam_params.K_,
            cam_params.distortion_coeff_mat_,
            R,
            P,
            cam_params.image_size_,
            kImageType,
            // Output:
            map_x_float,
            map_y_float);
      } break;
      case DistortionModel::EQUIDISTANT: {
        cv::fisheye::initUndistortRectifyMap(
            // Input,
            cam_params.K_,
            cam_params.distortion_coeff_mat_,
            R,
            P,
            cam_params.image_size_,
            kImageType,
            // Output:
            map_x_float,
            map_y_float);
      } break;
      default: {
        LOG(FATAL) << "Unknown distortion model: "
                   << VIO::to_underlying(cam_params.distortion_model_);
      }
    }

    // The reason we convert from floating to fixed-point representations
    // of a map is that they can yield much faster (~2x) remapping operations.
    cv::convertMaps(map_x_float, map_y_float, *map_x, *map_y, CV_16SC2, false);
  }

 protected:
  cv::Mat map_x_;
  cv::Mat map_y_;

  // Replicate instead of constant is more efficient for GPUs to calculate.
  bool remap_use_constant_border_type_ = false;
  int remap_interpolation_type_ = cv::INTER_LINEAR;
};

class Camera {
 public:
  KIMERA_POINTER_TYPEDEFS(Camera);
  KIMERA_DELETE_COPY_CONSTRUCTORS(Camera);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Camera(const CameraParams& cam_params)
      : camera_impl_(),
        calibration_(cam_params.intrinsics_.at(0),
                     cam_params.intrinsics_.at(1),
                     0.0,  // No skew
                     cam_params.intrinsics_.at(2),
                     cam_params.intrinsics_.at(3)),
        cam_params_(cam_params) {}

  virtual ~Camera() = default;

 public:
  /** NOT TESTED
   * @brief project Lmks into images, doesn't do any check...
   * @param lmks
   * @param kpts
   */
  void project(const LandmarksCV& lmks, KeypointsCV* kpts) const {
    CHECK_NOTNULL(kpts)->clear();
    const auto& n_lmks = lmks.size();
    kpts->reserve(n_lmks);
    for (const auto& lmk : lmks) {
      const gtsam::StereoPoint2& kp =
          camera_impl_.project2(gtsam::Point3(lmk.x, lmk.y, lmk.z));
      kpts->push_back(KeypointCV(kp.uL(), kp.v()));
    }
  }

  /** NOT TESTED
   * @brief backProject keypoints given depth
   * @param kps
   * @param disparity_img
   */
  void backProject(const KeypointsCV& kps,
                   const double& depth,
                   LandmarksCV* lmks) {
    CHECK_NOTNULL(lmks)->clear();
    lmks->reserve(kps.size());
    for (const KeypointCV& kp : kps) {
      gtsam::Point2 z(kp.x, kp.y);
      gtsam::Point3 lmk = camera_impl_.backproject(z, depth);
      lmks->push_back(LandmarkCV(lmk.x(), lmk.y(), lmk.z()));
    }
  }

 private:
  CameraParams cam_params_;
  gtsam::Cal3_S2 calibration_;
  gtsam::PinholeCamera<gtsam::Cal3_S2> camera_impl_;
};

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
