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

#include <glog/logging.h>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

//
// class MultiCamera : public Camera {
//
//};

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

  Camera(const CameraParams& cam_params) {}

  virtual ~Camera() = default;

 public:
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
               const StereoMatchingParams& stereo_matching_params)
      : stereo_camera_impl_(),
        stereo_calibration_(nullptr),
        baseline_(0.0),
        left_cam_params_(left_cam_params),
        right_cam_params_(right_cam_params),
        left_cam_undistort_rectifier_(nullptr),
        right_cam_undistort_rectifier_(nullptr),
        stereo_matching_params_(stereo_matching_params) {
    computeRectificationParameters(left_cam_params,
                                   right_cam_params,
                                   &R1_,
                                   &R2_,
                                   &P1_,
                                   &P2_,
                                   &Q_,
                                   &ROI1_,
                                   &ROI2_);

    // Calc left camera pose after rectification
    // NOTE: OpenCV pose convention is the opposite, therefore the inverse.
    const gtsam::Rot3& camL_Rot_camLrect =
        UtilsOpenCV::cvMatToGtsamRot3(R1_).inverse();
    gtsam::Pose3 camL_Pose_camLrect(camL_Rot_camLrect, gtsam::Point3::Zero());
    B_Pose_camLrect_ =
        left_cam_params.body_Pose_cam_.compose(camL_Pose_camLrect);

    // Calc baseline
    baseline_ =
        left_cam_params.body_Pose_cam_.between(right_cam_params.body_Pose_cam_)
            .translation()
            .x();
    CHECK_GT(baseline_, 0.0);

    //! Create stereo camera calibration after rectification and undistortion.
    gtsam::Cal3_S2 left_undist_rect_cam_mat = UtilsOpenCV::Cvmat2Cal3_S2(P1_);
    stereo_calibration_ = boost::make_shared<gtsam::Cal3_S2Stereo>(
        left_undist_rect_cam_mat.fx(),
        left_undist_rect_cam_mat.fy(),
        left_undist_rect_cam_mat.skew(),
        left_undist_rect_cam_mat.px(),
        left_undist_rect_cam_mat.py(),
        baseline_);

    //! Create undistort rectifiers: these should be called after
    //! computeRectificationParameters.
    left_cam_undistort_rectifier_ =
        VIO::make_unique<UndistorterRectifier>(P1_, left_cam_params, R1_);
    right_cam_undistort_rectifier_ =
        VIO::make_unique<UndistorterRectifier>(P2_, right_cam_params, R2_);

    //! Create stereo camera implementation
    stereo_camera_impl_ =
        gtsam::StereoCamera(B_Pose_camLrect_, stereo_calibration_);
  }

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
               KeypointsCV* right_kpts) const {
    CHECK_NOTNULL(left_kpts)->clear();
    CHECK_NOTNULL(right_kpts)->clear();
    const auto& n_lmks = lmks.size();
    left_kpts->reserve(n_lmks);
    right_kpts->reserve(n_lmks);
    for (const auto& lmk : lmks) {
      const gtsam::StereoPoint2& kp =
          stereo_camera_impl_.project2(gtsam::Point3(lmk.x, lmk.y, lmk.z));
      left_kpts->push_back(KeypointCV(kp.uL(), kp.v()));
      right_kpts->push_back(KeypointCV(kp.uR(), kp.v()));
    }
  }

  /**
   * @brief backProject keypoints given disparity image
   * @param kps
   * @param disparity_img
   */
  void backProject(const KeypointsCV& kps,
                   const cv::Mat& disparity_img,
                   LandmarksCV* lmks) {
    CHECK_NOTNULL(lmks)->clear();
    CHECK_EQ(disparity_img.type(), CV_16S);  // 16-bit signed
    lmks->reserve(kps.size());
    for (const KeypointCV& kp : kps) {
      // Some algorithms, like StereoBM or StereoSGBM compute 16-bit fixed-point
      // disparity map (where each disparity value has 4 fractional bits),
      // whereas other algorithms output 32-bit floating-point disparity map.
      auto disparity = disparity_img.at<int16_t>(kp);
      // TODO(TONI): check disparity is valid!!
      CHECK_GE(disparity, 0);
      // WARNING gtsam's stereo points are doubles, our keypoints are floats,
      // and disparity is int16_t...
      gtsam::StereoPoint2 z(kp.x, kp.x + disparity, kp.y);
      gtsam::Point3 lmk = stereo_camera_impl_.backproject2(z);
      lmks->push_back(LandmarkCV(lmk.x(), lmk.y(), lmk.z()));
    }
  }

  void stereoDisparityReconstruction(const cv::Mat& left_img,
                                     const cv::Mat& right_img,
                                     cv::Mat* disparity_img) {
    CHECK_NOTNULL(disparity_img);
    CHECK_EQ(disparity_img->cols, left_img.cols);
    CHECK_EQ(right_img.cols, left_img.cols);
    CHECK_EQ(disparity_img->rows, left_img.rows);
    CHECK_EQ(right_img.rows, left_img.rows);
    CHECK_EQ(right_img.type(), left_img.type());
    CHECK_EQ(disparity_img->type(), CV_32F);

    // Setup stereo matcher
    cv::Ptr<cv::StereoMatcher> stereo_matcher;
    if (use_sgbm_) {
      int mode;
      if (use_mode_HH_) {
        mode = cv::StereoSGBM::MODE_HH;
      } else {
        mode = cv::StereoSGBM::MODE_SGBM;
      }
      stereo_matcher = cv::StereoSGBM::create(min_disparity_,
                                              num_disparities_,
                                              sad_window_size_,
                                              p1_,
                                              p2_,
                                              disp_12_max_diff_,
                                              pre_filter_cap_,
                                              uniqueness_ratio_,
                                              speckle_window_size_,
                                              speckle_range_,
                                              mode);
    } else {
      cv::Ptr<cv::StereoBM> sbm =
          cv::StereoBM::create(num_disparities_, sad_window_size_);

      sbm->setPreFilterType(pre_filter_type_);
      sbm->setPreFilterSize(pre_filter_size_);
      sbm->setPreFilterCap(pre_filter_cap_);
      sbm->setMinDisparity(min_disparity_);
      sbm->setTextureThreshold(texture_threshold_);
      sbm->setUniquenessRatio(uniqueness_ratio_);
      sbm->setSpeckleRange(speckle_range_);
      sbm->setSpeckleWindowSize(speckle_window_size_);
      if (!ROI1_.empty() && !ROI2_.empty()) {
        sbm->setROI1(ROI1_);
        sbm->setROI2(ROI2_);
      } else {
        LOG(WARNING) << "ROIs are empty.";
      }

      stereo_matcher = sbm;
    }

    // Reconstruct scene
    stereo_matcher->compute(left_img, right_img, *disparity_img);

    // Optionally, post-filter disparity
    if (post_filter_disparity_) {
      // Use disparity post-filter
      // wls_filter = createDisparityWLSFilter(left_matcher);
      // Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
      // See
      // https://docs.opencv.org/3.3.1/d3/d14/tutorial_ximgproc_disparity_filtering.html#gsc.tab=0
    }

    // Optionally, smooth the disparity image
    if (median_blur_disparity_) {
      cv::medianBlur(*disparity_img, *disparity_img, 5);
    }

    static constexpr bool debug = false;
    if (debug) {
      // cv::Mat raw_disp_vis;
      // cv::ximgproc::getDisparityVis(left_disp,raw_disp_vis,vis_mult);
      // cv::namedWindow("raw disparity", WINDOW_AUTOSIZE);
      // cv::imshow("raw disparity", raw_disp_vis);
      // cv::Mat filtered_disp_vis;
      // cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
      // cv::namedWindow("filtered disparity", WINDOW_AUTOSIZE);
      // cv::imshow("filtered disparity", filtered_disp_vis);
      // cv::waitKey();
    }
  }

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
  void backProjectDisparityTo3D(const cv::Mat& disparity_img, cv::Mat* depth) {
    CHECK_NOTNULL(depth);
    // Check disparity img is CV_32F, if it is CV_16S it should have been
    // divided by 16, see StereoBM/stereoSGBM output.
    CHECK(!disparity_img.empty());
    CHECK_EQ(disparity_img.type(), CV_32F)
        << "Wrong type for disparity img, mind that if the disparity image is "
           "of type CV_16S, you might need to divide the values by 16.";
    // We use the Q_ pre-computed during undistortRectification, so check it has
    // been computed: in principle, we calculate that at ctor time, so it is
    // always computed.
    LOG(INFO) << "Back projecting disparity to 3D.";
    static constexpr bool kHandleMissingValues = true;
    cv::reprojectImageTo3D(
        disparity_img, *depth, Q_, kHandleMissingValues, CV_32F);
  }

  /**
   * @brief getLeftCamRectPose Get left camera pose after rectification with
   * respect to the body frame.
   * @return
   */
  gtsam::Pose3 getLeftCamRectPose() const { return B_Pose_camLrect_; }

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
   * @brief rectifyUndistortStereoFrame
   * @param stereo_frame
   */
  void undistortRectifyStereoFrame(StereoFrame* stereo_frame) {
    CHECK_NOTNULL(stereo_frame);
    LOG_IF(WARNING, stereo_frame->isRectified())
        << "Rectifying already rectified stereo frame ...";
    stereo_frame->setIsRectified(true);
    //! Rectify and undistort images using precomputed maps.
    CHECK(left_cam_undistort_rectifier_ != nullptr);
    left_cam_undistort_rectifier_->undistortRectifyImage(
        stereo_frame->getLeftFrameMutable()->img_,
        &stereo_frame->left_img_rectified_);
    CHECK(right_cam_undistort_rectifier_ != nullptr);
    right_cam_undistort_rectifier_->undistortRectifyImage(
        stereo_frame->getRightFrameMutable()->img_,
        &stereo_frame->right_img_rectified_);
  }

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
      cv::Rect* ROI2) {
    CHECK_NOTNULL(R1);
    CHECK_NOTNULL(R2);
    CHECK_NOTNULL(P1);
    CHECK_NOTNULL(P2);
    CHECK_NOTNULL(Q);
    CHECK_NOTNULL(ROI1);
    CHECK_NOTNULL(ROI2);

    //! Extrinsics of the stereo (not rectified) relative pose between cameras
    gtsam::Pose3 camL_Pose_camR = (left_cam_params.body_Pose_cam_)
                                      .between(right_cam_params.body_Pose_cam_);

    // Get extrinsics in open CV format.
    // NOTE: openCV pose convention is the opposite, that's why we have to
    // invert
    cv::Mat camL_Rot_camR, camL_Tran_camR;
    boost::tie(camL_Rot_camR, camL_Tran_camR) =
        UtilsOpenCV::Pose2cvmats(camL_Pose_camR.inverse());

    // kAlpha is -1 by default, here we set to 0 so we get only valid pixels...
    static constexpr int kAlpha = 0;
    switch (left_cam_params.distortion_model_) {
      case DistortionModel::RADTAN: {
        cv::stereoRectify(
            // Input
            left_cam_params.K_,
            left_cam_params.distortion_coeff_mat_,
            right_cam_params.K_,
            right_cam_params.distortion_coeff_mat_,
            left_cam_params.image_size_,
            camL_Rot_camR,
            camL_Tran_camR,
            // Output
            *R1,
            *R2,
            *P1,
            *P2,
            *Q,
            cv::CALIB_ZERO_DISPARITY,
            kAlpha,
            cv::Size(),
            ROI1,
            ROI2);
      } break;
      case DistortionModel::EQUIDISTANT: {
        cv::fisheye::stereoRectify(
            // Input
            left_cam_params.K_,
            left_cam_params.distortion_coeff_mat_,
            right_cam_params.K_,
            right_cam_params.distortion_coeff_mat_,
            left_cam_params.image_size_,
            camL_Rot_camR,
            camL_Tran_camR,
            // Output
            *R1,
            *R2,
            *P1,
            *P2,
            *Q,
            // TODO: Flag to maximise area???
            cv::CALIB_ZERO_DISPARITY);
      } break;
      default: {
        LOG(FATAL) << "Unknown DistortionModel: "
                   << VIO::to_underlying(left_cam_params.distortion_model_);
      }
    }
  }

 protected:
  //! Stereo camera implementation
  gtsam::StereoCamera stereo_camera_impl_;

  //! Stereo camera calibration
  gtsam::Cal3_S2Stereo::shared_ptr stereo_calibration_;

  gtsam::Pose3 B_Pose_camLrect_;

  //! Non-rectified parameters.
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
  /// cv::reprojectImageTo3D).
  cv::Mat Q_;

  cv::Rect ROI1_, ROI2_;

  //! Stereo baseline
  double baseline_;
  // TODO(Toni): perhaps wrap these params in a struct instead.

  // TODO(Toni): put on its own, dense stereo depth reconstruction
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
