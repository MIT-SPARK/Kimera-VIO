/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoCamera.cpp
 * @brief  Class describing a stereo camera.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/StereoCamera.h"

#include <Eigen/Core>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <boost/utility.hpp>  // for tie

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>

#include <glog/logging.h>

#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/frontend/UndistorterRectifier.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

StereoCamera::StereoCamera(const CameraParams& left_cam_params,
                           const CameraParams& right_cam_params)
    : original_left_camera_(nullptr),
      original_right_camera_(nullptr),
      undistorted_rectified_stereo_camera_impl_(),
      stereo_calibration_(nullptr),
      stereo_baseline_(0.0),
      left_cam_undistort_rectifier_(nullptr),
      right_cam_undistort_rectifier_(nullptr) {
  computeRectificationParameters(left_cam_params,
                                 right_cam_params,
                                 &R1_,
                                 &R2_,
                                 &P1_,
                                 &P2_,
                                 &Q_,
                                 &ROI1_,
                                 &ROI2_);
  original_left_camera_ = std::make_shared<VIO::Camera>(left_cam_params);
  original_right_camera_ = std::make_shared<VIO::Camera>(right_cam_params);

  // Calc left camera pose after rectification
  // NOTE: OpenCV pose convention is the opposite, therefore the inverse.
  const gtsam::Rot3& camL_Rot_camLrect =
      UtilsOpenCV::cvMatToGtsamRot3(R1_).inverse();
  gtsam::Pose3 camL_Pose_camLrect(camL_Rot_camLrect, gtsam::Point3::Zero());
  B_Pose_camLrect_ = left_cam_params.body_Pose_cam_.compose(camL_Pose_camLrect);

  const gtsam::Rot3& camR_Rot_camRrect =
      UtilsOpenCV::cvMatToGtsamRot3(R2_).inverse();
  gtsam::Pose3 camR_Pose_camRrect(camR_Rot_camRrect, gtsam::Point3::Zero());
  B_Pose_camRrect_ = left_cam_params.body_Pose_cam_.compose(camR_Pose_camRrect);

  // Calc baseline (see L.2700 and L.2616 in
  // https://github.com/opencv/opencv/blob/master/modules/calib3d/src/calibration.cpp
  // NOTE: OpenCV pose convention is the opposite, therefore the missing -1.0
  CHECK_NE(Q_.at<double>(3, 2), 0.0);
  stereo_baseline_ = 1.0 / Q_.at<double>(3, 2);
  CHECK_GT(stereo_baseline_, 0.0);

  //! Create stereo camera calibration after rectification and undistortion.
  const gtsam::Cal3_S2& left_undist_rect_cam_mat =
      UtilsOpenCV::Cvmat2Cal3_S2(P1_);
  stereo_calibration_ =
      boost::make_shared<gtsam::Cal3_S2Stereo>(left_undist_rect_cam_mat.fx(),
                                               left_undist_rect_cam_mat.fy(),
                                               left_undist_rect_cam_mat.skew(),
                                               left_undist_rect_cam_mat.px(),
                                               left_undist_rect_cam_mat.py(),
                                               stereo_baseline_);

  //! Create undistort rectifiers: these should be called after
  //! computeRectificationParameters.
  left_cam_undistort_rectifier_ =
      VIO::make_unique<UndistorterRectifier>(P1_, left_cam_params, R1_);
  right_cam_undistort_rectifier_ =
      VIO::make_unique<UndistorterRectifier>(P2_, right_cam_params, R2_);

  //! Create stereo camera implementation
  undistorted_rectified_stereo_camera_impl_ =
      gtsam::StereoCamera(B_Pose_camLrect_, stereo_calibration_);
}

StereoCamera::StereoCamera(Camera::ConstPtr left_camera, Camera::ConstPtr right_camera)
    : StereoCamera(
          left_camera->getCamParams(),
          right_camera->getCamParams()) {}

void StereoCamera::project(const LandmarksCV& lmks,
                           KeypointsCV* left_kpts,
                           KeypointsCV* right_kpts) const {
  CHECK_NOTNULL(left_kpts)->clear();
  CHECK_NOTNULL(right_kpts)->clear();
  left_kpts->clear();
  right_kpts->clear();
  const auto& n_lmks = lmks.size();
  left_kpts->resize(n_lmks);
  right_kpts->resize(n_lmks);
  // Can be greatly optimized using matrix multiplication/vectorization.
  for (size_t i = 0u; i < n_lmks; i++) {
    project(lmks[i], &(*left_kpts)[i], &(*right_kpts)[i]);
  }
}

void StereoCamera::project(const LandmarkCV& lmk,
                           KeypointCV* left_kpt,
                           KeypointCV* right_kpt) const {
  CHECK_NOTNULL(left_kpt);
  CHECK_NOTNULL(right_kpt);
  const gtsam::StereoPoint2& kp =
      undistorted_rectified_stereo_camera_impl_.project2(
          gtsam::Point3(lmk.x, lmk.y, lmk.z));
  *left_kpt = KeypointCV(kp.uL(), kp.v());
  *right_kpt = KeypointCV(kp.uR(), kp.v());
}

void StereoCamera::backProjectDepth(const KeypointCV& kp,
                                    const Depth& depth,
                                    LandmarkCV* lmk) const {
  CHECK_NOTNULL(lmk);
  CHECK(stereo_calibration_ != nullptr);
  CHECK_GT(depth, 0.0) << "Requested back projection of a keypoint :" << kp
                       << "\n with negative depth: " << depth;
  auto disparity =
      stereo_calibration_->fy() * stereo_calibration_->baseline() / depth;
  backProjectDisparity(kp, disparity, lmk);
}

void StereoCamera::backProjectDisparity(const KeypointCV& kp,
                                        const double& disparity,
                                        LandmarkCV* lmk) const {
  CHECK_NOTNULL(lmk);
  CHECK(stereo_calibration_ != nullptr);
  CHECK_GT(disparity, 0.0) << "Requested back projection of a keypoint :" << kp
                           << "\n with negative disparity: " << disparity;
  gtsam::StereoPoint2 z(kp.x, kp.x - disparity, kp.y);
  gtsam::Point3 gtsam_lmk =
      undistorted_rectified_stereo_camera_impl_.backproject2(z);
  *lmk = LandmarkCV(gtsam_lmk.x(), gtsam_lmk.y(), gtsam_lmk.z());
}

// NOT TESTED and probably wrong!
void StereoCamera::backProject(const KeypointsCV& kps,
                               const cv::Mat& disparity_img,
                               LandmarksCV* lmks) const {
  CHECK_NOTNULL(lmks)->clear();
  CHECK_EQ(disparity_img.type(), CV_16S);  // 16-bit signed
  lmks->reserve(kps.size());
  for (const KeypointCV& kp : kps) {
    // Some algorithms, like StereoBM or StereoSGBM compute 16-bit fixed-point
    // disparity map (where each disparity value has 4 fractional bits),
    // whereas other algorithms output 32-bit floating-point disparity map.
    // !!!!!!!! WARNING gtsam's stereo points are doubles, our keypoints are
    // floats,
    // and disparity is int16_t...
    double disparity = static_cast<double>(disparity_img.at<int16_t>(kp));
    // TODO(TONI): check disparity is valid!!
    CHECK_GE(disparity, 0.0);
    LandmarkCV lmk;
    backProjectDisparity(kp, disparity, &lmk);
    lmks->push_back(lmk);
  }
}

void StereoCamera::backProjectDisparityTo3D(const cv::Mat& disparity_img,
                                            cv::Mat* depth) const {
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
  CHECK_NE(Q_.at<double>(3, 2), 0.0);
  CHECK_EQ(Q_.at<double>(3, 3), 0.0)
      << "Set CALIB_ZERO_DISPARITY when doing stereo rectification.";
  static constexpr bool kHandleMissingValues = true;
  cv::reprojectImageTo3D(
      disparity_img, *depth, Q_, kHandleMissingValues, CV_32F);
  CHECK_EQ(disparity_img.rows, depth->rows);
  CHECK_EQ(disparity_img.cols, depth->cols);
}

void StereoCamera::backProjectDisparityTo3DManual(const cv::Mat& disparity_img,
                                                  cv::Mat* depth) const {
  CHECK_NOTNULL(depth);
  CHECK_EQ(disparity_img.type(), CV_32F);
  CHECK(!disparity_img.empty());
  CHECK_EQ(Q_.type(), CV_64F);
  CHECK_EQ(Q_.rows, 4);
  CHECK_EQ(Q_.cols, 4);

  // Same size than the disparity_img
  *depth = cv::Mat::zeros(disparity_img.size(), CV_32FC3);

  // Non-zero elements from Q
  double Q03 = Q_.at<double>(0, 3);  // -c_x
  double Q13 = Q_.at<double>(1, 3);  // -c_y
  double Q23 = Q_.at<double>(2, 3);  // f
  double Q32 = Q_.at<double>(3, 2);  // -1.0 / T_x where T_x is the baseline
  double Q33 = Q_.at<double>(3, 3);  // (c_x - c_x') / T_x

  // Get xyz from disparity
  for (size_t i = 0u; i < disparity_img.rows; i++) {
    // Loop over rows
    const float* disp_ptr = disparity_img.ptr<float>(i);
    cv::Vec3f* xyz_ptr = depth->ptr<cv::Vec3f>(i);

    for (size_t j = 0u; j < disparity_img.cols; j++) {
      // Loop over cols
      const float pw = 1.0f / (disp_ptr[j] * Q32 + Q33);

      // For each pixel, generate 3D point
      cv::Vec3f& point = xyz_ptr[j];
      point[0] = (static_cast<float>(j) + Q03) * pw;
      point[1] = (static_cast<float>(i) + Q13) * pw;
      point[2] = Q23 * pw;
    }
  }
}

void StereoCamera::undistortRectifyLeftKeypoints(
    const KeypointsCV& keypoints,
    StatusKeypointsCV* status_keypoints_rectified) const {
  KeypointsCV undistorted_rectified_keypoints;
  CHECK(left_cam_undistort_rectifier_);
  left_cam_undistort_rectifier_->undistortRectifyKeypoints(
      keypoints, &undistorted_rectified_keypoints);
  left_cam_undistort_rectifier_->checkUndistortedRectifiedLeftKeypoints(
      keypoints, undistorted_rectified_keypoints, status_keypoints_rectified);
}

void StereoCamera::distortUnrectifyRightKeypoints(
    const StatusKeypointsCV& status_keypoints_rectified,
    KeypointsCV* keypoints) const {
  right_cam_undistort_rectifier_->distortUnrectifyKeypoints(
      status_keypoints_rectified, keypoints);
}

void StereoCamera::undistortRectifyStereoFrame(StereoFrame* stereo_frame) const {
  CHECK_NOTNULL(stereo_frame);
  //! Warn if stupid behavior from user
  LOG_IF(WARNING, stereo_frame->isRectified())
      << "Rectifying already rectified stereo frame ...";

  //! Left img
  CHECK(left_cam_undistort_rectifier_);
  cv::Mat left_img_rectified;
  left_cam_undistort_rectifier_->undistortRectifyImage(
      stereo_frame->left_frame_.img_, &left_img_rectified);

  //! Right img
  CHECK(right_cam_undistort_rectifier_);
  cv::Mat right_img_rectified;
  right_cam_undistort_rectifier_->undistortRectifyImage(
      stereo_frame->right_frame_.img_, &right_img_rectified);

  //! Update stereo_frame
  stereo_frame->setRectifiedImages(left_img_rectified, right_img_rectified);
}

void StereoCamera::computeRectificationParameters(
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
  gtsam::Pose3 camL_Pose_camR =
      (left_cam_params.body_Pose_cam_).between(right_cam_params.body_Pose_cam_);

  // Get extrinsics in open CV format.
  // NOTE: openCV pose convention is the opposite, that's why we have to
  // invert
  cv::Mat camL_Rot_camR, camL_Tran_camR;
  boost::tie(camL_Rot_camR, camL_Tran_camR) =
      UtilsOpenCV::Pose2cvmats(camL_Pose_camR.inverse());

  // kAlpha is -1 by default, but that introduces invalid keypoints!
  // here we should use kAlpha = 0 so we get only valid pixels...
  // But that has an issue that it removes large part of the image, check:
  // https://github.com/opencv/opencv/issues/7240 for this issue with kAlpha
  // Setting to -1 to make it easy, but it should NOT be -1!
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

}  // namespace VIO
