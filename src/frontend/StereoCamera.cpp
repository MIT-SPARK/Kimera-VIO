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

#include "kimera-vio/frontend/UndistorterRectifier.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

StereoCamera::StereoCamera(const CameraParams& left_cam_params,
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
  B_Pose_camLrect_ = left_cam_params.body_Pose_cam_.compose(camL_Pose_camLrect);

  // Calc baseline (see L.2700 and L.2616 in
  // https://github.com/opencv/opencv/blob/master/modules/calib3d/src/calibration.cpp
  // NOTE: OpenCV pose convention is the opposite, therefore the missing -1.0
  CHECK_NE(Q_.at<double>(3, 2), 0.0);
  baseline_ = 1.0 / Q_.at<double>(3, 2);
  CHECK_GT(baseline_, 0.0);

  //! Create stereo camera calibration after rectification and undistortion.
  gtsam::Cal3_S2 left_undist_rect_cam_mat = UtilsOpenCV::Cvmat2Cal3_S2(P1_);
  stereo_calibration_ =
      boost::make_shared<gtsam::Cal3_S2Stereo>(left_undist_rect_cam_mat.fx(),
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

void StereoCamera::project(const LandmarksCV& lmks,
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

void StereoCamera::stereoDisparityReconstruction(const cv::Mat& left_img,
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

void StereoCamera::backProjectDisparityTo3D(const cv::Mat& disparity_img,
                                            cv::Mat* depth) {
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
  static constexpr bool kHandleMissingValues = true;
  cv::reprojectImageTo3D(
      disparity_img, *depth, Q_, kHandleMissingValues, CV_32F);
}

void StereoCamera::undistortRectifyStereoFrame(StereoFrame* stereo_frame) {
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
  static constexpr int kAlpha = -1;
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
