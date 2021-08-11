/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoMatcher.cpp
 * @brief  Class describing stereo matching algorithms.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/StereoMatcher.h"

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>

#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

StereoMatcher::StereoMatcher(const StereoCamera::ConstPtr& stereo_camera,
                             const StereoMatchingParams& stereo_matching_params)
    : stereo_camera_(stereo_camera),
      stereo_matching_params_(stereo_matching_params),
      dense_stereo_params_() {}

void StereoMatcher::denseStereoReconstruction(
    const cv::Mat& left_img_rectified,
    const cv::Mat& right_img_rectified,
    cv::Mat* disparity_img) {
  CHECK_NOTNULL(disparity_img);
  CHECK_EQ(disparity_img->cols, left_img_rectified.cols);
  CHECK_EQ(right_img_rectified.cols, left_img_rectified.cols);
  CHECK_EQ(disparity_img->rows, left_img_rectified.rows);
  CHECK_EQ(right_img_rectified.rows, left_img_rectified.rows);
  CHECK_EQ(right_img_rectified.type(), left_img_rectified.type());
  CHECK_EQ(disparity_img->type(), CV_32F);
  CHECK(stereo_camera_);

  // Setup stereo matcher
  cv::Ptr<cv::StereoMatcher> cv_stereo_matcher;
  if (dense_stereo_params_.use_sgbm_) {
    int mode;
    if (dense_stereo_params_.use_mode_HH_) {
      mode = cv::StereoSGBM::MODE_HH;
    } else {
      mode = cv::StereoSGBM::MODE_SGBM;
    }
    cv_stereo_matcher =
        cv::StereoSGBM::create(dense_stereo_params_.min_disparity_,
                               dense_stereo_params_.num_disparities_,
                               dense_stereo_params_.sad_window_size_,
                               dense_stereo_params_.p1_,
                               dense_stereo_params_.p2_,
                               dense_stereo_params_.disp_12_max_diff_,
                               dense_stereo_params_.pre_filter_cap_,
                               dense_stereo_params_.uniqueness_ratio_,
                               dense_stereo_params_.speckle_window_size_,
                               dense_stereo_params_.speckle_range_,
                               mode);
  } else {
    cv::Ptr<cv::StereoBM> sbm =
        cv::StereoBM::create(dense_stereo_params_.num_disparities_,
                             dense_stereo_params_.sad_window_size_);

    sbm->setPreFilterType(dense_stereo_params_.pre_filter_type_);
    sbm->setPreFilterSize(dense_stereo_params_.pre_filter_size_);
    sbm->setPreFilterCap(dense_stereo_params_.pre_filter_cap_);
    sbm->setMinDisparity(dense_stereo_params_.min_disparity_);
    sbm->setTextureThreshold(dense_stereo_params_.texture_threshold_);
    sbm->setUniquenessRatio(dense_stereo_params_.uniqueness_ratio_);
    sbm->setSpeckleRange(dense_stereo_params_.speckle_range_);
    sbm->setSpeckleWindowSize(dense_stereo_params_.speckle_window_size_);
    const cv::Rect& roi1 = stereo_camera_->getROI1();
    const cv::Rect& roi2 = stereo_camera_->getROI2();
    if (!roi1.empty() && !roi2.empty()) {
      sbm->setROI1(roi1);
      sbm->setROI2(roi2);
    } else {
      LOG(WARNING) << "ROIs are empty.";
    }

    cv_stereo_matcher = sbm;
  }

  // Reconstruct scene
  cv_stereo_matcher->compute(
      left_img_rectified, right_img_rectified, *disparity_img);

  // Optionally, post-filter disparity
  if (dense_stereo_params_.post_filter_disparity_) {
    // Use disparity post-filter
    // wls_filter = createDisparityWLSFilter(left_matcher);
    // Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
    // See
    // https://docs.opencv.org/3.3.1/d3/d14/tutorial_ximgproc_disparity_filtering.html#gsc.tab=0
  }

  // Optionally, smooth the disparity image
  if (dense_stereo_params_.median_blur_disparity_) {
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

void StereoMatcher::sparseStereoReconstruction(StereoFrame* stereo_frame) {
  CHECK_NOTNULL(stereo_frame);
  //! Undistort rectify left/right images
  // CHECK(!stereo_frame->isRectified());
  // TODO(marcus): LoopClosureDetector rewrites stereoframes that are already
  //   rectified using this function! That's why the above check doesn't work...
  if (stereo_frame->isRectified()) {
    VLOG(1) << "sparseStereoMatching: StereoFrame is already rectified!";
  }
  stereo_camera_->undistortRectifyStereoFrame(stereo_frame);
  CHECK(stereo_frame->isRectified());

  //! Undistort rectify left keypoints
  CHECK_GT(stereo_frame->left_frame_.keypoints_.size(), 0u)
      << "Call feature detection on left frame first...";
  stereo_camera_->undistortRectifyLeftKeypoints(
      stereo_frame->left_frame_.keypoints_,
      &stereo_frame->left_keypoints_rectified_);
  sparseStereoReconstruction(stereo_frame->getLeftImgRectified(),
                             stereo_frame->getRightImgRectified(),
                             stereo_frame->left_keypoints_rectified_,
                             &stereo_frame->right_keypoints_rectified_);

  //! Fill out keypoint depths
  getDepthFromRectifiedMatches(stereo_frame->left_keypoints_rectified_,
                               stereo_frame->right_keypoints_rectified_,
                               &stereo_frame->keypoints_depth_);

  //! Fill out right frame keypoints
  CHECK_GT(stereo_frame->right_keypoints_rectified_.size(), 0);
  stereo_camera_->distortUnrectifyRightKeypoints(
      stereo_frame->right_keypoints_rectified_,
      &stereo_frame->right_frame_.keypoints_);

  //! Fill out 3D keypoints in ref frame of left camera
  stereo_frame->keypoints_3d_.clear();
  stereo_frame->keypoints_3d_.reserve(
      stereo_frame->right_keypoints_rectified_.size());
  for (size_t i = 0; i < stereo_frame->right_keypoints_rectified_.size(); i++) {
    if (stereo_frame->right_keypoints_rectified_[i].first ==
        KeypointStatus::VALID) {
      // NOTE: versors are already in the rectified frame.
      Vector3 versor = stereo_frame->left_frame_.versors_[i];
      CHECK_GE(versor(2), 1e-3)
          << "sparseStereoMatching: found point with nonpositive depth!";
      // keypoints_depth_ is not the norm of the vector, it is the z component.
      stereo_frame->keypoints_3d_.push_back(
          versor * stereo_frame->keypoints_depth_[i] / versor(2));
    } else {
      stereo_frame->keypoints_3d_.push_back(Vector3::Zero());
    }
  }
}

void StereoMatcher::sparseStereoReconstruction(
    const cv::Mat& left_img_rectified,
    const cv::Mat& right_img_rectified,
    const StatusKeypointsCV& left_keypoints_rectified,
    StatusKeypointsCV* right_keypoints_rectified) {
  CHECK_NOTNULL(right_keypoints_rectified);
  CHECK(stereo_camera_);
  const auto& stereo_calib = stereo_camera_->getStereoCalib();
  CHECK(stereo_calib);
  const auto& baseline = stereo_calib->baseline();
  const auto& fx = stereo_calib->fx();
  getRightKeypointsRectified(left_img_rectified,
                             right_img_rectified,
                             left_keypoints_rectified,
                             fx,
                             baseline,
                             right_keypoints_rectified);
}

void StereoMatcher::getRightKeypointsRectified(
    const cv::Mat& left_img_rectified,
    const cv::Mat& right_img_rectified,
    const StatusKeypointsCV& left_keypoints_rectified,
    const double& fx,
    const double& baseline,
    StatusKeypointsCV* right_keypoints_rectified) const {
  CHECK_NOTNULL(right_keypoints_rectified)->clear();
  right_keypoints_rectified->reserve(left_keypoints_rectified.size());

  int verbosity = 0;  // Change back to 0

  // The stripe has to be placed in the right image, on the left-hand-side wrt
  // x of the left feature, since: disparity = left_px.x - right_px.x, hence
  // we check: right_px.x < left_px.x a stripe to select in the right image
  // (this must contain match as epipolar lines are horizontal)
  // must be odd; p/m stripe_extra_rows/2 pixels
  // to deal with rectification error
  int stripe_rows = stereo_matching_params_.templ_rows_ +
                    stereo_matching_params_.stripe_extra_rows_;

  // dimension of the search space in right camera is defined by min depth:
  // depth = fx * b / disparity => max disparity = fx * b / minDepth;
  int stripe_cols =
      std::round(fx * baseline / stereo_matching_params_.min_point_dist_) +
      stereo_matching_params_.templ_cols_ + 4;  // 4 is a tolerance

  if (stripe_cols % 2 != 1) {
    // make it odd, if it is not
    stripe_cols += 1;
  }

  if (stripe_cols > right_img_rectified.cols) {
    // if we exagerated with the stripe columns
    stripe_cols = right_img_rectified.cols;
  }

  // Serial version (could be parallelized).
  for (const StatusKeypointCV& left_keypoint_rectified :
       left_keypoints_rectified) {
    // If left point is invalid, set right point to be invalid and continue
    if (left_keypoint_rectified.first != KeypointStatus::VALID) {
      // Skip invalid points (fill in with placeholders in right)
      // Gtsam is able to deal with non-valid stereo matches.
      right_keypoints_rectified->push_back(
          std::make_pair(left_keypoint_rectified.first, KeypointCV(0.0, 0.0)));
      continue;
    }

    // Do left->right matching
    const KeypointCV& left_rectified_i = left_keypoint_rectified.second;
    StatusKeypointCV right_rectified_i_candidate;
    double matching_val_LR;
    searchRightKeypointEpipolar(left_img_rectified,
                                left_rectified_i,
                                right_img_rectified,
                                stripe_cols,
                                stripe_rows,
                                stereo_matching_params_,
                                &right_rectified_i_candidate,
                                &matching_val_LR);

    // TODO(Toni): Here we could perform bidirectional checking...

    right_keypoints_rectified->push_back(right_rectified_i_candidate);
  }

  if (verbosity > 0) {
    cv::Mat left_img_with_keypoints =
        UtilsOpenCV::DrawCircles(left_img_rectified, left_keypoints_rectified);
    cv::Mat right_img_with_keypoints = UtilsOpenCV::DrawCircles(
        right_img_rectified, *right_keypoints_rectified);
    UtilsOpenCV::showImagesSideBySide(left_img_with_keypoints,
                                      right_img_with_keypoints,
                                      "result_getRightKeypointsRectified",
                                      verbosity == 1,
                                      verbosity == 2);
  }
}

void StereoMatcher::searchRightKeypointEpipolar(
    const cv::Mat& left_img_rectified,
    const KeypointCV& left_keypoint_rectified,
    const cv::Mat& right_rectified,
    const int& stripe_cols,
    const int& stripe_rows,
    const StereoMatchingParams& stereo_matching_params,
    StatusKeypointCV* right_keypoint_rectified,
    double* score) const {
  CHECK_NOTNULL(right_keypoint_rectified);
  CHECK_NOTNULL(score);

  // Correlation matrix
  cv::Mat result;

  int rounded_left_rectified_i_x = round(left_keypoint_rectified.x);
  int rounded_left_rectified_i_y = round(left_keypoint_rectified.y);

  // CORRECTLY PLACE THE TEMPLATE (IN LEFT IMAGE)
  // y-component of upper left corner of template
  int temp_corner_y =
      rounded_left_rectified_i_y - (stereo_matching_params.templ_rows_ - 1) / 2;
  if (temp_corner_y < 0 || temp_corner_y + stereo_matching_params.templ_rows_ >
                               left_img_rectified.rows - 1) {
    // template exceeds bottom or top of the image
    // skip point too close to up or down boundary
    *score = -1.0;
    *right_keypoint_rectified =
        std::make_pair(KeypointStatus::NO_RIGHT_RECT, KeypointCV(0.0, 0.0));
    return;
  }
  // Compensate when the template falls off the image.
  int offset_temp = 0;
  int temp_corner_x =
      rounded_left_rectified_i_x - (stereo_matching_params.templ_cols_ - 1) / 2;
  // Template exceeds on the left-hand-side of the image.
  if (temp_corner_x < 0) {
    // offset_temp a bit to make the template inside the image.
    offset_temp = temp_corner_x;
    // Because of the offset_temp, the template corner ends up on the image
    // border
    temp_corner_x = 0;
  }
  // Template exceeds on the right-hand-side of the image
  if (temp_corner_x + stereo_matching_params.templ_cols_ >
      left_img_rectified.cols - 1) {
    LOG_IF(FATAL, offset_temp != 0)
        << "Offset_temp cannot exceed in both directions!";
    // Amount that exceeds
    offset_temp = (temp_corner_x + stereo_matching_params.templ_cols_) -
                  (left_img_rectified.cols - 1);
    // Corner has to be offset_temp to the left by the amount that exceeds
    temp_corner_x -= offset_temp;
  }

  // Create template
  cv::Rect templ_selector(temp_corner_x,
                          temp_corner_y,
                          stereo_matching_params.templ_cols_,
                          stereo_matching_params.templ_rows_);
  cv::Mat templ(left_img_rectified, templ_selector);

  // CORRECTLY PLACE THE STRIPE (IN RIGHT IMAGE)
  // y-component of upper left corner of stripe
  int stripe_corner_y = rounded_left_rectified_i_y - (stripe_rows - 1) / 2;
  if (stripe_corner_y < 0 ||
      stripe_corner_y + stripe_rows > right_rectified.rows - 1) {
    // stripe exceeds bottom or top of the image
    *score = -1.0;
    *right_keypoint_rectified =
        std::make_pair(KeypointStatus::NO_RIGHT_RECT, KeypointCV(0.0, 0.0));
    return;
  }

  // Compensate when the template falls off the image
  int offset_stripe = 0;
  // y-component of upper left corner of stripe
  int stripe_corner_x = rounded_left_rectified_i_x +
                        (stereo_matching_params.templ_cols_ - 1) / 2 -
                        stripe_cols;
  if (stripe_corner_x + stripe_cols > right_rectified.cols - 1) {
    // stripe exceeds on the right of image
    // amount that exceeds
    offset_stripe =
        (stripe_corner_x + stripe_cols) - (right_rectified.cols - 1);
    stripe_corner_x -= offset_stripe;
  }

  // Stripe exceeds on the left of the image
  // set to left-most column
  if (stripe_corner_x < 0) {
    stripe_corner_x = 0;
  }

  // Create stripe
  cv::Rect stripe_selector(
      stripe_corner_x, stripe_corner_y, stripe_cols, stripe_rows);
  cv::Mat stripe(right_rectified, stripe_selector);

  // Find template and normalize results
  double min_val;
  double max_val;
  cv::Point min_loc;
  cv::Point max_loc;

  cv::matchTemplate(stripe, templ, result, CV_TM_SQDIFF_NORMED);
  // Localizing the best match with minMaxLoc
  cv::minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc, cv::Mat());

  // normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() ); //
  // TODO:
  // do we need to normalize??

  // Position within the result matrix
  cv::Point matchLoc = min_loc;
  matchLoc.x += stripe_corner_x + (stereo_matching_params.templ_cols_ - 1) / 2 +
                offset_temp;
  // From result to image
  matchLoc.y += stripe_corner_y + (stereo_matching_params.templ_rows_ - 1) / 2;
  // Our desired pixel match
  KeypointCV match_px(matchLoc.x, matchLoc.y);

  // Refine keypoint with subpixel accuracy.
  if (stereo_matching_params.subpixel_refinement_) {
    // TODO(Toni): removed hardcoded!
    static const cv::TermCriteria criteria(
        CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
    static const cv::Size winSize(10, 10);
    static const cv::Size zeroZone(-1, -1);
    std::vector<cv::Point2f> corner = {match_px};
    cv::cornerSubPix(right_rectified, corner, winSize, zeroZone, criteria);
    match_px = corner[0];
  }

  *score = min_val;
  if (min_val < stereo_matching_params.tolerance_template_matching_) {
    // Valid point with small mismatch wrt template
    *right_keypoint_rectified = std::make_pair(KeypointStatus::VALID, match_px);
  } else {
    *right_keypoint_rectified =
        std::make_pair(KeypointStatus::NO_RIGHT_RECT, match_px);
  }
}

void StereoMatcher::getDepthFromRectifiedMatches(
    StatusKeypointsCV& left_keypoints_rectified,
    StatusKeypointsCV& right_keypoints_rectified,
    std::vector<double>* keypoints_depth) const {
  CHECK_NOTNULL(keypoints_depth)->clear();
  // depth = fx * baseline / disparity (should be fx = focal * sensorsize)
  double fx_b =
      stereo_camera_->getStereoCalib()->fx() * stereo_camera_->getBaseline();

  CHECK_EQ(left_keypoints_rectified.size(), right_keypoints_rectified.size())
      << "getDepthFromRectifiedMatches: size mismatch!";
  keypoints_depth->reserve(left_keypoints_rectified.size());

  int nrValidDepths = 0;
  // disparity = left_px.x - right_px.x, hence we check: right_px.x < left_px.x
  size_t i = 0;
  for (i = 0; i < left_keypoints_rectified.size(); i++) {
    if (left_keypoints_rectified[i].first == KeypointStatus::VALID &&
        right_keypoints_rectified[i].first == KeypointStatus::VALID) {
      KeypointCV left_px = left_keypoints_rectified[i].second;
      KeypointCV right_px = right_keypoints_rectified[i].second;
      double disparity = left_px.x - right_px.x;
      if (disparity >= 0.0) {
        // Valid.
        nrValidDepths += 1;
        double depth = fx_b / disparity;
        if (depth < stereo_matching_params_.min_point_dist_ ||
            depth > stereo_matching_params_.max_point_dist_) {
          right_keypoints_rectified[i].first = KeypointStatus::NO_DEPTH;
          keypoints_depth->push_back(0.0);
        } else {
          keypoints_depth->push_back(depth);
        }
      } else {
        // Right match was wrong.
        right_keypoints_rectified[i].first = KeypointStatus::NO_DEPTH;
        keypoints_depth->push_back(0.0);
      }
    } else {
      // Something is wrong.
      if (left_keypoints_rectified[i].first != KeypointStatus::VALID &&
          right_keypoints_rectified.at(i).first !=
              left_keypoints_rectified[i].first) {
        // We cannot have a valid right, without a valid left keypoint.
        LOG(WARNING)
            << "Cannot have a valid right kpt without also a valid left kpt!"
            << "\nLeft kpt status: "
            << to_underlying(left_keypoints_rectified[i].first)
            << "\nRight kpt status: "
            << to_underlying(right_keypoints_rectified.at(i).first);
        right_keypoints_rectified.at(i).first =
            left_keypoints_rectified[i].first;
      }
      keypoints_depth->push_back(0.0);
    }
  }
  CHECK_EQ(left_keypoints_rectified.size(), keypoints_depth->size())
      << "getDepthFromRectifiedMatches: depths size mismatch!";
}

}  // namespace VIO
