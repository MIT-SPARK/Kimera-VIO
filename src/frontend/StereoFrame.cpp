/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Frame.h
 * @brief  Class describing a pair of stereo images
 * @author Antoni Rosinol
 * @author Luca Carlone
 * @author Marcus Abate
 */

#include "kimera-vio/frontend/StereoFrame.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/core/core.hpp>

namespace VIO {

StereoFrame::StereoFrame(const FrameId& id,
                         const Timestamp& timestamp,
                         const Frame& left_frame,
                         const Frame& right_frame)
    : is_keyframe_(false),
      is_rectified_(false),
      left_img_rectified_(),
      right_img_rectified_(),
      id_(id),
      timestamp_(timestamp),
      // TODO(Toni): remove these copies
      left_frame_(left_frame),
      right_frame_(right_frame),
      left_keypoints_rectified_(),
      right_keypoints_rectified_(),
      keypoints_depth_(),
      keypoints_3d_() {
  CHECK_EQ(id_, left_frame_.id_);
  CHECK_EQ(id_, right_frame_.id_);
  CHECK_EQ(timestamp_, left_frame_.timestamp_);
  CHECK_EQ(timestamp_, right_frame_.timestamp_);
}

void StereoFrame::setRectifiedImages(const cv::Mat& left_rectified_img,
                                     const cv::Mat& right_rectified_img) {
  left_img_rectified_ = left_rectified_img;
  right_img_rectified_ = right_rectified_img;
  is_rectified_ = true;
}

void StereoFrame::checkStereoFrame() const {
  const size_t nrLeftKeypoints = left_frame_.keypoints_.size();
  CHECK_EQ(left_frame_.scores_.size(), nrLeftKeypoints)
      << "checkStereoFrame: left_frame_.scores.size()";
  CHECK_EQ(right_frame_.keypoints_.size(), nrLeftKeypoints)
      << "checkStereoFrame: right_frame_.keypoints_.size()";
  CHECK_EQ(keypoints_3d_.size(), nrLeftKeypoints)
      << "checkStereoFrame: keypoints_3d_.size()";
  CHECK_EQ(left_keypoints_rectified_.size(), nrLeftKeypoints)
      << "checkStereoFrame: left_keypoints_rectified_.size()";
  CHECK_EQ(right_keypoints_rectified_.size(), nrLeftKeypoints)
      << "checkStereoFrame: right_keypoints_rectified_.size()";

  for (size_t i = 0u; i < nrLeftKeypoints; i++) {
    if (right_keypoints_rectified_[i].first == KeypointStatus::VALID) {
      CHECK_LE(fabs(right_keypoints_rectified_[i].second.y -
                    left_keypoints_rectified_[i].second.y),
               3)
          << "checkStereoFrame: rectified keypoints have different y "
          << right_keypoints_rectified_[i].second.y << " vs. "
          << left_keypoints_rectified_[i].second.y;
    }

    if (right_keypoints_rectified_[i].first == KeypointStatus::VALID) {
      CHECK_NE(fabs(right_frame_.keypoints_[i].x) +
                   fabs(right_frame_.keypoints_[i].y),
               0)
          << "checkStereoFrame: right_frame_.keypoints_[i] is zero.";
      // Also: cannot have zero depth.
      CHECK_GT(keypoints_3d_[i](2), 0)
          << "checkStereoFrame: keypoints_3d_[i] has nonpositive "
             "for valid point: "
          << keypoints_3d_[i](2) << '\n'
          << "left_frame_.keypoints_[i] " << left_frame_.keypoints_[i] << '\n'
          << "right_frame_.keypoints_[i] " << right_frame_.keypoints_[i] << '\n'
          << '\n'
          << "right_keypoints_rectified_[i] st "
          << to_underlying(right_keypoints_rectified_[i].first)
          << '\n'
          << "right_keypoints_rectified_[i] kp "
          << right_keypoints_rectified_[i].second;
    } else {
      CHECK_LE(keypoints_3d_[i](2), 0)
          << "checkStereoFrame: keypoints_3d_[i] has positive "
             "for nonvalid point: "
          << keypoints_3d_[i](2);
    }
  }
}

void StereoFrame::checkStatusRightKeypoints(
    DebugTrackerInfo* debug_info) const {
  CHECK_NOTNULL(debug_info);
  debug_info->nrValidRKP_ = 0;
  debug_info->nrNoLeftRectRKP_ = 0;
  debug_info->nrNoRightRectRKP_ = 0;
  debug_info->nrNoDepthRKP_ = 0;
  debug_info->nrFailedArunRKP_ = 0;
  for (const StatusKeypointCV& right_keypoint : right_keypoints_rectified_) {
    KeypointStatus right_keypoint_status = right_keypoint.first;
    switch (right_keypoint_status) {
      case KeypointStatus::VALID: {
        debug_info->nrValidRKP_++;
        break;
      }
      case KeypointStatus::NO_LEFT_RECT: {
        debug_info->nrNoLeftRectRKP_++;
        break;
      }
      case KeypointStatus::NO_RIGHT_RECT: {
        debug_info->nrNoRightRectRKP_++;
        break;
      }
      case KeypointStatus::NO_DEPTH: {
        debug_info->nrNoDepthRKP_++;
        break;
      }
      case KeypointStatus::FAILED_ARUN: {
        debug_info->nrFailedArunRKP_++;
        break;
      }
      default: {
        LOG(FATAL) << "Unknown keypoint status.";
        break;
      }
    }
  }
}

void StereoFrame::print() const {
  LOG(INFO) << "=====================\n"
            << "id_: " << id_ << '\n'
            << "timestamp_: " << timestamp_ << '\n'
            << "isKeyframe_: " << is_keyframe_ << '\n'
            << "nr keypoints in left: " << left_frame_.keypoints_.size() << '\n'
            << "nr keypoints in right: " << right_frame_.keypoints_.size()
            << '\n'
            << "nr keypoints_3d_: " << keypoints_3d_.size() << '\n'
            << "left_frame_.cam_param_.body_Pose_cam_: "
            << left_frame_.cam_param_.body_Pose_cam_ << '\n'
            << "right_frame_.cam_param_.body_Pose_cam_: "
            << right_frame_.cam_param_.body_Pose_cam_;
}

cv::Mat StereoFrame::drawCornersMatches(
    const StereoFrame& stereo_frame_1,
    const StereoFrame& stereo_frame_2,
    const DMatchVec& matches,
    const bool& random_color) {
  return UtilsOpenCV::DrawCornersMatches(
      stereo_frame_1.left_img_rectified_,
      stereo_frame_1.left_keypoints_rectified_,
      stereo_frame_2.left_img_rectified_,
      stereo_frame_2.left_keypoints_rectified_,
      matches,
      random_color);
}

cv::Mat StereoFrame::drawLeftRightCornersMatches(
    const DMatchVec& matches,
    const bool& random_color) const {
  return UtilsOpenCV::DrawCornersMatches(left_img_rectified_,
                                         left_keypoints_rectified_,
                                         right_img_rectified_,
                                         right_keypoints_rectified_,
                                         matches,
                                         random_color);
}

// TODO(marcus): should we get rid of both these functions?
// They should be in UtilsOpenCV...
void StereoFrame::showOriginal(const int verbosity) const {
  showImagesSideBySide(
      left_frame_.img_, right_frame_.img_, "original: left-right", verbosity);
}

void StereoFrame::showRectified(const bool& visualize,
                                const bool& write) const {
  CHECK(is_rectified_);
  cv::Mat canvas_undistorted_rectified =
      drawEpipolarLines(left_img_rectified_, right_img_rectified_, 15);
  if (write) {
    std::string img_name =
        "./outputImages/rectified_" + std::to_string(id_) + ".png";
    cv::imwrite(img_name, canvas_undistorted_rectified);
  }
  if (visualize) {
    cv::imshow("Rectified!", canvas_undistorted_rectified);
    cv::waitKey(1);
  }
}

void StereoFrame::showImagesSideBySide(const cv::Mat imL,
                                       const cv::Mat imR,
                                       const std::string& title,
                                       const int& verbosity) const {
  if (verbosity == 0) return;

  cv::Mat originalLR = UtilsOpenCV::concatenateTwoImages(imL, imR);
  if (verbosity == 1) {
    cv::namedWindow(
        title, cv::WINDOW_AUTOSIZE);  // moved in here to allow saving images
    cv::imshow("originalLR", originalLR);
    cv::waitKey(1);
  } else if (verbosity == 2) {
    std::string img_name =
        "./outputImages/" + title + std::to_string(id_) + ".png";
    cv::imwrite(img_name, originalLR);
  }
}

cv::Mat StereoFrame::drawEpipolarLines(const cv::Mat img1,
                                       const cv::Mat img2,
                                       const int& num_lines,
                                       const bool& write) const {
  cv::Mat canvas = UtilsOpenCV::concatenateTwoImages(img1, img2);
  int line_gap = canvas.rows / (num_lines + 1);
  for (int l = 0; l < num_lines; l++) {
    float y_pos = (l + 1) * line_gap;
    cv::line(canvas,
             cv::Point2f(0.0f, y_pos),
             cv::Point2f(canvas.cols - 1, y_pos),
             cv::Scalar(0, 255, 0));
  }

  if (write) {
    std::string img_name =
        "./outputImages/drawEpipolarLines_" + std::to_string(id_) + ".png";
    cv::imwrite(img_name, canvas);
  }

  return canvas;
}

void StereoFrame::showLeftRightMatches() const {
  CHECK_EQ(left_frame_.keypoints_.size(), right_frame_.keypoints_.size())
      << "displayLeftRightMatches: error -  nr of corners in left and right "
         "cameras must be the same";

  // Draw the matchings: assumes that keypoints in the left and right keyframe
  // are ordered in the same way
  DMatchVec matches;
  for (size_t i = 0; i < left_frame_.keypoints_.size(); i++) {
    matches.push_back(cv::DMatch(i, i, 0));
  }
  cv::Mat match_vis = UtilsOpenCV::DrawCornersMatches(left_frame_.img_,
                                                      left_frame_.keypoints_,
                                                      right_frame_.img_,
                                                      right_frame_.keypoints_,
                                                      matches);
  cv::imshow("match_visualization", match_vis);
  cv::waitKey(1);
}

void StereoFrame::printKeypointStats(
    const StatusKeypointsCV& right_keypoints_rectified) const {
  size_t n_valid = 0u;
  size_t n_no_left_rect = 0u;
  size_t n_no_right_rect = 0u;
  size_t n_no_depth = 0u;
  size_t n_failed_arun_rkp = 0u;
  for (const StatusKeypointCV& right_keypoint_rectified :
       right_keypoints_rectified) {
    switch (right_keypoint_rectified.first) {
      case KeypointStatus::VALID: {
        n_valid++;
        break;
      }
      case KeypointStatus::NO_LEFT_RECT: {
        n_no_left_rect++;
        break;
      }
      case KeypointStatus::NO_RIGHT_RECT: {
        n_no_right_rect++;
        break;
      }
      case KeypointStatus::NO_DEPTH: {
        n_no_depth++;
        break;
      }
      case KeypointStatus::FAILED_ARUN: {
        n_failed_arun_rkp++;
        break;
      }
    }
  }
  LOG(INFO) << "Nr of right keypoints: " << right_keypoints_rectified.size()
            << " of which:\n"
            << "nrValid: " << n_valid << '\n'
            << "nrNoLeftRect: " << n_no_left_rect << '\n'
            << "nrNoRightRect: " << n_no_right_rect << '\n'
            << "nrNoDepth: " << n_no_depth << '\n'
            << "nrFailedArunRKP: " << n_failed_arun_rkp;
}

}  // namespace VIO
