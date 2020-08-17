/* -----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Frame.h
 * @brief  Class describing a pair of stereo images
 * @author Antoni Rosinol, Luca Carlone
 */

#pragma once

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/StereoPoint2.h>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame-definitions.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/utils/UtilsGeometry.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace VIO {

class StereoFrame {
 public:
  // TODO(Toni) Do it pls...
  // KIMERA_DELETE_COPY_CONSTRUCTORS(StereoFrame);
  KIMERA_POINTER_TYPEDEFS(StereoFrame);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StereoFrame(const FrameId& id,
              const Timestamp& timestamp,
              const Frame& left_frame,
              const Frame& right_frame);
  ~StereoFrame() = default;

 public:
  /// Setters
  inline void setIsKeyframe(bool is_kf) {
    is_keyframe_ = is_kf;
    left_frame_.isKeyframe_ = is_kf;
    right_frame_.isKeyframe_ = is_kf;
  }
  void setRectifiedImages(const cv::Mat& left_rectified_img,
                          const cv::Mat& right_rectified_img);

  /// Getters
  inline FrameId getFrameId() const { return id_; }
  inline Timestamp getTimestamp() const { return timestamp_; }

  inline bool isKeyframe() const { return is_keyframe_; }
  inline bool isRectified() const { return is_rectified_; }

  inline const Frame& getLeftFrame() const { return left_frame_; }
  inline const Frame& getRightFrame() const { return right_frame_; }

  inline const StatusKeypointsCV& getLeftKptsRectified() const {
    CHECK(is_rectified_);
    return left_keypoints_rectified_;
  }
  inline const StatusKeypointsCV& getRightKptsRectified() const {
    CHECK(is_rectified_);
    return right_keypoints_rectified_;
  }
  inline const std::vector<gtsam::Vector3>& get3DKpts() const {
    return keypoints_3d_;
  }

  // NON-THREAD SAFE, and potentially very hazardous, giving away rights to
  // modify class members is EVIL.
  inline Frame* getLeftFrameMutable() { return &left_frame_; }
  inline Frame* getRightFrameMutable() { return &right_frame_; }
  inline StatusKeypointsCV* getLeftKptsRectifiedMutable() {
    CHECK(is_rectified_);
    return &left_keypoints_rectified_;
  }
  inline StatusKeypointsCV* getRightKptsRectifiedMutable() {
    CHECK(is_rectified_);
    return &right_keypoints_rectified_;
  }
  inline std::vector<gtsam::Vector3>* get3DKptsMutable() {
    return &keypoints_3d_;
  }

  //! Return rectified images, assumes the images have already been computed.
  //! Note that we return const images, since these should not be modified
  //! by the user.
  // TODO(marcus): why not return const reference?
  inline const cv::Mat getLeftImgRectified() const {
    CHECK(is_rectified_);
    return left_img_rectified_;
  }
  inline const cv::Mat getRightImgRectified() const {
    CHECK(is_rectified_);
    return right_img_rectified_;
  }

  std::vector<double> getDepthFromRectifiedMatches(
      StatusKeypointsCV& left_keypoints_rectified,
      StatusKeypointsCV& right_keypoints_rectified,
      const double& fx,
      const double& baseline,
      const StereoMatchingParams& stereo_matching_params) const;

  void getSmartStereoMeasurements(StereoMeasurements* smart_stereo_measurements,
                                  const bool& use_stereo_measurements) const;

  /// Checkers
  void checkStereoFrame() const;
  /**
   * @brief checkStatusRightKeypoints Fill debug tracker info for logging with
   * keypoint status statistics
   * @param[out] debug_info Debug tracker info structure to update with stats.
   */
  void checkStatusRightKeypoints(DebugTrackerInfo* debug_info) const;

  /// Drawing functions
  /**
   * @brief drawCornersMatches
   * (friend function to avoid granting full access on the
   * stereo frame members to everyone).
   * @param stereo_frame_1
   * @param stereo_frame_2
   * @param matches
   * @param random_color
   * @return
   */
  static cv::Mat drawCornersMatches(const StereoFrame::Ptr& stereo_frame_1,
                                    const StereoFrame::Ptr& stereo_frame_2,
                                    const std::vector<cv::DMatch>& matches,
                                    const bool& random_color);

  cv::Mat drawLeftRightCornersMatches(const std::vector<cv::DMatch>& matches,
                                      const bool& random_color) const;

 private:
  //! Visualization functions
  void showOriginal(const int verbosity) const;
  void showRectified(const bool& visualize, const bool& write) const;
  void showImagesSideBySide(const cv::Mat imL,
                            const cv::Mat imR,
                            const std::string& title,
                            const int& verbosity = 0) const;
  cv::Mat drawEpipolarLines(const cv::Mat img1,
                            const cv::Mat img2,
                            const int& numLines = 20,
                            const bool& verbosity = false) const;
  void showLeftRightMatches() const;

  //! Printers
  void printKeypointStats(
      const StatusKeypointsCV& right_keypoints_rectified) const;
  void print() const;

 private:
  //! Unique id of this stereo frame (the user must ensure it is unique).
  const FrameId id_;
  //! Timestamp of this stereo frame.
  const Timestamp timestamp_;

  //! Original monocular left/right frames: these contain features as well.
  Frame left_frame_;
  Frame right_frame_;

  //! If this stereo frame is a keyframe
  bool is_keyframe_;

  bool is_rectified_;
  //! Rectified undistorted images for sparse stereo epipolar matching
  //! If the flag is_rectified_ is not true,
  //! the images have not been computed yet.
  cv::Mat left_img_rectified_;
  cv::Mat right_img_rectified_;

  //! Left/right keypoints being tracked expressed as rectified/undistorted
  //! points with validity flag.
  StatusKeypointsCV left_keypoints_rectified_;
  StatusKeypointsCV right_keypoints_rectified_;

  //! 3D positions of the stereo points as given by reprojection using stereo
  //! disparity
  std::vector<gtsam::Vector3> keypoints_3d_;

  /// Friends with access rights to the StereoFrame inner members.
  //! StereoMatcher computes the right keypoints for the left keypoints using
  //! the rectified left/right frames and left keypoints.
  friend class StereoMatcher;
};

}  // namespace VIO
