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
 * @author Antoni Rosinol
 * @author Luca Carlone
 * @author Marcus Abate
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
  inline void setIsRectified(bool is_rect) {
    is_rectified_ = is_rect;
  }
  void setRectifiedImages(const cv::Mat& left_rectified_img,
                          const cv::Mat& right_rectified_img);

  inline bool isKeyframe() const { return is_keyframe_; }
  inline bool isRectified() const { return is_rectified_; }

  //! Return rectified images, assumes the images have already been computed.
  //! Note that we return const images, since these should not be modified
  //! by the user.
  inline const cv::Mat getLeftImgRectified() const {
    CHECK(is_rectified_);
    return left_img_rectified_;
  }
  inline const cv::Mat getRightImgRectified() const {
    CHECK(is_rectified_);
    return right_img_rectified_;
  }

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
  static cv::Mat drawCornersMatches(const StereoFrame& stereo_frame_1,
                                    const StereoFrame& stereo_frame_2,
                                    const DMatchVec& matches,
                                    const bool& random_color);

  cv::Mat drawLeftRightCornersMatches(const DMatchVec& matches,
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
  // These must be private because they must always match with other members.
  // Can only be a kf if left and right frames are kf as well.
  // Can only be rectified if rectified images are filled.
  bool is_keyframe_;
  bool is_rectified_;

  //! Rectified undistorted images for sparse stereo epipolar matching
  //! If the flag is_rectified_ is not true,
  //! the images have not been computed yet.
  cv::Mat left_img_rectified_;
  cv::Mat right_img_rectified_;

 public:
  //! Unique id of this stereo frame (the user must ensure it is unique).
  const FrameId id_;
  //! Timestamp of this stereo frame.
  const Timestamp timestamp_;

  //! Original monocular left/right frames: these contain features as well.
  Frame left_frame_;
  Frame right_frame_;

  //! Left/right keypoints being tracked expressed as rectified/undistorted
  //! points with validity flag.
  StatusKeypointsCV left_keypoints_rectified_;
  StatusKeypointsCV right_keypoints_rectified_;

  //! Depths of keypoints
  // TODO(marcus): we got rid of this for a reason, find out what to replace it with...
  VIO::Depths keypoints_depth_;

  //! 3D positions of the stereo points as given by reprojection using stereo
  //! disparity; in the rectified left frame.
  std::vector<gtsam::Vector3> keypoints_3d_;
};

}  // namespace VIO
