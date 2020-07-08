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

namespace VIO {

class StereoFrame {
 public:
  // TODO(Toni) Do it pls...
  // KIMERA_DELETE_COPY_CONSTRUCTORS(StereoFrame);
  KIMERA_POINTER_TYPEDEFS(StereoFrame);
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StereoFrame(const FrameId& id,
              const Timestamp& timestamp,
              const Frame& left_frame,
              const Frame& right_frame,
              const StereoMatchingParams& stereo_matching_params);

  void setRectifiedImages(const cv::Mat& left_rectified_img,
                          const cv::Mat& right_rectified_img);

 public:
  // TODO these guys are flying around the code, as they are publicly accessible
  // by anyone... To make this self-contained and thread-safe, there should be
  // getters for each one of these, but this has the caveat of making copies
  // everytime a getter is called. Better would be to add them to a
  // Output queue.
  //! in the ref frame of the UNRECTIFIED left frame

 public:
  inline void setIsKeyframe(bool is_kf);


  void checkStereoFrame() const;

  std::vector<double> getDepthFromRectifiedMatches(
      StatusKeypointsCV& left_keypoints_rectified,
      StatusKeypointsCV& right_keypoints_rectified,
      const double& fx,
      const double& baseline,
      const StereoMatchingParams& stereo_matching_params) const;

 public:
  /// Getters
  // Thread-safe.
  inline FrameId getFrameId() const { return id_; }
  inline Timestamp getTimestamp() const { return timestamp_; }

  // NOT THREAD-SAFE, needs critical section.
  inline bool isKeyframe() const { return is_keyframe_; }
  inline void isRectified() const {
    return left_img_rectified_ && right_img_rectified_;
  }

  // NON-THREAD SAFE.
  inline const Frame& getLeftFrame() const { return left_frame_; }
  inline const Frame& getRightFrame() const { return right_frame_; }
  // NON-THREAD SAFE, and potentially very hazardous, giving away rights to
  // modify class members is EVIL.
  inline Frame* getLeftFrameMutable() { return &left_frame_; }
  inline Frame* getRightFrameMutable() { return &right_frame_; }
  // NON-THREAD SAFE, Get rectified images
  //! Return rectified images, assumes the images have already been computed.
  //! Note that we return const images, since these should not be modified
  //! by the user.
  inline const cv::Mat& getLeftImgRectified() const {
    CHECK(left_img_rectified_);
    return *left_img_rectified_;
  }
  inline const cv::Mat& getRightImgRectified() const {
    CHECK(right_img_rectified_);
    return *right_img_rectified_;
  }

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

  //! Rectified undistorted images for sparse stereo epipolar matching
  //! If the pointers are nullptr, the images have not been computed yet.
  std::unique_ptr<cv::Mat> left_img_rectified_;
  std::unique_ptr<cv::Mat> right_img_rectified_;

  //! Left/right keypoints being tracked expressed as rectified/undistorted
  KeypointsCV left_keypoints_rectified_;
  KeypointsCV right_keypoints_rectified_;

  //! 3D positions of the stereo points as given by reprojection using stereo
  //! disparity
  std::vector<gtsam::Vector3> keypoints_3d_;

  //! Validity status of the right keypoint detection
  std::vector<KeypointStatus> right_keypoints_status_;

 private:
  //! Visualization functions
  void showOriginal(const int verbosity) const;
  void showRectified(const bool& visualize,
                     const bool& write) const;
  void showImagesSideBySide(const cv::Mat imL,
                            const cv::Mat imR,
                            const std::string& title,
                            const int& verbosity = 0) const;
  cv::Mat drawEpipolarLines(const cv::Mat img1,
                            const cv::Mat img2,
                            const int& numLines = 20,
                            const int& verbosity = 0) const;
  void showLeftRightMatches() const;

  //! Printers
  void printKeypointStats(
      const StatusKeypointsCV& right_keypoints_rectified) const;
  void print() const;


};

}  // namespace VIO
