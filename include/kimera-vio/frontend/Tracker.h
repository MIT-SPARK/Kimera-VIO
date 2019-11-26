/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Tracker.h
 * @brief  Class describing temporal tracking
 * @author Antoni Rosinol, Luca Carlone
 */

// TODO(Toni): put tracker in another folder.

#pragma once

#include <time.h>

#include <boost/shared_ptr.hpp> // used for opengv
#include <boost/filesystem.hpp> // to create folders

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <gtsam/geometry/StereoCamera.h>

#include "kimera-vio/frontend/FeatureSelector.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

// implementation of feature selector, still within the tracker class
#include <gtsam/nonlinear/Marginals.h>

namespace VIO {

////////////////////////////////////////////////////////////////////////////////
class Tracker {
public:
  // Constructor
 Tracker(const VioFrontEndParams& trackerParams = VioFrontEndParams());

 // Tracker parameters.
 const VioFrontEndParams trackerParams_;

 // This is not const as for debugging we want to redirect the image save path
 // where we like.
 std::string outputImagesPath_;

 // Mask for features.
 cv::Mat camMask_;

 // Counters.
 int landmark_count_;  // incremental id assigned to new landmarks

 // Debug info.
 DebugTrackerInfo debugInfo_;

public:
  /* +++++++++++++++++++++ NONCONST FUNCTIONS +++++++++++++++++++++++++++++++ */
  void featureTracking(Frame* ref_frame,
                       Frame* cur_frame);
  void featureDetection(Frame* cur_frame);

  std::pair<TrackingStatus, gtsam::Pose3>
  geometricOutlierRejectionMono(Frame* ref_frame,
                                Frame* cur_frame);

  std::pair<TrackingStatus, gtsam::Pose3>
  geometricOutlierRejectionStereo(StereoFrame& ref_frame,
                                  StereoFrame& cur_frame);

  // Contrarily to the previous 2 this also returns a 3x3 covariance for the
  // translation estimate.
  std::pair<TrackingStatus, gtsam::Pose3>
  geometricOutlierRejectionMonoGivenRotation(
      Frame* ref_frame,
      Frame* cur_frame,
      const gtsam::Rot3& R);

  std::pair< std::pair<TrackingStatus,gtsam::Pose3> , gtsam::Matrix3 >
  geometricOutlierRejectionStereoGivenRotation(
      StereoFrame& ref_stereoFrame,
      StereoFrame& cur_stereoFrame,
      const gtsam::Rot3& R);

  void removeOutliersMono(
      Frame* ref_frame,
      Frame* cur_frame,
      const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
      const std::vector<int>& inliers,
      const int iterations);

  void removeOutliersStereo(
      StereoFrame& ref_stereoFrame,
      StereoFrame& cur_stereoFrame,
      const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
      const std::vector<int>& inliers,
      const int iterations);

  void checkStatusRightKeypoints(
      const std::vector<KeypointStatus>& right_keypoints_status);

  /* ---------------------------- CONST FUNCTIONS --------------------------- */
  // returns frame with markers
  cv::Mat displayFrame(
      const Frame& ref_frame,
      const Frame& cur_frame,
      bool write_frame = false,
      const std::string& img_title = "",
      const KeypointsCV& extra_corners_gray = KeypointsCV(),
      const KeypointsCV& extra_corners_blue = KeypointsCV()) const;

  /* ---------------------------- STATIC FUNCTIONS -------------------------- */
  static void findOutliers(
      const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
      std::vector<int> inliers,
      std::vector<int> *outliers);

  static void findMatchingKeypoints(
      const Frame& ref_frame,
      const Frame& cur_frame,
      std::vector<std::pair<size_t, size_t>>* matches_ref_cur);

  static void findMatchingStereoKeypoints(
      const StereoFrame& ref_stereoFrame,
      const StereoFrame& cur_stereoFrame,
      std::vector<std::pair<size_t, size_t>>* matches_ref_cur_stereo);

  static void findMatchingStereoKeypoints(
      const StereoFrame& ref_stereoFrame,
      const StereoFrame& cur_stereoFrame,
      const std::vector<std::pair<size_t, size_t>>& matches_ref_cur_mono,
      std::vector<std::pair<size_t, size_t>>* matches_ref_cur_stereo);

  static double computeMedianDisparity(const Frame& ref_frame,
                                       const Frame& cur_frame);

  // Returns landmark_count (updated from the new keypoints),
  // and nr or extracted corners.
  static std::pair<KeypointsCV, std::vector<double>>
  featureDetection(const Frame& cur_frame,
                   const VioFrontEndParams& trackerParams,
                   const cv::Mat& cam_mask,
                   const int need_n_corners);

  static std::pair< Vector3, Matrix3 > getPoint3AndCovariance(
      const StereoFrame& stereoFrame,
      const gtsam::StereoCamera& stereoCam,
      const size_t pointId,
      const gtsam::Matrix3& stereoPtCov,
      boost::optional<gtsam::Matrix3> Rmat = boost::none);

  // Get tracker info
  inline DebugTrackerInfo getTrackerDebugInfo() { return debugInfo_; }

 private:
  // Pixel offset for using center of image
  cv::Point2f pixelOffset_;

  // Flags
  const int verbosity_;
};

}  // namespace VIO
