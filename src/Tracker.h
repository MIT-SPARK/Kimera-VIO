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

#ifndef Tracker_H_
#define Tracker_H_

#include <boost/shared_ptr.hpp>     // used for opengv
#include <time.h>
#include "opencv2/opencv.hpp"
#include "Frame.h"
#include "StereoFrame.h"
#include <gtsam/geometry/StereoCamera.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "UtilsOpenCV.h"
#include <boost/filesystem.hpp> // to create folders

// OpenGV for Ransac
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/triangulation/methods.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/point_cloud/methods.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>

// implementation of feature selector, still within the tracker class
#include <gtsam/nonlinear/Marginals.h>
#include "FeatureSelector.h"

#define TRACKER_VERBOSITY 0 // Should be 1

namespace VIO {

// Mono
using ProblemMono = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem; // 5-point ransac
using AdapterMono = opengv::relative_pose::CentralRelativeAdapter;
// MonoTranslationOnly: TranslationOnlySacProblem
using ProblemMonoGivenRot = opengv::sac_problems::relative_pose::TranslationOnlySacProblem; // 2-point ransac
using AdapterMonoGivenRot = opengv::relative_pose::CentralRelativeAdapter;
// Stereo
using ProblemStereo = opengv::sac_problems::point_cloud::PointCloudSacProblem; // Arun's problem (3-point ransac)
using AdapterStereo = opengv::point_cloud::PointCloudAdapter;

///////////////////////////////////////////////////////////////////////////////////////
class DebugTrackerInfo {
public:
  // Info about feature detection, tracking and ransac.
  size_t nrDetectedFeatures_ = 0, nrTrackerFeatures_ = 0, nrMonoInliers_ = 0;
  size_t nrMonoPutatives_ = 0, nrStereoInliers_ = 0, nrStereoPutatives_ = 0;
  size_t monoRansacIters_ = 0, stereoRansacIters_ = 0;

  // Info about performance of sparse stereo matching (and ransac):
  // RPK = right keypoints
  size_t nrValidRKP_ = 0, nrNoLeftRectRKP_ = 0, nrNoRightRectRKP_ = 0;
  size_t nrNoDepthRKP_ = 0, nrFailedArunRKP_ = 0;

  // Info about timing.
  double featureDetectionTime_ = 0, featureTrackingTime_ = 0;
  double monoRansacTime_ = 0, stereoRansacTime_ = 0;

  // Info about feature selector.
  double featureSelectionTime_ = 0;
  size_t extracted_corners_ = 0, need_n_corners_ = 0;

  void printTimes() const {
    LOG(INFO) << "featureDetectionTime_: " << featureDetectionTime_ << " s\n"
              << "featureSelectionTime_: " << featureSelectionTime_ << " s\n"
              << "featureTrackingTime_: " << featureTrackingTime_ << " s\n"
              << "monoRansacTime_: " << monoRansacTime_ << " s\n"
              << "stereoRansacTime_: " << stereoRansacTime_ << " s";
  }
};

///////////////////////////////////////////////////////////////////////////////////////
class Tracker{
public:
  enum class TrackingStatus {
      VALID,
      LOW_DISPARITY,
      FEW_MATCHES,
      INVALID,
      DISABLED
  };

  // Constructor
  Tracker(const VioFrontEndParams& trackerParams = VioFrontEndParams(),
          const int saveImages = 1):
    trackerParams_(trackerParams),
    outputImagesPath_("./outputImages/"), // only for debugging and visualization
    landmark_count_(0),
    pixelOffset_(),
    saveImages_(saveImages),
    verbosity_(TRACKER_VERBOSITY) {}

  // Tracker parameters.
  const VioFrontEndParams trackerParams_;

  // This is not const as for debugging we want to redirect the image save path
  // where we like.
  std::string outputImagesPath_;

  // Mask for features.
  cv::Mat camMask_;

  // Counters.
  int landmark_count_; // incremental id assigned to new landmarks

  // Debug info.
  DebugTrackerInfo debugInfo_;

public:
  /* +++++++++++++++++++++ NONCONST FUNCTIONS +++++++++++++++++++++++++++++++ */
  void featureTracking(Frame& ref_frame,
                       Frame& cur_frame);
  void featureDetection(Frame& cur_frame);

  std::pair<Tracker::TrackingStatus, gtsam::Pose3>
  geometricOutlierRejectionMono(Frame& ref_frame,
                                Frame& cur_frame);

  std::pair<Tracker::TrackingStatus, gtsam::Pose3>
  geometricOutlierRejectionStereo(StereoFrame& ref_frame,
                                  StereoFrame& cur_frame);

  // Contrarily to the previous 2 this also returns a 3x3 covariance for the
  // translation estimate.
  std::pair<Tracker::TrackingStatus, gtsam::Pose3>
  geometricOutlierRejectionMonoGivenRotation(
      Frame& ref_frame,
      Frame& cur_frame,
      const gtsam::Rot3& R);

  std::pair< std::pair<Tracker::TrackingStatus,gtsam::Pose3> , gtsam::Matrix3 >
  geometricOutlierRejectionStereoGivenRotation(
      StereoFrame& ref_stereoFrame,
      StereoFrame& cur_stereoFrame,
      const gtsam::Rot3& R);

  void removeOutliersMono(
      Frame& ref_frame,
      Frame& cur_frame,
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
      const std::vector<Kstatus>& right_keypoints_status);

  /* ---------------------------- CONST FUNCTIONS ------------------------------------------- */
  // returns frame with markers
  cv::Mat displayFrame(
      const Frame& ref_frame,
      const Frame& cur_frame,
      int verbosity = 0,
      const KeypointsCV& extraCorners1 = KeypointsCV(),
      const KeypointsCV& extraCorners2 = KeypointsCV(),
      const std::string& extraString = "") const;

  /* ---------------------------- STATIC FUNCTIONS ------------------------------------------ */
  static void findOutliers(const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
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

  // returns landmark_count (updated from the new keypoints), and nr or extracted corners
  static std::pair<KeypointsCV, std::vector<double> >
  featureDetection(Frame& cur_frame,
                   const VioFrontEndParams& trackerParams,
                   const cv::Mat cam_mask, const int need_n_corners);

  static std::pair< Vector3, Matrix3 > getPoint3AndCovariance(
      const StereoFrame& stereoFrame,
      const gtsam::StereoCamera& stereoCam,
      const size_t pointId,
      const gtsam::Matrix3& stereoPtCov,
      boost::optional<gtsam::Matrix3> Rmat = boost::none);

private:
  // Pixel offset for using center of image
  cv::Point2f pixelOffset_;

  const int saveImages_; // 0: don't show, 1: show, 2: write & save

  // Flags
  const int verbosity_;

};

} // namespace VIO
#endif /* Tracker_H_ */
