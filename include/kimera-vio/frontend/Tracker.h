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
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

// TODO(Toni): put tracker in another folder.
#pragma once

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/StereoCamera.h>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/OpticalFlowPredictor.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/frontend/VisionFrontEndParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class Tracker {
 public:
  KIMERA_POINTER_TYPEDEFS(Tracker);
  KIMERA_DELETE_COPY_CONSTRUCTORS(Tracker);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Tracker tracks features from frame to frame.
   * @param tracker_params Parameters for feature tracking
   * @param camera_params Parameters for the camera used for tracking.
   */
  Tracker(const VisionFrontEndParams& tracker_params,
          const CameraParams& camera_params);

  // Tracker parameters.
  const VisionFrontEndParams tracker_params_;

  // Mask for features.
  cv::Mat cam_mask_;

 public:
  void featureTracking(Frame* ref_frame,
                       Frame* cur_frame,
                       const gtsam::Rot3& inter_frame_rotation);

  // Get good features to track from image (wrapper for opencv
  // goodFeaturesToTrack)
  static void findGoodFeaturesToTrack(
      const cv::Mat& image,
      const int& max_corners,
      const double& quality_level,
      const double& min_distance,  // Not const because modified dkwhy inside...
      const cv::Mat& mask,
      const int& blockSize,
      const bool& useHarrisDetector,
      const double& harrisK,
      KeypointsWithScores* corners_with_scores);

  void featureDetection(Frame* cur_frame);

  // TODO(Toni): this function is almost a replica of the Stereo version,
  // factorize.
  std::pair<TrackingStatus, gtsam::Pose3> geometricOutlierRejectionMono(
      Frame* ref_frame,
      Frame* cur_frame);

  // TODO(Toni): this function is almost a replica of the Mono version,
  // factorize.
  std::pair<TrackingStatus, gtsam::Pose3> geometricOutlierRejectionStereo(
      StereoFrame& ref_frame,
      StereoFrame& cur_frame);

  // Contrarily to the previous 2 this also returns a 3x3 covariance for the
  // translation estimate.
  std::pair<TrackingStatus, gtsam::Pose3>
  geometricOutlierRejectionMonoGivenRotation(Frame* ref_frame,
                                             Frame* cur_frame,
                                             const gtsam::Rot3& R);

  std::pair<std::pair<TrackingStatus, gtsam::Pose3>, gtsam::Matrix3>
  geometricOutlierRejectionStereoGivenRotation(StereoFrame& ref_stereoFrame,
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
  cv::Mat getTrackerImage(
      const Frame& ref_frame,
      const Frame& cur_frame,
      const KeypointsCV& extra_corners_gray = KeypointsCV(),
      const KeypointsCV& extra_corners_blue = KeypointsCV()) const;

  /* ---------------------------- STATIC FUNCTIONS -------------------------- */
  static void findOutliers(
      const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
      std::vector<int> inliers,
      std::vector<int>* outliers);

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
  KeypointsWithScores featureDetection(const Frame& cur_frame,
                                       const cv::Mat& cam_mask,
                                       const int& need_n_corners);

  static std::pair<Vector3, Matrix3> getPoint3AndCovariance(
      const StereoFrame& stereoFrame,
      const gtsam::StereoCamera& stereoCam,
      const size_t pointId,
      const gtsam::Matrix3& stereoPtCov,
      boost::optional<gtsam::Matrix3> Rmat = boost::none);

  // Get tracker info
  inline DebugTrackerInfo getTrackerDebugInfo() { return debug_info_; }

 private:
  // Camera params for the camera used to track: currently we only use K if the
  // rotational optical flow predictor is used
  const CameraParams camera_params_;

  // Feature tracking uses the optical flow predictor to have a better guess of
  // where the features moved from frame to frame.
  OpticalFlowPredictor::UniquePtr optical_flow_predictor_;

  // Debug info.
  DebugTrackerInfo debug_info_;

  // This is not const as for debugging we want to redirect the image save path
  // where we like.
  std::string output_images_path_;

  // Monocular RANSACs
  opengv::sac::Ransac<ProblemMono> mono_ransac_;
  opengv::sac::Ransac<ProblemMonoGivenRot> mono_ransac_given_rot_;

  // Stereo RANSAC
  opengv::sac::Ransac<ProblemStereo> stereo_ransac_;
};

}  // namespace VIO
