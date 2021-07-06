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

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/StereoCamera.h>

#include <opencv2/opencv.hpp>

#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/frontend/VisionImuFrontendParams.h"
#include "kimera-vio/frontend/optical-flow/OpticalFlowPredictor.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"

namespace VIO {

// TODO(Toni): Fast-forwarding bcs of an issue wiht includes:
// if you include here the display-definitions.h, a million errors appear, this
// should go away after properly cleaning what each file includes.
class DisplayInputBase;
using DisplayQueue = ThreadsafeQueue<std::unique_ptr<DisplayInputBase>>;

class Tracker {
 public:
  KIMERA_POINTER_TYPEDEFS(Tracker);
  KIMERA_DELETE_COPY_CONSTRUCTORS(Tracker);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  /**
   * @brief Tracker tracks features from frame to frame.
   * @param tracker_params Parameters for feature tracking
   * @param camera_params Parameters for the camera used for tracking.
   */
  Tracker(const FrontendParams& tracker_params,
          const Camera::ConstPtr& camera,
          DisplayQueue* display_queue = nullptr);

  virtual ~Tracker() = default;

  // Tracker parameters.
  const FrontendParams tracker_params_;

  // Mask for features.
  cv::Mat cam_mask_;

 public:
  void featureTracking(Frame* ref_frame,
                       Frame* cur_frame,
                       const gtsam::Rot3& inter_frame_rotation,
                       boost::optional<cv::Mat> R = boost::none);

  /**
   * @brief updateMap Updates the map of landmarks in the time horizon of
   * the backend. This is thread-safe to allow for asap updates from the
   * backend.
   * @param lmks_map New map of landmarks with optimized landmarks 3D positions.
   */
  inline void updateMap(const LandmarksMap& lmks_map) {
    std::lock_guard<std::mutex> lock(landmarks_map_mtx_);
    // std::stringstream out;
    // out << std::setw(6) << "Lmk Id"
    //     << " | " << std::setw(20) << "Lmk 3D" << '\n';
    // for (const auto& it : lmks_map) {
    //   out << std::setw(6) << it.first << ": " << std::setw(20) <<
    //   it.second[0]
    //       << ", " << it.second[1] << ", " << it.second[2] << '\n';
    // }
    // LOG(ERROR) << "Updating Backend Map:\n " << out.str();
    landmarks_map_ = lmks_map;
  }

  // TODO(Toni): this function is almost a replica of the Stereo version,
  // factorize.
  virtual std::pair<TrackingStatus, gtsam::Pose3> geometricOutlierRejectionMono(
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
  geometricOutlierRejectionMonoGivenRotation(
      Frame* ref_frame,
      Frame* cur_frame,
      const gtsam::Rot3& camLrectlkf_R_camLrectkf);

  std::pair<std::pair<TrackingStatus, gtsam::Pose3>, gtsam::Matrix3>
  geometricOutlierRejectionStereoGivenRotation(
      StereoFrame& ref_stereoFrame,
      StereoFrame& cur_stereoFrame,
      VIO::StereoCamera::ConstPtr stereo_camera,
      const gtsam::Rot3& camLrectlkf_R_camLrectkf);

  void removeOutliersMono(const std::vector<int>& inliers,
                          Frame* ref_frame,
                          Frame* cur_frame,
                          KeypointMatches* matches_ref_cur);

  void removeOutliersStereo(const std::vector<int>& inliers,
                            StereoFrame* ref_stereoFrame,
                            StereoFrame* cur_stereoFrame,
                            KeypointMatches* matches_ref_cur);

  /**
   * @brief pnp Absolute Pose estimation from 2D-3D correspondences.
   * @param cur_stereo_frame
   * @param w_Pose_cam
   * @param best_absolute_pose
   * @param[in/out] inliers
   */
  bool pnp(const StereoFrame& cur_stereo_frame,
           gtsam::Pose3* best_absolute_pose,
           std::vector<int>* inliers,
           gtsam::Pose3* w_Pose_cam = nullptr);

  /* ---------------------------- CONST FUNCTIONS --------------------------- */
  // returns frame with markers
  cv::Mat getTrackerImage(
      const Frame& ref_frame,
      const Frame& cur_frame,
      const KeypointsCV& extra_corners_gray = KeypointsCV(),
      const KeypointsCV& extra_corners_blue = KeypointsCV()) const;

  /* ---------------------------- STATIC FUNCTIONS -------------------------- */
  static void findOutliers(const KeypointMatches& matches_ref_cur,
                           std::vector<int> inliers,
                           std::vector<int>* outliers);

  static void findMatchingKeypoints(const Frame& ref_frame,
                                    const Frame& cur_frame,
                                    KeypointMatches* matches_ref_cur);

  static void findMatchingStereoKeypoints(
      const StereoFrame& ref_stereoFrame,
      const StereoFrame& cur_stereoFrame,
      KeypointMatches* matches_ref_cur_stereo);

  static void findMatchingStereoKeypoints(
      const StereoFrame& ref_stereoFrame,
      const StereoFrame& cur_stereoFrame,
      const KeypointMatches& matches_ref_cur_mono,
      KeypointMatches* matches_ref_cur_stereo);

  static bool computeMedianDisparity(const KeypointsCV& ref_frame_kpts,
                                     const KeypointsCV& cur_frame_kpts,
                                     const KeypointMatches& matches_ref_cur,
                                     double* median_disparity);

  static std::pair<Vector3, Matrix3> getPoint3AndCovariance(
      const StereoFrame& stereoFrame,
      const gtsam::StereoCamera& stereoCam,
      const size_t pointId,
      const gtsam::Matrix3& stereoPtCov,
      boost::optional<gtsam::Matrix3> Rmat = boost::none);

 private:
  /**
   * @brief runPnpRansac
   * @param absolute_pose_problem_ptr
   * @param threshold Quality threshold to stop
   * @param max_iterations
   * @param [out] The best pose estimate
   * @param inliers The inliers from the input data
   * @return true on success, false on failure.
   */
  bool runPnpRansac(
      std::shared_ptr<
          opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
          absolute_pose_problem_ptr,
      const double& threshold,
      const int& max_iterations,
      const double& probability,
      gtsam::Pose3* best_pose,
      std::vector<int>* inliers);

 public:
  //! Debug info (its public to allow stereo frames to populate it).
  DebugTrackerInfo debug_info_;

 private:
  // Incremental id assigned to new landmarks.
  LandmarkId landmark_count_;

  // StereoCamera object for the camera we are tracking. We use the left camera.
  // TODO(marcus): this should be general to all camera types!
  Camera::ConstPtr camera_;

  // Feature tracking uses the optical flow predictor to have a better guess of
  // where the features moved from frame to frame.
  OpticalFlowPredictor::UniquePtr optical_flow_predictor_;

  // Display queue: push to this queue if you want to display an image.
  DisplayQueue* display_queue_;

  // This is not const as for debugging we want to redirect the image save path
  // where we like.
  std::string output_images_path_;

  // Monocular RANSACs
  opengv::sac::Ransac<ProblemMono> mono_ransac_;
  opengv::sac::Ransac<ProblemMonoGivenRot> mono_ransac_given_rot_;

  // Stereo RANSAC
  opengv::sac::Ransac<ProblemStereo> stereo_ransac_;

  // Pnp RANSAC
  opengv::sac::Ransac<ProblemPnp> pnp_ransac_;

  mutable std::mutex landmarks_map_mtx_;
  //! Most up-to-date map of landmarks with optimized 3D poses from backend.
  //! WARNING: do not use this without locking first its mutex.
  LandmarksMap landmarks_map_;
};

}  // namespace VIO
