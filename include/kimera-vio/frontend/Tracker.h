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

#include <optional>

#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/frontend/VisionImuTrackerParams.h"
#include "kimera-vio/frontend/optical-flow/OpticalFlowPredictor.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Display-definitions.h"

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
   * @param Camera Camera used for tracking
   */
  Tracker(const TrackerParams& tracker_params,
          const Camera::ConstPtr& camera,
          DisplayQueue* display_queue = nullptr);

  virtual ~Tracker() = default;

  // Tracker parameters.
  const TrackerParams tracker_params_;

  // Mask for features.
  cv::Mat cam_mask_;

 public:
  void featureTracking(Frame* ref_frame,
                       Frame* cur_frame,
                       const gtsam::Rot3& inter_frame_rotation,
                       const FeatureDetectorParams& feature_detector_params,
                       std::optional<cv::Mat> R = std::nullopt);

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

  // gtsam::pose3 has to be rectified cameras bcs bearings are in rectified cams
  virtual TrackingStatusPose geometricOutlierRejection2d2d(
      Frame* ref_frame,
      Frame* cur_frame,
      const gtsam::Pose3& cam_lkf_Pose_cam_kf = gtsam::Pose3());

  virtual TrackingStatusPose geometricOutlierRejection2d2d(
      const BearingVectors& ref_bearings,
      const BearingVectors& cur_bearings,
      const KeypointMatches& matches_ref_cur,
      std::vector<int>* inliers,
      const gtsam::Pose3& cam_lkf_Pose_cam_kf = gtsam::Pose3());

  /**
   * @brief geometricOutlierRejection3d3d
   * @param ref_stereo_frame
   * @param cur_stereo_frame
   * @param cam_lkf_Pose_cam_kf inital guess of the transformation between 3d
   * pointclouds (this is only used in case of nonlinear optimization).
   * @return
   */
  TrackingStatusPose geometricOutlierRejection3d3d(
      StereoFrame* ref_stereo_frame,
      StereoFrame* cur_stereo_frame,
      const gtsam::Pose3& cam_lkf_Pose_cam_kf = gtsam::Pose3());

  // TODO(marcus); doc differences between two versions, esp no outlier removal
  TrackingStatusPose geometricOutlierRejection3d3d(
      const Landmarks& ref_keypoints_3d,
      const Landmarks& cur_keypoints_3d,
      const KeypointMatches& matches_ref_cur,
      std::vector<int>* inliers,
      const gtsam::Pose3& cam_lkf_Pose_cam_kf = gtsam::Pose3());

  std::pair<TrackingStatusPose, gtsam::Matrix3>
  geometricOutlierRejection3d3dGivenRotation(
      StereoFrame& ref_stereo_frame,
      StereoFrame& cur_stereo_frame,
      const gtsam::StereoCamera& stereo_cam,
      const gtsam::Rot3& camLrectlkf_R_camLrectkf);

  std::pair<TrackingStatusPose, gtsam::Matrix3>
  geometricOutlierRejection3d3dGivenRotation(
      const StatusKeypointsCV& ref_keypoints_status_left,
      const StatusKeypointsCV& ref_keypoints_status_right,
      const StatusKeypointsCV& cur_keypoints_status_left,
      const StatusKeypointsCV& cur_keypoints_status_right,
      const Landmarks& ref_keypoints_3d,
      const Landmarks& cur_keypoints_3d,
      const gtsam::StereoCamera& stereo_cam,
      const KeypointMatches& matches_ref_cur,
      const gtsam::Rot3& camLrectlkf_R_camLrectkf,
      std::vector<int>* inliers);

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
   * @param [in] cam_bearing_vectors Bearing vectors in camera frame.
   * @param [in] F_points 3D landmarks in the generic frame F.
   * @param [out] F_Pose_cam_estimate Output of pnp ransac.
   * @param[in/out] inliers
   * @param [in] F_Pose_cam_prior Optional prior pose of the camera with respect to
   * reference frame F.
   */
  bool pnp(const BearingVectors& cam_bearing_vectors,
           const Landmarks& F_points,
           gtsam::Pose3* F_Pose_cam_estimate,
           std::vector<int>* inliers,
           gtsam::Pose3* F_Pose_cam_prior = nullptr);

  /**
   * @brief pnp Absolute Pose estimation from 2D-3D correspondences.
   * @param cur_stereo_frame
   * @param W_Pose_cam_prior
   * @param W_Pose_cam_estimate
   * @param[in/out] inliers
   */
  bool pnp(const StereoFrame& cur_stereo_frame,
           gtsam::Pose3* W_Pose_cam_estimate,
           std::vector<int>* inliers,
           gtsam::Pose3* W_Pose_cam_prior = nullptr);

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

  // TODO(Toni): remove
  static std::pair<gtsam::Vector3, gtsam::Matrix3> getPoint3AndCovariance(
      const StereoFrame& stereoFrame,
      const gtsam::StereoCamera& stereoCam,
      const size_t pointId,
      const gtsam::Matrix3& stereoPtCov,
      std::optional<gtsam::Matrix3> Rmat = std::nullopt);

  static std::pair<Vector3, Matrix3> getPoint3AndCovariance(
      const StatusKeypointsCV& keypoints_undistorted_left,
      const StatusKeypointsCV& keypoints_undistorted_right,
      const BearingVectors& keypoints_3d,
      const gtsam::StereoCamera& stereo_cam,
      const size_t point_id,
      const gtsam::Matrix3& stereo_point_covariance,
      std::optional<gtsam::Matrix3> Rmat);

 private:
  /**
   * @brief runRansac
   * @param [in] sample_consensus_problem_ptr
   * @param [in] threshold Quality threshold to stop
   * @param [in] max_iterations
   * @param [out] The best pose estimate
   * @param [out] inliers The inliers from the input data
   * @return true on success, false on failure.
   */
  template <class SampleConsensusProblem>
  bool runRansac(
      std::shared_ptr<SampleConsensusProblem> sample_consensus_problem_ptr,
      const double& threshold,
      const int& max_iterations,
      const double& probability,
      const bool& do_nonlinear_optimization,
      gtsam::Pose3* best_pose,
      std::vector<int>* inliers) {
    CHECK_NOTNULL(best_pose);
    CHECK(sample_consensus_problem_ptr);

    //! Create ransac
    opengv::sac::Ransac<SampleConsensusProblem> ransac(
        max_iterations, threshold, probability);

    //! Setup ransac
    ransac.sac_model_ = sample_consensus_problem_ptr;

    //! Run ransac
    bool success = ransac.computeModel(VLOG_IS_ON(10) ? 1 : 0);

    if (success) {
      if (ransac.iterations_ >= max_iterations && ransac.inliers_.empty()) {
        success = false;
        *best_pose = gtsam::Pose3();
        if (inliers) *inliers = {};
      } else {
        *best_pose =
            UtilsOpenCV::openGvTfToGtsamPose3(ransac.model_coefficients_);
        if (inliers) *inliers = ransac.inliers_;

        if (do_nonlinear_optimization) {
          CHECK(inliers);
          opengv::transformation_t optimized_pose;
          sample_consensus_problem_ptr->optimizeModelCoefficients(
              *inliers, ransac.model_coefficients_, optimized_pose);
          *best_pose = Eigen::MatrixXd(optimized_pose);
        }
      }
    } else {
      CHECK(ransac.inliers_.empty());
      *best_pose = gtsam::Pose3();
      if (inliers) *inliers = {};
    }

    VLOG(5) << printRansacStats(
        ransac, sample_consensus_problem_ptr->indices_->size(), "todo");
    return success;
  }

  // Printers
  template <class T>
  std::string printRansacStats(const opengv::sac::Ransac<T>& ransac,
                               const size_t& total_correspondences,
                               const std::string& ransac_type) const {
    std::stringstream out;
    out << "RANSAC (" << ransac_type << "): \n"
        << "- #iter = " << ransac.iterations_ << '\n'
        << "- #inliers = " << ransac.inliers_.size() << '\n'
        << "- #outliers = " << total_correspondences - ransac.inliers_.size()
        << '\n'
        << "Total = " << total_correspondences;
    return out.str();
  }

 public:
  //! Debug info (its public to allow stereo frames to populate it).
  DebugTrackerInfo debug_info_;

 private:
  // Incremental id assigned to new landmarks.
  LandmarkId landmark_count_;

  // Camera object for the camera we are tracking. For stereo, use left.
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
  opengv::sac::Ransac<Problem2d2d> mono_ransac_;
  opengv::sac::Ransac<Problem2d2dGivenRot> mono_ransac_given_rot_;

  // Stereo RANSAC
  opengv::sac::Ransac<Problem3d3d> stereo_ransac_;

  // Pnp RANSAC
  opengv::sac::Ransac<ProblemPnP> pnp_ransac_;

  mutable std::mutex landmarks_map_mtx_;
  //! Most up-to-date map of landmarks with optimized 3D poses from backend.
  //! WARNING: do not use this without locking first its mutex.
  LandmarksMap landmarks_map_;
};

}  // namespace VIO
