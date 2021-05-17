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

#include <opengv/absolute_pose/methods.hpp>

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

  /**
   * @brief Tracker tracks features from frame to frame.
   * @param tracker_params Parameters for feature tracking
   * @param camera_params Parameters for the camera used for tracking.
   */
  Tracker(const FrontendParams& tracker_params,
          const Camera::ConstPtr& camera,
          DisplayQueue* display_queue = nullptr);

  // Tracker parameters.
  const FrontendParams tracker_params_;

  // Mask for features.
  cv::Mat cam_mask_;

 public:
  void featureTracking(Frame* ref_frame,
                       Frame* cur_frame,
                       const gtsam::Rot3& inter_frame_rotation,
                       boost::optional<cv::Mat> R = boost::none);

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

  bool pnp(const StereoFrame& cur_stereo_frame,
           const StereoCamera::ConstPtr& stereo_camera,
           const gtsam::Rot3& camLrectlkf_R_camLrectkf,
           const gtsam::Point3& camLrectlkf_t_camLrectkf,
           gtsam::Pose3* best_absolute_pose) {
    CHECK_NOTNULL(best_absolute_pose);

    //! Given 3D-2D correspondences, get cur_stereo_frame pose.
    // BearingVectors bearing_vectors =
    //     cur_stereo_frame.keypoints_3d_;  //! 3D bearing vectors
    // LandmarkCV lmks =
    //     cur_stereo_frame.keypoints_depth_ * bearing_vectors;  //! 2D
    //     keypoints

    const opengv::rotation_t& rotation_prior =
        camLrectlkf_R_camLrectkf.matrix();
    const opengv::translation_t& translation_prior =
        camLrectlkf_t_camLrectkf.matrix();

    // Not sure what are these...
    std::vector<int> indices2;

    opengv::bearingVectors_t bearing_vectors;
    opengv::points_t points;

    // Create the central adapter
    AdapterPnp adapter(bearing_vectors, points);

    static constexpr int threshold = 10;
    static constexpr int max_iterations = 10;

    opengv::transformation_t best_transformation;
    switch (pnp_method_) {
      case PnpMethod::KneipP2P: {
        // Uses rotation prior from adapter
        adapter.setR(rotation_prior);
        best_transformation = runPnpRansac(
            VIO::make_unique<
                opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>(
                adapter,
                opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::
                    TWOPT),
            threshold,
            max_iterations);
        break;
      }
      case PnpMethod::KneipP3P: {
        best_transformation = runPnpRansac(
            VIO::make_unique<
                opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>(
                adapter,
                opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::
                    KNEIP),
            threshold,
            max_iterations);
        break;
      }
      case PnpMethod::GaoP3P: {
        // Get the result
        best_transformation = runPnpRansac(
            VIO::make_unique<
                opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>(
                adapter,
                opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::
                    GAO),
            threshold,
            max_iterations);
        break;
      }
      case PnpMethod::EPNP: {
        // Uses all correspondences.
        best_transformation = runPnpRansac(
            VIO::make_unique<
                opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>(
                adapter,
                opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::
                    EPNP),
            threshold,
            max_iterations);
        break;
      }
      case PnpMethod::UPNP: {
        // Uses all correspondences.
        const opengv::transformations_t& upnp_transformations =
            opengv::absolute_pose::upnp(adapter);
        // TODO(TONI): what about the rest of transformations?
        CHECK_GT(upnp_transformations.size(), 0);
        best_transformation = upnp_transformations[0];
        break;
      }
      case PnpMethod::UP3P: {
        // Uses three correspondences.
        const opengv::transformations_t& upnp_transformations =
            opengv::absolute_pose::upnp(adapter, indices2);
        // TODO(TONI): what about the rest of transformations?
        CHECK_GT(upnp_transformations.size(), 0);
        best_transformation = upnp_transformations[0];
        break;
      }
      case PnpMethod::NonlinearOptimization: {
        // Uses all correspondences.
        adapter.sett(translation_prior);
        adapter.setR(rotation_prior);
        best_transformation =
            opengv::absolute_pose::optimize_nonlinear(adapter);
        break;
      }
      case PnpMethod::MLPNP: {
        // TODO(TONI): needs fork of opengv, can we make a static check and use
        // this iff we are having MLPNP support?
        LOG(FATAL) << "Not implemented...";
      }
      default: {
        LOG(ERROR) << "Unknown PnP method selected: "
                   << VIO::to_underlying(pnp_method_);
        break;
      }
    }

    *best_absolute_pose = Eigen::MatrixXd(best_transformation);

    return true;
  }

  opengv::transformation_t runPnpRansac(
      std::shared_ptr<
          opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
          absolute_pose_problem_ptr,
      const int& threshold,
      const int& max_iterations) {
    // Create a Ransac object
    opengv::sac::Ransac<
        opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
        pnp_ransac;

    // Setup pnp ransac
    pnp_ransac.sac_model_ = absolute_pose_problem_ptr;
    pnp_ransac.threshold_ = threshold;
    pnp_ransac.max_iterations_ = max_iterations;

    // Run pnp ransac
    pnp_ransac.computeModel();

    return pnp_ransac.model_coefficients_;
  }

  void removeOutliersMono(const std::vector<int>& inliers,
                          Frame* ref_frame,
                          Frame* cur_frame,
                          KeypointMatches* matches_ref_cur);

  void removeOutliersStereo(const std::vector<int>& inliers,
                            StereoFrame* ref_stereoFrame,
                            StereoFrame* cur_stereoFrame,
                            KeypointMatches* matches_ref_cur);

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

  PnpMethod pnp_method_;
};

}  // namespace VIO
