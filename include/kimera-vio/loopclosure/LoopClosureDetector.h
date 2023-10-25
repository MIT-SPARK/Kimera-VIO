/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoopClosureDetector.h
 * @brief  Pipeline for detection and reporting of Loop Closures between frames
 * @author Marcus Abate
 * @author Luca Carlone
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <limits>
#include <memory>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "kimera-vio/frontend/RgbdCamera.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/loopclosure/FrameCache.h"
#include "kimera-vio/loopclosure/LcdOutputPacket.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"
#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"

/* ------------------------------------------------------------------------ */
// Forward declare KimeraRPGO, a private dependency.
namespace KimeraRPGO {
class RobustSolver;
}

namespace DBoW2 {
class BowVector;
class FORB;

template <class D, class F>
class TemplatedVocabulary;

template <class D, class F>
class TemplatedDatabase;

}  // namespace DBoW2

typedef DBoW2::TemplatedVocabulary<cv::Mat, DBoW2::FORB> OrbVocabulary;
typedef DBoW2::TemplatedDatabase<cv::Mat, DBoW2::FORB> OrbDatabase;

namespace VIO {

class LcdThirdPartyWrapper;  // forward declare to avoid DBoW2 header

// quick wrapper class to get around forward declaration of OrbVocabulary
struct PreloadedVocab {
  KIMERA_DELETE_COPY_CONSTRUCTORS(PreloadedVocab);
  using Ptr = std::unique_ptr<PreloadedVocab>;

  PreloadedVocab();
  PreloadedVocab(PreloadedVocab&& other);
  ~PreloadedVocab();

  std::unique_ptr<OrbVocabulary> vocab;
};

/* ------------------------------------------------------------------------ */
class LoopClosureDetector {
 public:
  KIMERA_POINTER_TYPEDEFS(LoopClosureDetector);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LoopClosureDetector);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using IsBackendQueueFilledCallback = std::function<bool()>;

  /* ------------------------------------------------------------------------ */
  /** @brief Constructor: detects loop-closures and updates internal PGO.
   * @param[in] lcd_params Parameters for the instance of LoopClosureDetector.
   * @param[in] log_output Output-logging flag. If set to true, the logger is
   *  instantiated and output/statistics are logged at every spinOnce().
   */
  LoopClosureDetector(
      const LoopClosureDetectorParams& lcd_params,
      const CameraParams& tracker_cam_params,
      const gtsam::Pose3& B_Pose_Cam,
      const std::optional<VIO::StereoCamera::ConstPtr>& stereo_camera =
          std::nullopt,
      const std::optional<StereoMatchingParams>& stereo_matching_params =
          std::nullopt,
      const std::optional<VIO::RgbdCamera::ConstPtr>& rgbd_camera =
          std::nullopt,
      bool log_output = false,
      PreloadedVocab::Ptr&& preloaded_vocab = nullptr);

  /* ------------------------------------------------------------------------ */
  virtual ~LoopClosureDetector();

  /* ------------------------------------------------------------------------ */
  /** @brief Processed a single input payload and runs it through the pipeline.
   * @param[in] input A shared_ptr referencing an input payload.
   * @return The output payload from the pipeline.
   */
  virtual LcdOutput::UniquePtr spinOnce(const LcdInput& input);

  /* ------------------------------------------------------------------------ */
  /** @brief Register callback for checking the size of the input queue. Knowing
   * this can help determine when to optimize the factor graph and when to wait
   * for additional inputs to be added first.
   * @param[in] cb A callback function.
   */
  inline void registerIsBackendQueueFilledCallback(
      const IsBackendQueueFilledCallback& cb) {
    is_backend_queue_filled_cb_ = cb;
  }

  /* ------------------------------------------------------------------------ */
  /** @brief Processed a single frame and adds it to relevant internal
   * databases. Also generates associated bearing vectors for PnP.
   * @param[in] frame A Frame object with one images, landmarks, and a pose to
   * the body frame at a minimum. Other fields may also be populated.
   * @param[in] points_with_ids A PointsWithIdMap object obtained from the
   * backend's output. Expressed in world frame.
   * @param[in] W_Pose_Blkf A gtsam::Pose3 representing the VIO estimate of
   * the pose of the body wrt to world frame at the given keframe.
   * @return The local ID of the frame after it is added to the databases.
   */
  FrameId processAndAddMonoFrame(const Frame& frame,
                                 const PointsWithIdMap& W_points_with_ids,
                                 const gtsam::Pose3& W_Pose_Blkf);

  /* ------------------------------------------------------------------------ */
  /** @brief Processed a single rgbd-frame and adds it to relevant internal
   * databases.
   * @param[in] rgbd_frame A RgbdFrame object with two images and a pose to
   * the body frame at a minimum. Other fields may also be populated.
   * @return The local ID of the frame after it is added to the databases.
   */
  FrameId processAndAddRgbdFrame(const RgbdFrame& rgbd_frame);

  /* ------------------------------------------------------------------------ */
  /** @brief Processed a single stereo-frame and adds it to relevant internal
   * databases.
   * @param[in] stereo_frame A StereoFrame object with two images and a pose to
   * the body frame at a minimum. Other fields may also be populated.
   * @return The local ID of the frame after it is added to the databases.
   */
  FrameId processAndAddStereoFrame(const StereoFrame& stereo_frame);

  /* ------------------------------------------------------------------------ */
  /** @brief Find a loop closure given a frame id
   * @param[in] frame_id A FrameId representing the ID of the latest LCDFrame
   *                     added to the database.
   * @param[out] result A pointer to the LoopResult that is filled with the
   *                    result of the loop-closure detection stage.
   */
  void detectLoopById(const FrameId& frame_id, LoopResult* result);

  /* ------------------------------------------------------------------------ */
  /** @brief Runs all checks on a frame and determines whether it a loop-closure
   *         with a previous frame or not. Fills the LoopResult with this
   *         information.
   * @param[in] frame_id A FrameId representing the ID of the latest LCDFrame
   * added to the database, which will be used to detect loop-closures.
   * @param[in] bow_vec A descriptor vector for the latest LCDFrame to be scored
   *                    and added to the database.
   * @param[out] result A pointer to the LoopResult that is filled with the
   *                    result of the loop-closure detection stage.
   */
  void detectLoop(const FrameId& frame_id,
                  const DBoW2::BowVector& bow_vec,
                  LoopResult* result);

  /* ------------------------------------------------------------------------ */
  /** @brief Perform geometric verification and pose recovery for a match
   *  @param Input and ouput loop result (uses match and query id)
   */
  void verifyAndRecoverPose(LoopResult* result);

  /* ------------------------------------------------------------------------ */
  /** @brief Verify that the geometry between two frames is close enough to be
      considered a match, and generate a monocular transformation between them.
   * @param[in] ref_id The frame ID of the match image in the databse.
   * @param[in] cur_id The frame ID of the query image in the database.
   * @param[out] camMatch_T_camQuery_2d The pose between the match frame and
   * the query frame, in the coordinates of the match frame.
   * @param[out] inliers The IDs of inliers in the keypoint matches.
   * @return True if the verification check passes, false otherwise.
   */
  bool geometricVerificationCam2d2d(const LCDFrame& ref_frame,
                                    const LCDFrame& cur_frame,
                                    const KeypointMatches& matches_query_match,
                                    gtsam::Pose3* camMatch_T_camQuery_2d,
                                    std::vector<int>* inliers);

  /* ------------------------------------------------------------------------ */
  /** @brief Determine the 3D pose betwen two frames.
   * @param[in] ref_id The frame ID of the match image in the database.
   * @param[in] cur_id The frame ID of the query image in the database.
   * @param[in] camMatch_T_camQuery_2d The relative pose between the match
   * frame
   *  and the query frame, in the coordinates of the match frame.
   * @param[out] bodyMatch_T_bodyQuery_3d The 3D pose between the match
   * frame
   *  and the query frame, in the coordinates of the match frame.
   * @param[out] inliers The inliers to use from the keypoint matches,
   * determined at the geometricVerificationCam2d2d stage.
   * @return True if the pose is recovered successfully, false otherwise.
   */
  bool recoverPoseBody(const LCDFrame& ref_frame,
                       const LCDFrame& cur_frame,
                       const gtsam::Pose3& camMatch_T_camQuery_2d,
                       const KeypointMatches& matches_query_match,
                       gtsam::Pose3* bodyMatch_T_bodyQuery_3d,
                       std::vector<int>* inliers);

  /* ------------------------------------------------------------------------ */
  /**
   * @brief Register a loop closure between two frames in a threadsafe manner
   * @param[in] query_id most recent frame
   * @param[in] match_id previous frame that was matched to query
   */
  LoopResult registerFrames(FrameId query_id, FrameId match_id);

 public:
  /* ------------------------------------------------------------------------ */
  /** @brief Returns the RAW pointer to the BoW database.
   * @return A pointer to the BoW database.
   *
   * WARNING: This is a potentially dangerous method to use because it requires
   *  a manual deletion of the pointer before it goes out of scope.
   */
  inline const OrbDatabase* getBoWDatabase() const { return db_BoW_.get(); }

  /**
   * @brief Get cache of LCD keyframes.
   * @return The FrameCache containing keyframe information.
   */
  const FrameCache& getFrameCache() const { return cache_; }

  /* ------------------------------------------------------------------------ */
  /** @brief Returns the pose between the inertial world-reference frame and the
   *  "map" frame, which is the error between the VIO and the PGO trajectories.
   * @return The pose of the map frame relative to the world frame.
   */
  const gtsam::Pose3 getWPoseMap() const;

  /* ------------------------------------------------------------------------ */
  /** @brief Returns the pose between the optimized world reference frame (map)
   * and the VIO world reference frame (odom).
   * @return The pose of the odom frame relative to the map frame.
   */
  const gtsam::Pose3 getMapPoseOdom() const;

  /* ------------------------------------------------------------------------ */
  /** @brief Returns the values of the PGO, which is the full trajectory of the
   *  PGO.
   * @return The gtsam::Values (poses) of the PGO.
   */
  const gtsam::Values getPGOTrajectory() const;

  /* ------------------------------------------------------------------------ */
  /** @brief Returns the Nonlinear-Factor-Graph from the PGO.
   * @return The gtsam::NonlinearFactorGraph of the optimized trajectory from
   *  the PGO.
   */
  const gtsam::NonlinearFactorGraph getPGOnfg() const;

  /* ------------------------------------------------------------------------ */
  /** @brief Set the OrbDatabase internal member.
   * @param[in] db An OrbDatabase object.
   */
  void setDatabase(const OrbDatabase& db);

  /* @brief Set the vocabulary of the BoW detector.
   * @param[in] voc An OrbVocabulary object.
   */
  void setVocabulary(const OrbVocabulary& voc);

  /* ------------------------------------------------------------------------ */
  /* @brief Prints parameters and other statistics on the LoopClosureDetector.
   */
  void print() const;

  /* ------------------------------------------------------------------------ */
  /** @brief Clears all keypoints and features from an input StereoFrame and
   *  fills it with ORB features.
   * @param[in] keypoints A vector of KeyPoints representing the ORB keypoints
   *  identified by an ORB detector.
   * @param[out] A StereoFrame initially filled with front-end features,
   *  which is then replaced with ORB features from the keypoints parameter.
   */
  // TODO(marcus): utils and reorder (or just static)
  void rewriteStereoFrameFeatures(const std::vector<cv::KeyPoint>& keypoints,
                                  StereoFrame* stereo_frame) const;

  /* ------------------------------------------------------------------------ */
  /** @brief Gives the transform between two frames in the body frame given
   *  that same transform in the camera frame.
   * @param[in] camMatch_T_camQuery The relative pose between two frames in the
   *  camera coordinate frame.
   * @param[out] bodyMatch_T_bodyQuery The relative pose between two frames in
   * the
   *  body coordinate frame.
   */
  // TODO(marcus): these should be private or util
  void transformCameraPoseToBodyPose(const gtsam::Pose3& camMatch_T_camQuery,
                                     gtsam::Pose3* bodyMatch_T_bodyQuery) const;

  /* ------------------------------------------------------------------------ */
  /** @brief The inverse of transformCameraPoseToBodyPose.
   * @param[in] bodyMatch_T_bodyQuery The relative pose between two frames in
   * the
   *  body coordinate frame.
   * @param[out] camMatch_T_camQuery The relative pose between two frames in the
   *  camera coordinate frame.
   * @return
   */
  void transformBodyPoseToCameraPose(const gtsam::Pose3& bodyMatch_T_bodyQuery,
                                     gtsam::Pose3* camMatch_T_camQuery) const;

  /* ------------------------------------------------------------------------ */
  /** @brief Adds an odometry factor to the PGO and optimizes the trajectory.
   *  No actual optimization is performed on the RPGO side for odometry.
   * @param[in] factor An OdometryFactor representing the Backend's guess for
   *  odometry between two consecutive keyframes.
   */
  void addOdometryFactorAndOptimize(const OdometryFactor& factor);

  /* ------------------------------------------------------------------------ */
  /** @brief Adds a loop-closure factor to the PGO and optimizes the trajectory.
   * @param[in] factor A LoopClosureFactor representing the relative pose
   *  between two frames that are not (necessarily) consecutive.
   */
  void addLoopClosureFactorAndOptimize(const LoopClosureFactor& factor);

  /* ------------------------------------------------------------------------ */
  /** @brief Initializes the RobustSolver member with an initial prior factor,
   *  which can be the first OdometryFactor given by the Backend.
   * @param[in] factor An OdometryFactor representing the pose between the
   *  initial state of the vehicle and the first keyframe.
   */
  void initializePGO(const OdometryFactor& factor);

  /* ------------------------------------------------------------------------ */
  /** @brief Computes the indices of keypoints that match between two frames.
   * @param[in] ref_descriptors The descriptors from the query frame.
   * @param[in] cur_descriptors The descriptors from the match frame.
   * @param[out] matches_match_query Map of matching keypoint indices between
   * match frame and query frame.
   * @param[in] cut_matches If true, Lowe's Ratio Test will be used to cut
   *  out bad matches before sending output.
   */
  void computeDescriptorMatches(const OrbDescriptor& ref_descriptors,
                                const OrbDescriptor& cur_descriptors,
                                KeypointMatches* matches_match_query,
                                bool cut_matches = false) const;

 private:
  /* ------------------------------------------------------------------------ */
  /** @brief Detect features in frame for use with BoW and return keypoints and
   * descriptors. Currently only ORB features are supported.
   * @param[in] img Image from which to get features and descriptors.
   * @param[out] keypoints The ORB keypoints that are detected in the image.
   * @param[out] descriptors_mat The descriptors associated with the ORB
   * keypoints in a matrix form.
   * @param[out] descriptors_vec The descriptors vectorized.
   */
  void getNewFeaturesAndDescriptors(const cv::Mat& img,
                                    std::vector<cv::KeyPoint>* keypoints,
                                    OrbDescriptor* descriptors_mat);

  /* ------------------------------------------------------------------------ */
  /** @brief Convert an ORB descriptor from matrix form to vector form for
   * use with BoW.
   * @param[in] descriptors_mat An OrbDescriptor matrix with input descriptors
   * @param[out] descriptors_vec The descriptors in vectorize format
   */
  void descriptorMatToVec(const OrbDescriptor& descriptors_mat,
                          OrbDescriptorVec* descriptors_vec);

  /* ------------------------------------------------------------------------ */
  /** @brief Refine relative pose given by ransac using smart factors.
   * @param[in] ref_id The frame ID of the match image in the database.
   * @param[in] cur_id The frame ID of the query image in the database.
   * @param[in] camMatch_T_camQuery_3d The relative pose between the match
   * frame
   *  and the query frame, in the coordinates of the match frame.
   * @param[in] inlier correspondences (from ransac) in the query frame
   * @param[in] inlier correspondences (from ransac) in the match frame
   * @return refined relative pose
   */
  gtsam::Pose3 refinePoses(const StereoLCDFrame& ref_frame,
                           const StereoLCDFrame& cur_frame,
                           const gtsam::Pose3& camMatch_T_camQuery_3d,
                           const KeypointMatches& matches_query_match);

 private:
  enum class LcdState {
    Bootstrap,  //! Lcd is initializing
    Nominal     //! Lcd is running in nominal mode
  };
  LcdState lcd_state_ = LcdState::Bootstrap;

  // Parameter members
  LoopClosureDetectorParams lcd_params_;
  const bool log_output_ = false;

  // TODO(Toni): we should be using the FeatureDetector/Description class...
  // ORB extraction and matching members
  cv::Ptr<cv::ORB> orb_feature_detector_;
  cv::Ptr<cv::DescriptorMatcher> orb_feature_matcher_;

  // TODO(marcus): want to move outlier-rejection to its own file
  // Tracker for outlier rejection
  Tracker::UniquePtr tracker_;

  // BoW database
  std::unique_ptr<OrbDatabase> db_BoW_;
  FrameCache cache_;
  FrameIDTimestampMap timestamp_map_;

  // Store latest computed objects for temporal matching and nss scoring
  std::unique_ptr<LcdThirdPartyWrapper> lcd_tp_wrapper_;
  std::unique_ptr<DBoW2::BowVector> latest_bowvec_;

  // Store camera parameters and StereoFrame stuff once
  gtsam::Pose3 B_Pose_Cam_;
  StereoCamera::ConstPtr stereo_camera_;
  StereoMatcher::UniquePtr stereo_matcher_;

  // Rgbd specific camera
  RgbdCamera::ConstPtr rgbd_camera_;

  // Robust PGO members
  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_;
  std::pair<gtsam::Symbol, gtsam::Pose3> W_Pose_B_kf_vio_;
  gtsam::SharedNoiseModel shared_noise_model_;

  // Queue-checking callback
  IsBackendQueueFilledCallback is_backend_queue_filled_cb_;
  int num_lc_unoptimized_;

  // Logging members
  std::unique_ptr<LoopClosureDetectorLogger> logger_;
  LcdDebugInfo debug_info_;
};

}  // namespace VIO
