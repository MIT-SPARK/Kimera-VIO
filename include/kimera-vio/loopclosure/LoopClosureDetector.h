/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoopClosureDetector.cpp
 * @brief  Pipeline for detection and reporting of Loop Closures between frames
 * @author Marcus Abate, Luca Carlone
 */

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>

#include <DBoW2/DBoW2.h>

#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"
#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"

/* ------------------------------------------------------------------------ */
// Forward declare KimeraRPGO, a private dependency.
namespace KimeraRPGO {
class RobustSolver;
}

namespace VIO {

/* ------------------------------------------------------------------------ */
typedef std::function<void(const LoopClosureDetectorOutputPayload&)>
    LoopClosurePGOCallback;

/* ------------------------------------------------------------------------ */
class LoopClosureDetector {
 public:
  /* ------------------------------------------------------------------------ */
  /* @brief Constructor: detects loop-closures and updates internal PGO.
   * @param[in] lcd_params Parameters for the instance of LoopClosureDetector.
   * @param[in] log_output Output-logging flag. If set to true, the logger is
   *  instantiated and output/statistics are logged at every spinOnce().
   */
  LoopClosureDetector(const LoopClosureDetectorParams& lcd_params,
                      const bool log_output = false);

  /* ------------------------------------------------------------------------ */
  /* @brief Destructor.
   */
  virtual ~LoopClosureDetector();

  /* ------------------------------------------------------------------------ */
  /* @brief Main spin function for the LoopClosureDetector pipeline.
   * @param[in] input_queue A ThreadsafeQueue into which input payloads are
   * pushed.
   * @param[in] parallel_run The parallelized flag. If set to false, the spin
   * won't loop and instead will return at the end of each run.
   * @reutrn True if everything goes well.
   */
  bool spin(ThreadsafeQueue<LoopClosureDetectorInputPayload>& input_queue,
            bool parallel_run = true);

  /* ------------------------------------------------------------------------ */
  /* @brief Processed a single input payload and runs it through the pipeline.
   * @param[in] input A shared_ptr referencing an input payload.
   * @return The output payload from the pipeline.
   */
  LoopClosureDetectorOutputPayload spinOnce(
      const std::shared_ptr<LoopClosureDetectorInputPayload>& input);

  /* ------------------------------------------------------------------------ */
  /* @brief Processed a single frame and adds it to relevant internal databases.
   * @param[in] stereo_frame A StereoFrame object with two images and a pose to
   * the body frame at a minimum. Other fields may also be populated.
   * @return The local ID of the frame after it is added to the databases.
   */
  FrameId processAndAddFrame(const StereoFrame& stereo_frame);

  /* ------------------------------------------------------------------------ */
  /* @brief Runs all checks on a frame and determines whether it a loop-closure
      with a previous frame or not. Fills the LoopResult with this information.
   * @param[in] stereo_frame A stereo_frame that has already been "rewritten" by
   *  the pipeline to have ORB features and keypoints.
   * @param[out] result A pointer to the LoopResult that is filled with the
   *  result of the loop-closure detection stage.
   * @return True if the frame is declared a loop-closure with a previous frame,
   *  false otherwise.
   */
  bool detectLoop(const StereoFrame& stereo_frame, LoopResult* result);

  /* ------------------------------------------------------------------------ */
  /* @brief Verify that the geometry between two frames is close enough to be
      considered a match, and generate a monocular transformation between them.
   * @param[in] query_id The frame ID of the query image in the database.
   * @param[in] match_id The frame ID of the match image in the databse.
   * @param[out] camCur_T_camRef_mono The pose between the match frame and the
   *  query frame, in the coordinates of the match frame.
   * @return True if the verification check passes, false otherwise.
   */
  bool geometricVerificationCheck(const FrameId& query_id,
                                  const FrameId& match_id,
                                  gtsam::Pose3* camCur_T_camRef_mono);

  /* ------------------------------------------------------------------------ */
  /* @brief Determine the 3D pose betwen two frames.
   * @param[in] query_id The frame ID of the query image in the database.
   * @param[in] match_id The frame ID of the match image in the database.
   * @param[in] camCur_T_camRef_mono The relative pose between the match frame
   *  and the query frame, in the coordinates of the match frame.
   * @param[out] bodyCur_T_bodyRef_stereo The 3D pose between the match frame
   *  and the query frame, in the coordinates of the match frame.
   * @return True if the pose is recovered successfully, false otherwise.
   */
  bool recoverPose(const FrameId& query_id,
                   const FrameId& match_id,
                   const gtsam::Pose3& camCur_T_camRef_mono,
                   gtsam::Pose3* bodyCur_T_bodyRef_stereo);

  /* ------------------------------------------------------------------------ */
  /* @brief Shuts-down the LoopClosureDetector pipeline.
   */
  inline void shutdown() {
    LOG_IF(WARNING, shutdown_) << "Shutdown requested, but LoopClosureDetector "
                                  "was already shutdown.";
    LOG(INFO) << "Shutting down LoopClosureDetector.";
    shutdown_ = true;
  }

  /* ------------------------------------------------------------------------ */
  /* @brief Restarts the LoopClosureDetector pipeline.
   */
  inline void restart() {
    LOG(INFO) << "Resetting shutdown LoopClosureDetector flag to false.";
    shutdown_ = false;
  }

  /* ------------------------------------------------------------------------ */
  /* @brief Returns the working status of the thread.
   * @return True if the thread is working, false otherwise.
   */
  inline bool isWorking() const { return is_thread_working_; }

  /* ------------------------------------------------------------------------ */
  /* @brief Gets a copy of the parameters of the LoopClosureDetector.
   * @return The local parameters of the LoopClosureDetector.
   */
  inline LoopClosureDetectorParams getLCDParams() const { return lcd_params_; }

  /* ------------------------------------------------------------------------ */
  /* @brief Returns a pointer to the parameters of the LoopClosureDetector.
   * @return A pointer to the parameters of the LoopClosureDetector.
   */
  inline LoopClosureDetectorParams* getLCDParamsMutable() {
    return &lcd_params_;
  }

  /* ------------------------------------------------------------------------ */
  /* @brief Returns the RAW pointer to the BoW database.
   * @return A pointer to the BoW database.
   *
   * WARNING: This is a potentially dangerous method to use because it requires
   *  a manual deletion of the pointer before it goes out of scope.
   */
  inline const OrbDatabase* getBoWDatabase() const { return db_BoW_.get(); }

  /* ------------------------------------------------------------------------ */
  /* @brief Returns a pointer to the database of LCDFrames.
   * @return A pointer to the LCDFrame database.
   *
   * WARNING: This is a potentially dangerous method to use because it requires
   *  a manual deletion of the pointer before it goes out of scope.
   */
  inline const std::vector<LCDFrame>* getFrameDatabasePtr() const {
    return &db_frames_;
  }

  /* ------------------------------------------------------------------------ */
  /* @brief Returns the "intrinsics flag", which is true if the pipeline has
   *  recieved the dimensions, principle point, and focal length of the images
   *  in the frames, as well as the transformation from body to camera.
   * @return True if the intrinsics have been recieved, false otherwise.
   */
  inline const bool getIntrinsicsFlag() const { return set_intrinsics_; }

  /* ------------------------------------------------------------------------ */
  /* @brief Returns the pose between the inertial world-reference frame and the
   *  "map" frame, which is the error between the VIO and the PGO trajectories.
   * @return The pose of the map frame relative to the world frame.
   */
  const gtsam::Pose3 getWPoseMap() const;

  /* ------------------------------------------------------------------------ */
  /* @brief Returns the values of the PGO, which is the full trajectory of the
   *  PGO.
   * @return The gtsam::Values (poses) of the PGO.
   */
  const gtsam::Values getPGOTrajectory() const;

  /* ------------------------------------------------------------------------ */
  /* @brief Returns the Nonlinear-Factor-Graph from the PGO.
   * @return The gtsam::NonlinearFactorGraph of the optimized trajectory from
   *  the PGO.
   */
  const gtsam::NonlinearFactorGraph getPGOnfg() const;

  /* ------------------------------------------------------------------------ */
  /* @brief Registers an external callback to which output payloads are sent.
   *  The callback is added to a vector of callbacks, enabling multiple
   *  functions for each output payload.
   * @param[in] callback A LoopClosurePGOCallback function which will parse
   *  LoopClosureDetectorOutputPayload objects.
   */
  inline void registerLcdPgoOutputCallback(
      const LoopClosurePGOCallback& callback) {
    lcd_pgo_output_callbacks_.push_back(callback);
  }

  /* ------------------------------------------------------------------------ */
  /* @brief Set the bool set_intrinsics as well as the parameter members
   *  representing the principle point, image dimensions, focal length and
   *  camera-to-body pose.
   * @param[in] stereo_frame A StereoFrame with the calibration and parameters
   *  needed to set the intrinsics.
   */
  // TODO(marcus): this should be private. But that makes testing harder.
  void setIntrinsics(const StereoFrame& stereo_frame);

  /* ------------------------------------------------------------------------ */
  /* @brief Set the OrbDatabase internal member.
   * @param[in] db An OrbDatabase object.
   */
  void setDatabase(const OrbDatabase& db);

  /* ------------------------------------------------------------------------ */
  /* @brief Prints parameters and other statistics on the LoopClosureDetector.
   */
  void print() const;

  /* ------------------------------------------------------------------------ */
  /* @brief Clears all keypoints and features from an input StereoFrame and
   *  fills it with ORB features.
   * @param[in] keypoints A vector of KeyPoints representing the ORB keypoints
   *  identified by an ORB detector.
   * @param[in/out] A StereoFrame initially filled with front-end features,
   *  which is then replaced with ORB features from the keypoints parameter.
   */
  // TODO(marcus): utils and reorder (or just static)
  void rewriteStereoFrameFeatures(const std::vector<cv::KeyPoint>& keypoints,
                                  StereoFrame* stereo_frame) const;

  /* ------------------------------------------------------------------------ */
  /* @brief Creates an image with matched ORB features between two frames.
   *  This is a utility for debugging the ORB feature matcher and isn't used
   *  in the main pipeline.
   * @param[in] query_img The image of the query frame in the database.
   * @param[in] match_img The image of the match frame in the database.
   * @param[in] query_id The frame ID of the query frame in the database.
   * @param[in] match_id The frame ID of the match frame in the database.
   * @param[in] cut_matches Determines if the Lowe Ratio Test is used to
   *  pare down matches that are bad.
   * @return A cv::Mat representing the matches between the two images.
   */
  // TODO(marcus): it would be nice if this could be a util
  // TODO(marcus): can this be static even though it requires id? Maybe feed it
  // descriptors instead
  cv::Mat computeAndDrawMatchesBetweenFrames(const cv::Mat& query_img,
                                             const cv::Mat& match_img,
                                             const FrameId& query_id,
                                             const FrameId& match_id,
                                             bool cut_matches = false) const;

  /* ------------------------------------------------------------------------ */
  /* @brief Gives the transform between two frames in the body frame given
   *  that same transform in the camera frame.
   * @param[in] camCur_T_camRef The relative pose between two frames in the
   *  camera coordinate frame.
   * @param[out] bodyCur_T_bodyRef The relative pose between two frames in the
   *  body coordinate frame.
   */
  // TODO(marcus): these should be private or util
  void transformCameraPoseToBodyPose(const gtsam::Pose3& camCur_T_camRef,
                                     gtsam::Pose3* bodyCur_T_bodyRef) const;

  /* ------------------------------------------------------------------------ */
  /* @brief The inverse of transformCameraPoseToBodyPose.
   * @param[in] bodyCur_T_bodyRef The relative pose between two frames in the
   *  body coordinate frame.
   * @param[out] camCur_T_camRef The relative pose between two frames in the
   *  camera coordinate frame.
   * @return
   */
  void transformBodyPoseToCameraPose(const gtsam::Pose3& bodyCur_T_bodyRef,
                                     gtsam::Pose3* camCur_T_camRef) const;

  /* ------------------------------------------------------------------------ */
  /* @brief Adds an odometry factor to the PGO and optimizes the trajectory.
   *  No actual optimization is performed on the RPGO side for odometry.
   * @param[in] factor An OdometryFactor representing the backend's guess for
   *  odometry between two consecutive keyframes.
   */
  void addOdometryFactorAndOptimize(const OdometryFactor& factor);

  /* ------------------------------------------------------------------------ */
  /* @brief Adds a loop-closure factor to the PGO and optimizes the trajectory.
   * @param[in] factor A LoopClosureFactor representing the relative pose
   *  between two frames that are not (necessarily) consecutive.
   */
  void addLoopClosureFactorAndOptimize(const LoopClosureFactor& factor);

  /* ------------------------------------------------------------------------ */
  /* @brief Initializes the RobustSolver member with no prior, or a neutral
   *  starting pose.
   */
  void initializePGO();

  /* ------------------------------------------------------------------------ */
  /* @brief Initializes the RobustSolver member with an initial prior factor,
   *  which can be the first OdometryFactor given by the backend.
   * @param[in] factor An OdometryFactor representing the pose between the
   *  initial state of the vehicle and the first keyframe.
   */
  void initializePGO(const OdometryFactor& factor);

 private:
  /* ------------------------------------------------------------------------ */
  /* @brief Determines whether a frame meets the temoral constraint given by
   *  a MatchIsland.
   * @param[in] id The frame ID of the frame being processed in the database.
   * @param[in] island A MatchIsland representing several potential matches.
   * @return True if the constraint is met, false otherwise.
   */
  // TODO(marcus): unit tests
  bool checkTemporalConstraint(const FrameId& id, const MatchIsland& island);

  /* ------------------------------------------------------------------------ */
  /* @brief Computes the various islands created by a QueryResult, which is
   *  given by the OrbDatabase.
   * @param[in] q A QueryResults object containing all the resulting possible
   *  matches with a frame.
   * @param[out] A vector of MatchIslands, each of which is an island of
   *  nearby possible matches with the frame being queried.
   * @return
   */
  // TODO(marcus): unit tests
  void computeIslands(DBoW2::QueryResults& q,
                      std::vector<MatchIsland>* islands) const;

  /* ------------------------------------------------------------------------ */
  /* @brief Compute the overall score of an island.
   * @param[in] q A QueryResults object containing all the possible matches
   *  with a frame.
   * @param[in] start_id The frame ID that starts the island.
   * @param[in] end_id The frame ID that ends the island.
   * @reutrn The score of the island.
   */
  double computeIslandScore(const DBoW2::QueryResults& q,
                            const FrameId& start_id,
                            const FrameId& end_id) const;

  /* ------------------------------------------------------------------------ */
  /* @brief Computes the indices of keypoints that match between two frames.
   * @param[in] query_id The frame ID of the query frame in the database.
   * @param[in] match_id The frame ID of the match frame in the database.
   * @param[out] i_query A vector of indices that match in the query frame.
   * @param[out] i_match A vector of indices that match in the match frame.
   * @param[in] cut_matches If true, Lowe's Ratio Test will be used to cut
   *  out bad matches before sending output.
   */
  void computeMatchedIndices(const FrameId& query_id,
                             const FrameId& match_id,
                             std::vector<int>* i_query,
                             std::vector<int>* i_match,
                             bool cut_matches = false) const;

  /* ------------------------------------------------------------------------ */
  /* @brief Checks geometric verification and determines a pose with
   *  a translation up to a scale factor between two frames, using Nister's
   *  five-point method.
   * @param[in] query_id The frame ID of the query frame in the database.
   * @param[in] match_id The frame ID of the match frame in the database.
   * @param[out] camCur_T_camRef_mono The relative pose between the two frames,
   *  with translation up to a scale factor.
   * @return True if the verification passes, false otherwise.
   */
  bool geometricVerificationNister(const FrameId& query_id,
                                   const FrameId& match_id,
                                   gtsam::Pose3* camCur_T_camRef_mono);

  /* ------------------------------------------------------------------------ */
  /* @brief Checks geometric verification and determines a pose that is
   *  "stereo" - correct in translation scale using Arun's three-point method.
   * @param[in] query_id The frame ID of the query frame in the database.
   * @param[in] match_id The frame ID of the match frame in the database.
   * @param[out] bodyCur_T_bodyRef The relative pose between the two frames.
   * @return True if the verification passes, false otherwise.
   */
  bool recoverPoseArun(const FrameId& query_id,
                       const FrameId& match_id,
                       gtsam::Pose3* bodyCur_T_bodyRef);

  /* ------------------------------------------------------------------------ */
  /* @brief Checks geometric verification and determines a pose that is
   *  "stereo" - correct in translation scale using the median of all
   *  3D keypoints matched between the frames.
   * @param[in] query_id The frame ID of the query frame in the database.
   * @param[in] match_id The frame ID of the match frame in the database.
   * @param[out] bodyCur_T_bodyRef The relative pose between the two frames.
   * @return True if the verification passes, false otherwise.
   */
  bool recoverPoseGivenRot(const FrameId& query_id,
                           const FrameId& match_id,
                           const gtsam::Pose3& camCur_T_camRef_mono,
                           gtsam::Pose3* bodyCur_T_bodyRef);

 private:
  // Output callback(s).
  std::vector<LoopClosurePGOCallback> lcd_pgo_output_callbacks_;

  // Parameter members
  LoopClosureDetectorParams lcd_params_;
  const bool log_output_ = {false};
  bool set_intrinsics_ = {false};

  // Thread related members
  std::atomic_bool shutdown_ = {false};
  std::atomic_bool is_thread_working_ = {false};

  // ORB extraction and matching members
  cv::Ptr<cv::ORB> orb_feature_detector_;
  cv::Ptr<cv::DescriptorMatcher> orb_feature_matcher_;

  // BoW and Loop Detection database and members
  std::unique_ptr<OrbDatabase> db_BoW_;
  std::vector<LCDFrame> db_frames_;
  FrameIDTimestampMap timestamp_map_;

  // Store latest computed objects for temporal matching and nss scoring
  DBoW2::BowVector latest_bowvec_;
  MatchIsland latest_matched_island_;
  FrameId latest_query_id_;
  int temporal_entries_;

  // Store camera parameters and StereoFrame stuff once
  gtsam::Pose3 B_Pose_camLrect_;

  // Robust PGO members
  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_;
  std::vector<gtsam::Pose3> W_Pose_Blkf_estimates_;
  gtsam::SharedNoiseModel
      shared_noise_model_;  // TODO(marcus): make accurate
                            // should also come in with input

  // Logging members
  std::unique_ptr<LoopClosureDetectorLogger> logger_;
  LcdDebugInfo debug_info_;
};  // class LoopClosureDetector

}  // namespace VIO
