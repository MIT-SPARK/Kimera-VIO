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

#include <DBoW2/DBoW2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <limits>
#include <memory>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/loopclosure/LcdThirdPartyWrapper.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"
#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"

/* ------------------------------------------------------------------------ */
// Forward declare KimeraRPGO, a private dependency.
namespace KimeraRPGO {
class RobustSolver;
}

namespace VIO {

/* ------------------------------------------------------------------------ */
class LoopClosureDetector {
 public:
  KIMERA_POINTER_TYPEDEFS(LoopClosureDetector);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LoopClosureDetector);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* ------------------------------------------------------------------------ */
  /** @brief Constructor: detects loop-closures and updates internal PGO.
   * @param[in] lcd_params Parameters for the instance of LoopClosureDetector.
   * @param[in] log_output Output-logging flag. If set to true, the logger is
   *  instantiated and output/statistics are logged at every spinOnce().
   */
  LoopClosureDetector(const LoopClosureDetectorParams& lcd_params,
                      const StereoCamera::ConstPtr& stereo_camera,
                      const StereoMatchingParams& stereo_matching_params,
                      bool log_output);

  /* ------------------------------------------------------------------------ */
  virtual ~LoopClosureDetector();

  /* ------------------------------------------------------------------------ */
  /** @brief Processed a single input payload and runs it through the pipeline.
   * @param[in] input A shared_ptr referencing an input payload.
   * @return The output payload from the pipeline.
   */
  virtual LcdOutput::UniquePtr spinOnce(const LcdInput& input);

  /* ------------------------------------------------------------------------ */
  /** @brief Processed a single frame and adds it to relevant internal
   * databases.
   * @param[in] stereo_frame A StereoFrame object with two images and a pose to
   * the body frame at a minimum. Other fields may also be populated.
   * @return The local ID of the frame after it is added to the databases.
   */
  FrameId processAndAddFrame(const StereoFrame& stereo_frame);

  /* ------------------------------------------------------------------------ */
  /** @brief Runs all checks on a frame and determines whether it a loop-closure
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
  /** @brief Verify that the geometry between two frames is close enough to be
      considered a match, and generate a monocular transformation between them.
   * @param[in] query_id The frame ID of the query image in the database.
   * @param[in] match_id The frame ID of the match image in the databse.
   * @param[out] camMatch_T_camQuery_mono The pose between the match frame and the
   *  query frame, in the coordinates of the match frame.
   * @return True if the verification check passes, false otherwise.
   */
  bool geometricVerificationCheck(
      const FrameId& query_id,
      const FrameId& match_id,
      gtsam::Pose3* camMatch_T_camQuery_mono,
      std::vector<FrameId>* inlier_id_in_query_frame,
      std::vector<FrameId>* inlier_id_in_match_frame);

  /* ------------------------------------------------------------------------ */
  /** @brief Determine the 3D pose betwen two frames.
   * @param[in] query_id The frame ID of the query image in the database.
   * @param[in] match_id The frame ID of the match image in the database.
   * @param[in] camMatch_T_camQuery_mono The relative pose between the match frame
   *  and the query frame, in the coordinates of the match frame.
   * @param[out] bodyMatch_T_bodyQuery_stereo The 3D pose between the match frame
   *  and the query frame, in the coordinates of the match frame.
   * @return True if the pose is recovered successfully, false otherwise.
   */
  bool recoverPose(const FrameId& query_id,
                   const FrameId& match_id,
                   const gtsam::Pose3& camMatch_T_camQuery_mono,
                   gtsam::Pose3* bodyMatch_T_bodyQuery_stereo,
                   std::vector<FrameId>* inlier_id_in_query_frame,
                   std::vector<FrameId>* inlier_id_in_match_frame);

  /* ------------------------------------------------------------------------ */
  /** @brief Refine relative pose given by ransac using smart factors.
   * @param[in] query_id The frame ID of the query image in the database.
   * @param[in] match_id The frame ID of the match image in the database.
   * @param[in] camMatch_T_camQuery_stereo The relative pose between the match frame
   *  and the query frame, in the coordinates of the match frame.
   * @param[in] inlier correspondences (from ransac) in the query frame
   * @param[in] inlier correspondences (from ransac) in the match frame
   * @return refined relative pose
   */
  gtsam::Pose3 refinePoses(
      const FrameId query_id,
      const FrameId match_id,
      const gtsam::Pose3& camMatch_T_camQuery_stereo,
      const std::vector<FrameId>& inlier_id_in_query_frame,
      const std::vector<FrameId>& inlier_id_in_match_frame);

  /* ------------------------------------------------------------------------ */
  /** @brief Gets a copy of the parameters of the LoopClosureDetector.
   * @return The local parameters of the LoopClosureDetector.
   */
  inline LoopClosureDetectorParams getLCDParams() const { return lcd_params_; }

  /* ------------------------------------------------------------------------ */
  /** @brief Returns a pointer to the parameters of the LoopClosureDetector.
   * @return A pointer to the parameters of the LoopClosureDetector.
   */
  inline LoopClosureDetectorParams* getLCDParamsMutable() {
    return &lcd_params_;
  }

  /* ------------------------------------------------------------------------ */
  /** @brief Returns the RAW pointer to the BoW database.
   * @return A pointer to the BoW database.
   *
   * WARNING: This is a potentially dangerous method to use because it requires
   *  a manual deletion of the pointer before it goes out of scope.
   */
  inline const OrbDatabase* getBoWDatabase() const { return db_BoW_.get(); }

  /* ------------------------------------------------------------------------ */
  /** @brief Returns a pointer to the database of LCDFrames.
   * @return A pointer to the LCDFrame database.
   *
   * WARNING: This is a potentially dangerous method to use because it requires
   *  a manual deletion of the pointer before it goes out of scope.
   */
  inline const std::vector<LCDFrame>* getFrameDatabasePtr() const {
    return &db_frames_;
  }

  /* ------------------------------------------------------------------------ */
  /** @brief Returns the pose between the inertial world-reference frame and the
   *  "map" frame, which is the error between the VIO and the PGO trajectories.
   * @return The pose of the map frame relative to the world frame.
   */
  const gtsam::Pose3 getWPoseMap() const;

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
   * @param[in/out] A StereoFrame initially filled with front-end features,
   *  which is then replaced with ORB features from the keypoints parameter.
   */
  // TODO(marcus): utils and reorder (or just static)
  void rewriteStereoFrameFeatures(const std::vector<cv::KeyPoint>& keypoints,
                                  StereoFrame* stereo_frame) const;

  /* ------------------------------------------------------------------------ */
  /** @brief Creates an image with matched ORB features between two frames.
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
  /** @brief Gives the transform between two frames in the body frame given
   *  that same transform in the camera frame.
   * @param[in] camMatch_T_camQuery The relative pose between two frames in the
   *  camera coordinate frame.
   * @param[out] bodyMatch_T_bodyQuery The relative pose between two frames in the
   *  body coordinate frame.
   */
  // TODO(marcus): these should be private or util
  void transformCameraPoseToBodyPose(const gtsam::Pose3& camMatch_T_camQuery,
                                     gtsam::Pose3* bodyMatch_T_bodyQuery) const;

  /* ------------------------------------------------------------------------ */
  /** @brief The inverse of transformCameraPoseToBodyPose.
   * @param[in] bodyMatch_T_bodyQuery The relative pose between two frames in the
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
   * @param[in] query_id The frame ID of the query frame in the database.
   * @param[in] match_id The frame ID of the match frame in the database.
   * @param[out] i_query A vector of indices that match in the query frame.
   * @param[out] i_match A vector of indices that match in the match frame.
   * @param[in] cut_matches If true, Lowe's Ratio Test will be used to cut
   *  out bad matches before sending output.
   */
  void computeMatchedIndices(const FrameId& query_id,
                             const FrameId& match_id,
                             std::vector<FrameId>* i_query,
                             std::vector<FrameId>* i_match,
                             bool cut_matches = false) const;

 private:
  /* ------------------------------------------------------------------------ */
  /** @brief Checks geometric verification and determines a pose with
   *  a translation up to a scale factor between two frames, using Nister's
   *  five-point method.
   * @param[in] query_id The frame ID of the query frame in the database.
   * @param[in] match_id The frame ID of the match frame in the database.
   * @param[out] camMatch_T_camQuery_mono The relative pose between the two frames,
   *  with translation up to a scale factor.
   * @return True if the verification passes, false otherwise.
   */
  bool geometricVerificationNister(
      const FrameId& query_id,
      const FrameId& match_id,
      gtsam::Pose3* camMatch_T_camQuery_mono,
      std::vector<FrameId>* inlier_id_in_query_frame,
      std::vector<FrameId>* inlier_id_in_match_frame);

  /* ------------------------------------------------------------------------ */
  /** @brief Checks geometric verification and determines a pose that is
   *  "stereo" - correct in translation scale using Arun's three-point method.
   * @param[in] query_id The frame ID of the query frame in the database.
   * @param[in] match_id The frame ID of the match frame in the database.
   * @param[out] bodyMatch_T_bodyQuery The relative pose between the two frames.
   * @return True if the verification passes, false otherwise.
   */
  bool recoverPoseArun(const FrameId& query_id,
                       const FrameId& match_id,
                       gtsam::Pose3* bodyMatch_T_bodyQuery,
                       std::vector<FrameId>* inlier_id_in_query_frame,
                       std::vector<FrameId>* inlier_id_in_match_frame);

  /* ------------------------------------------------------------------------ */
  /** @brief Checks geometric verification and determines a pose that is
   *  "stereo" - correct in translation scale using the median of all
   *  3D keypoints matched between the frames.
   * @param[in] query_id The frame ID of the query frame in the database.
   * @param[in] match_id The frame ID of the match frame in the database.
   * @param[out] bodyMatch_T_bodyQuery The relative pose between the two frames.
   * @return True if the verification passes, false otherwise.
   */
  bool recoverPoseGivenRot(const FrameId& query_id,
                           const FrameId& match_id,
                           const gtsam::Pose3& camMatch_T_camQuery_mono,
                           gtsam::Pose3* bodyMatch_T_bodyQuery,
                           std::vector<FrameId>* inlier_id_in_query_frame,
                           std::vector<FrameId>* inlier_id_in_match_frame);

 private:
  enum class LcdState {
    Bootstrap,  //! Lcd is initializing
    Nominal     //! Lcd is running in nominal mode
  };
  LcdState lcd_state_ = LcdState::Bootstrap;

  // Parameter members
  LoopClosureDetectorParams lcd_params_;
  const bool log_output_ = false;

  // ORB extraction and matching members
  cv::Ptr<cv::ORB> orb_feature_detector_;
  cv::Ptr<cv::DescriptorMatcher> orb_feature_matcher_;

  // BoW and Loop Detection database and members
  std::unique_ptr<OrbDatabase> db_BoW_;
  std::vector<LCDFrame> db_frames_;
  FrameIDTimestampMap timestamp_map_;

  // Store latest computed objects for temporal matching and nss scoring
  LcdThirdPartyWrapper::UniquePtr lcd_tp_wrapper_;
  DBoW2::BowVector latest_bowvec_;

  // Store camera parameters and StereoFrame stuff once
  gtsam::Pose3 B_Pose_camLrect_;
  StereoCamera::ConstPtr stereo_camera_;
  StereoMatcher::UniquePtr stereo_matcher_;

  // Robust PGO members
  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_;
  std::vector<gtsam::Pose3> W_Pose_Blkf_estimates_;
  gtsam::SharedNoiseModel
      shared_noise_model_;  // TODO(marcus): make accurate
                            // should also come in with input

  // Logging members
  std::unique_ptr<LoopClosureDetectorLogger> logger_;
  LcdDebugInfo debug_info_;

 private:
  // Lcd typedefs
  using AdapterMono = opengv::relative_pose::CentralRelativeAdapter;
  using SacProblemMono =
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
  using AdapterStereo = opengv::point_cloud::PointCloudAdapter;
  using SacProblemStereo =
      opengv::sac_problems::point_cloud::PointCloudSacProblem;
};  // class LoopClosureDetector

enum class LoopClosureDetectorType {
  //! Bag of Words approach
  BoW = 0u,
};

class LcdFactory {
 public:
  KIMERA_POINTER_TYPEDEFS(LcdFactory);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LcdFactory);
  LcdFactory() = delete;
  virtual ~LcdFactory() = default;

  static LoopClosureDetector::UniquePtr createLcd(
      const LoopClosureDetectorType& lcd_type,
      const LoopClosureDetectorParams& lcd_params,
      const StereoCamera::ConstPtr& stereo_camera,
      const StereoMatchingParams& stereo_matching_params,
      bool log_output) {
    switch (lcd_type) {
      case LoopClosureDetectorType::BoW: {
        return VIO::make_unique<LoopClosureDetector>(lcd_params,
                                                     stereo_camera,
                                                     stereo_matching_params,
                                                     log_output);
      }
      default: {
        LOG(FATAL) << "Requested loop closure detector type is not supported.\n"
                   << "Currently supported loop closure detector types:\n"
                   << "0: BoW \n but requested loop closure detector: "
                   << static_cast<int>(lcd_type);
      }
    }
  }
};

class LcdModule : public MIMOPipelineModule<LcdInput, LcdOutput> {
 public:
  KIMERA_POINTER_TYPEDEFS(LcdModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LcdModule);
  using LcdFrontendInput = FrontendOutputPacketBase::Ptr;
  using LcdBackendInput = BackendOutput::Ptr;

  LcdModule(bool parallel_run, LoopClosureDetector::UniquePtr lcd)
      : MIMOPipelineModule<LcdInput, LcdOutput>("Lcd", parallel_run),
        frontend_queue_("lcd_frontend_queue"),
        backend_queue_("lcd_backend_queue"),
        lcd_(std::move(lcd)) {}
  virtual ~LcdModule() = default;

  //! Callbacks to fill queues: they should be all lighting fast.
  inline void fillFrontendQueue(const LcdFrontendInput& frontend_payload) {
    frontend_queue_.push(frontend_payload);
  }
  inline void fillBackendQueue(const LcdBackendInput& backend_payload) {
    backend_queue_.push(backend_payload);
  }

 protected:
  //! Synchronize input queues.
  inline InputUniquePtr getInputPacket() override {
    // TODO(X): this is the same or very similar to the Mesher getInputPacket.
    LcdBackendInput backend_payload;
    bool queue_state = false;
    if (PIO::parallel_run_) {
      queue_state = backend_queue_.popBlocking(backend_payload);
    } else {
      queue_state = backend_queue_.pop(backend_payload);
    }
    if (!queue_state) {
      LOG_IF(WARNING, PIO::parallel_run_)
          << "Module: " << name_id_ << " - Backend queue is down";
      VLOG_IF(1, !PIO::parallel_run_)
          << "Module: " << name_id_ << " - Backend queue is empty or down";
      return nullptr;
    }
    CHECK(backend_payload);
    const Timestamp& timestamp = backend_payload->W_State_Blkf_.timestamp_;

    // Look for the synchronized packet in Frontend payload queue
    // This should always work, because it should not be possible to have
    // a Backend payload without having a Frontend one first!
    LcdFrontendInput frontend_payload = nullptr;
    PIO::syncQueue(timestamp, &frontend_queue_, &frontend_payload);
    CHECK(frontend_payload);
    CHECK(frontend_payload->is_keyframe_);
    CHECK_EQ(timestamp, frontend_payload->timestamp_);

    // Push the synced messages to the lcd's input queue
    const gtsam::Pose3& body_pose = backend_payload->W_State_Blkf_.pose_;
    return VIO::make_unique<LcdInput>(
        timestamp, frontend_payload, backend_payload->cur_kf_id_, body_pose);
  }

  OutputUniquePtr spinOnce(LcdInput::UniquePtr input) override {
    return lcd_->spinOnce(*input);
  }

  //! Called when general shutdown of PipelineModule is triggered.
  void shutdownQueues() override {
    LOG(INFO) << "Shutting down queues for: " << name_id_;
    frontend_queue_.shutdown();
    backend_queue_.shutdown();
  }

  //! Checks if the module has work to do (should check input queues are empty)
  bool hasWork() const override {
    // We don't check Frontend queue because it runs faster than Backend queue.
    return !backend_queue_.empty();
  }

 private:
  //! Input Queues
  ThreadsafeQueue<LcdFrontendInput> frontend_queue_;
  ThreadsafeQueue<LcdBackendInput> backend_queue_;

  //! Lcd implementation
  LoopClosureDetector::UniquePtr lcd_;
};

}  // namespace VIO
