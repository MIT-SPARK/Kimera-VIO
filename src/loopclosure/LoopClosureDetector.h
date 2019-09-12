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
#include <vector>

#include "loopclosure/LoopClosureDetector-definitions.h"
#include "loopclosure/LoopClosureDetectorParams.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>

#include <DBoW2/DBoW2.h>

#include "utils/ThreadsafeQueue.h"
#include "StereoFrame.h"
#include "logging/Logger.h"

// Forward declare RobustPGO, a private dependency.
namespace RobustPGO {
  class RobustSolver;
}

namespace VIO {

class LoopClosureDetector {
 public:
  LoopClosureDetector(const LoopClosureDetectorParams& lcd_params,
                      const bool log_output = false);

  virtual ~LoopClosureDetector();

  bool spin(ThreadsafeQueue<LoopClosureDetectorInputPayload>& input_queue,
            ThreadsafeQueue<LoopClosureDetectorOutputPayload>& output_queue,
            bool parallel_run = true);

  LoopClosureDetectorOutputPayload spinOnce(
      const std::shared_ptr<LoopClosureDetectorInputPayload>& input);

  FrameId processAndAddFrame(const StereoFrame& stereo_frame);

  bool detectLoop(const StereoFrame& stereo_frame, LoopResult* result);

  bool geometricVerificationCheck(const FrameId& query_id,
                                  const FrameId& match_id,
                                  gtsam::Pose3* camCur_T_camRef_mono) const;

  bool recoverPose(const FrameId& query_id, const FrameId& match_id,
                   const gtsam::Pose3& camCur_T_camRef_mono,
                   gtsam::Pose3* bodyCur_T_bodyRef_stereo) const;

  inline void shutdown() {
    LOG_IF(WARNING, shutdown_) << "Shutdown requested, but LoopClosureDetector "
                                  "was already shutdown.";
    LOG(INFO) << "Shutting down LoopClosureDetector.";
    shutdown_ = true;
  }

  inline void restart() {
    LOG(INFO) << "Resetting shutdown LoopClosureDetector flag to false.";
    shutdown_ = false;
  }

  inline bool isWorking() const { return is_thread_working_; }

  inline LoopClosureDetectorParams getLCDParams() const {
    return lcd_params_;
  }

  inline LoopClosureDetectorParams* getLCDParamsMutable() {
    return &lcd_params_;
  }

  inline const OrbDatabase* getBoWDatabase() const { return db_BoW_.get(); }

  inline const std::vector<LCDFrame>* getFrameDatabasePtr() const {
    return &db_frames_;
  }

  inline const bool getIntrinsicsFlag() const { return set_intrinsics_; }

  const gtsam::Pose3 getWPoseMap() const;

  const gtsam::Values getPGOTrajectory() const;

  const gtsam::NonlinearFactorGraph getPGOnfg() const;

  // TODO(marcus): maybe this should be private. But that makes testing harder.
  void setIntrinsics(const StereoFrame& stereo_frame);

  void setDatabase(const OrbDatabase& db);

  void allocate(size_t n);

  inline void clear();

  void print() const;

  // TODO(marcus): utils and reorder (or just static)
  void rewriteStereoFrameFeatures(const std::vector<cv::KeyPoint>& keypoints,
                                  StereoFrame* stereo_frame) const;

  // TODO(marcus): it would be nice if this could be a util
  // TODO(marcus): can this be static even though it requires id? Maybe feed it
  // descriptors instead
  cv::Mat computeAndDrawMatchesBetweenFrames(const cv::Mat& query_img,
                                             const cv::Mat& match_img,
                                             const FrameId& query_id,
                                             const FrameId& match_id,
                                             bool cut_matches = false) const;

  // TODO(marcus): maybe these should be private?
  void transformCameraPose2BodyPose(const gtsam::Pose3& camCur_T_camRef,
                                    gtsam::Pose3* bodyCur_T_bodyRef) const;

  void transformBodyPose2CameraPose(const gtsam::Pose3& bodyCur_T_bodyRef,
                                    gtsam::Pose3* camCur_T_camRef) const;

  void addOdometryFactorAndOptimize(const OdometryFactor& factor);

  void addLoopClosureFactorAndOptimize(const LoopClosureFactor& factor);

  void initializePGO();

  void initializePGO(const OdometryFactor& factor);

 private:
  // TODO(marcus): review this one
  bool checkTemporalConstraint(const FrameId& id, const MatchIsland& island);

  // TODO(marcus): review these
  void computeIslands(DBoW2::QueryResults& q,
      std::vector<MatchIsland>* islands) const;

  double computeIslandScore(const DBoW2::QueryResults& q,
                            const FrameId& start_id,
                            const FrameId& end_id) const;

  void computeMatchedIndices(const FrameId& query_id, const FrameId& match_id,
                             std::vector<unsigned int>* i_query,
                             std::vector<unsigned int>* i_match,
                             bool cut_matches = false) const;

  bool geometricVerificationNister(const FrameId& query_id,
                                   const FrameId& match_id,
                                   gtsam::Pose3* camCur_T_camRef_mono) const;

  bool recoverPoseArun(const FrameId& query_id, const FrameId& match_id,
                       gtsam::Pose3* bodyCur_T_bodyRef) const;

  bool recoverPoseGivenRot(const FrameId& query_id, const FrameId& match_id,
                           const gtsam::Pose3& camCur_T_camRef_mono,
                           gtsam::Pose3* bodyCur_T_bodyRef) const;

 private:
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
  std::unordered_map<FrameId, Timestamp> ts_map_;

  // Store latest computed objects for temporal matching and nss scoring
  DBoW2::BowVector latest_bowvec_;
  MatchIsland latest_matched_island_;
  FrameId latest_query_id_;
  int temporal_entries_;

  // Store camera parameters and StereoFrame stuff once
  gtsam::Pose3 B_Pose_camLrect_;

  // Robust PGO members
  std::unique_ptr<RobustPGO::RobustSolver> pgo_;
  std::vector<gtsam::Pose3> W_Pose_Blkf_estimates_;
  gtsam::SharedNoiseModel shared_noise_model_; // TODO(marcus): make accurate
                                               // should also come in with input

  // LoopCLosureDetector Logger
  std::unique_ptr<LoopClosureDetectorLogger> logger_;
};  // class LoopClosureDetector

}  // namespace VIO
