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

#ifndef LoopClosureDetector_H_
#define LoopClosureDetector_H_

#include <string>

#include <boost/shared_ptr.hpp> // used for opengv
#include <gtsam/base/Matrix.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include "UtilsOpenCV.h"

#include <DBoW2/DBoW2.h>
#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>

#include "utils/ThreadsafeQueue.h"
#include "utils/Timer.h"
#include "utils/Statistics.h"
#include "StereoFrame.h"
#include "LoopClosureDetectorParams.h"
#include "LoopClosureDetector-definitions.h"

namespace VIO {

class LoopClosureDetector {
public:
  LoopClosureDetector(const LoopClosureDetectorParams& lcd_params,
      const bool log_output=false);

  virtual ~LoopClosureDetector() {
    LOG(INFO) << "LoopClosureDetector desctuctor called.";
  }

  bool spin(ThreadsafeQueue<LoopClosureDetectorInputPayload>& input_queue,
      ThreadsafeQueue<LoopClosureDetectorOutputPayload>& output_queue,
      bool parallel_run = true);

  LoopClosureDetectorOutputPayload spinOnce(
      const std::shared_ptr<LoopClosureDetectorInputPayload>& input);

  LoopClosureDetectorOutputPayload checkLoopClosure(StereoFrame& stereo_frame);

  FrameId processAndAddFrame(StereoFrame& stereo_frame);

  LoopResult detectLoop(size_t frame_id);

  bool geometricVerificationCheck(const FrameId& query_id,
      const FrameId& match_id, LoopResult& result);

  gtsam::Pose3 compute3DPose(const FrameId& query_id, const FrameId& match_id,
      gtsam::Pose3& pose_estimate);

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

  inline LoopClosureDetectorParams& getLCDParams() { return lcd_params_; }

  inline LoopClosureDetectorParams* getLCDParamsMutable() {
    return &lcd_params_;
  }

  inline const OrbVocabulary& getVocabulary() const { return vocab_; }

  inline const OrbDatabase* getBoWDatabase() const { return &db_BoW_; }

  inline const std::vector<LCDFrame>* getFrameDatabase() const {
    return &db_frames_;
  }

  inline bool getIntrinsicsFlag() const { return set_intrinsics_; }

  // TODO: maybe this should be private. But that makes testing harder.
  void setIntrinsics(const StereoFrame& stereo_frame);

  void setDatabase(const OrbDatabase& db);

  void setVocabulary(const OrbVocabulary& vocab);

  void allocate(size_t n);

  inline void clear();

  void print() const;

  // TODO: utils and reorder (or just static)
  void rewriteStereoFrameFeatures(StereoFrame& stereo_frame,
    const std::vector<cv::KeyPoint>& keypoints);

  // TODO: it would be nice if this could be a util
  // TODO: can this be static even though it requires id? Maybe feed it descriptors instead
  cv::Mat computeAndDrawMatchesBetweenFrames(const cv::Mat& query_img,
    const cv::Mat& match_img, const size_t& query_id, const size_t& match_id,
    bool cut_matches=false);

  static gtsam::Pose3 mat2pose(const cv::Mat& R, const cv::Mat& t);

  // TODO: maybe these should be private?
  void transformCameraPose2BodyPose(gtsam::Pose3& camRef_T_camCur,
      gtsam::Pose3* bodyRef_T_bodyCur) const;

  void transformBodyPose2CameraPose(gtsam::Pose3& bodyRef_T_bodyCur,
      gtsam::Pose3* camRef_T_camCur) const;

private:
  bool checkTemporalConstraint(const FrameId& id, const MatchIsland& island);

  void computeIslands(DBoW2::QueryResults& q,
      std::vector<MatchIsland>& islands) const;

  double computeIslandScore(const DBoW2::QueryResults& q,
      const FrameId& start_id, const FrameId& end_id) const;

  void computeMatchedIndices(const FrameId& query_id, const FrameId& match_id,
      std::vector<unsigned int>& i_query, std::vector<unsigned int>& i_match,
      bool cut_matches=false);

  bool geometricVerification_cv(const FrameId& query_id,
      const FrameId& match_id, LoopResult& result);

  bool geometricVerification_gv(const FrameId& query_id,
      const FrameId& match_id, LoopResult& result);

  gtsam::Pose3 compute3DPose3pt(const FrameId& query_id,
      const FrameId& match_id);

  gtsam::Pose3 compute3DPoseGiven2D(const FrameId& query_id,
      const FrameId& match_id, gtsam::Pose3& pose_2d);

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
  OrbDatabase db_BoW_;
  std::vector<LCDFrame> db_frames_;
  OrbVocabulary vocab_;

  // Store latest computed objects for temporal matching and nss scoring
  DBoW2::BowVector latest_bowvec_;
  MatchIsland latest_matched_island_;
  FrameId latest_query_id_;
  int temporal_entries_;

  // Store camera parameters and StereoFrame stuff once
  gtsam::Pose3 B_Pose_camLrect_;


}; // class LoopClosureDetector

} // namespace VIO
#endif /* LoopClosureDetector_H_ */
