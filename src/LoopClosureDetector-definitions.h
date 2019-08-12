/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoopClosureDetector-definitions.h
 * @brief  Definitions for LoopClosureDetector
 * @author Marcus Abate, Luca Carlone
 */

#pragma once

#include <vector>

#include "common/vio_types.h"
#include "StereoFrame.h"

#include <DBoW2/DBoW2.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/NoiseModel.h>

namespace VIO {

enum LCDStatus {
  LOOP_DETECTED,
  NO_MATCHES,
  LOW_NSS_FACTOR,
  LOW_SCORE,
  NO_GROUPS,
  FAILED_TEMPORAL_CONSTRAINT,
  FAILED_GEOM_VERIFICATION,
  FAILED_POSE_RECOVERY
};

enum GeomVerifOption {
  NISTER,
  NONE
};

enum PoseRecoveryOption { RANSAC_ARUN, GIVEN_ROT };

struct LCDFrame {
  LCDFrame() {}
  LCDFrame(const Timestamp& timestamp,
           const FrameId& id,
           const FrameId& id_kf,
           const std::vector<cv::KeyPoint>& keypoints,
           const std::vector<gtsam::Vector3>& keypoints_3d,
           const std::vector<cv::Mat>& descriptors_vec,
           const cv::Mat& descriptors_mat,
           const BearingVectors& versors)
    : timestamp_(timestamp),
      id_(id),
      id_kf_(id_kf),
      keypoints_(keypoints),
      keypoints_3d_(keypoints_3d),
      descriptors_vec_(descriptors_vec),
      descriptors_mat_(descriptors_mat),
      versors_(versors) {}

  Timestamp timestamp_;
  FrameId id_;
  FrameId id_kf_;
  std::vector<cv::KeyPoint> keypoints_;
  std::vector<gtsam::Vector3> keypoints_3d_;
  std::vector<cv::Mat> descriptors_vec_;
  cv::Mat descriptors_mat_;
  BearingVectors versors_;
};  // struct LCDFrame

struct MatchIsland {
  MatchIsland()
    : start_id_(0),
      end_id_(0),
      island_score_(0),
      best_id_(0),
      best_score_(0) {}

  MatchIsland(FrameId start, FrameId end)
    : start_id_(start),
      end_id_(end),
      island_score_(0),
      best_id_(0),
      best_score_(0) {}

  MatchIsland(FrameId start, FrameId end, double score)
    : start_id_(start),
      end_id_(end),
      island_score_(score),
      best_id_(0),
      best_score_(0) {}

  inline bool operator < (const MatchIsland& other) const {
    return this->island_score_ < other.island_score_;
  }

  inline bool operator > (const MatchIsland& other) const {
    return this->island_score_ > other.island_score_;
  }

  inline size_t size() const { return end_id_ - start_id_ + 1; }

  inline void clear() {
    start_id_ = 0;
    end_id_ = 0;
    island_score_ = 0;
    best_id_ = 0;
    best_score_ = 0;
  }

  FrameId start_id_;
  FrameId end_id_;
  double island_score_;
  FrameId best_id_;
  double best_score_;
};  // struct MatchIsland

struct LoopResult {
  inline bool isLoop() const { return status_ == LCDStatus::LOOP_DETECTED; }

  LCDStatus status_;
  FrameId query_id_;
  FrameId match_id_;
  gtsam::Pose3 relative_pose_;
};  // struct LoopResult

struct VioFactor {
  VioFactor(const FrameId& cur_key,
            const gtsam::Pose3& W_Pose_Blkf,
            const gtsam::SharedNoiseModel& noise)
      : cur_key_(cur_key),
      W_Pose_Blkf_(W_Pose_Blkf),
      noise_(noise) {}

  const FrameId cur_key_;
  const gtsam::Pose3 W_Pose_Blkf_;
  const gtsam::SharedNoiseModel noise_;
}; // struct VioFactor

struct LoopClosureFactor {
  LoopClosureFactor(const FrameId& ref_key,
                    const FrameId& cur_key,
                    const gtsam::Pose3& ref_Pose_cur,
                    const gtsam::SharedNoiseModel& noise)
      : ref_key_(ref_key),
        cur_key_(cur_key),
        ref_Pose_cur_(ref_Pose_cur),
        noise_(noise) {}

  const FrameId ref_key_;
  const FrameId cur_key_;
  const gtsam::Pose3 ref_Pose_cur_;
  const gtsam::SharedNoiseModel noise_;
}; // struct LoopClosureFactor

struct LoopClosureDetectorInputPayload {
  LoopClosureDetectorInputPayload(const Timestamp& timestamp_kf,
                                  const FrameId& cur_kf_id,
                                  const StereoFrame& stereo_frame,
                                  const gtsam::Pose3& W_Pose_Blkf)
      : timestamp_kf_(timestamp_kf),
        cur_kf_id_(cur_kf_id),
        stereo_frame_(stereo_frame),
        W_Pose_Blkf_(W_Pose_Blkf) {}

  const Timestamp timestamp_kf_;
  const FrameId cur_kf_id_;
  const StereoFrame stereo_frame_;
  const gtsam::Pose3 W_Pose_Blkf_;
};  // struct LoopClosureDetectorInputPayload

struct LoopClosureDetectorOutputPayload {
  LoopClosureDetectorOutputPayload(bool is_loop_closure,
                                   const Timestamp& timestamp_kf,
                                   const FrameId& id_match,
                                   const FrameId& id_recent,
                                   const gtsam::Pose3& relative_pose,
                                   const gtsam::Pose3& W_Pose_Map,
                                   const gtsam::Values& states)
      : is_loop_closure_(is_loop_closure),
        timestamp_kf_(timestamp_kf),
        id_match_(id_match),
        id_recent_(id_recent),
        relative_pose_(relative_pose),
        W_Pose_Map_(W_Pose_Map),
        states_(states) {}

  // TODO(marcus): inlude stats/score of match
  const bool is_loop_closure_;
  const Timestamp timestamp_kf_;
  const FrameId id_match_;
  const FrameId id_recent_;
  const gtsam::Pose3 relative_pose_;
  const gtsam::Pose3 W_Pose_Map_;
  const gtsam::Values states_;
};  // struct LoopClosureDetectorOutputPayload

}  // namespace VIO
