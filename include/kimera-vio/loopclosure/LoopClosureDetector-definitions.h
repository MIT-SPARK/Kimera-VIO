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
 * @author Marcus Abate
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/FrontendOutputPacketBase.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

typedef cv::Mat OrbDescriptor;
typedef std::vector<OrbDescriptor> OrbDescriptorVec;

enum class LoopClosureDetectorType {
  BoW = 0u,  //! Bag of Words approach
};

enum class LCDStatus : int {
  LOOP_DETECTED,
  NO_MATCHES,
  LOW_NSS_FACTOR,
  LOW_SCORE,
  NO_GROUPS,
  FAILED_TEMPORAL_CONSTRAINT,
  FAILED_GEOM_VERIFICATION,
  FAILED_POSE_RECOVERY
};

struct LCDFrame {
  KIMERA_POINTER_TYPEDEFS(LCDFrame);
  LCDFrame() = default;

  LCDFrame(const Timestamp& timestamp,
           const FrameId& id,
           const FrameId& id_kf,
           const std::vector<cv::KeyPoint>& keypoints,
           const Landmarks& keypoints_3d,
           const OrbDescriptorVec& descriptors_vec,
           const OrbDescriptor& descriptors_mat,
           const BearingVectors& bearing_vectors)
      : timestamp_(timestamp),
        id_(id),
        id_kf_(id_kf),
        keypoints_(keypoints),
        keypoints_3d_(keypoints_3d),
        descriptors_vec_(descriptors_vec),
        descriptors_mat_(descriptors_mat),
        bearing_vectors_(bearing_vectors) {}

  virtual ~LCDFrame() = default;

  virtual void save(std::ostream& buffer) const;

  static LCDFrame::Ptr load(std::istream& buffer);

  Timestamp timestamp_;
  FrameId id_;
  FrameId id_kf_;
  std::vector<cv::KeyPoint> keypoints_;
  Landmarks keypoints_3d_;
  OrbDescriptorVec descriptors_vec_;
  OrbDescriptor descriptors_mat_;
  BearingVectors bearing_vectors_;

 protected:
  virtual void saveBytes(std::ostream& buffer) const;

  virtual void loadBytes(std::istream& buffer);
};

struct StereoLCDFrame : LCDFrame {
  KIMERA_POINTER_TYPEDEFS(StereoLCDFrame);
  StereoLCDFrame() = default;

  StereoLCDFrame(const Timestamp& timestamp,
                 const FrameId& id,
                 const FrameId& id_kf,
                 const std::vector<cv::KeyPoint>& keypoints,
                 const Landmarks& keypoints_3d,
                 const OrbDescriptorVec& descriptors_vec,
                 const OrbDescriptor& descriptors_mat,
                 const BearingVectors& bearing_vectors,
                 const StatusKeypointsCV& left_keypoints_rectified,
                 const StatusKeypointsCV& right_keypoints_rectified)
      : LCDFrame(timestamp,
                 id,
                 id_kf,
                 keypoints,
                 keypoints_3d,
                 descriptors_vec,
                 descriptors_mat,
                 bearing_vectors),
        left_keypoints_rectified_(left_keypoints_rectified),
        right_keypoints_rectified_(right_keypoints_rectified) {}

  virtual ~StereoLCDFrame() = default;

  void save(std::ostream& buffer) const override;

  StatusKeypointsCV left_keypoints_rectified_;
  StatusKeypointsCV right_keypoints_rectified_;

 protected:
  void saveBytes(std::ostream& buffer) const override;

  void loadBytes(std::istream& buffer) override;
};

struct MatchIsland {
  MatchIsland()
      : start_id_(0),
        end_id_(0),
        island_score_(0),
        best_id_(0),
        best_score_(0) {}

  MatchIsland(const FrameId& start, const FrameId& end)
      : start_id_(start),
        end_id_(end),
        island_score_(0),
        best_id_(0),
        best_score_(0) {}

  MatchIsland(const FrameId& start, const FrameId& end, const double& score)
      : start_id_(start),
        end_id_(end),
        island_score_(score),
        best_id_(0),
        best_score_(0) {}

  inline bool operator<(const MatchIsland& other) const {
    return island_score_ < other.island_score_;
  }

  inline bool operator>(const MatchIsland& other) const {
    return island_score_ > other.island_score_;
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

  static std::string asString(const LCDStatus& status) {
    std::string status_str = "";
    switch (status) {
      case LCDStatus::LOOP_DETECTED: {
        status_str = "LOOP_DETECTED";
        break;
      }
      case LCDStatus::NO_MATCHES: {
        status_str = "NO_MATCHES";
        break;
      }
      case LCDStatus::LOW_NSS_FACTOR: {
        status_str = "LOW_NSS_FACTOR";
        break;
      }
      case LCDStatus::LOW_SCORE: {
        status_str = "LOW_SCORE";
        break;
      }
      case LCDStatus::NO_GROUPS: {
        status_str = "NO_GROUPS";
        break;
      }
      case LCDStatus::FAILED_TEMPORAL_CONSTRAINT: {
        status_str = "FAILED_TEMPORAL_CONSTRAINT";
        break;
      }
      case LCDStatus::FAILED_GEOM_VERIFICATION: {
        status_str = "FAILED_GEOM_VERIFICATION";
        break;
      }
      case LCDStatus::FAILED_POSE_RECOVERY: {
        status_str = "FAILED_POSE_RECOVERY";
        break;
      }
    }
    return status_str;
  }

  LCDStatus status_ = LCDStatus::NO_MATCHES;
  FrameId query_id_;
  FrameId match_id_;
  gtsam::Pose3 relative_pose_;
};  // struct LoopResult

struct LcdDebugInfo {
  LcdDebugInfo() = default;

  Timestamp timestamp_;
  LoopResult loop_result_;

  size_t mono_input_size_;
  size_t mono_inliers_;
  int mono_iter_;

  size_t stereo_input_size_;
  size_t stereo_inliers_;
  int stereo_iter_;

  size_t pgo_size_;
  size_t pgo_lc_count_;
  size_t pgo_lc_inliers_;
};  // struct LcdDebugInfo

struct OdometryFactor {
  OdometryFactor(const FrameId& cur_key,
                 const gtsam::Pose3& W_Pose_Blkf,
                 const gtsam::SharedNoiseModel& noise)
      : cur_key_(cur_key), W_Pose_Blkf_(W_Pose_Blkf), noise_(noise) {}

  const FrameId cur_key_;
  const gtsam::Pose3 W_Pose_Blkf_;
  const gtsam::SharedNoiseModel noise_;
};  // struct OdometryFactor

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
};  // struct LoopClosureFactor

struct LcdInput : public PipelinePayload {
  KIMERA_POINTER_TYPEDEFS(LcdInput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LcdInput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LcdInput(const Timestamp& timestamp,
           const FrontendOutputPacketBase::Ptr& frontend_output,
           const FrameId& cur_kf_id,
           const PointsWithIdMap& W_points_with_ids,
           const gtsam::Pose3& W_Pose_Blkf)
      : PipelinePayload(timestamp),
        frontend_output_(frontend_output),
        cur_kf_id_(cur_kf_id),
        W_points_with_ids_(W_points_with_ids),
        W_Pose_Blkf_(W_Pose_Blkf) {
    CHECK(frontend_output);
    CHECK_EQ(timestamp, frontend_output->timestamp_);
  }

  const FrontendOutputPacketBase::Ptr frontend_output_;
  const FrameId cur_kf_id_;
  const PointsWithIdMap W_points_with_ids_;
  const gtsam::Pose3 W_Pose_Blkf_;
};

}  // namespace VIO
