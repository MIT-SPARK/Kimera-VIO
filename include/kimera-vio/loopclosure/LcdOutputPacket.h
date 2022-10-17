/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LcdOutputPacket.h
 * @brief  Loop closure output packet
 * @author Marcus Abate
 * @author Antoni Rosinol
 * @author Luca Carlone
 * @author Nathan Hughes
 * @author Yun Chang
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/pipeline/PipelinePayload.h"
#include "kimera-vio/utils/Macros.h"

namespace DBoW2 {
class BowVector;  // forward declare to avoid public dbow dependency
}

namespace VIO {

typedef std::unordered_map<FrameId, Timestamp> FrameIDTimestampMap;

struct LcdOutput : PipelinePayload {
  KIMERA_POINTER_TYPEDEFS(LcdOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LcdOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LcdOutput(bool is_loop_closure,
            const Timestamp& timestamp_kf,
            const Timestamp& timestamp_query,
            const Timestamp& timestamp_match,
            const FrameId& id_match,
            const FrameId& id_recent,
            const gtsam::Pose3& relative_pose);

  explicit LcdOutput(const Timestamp& timestamp_kf);

  void setMapInformation(const gtsam::Pose3& W_Pose_Map,
                         const gtsam::Pose3& Map_Pose_Odom,
                         const gtsam::Values& states,
                         const gtsam::NonlinearFactorGraph& nfg);

  void setFrameInformation(const Landmarks& keypoints_3d,
                           const BearingVectors& versors,
                           const DBoW2::BowVector& bow_vec,
                           const cv::Mat& descriptors_mat);

  // TODO(marcus): inlude stats/score of match
  bool is_loop_closure_;
  Timestamp timestamp_query_;
  Timestamp timestamp_match_;
  FrameId id_match_;
  FrameId id_recent_;
  gtsam::Pose3 relative_pose_;
  // map information
  gtsam::Pose3 W_Pose_Map_;
  gtsam::Pose3 Map_Pose_Odom_;  // Map frame is the optimal (RPGO) global frame
                                // and odom is the VIO estimate global frame
  gtsam::Values states_;
  gtsam::NonlinearFactorGraph nfg_;
  // frame information
  Landmarks keypoints_3d_;
  BearingVectors versors_;
  std::map<int, double> bow_vec_;
  cv::Mat descriptors_mat_;
  FrameIDTimestampMap timestamp_map_;
};

}  // namespace VIO
