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
 */

 #pragma once

#include "common/vio_types.h"

namespace VIO {

struct LoopClosureDetectorInputPayload {
  LoopClosureDetectorInputPayload(const Timestamp& timestamp_kf,
                                  const StereoFrame stereo_frame)
    : timestamp_kf_(timestamp_kf),
      stereo_frame_(stereo_frame) {}

  const Timestamp timestamp_kf_;
  const StereoFrame stereo_frame_;

}; // struct LoopClosureInputPayload

// TODO: Add transform between the two frames
struct LoopClosureDetectorOutputPayload {
  LoopClosureDetectorOutputPayload(bool is_loop,
                                   const Timestamp& timestamp_kf,
                                   const FrameId& id_recent,
                                   const FrameId& id_match,
                                   const cv::Mat& translation,
                                   const cv::Mat& rotation)
    : is_loop_(is_loop),
      timestamp_kf_(timestamp_kf),
      id_recent_(id_recent),
      id_match_(id_match),
      translation_(translation),
      rotation_(rotation) {}

  // TODO: inlude score of match
  const bool is_loop_;
  const Timestamp timestamp_kf_;
  const FrameId id_recent_;
  const FrameId id_match_;
  const cv::Mat translation_; // TODO: should be gtsam::Pose3
  const cv::Mat rotation_; // TODO: should be packaged into gtsam::Pose3 above
}; // struct LoopClosureOutputPayload

} // namespace VIO
