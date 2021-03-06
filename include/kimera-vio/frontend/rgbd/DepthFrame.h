/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DepthFrame.h
 * @brief  Class describing a single Depth image
 * @author Antoni Rosinol
 */

#pragma once

#include <opencv2/core/core.hpp>

#include "kimera-vio/pipeline/PipelinePayload.h"

namespace VIO {

class DepthFrame : public PipelinePayload {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(DepthFrame);
  KIMERA_POINTER_TYPEDEFS(DepthFrame);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DepthFrame(const FrameId& id,
             const Timestamp& timestamp,
             const cv::Mat& depth_img)
      : PipelinePayload(timestamp), id_(id), depth_img_(depth_img) {
    CHECK(depth_img_.type() == CV_32FC1 || depth_img_.type() == CV_16UC1);
  }

 public:
  const FrameId id_;
  const cv::Mat depth_img_;
};

}  // namespace VIO
