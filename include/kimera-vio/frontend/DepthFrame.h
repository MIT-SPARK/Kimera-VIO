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

#include "kimera-vio/pipeline/PipelinePayload.h"

namespace VIO {

// foward declare to avoid header includes
struct CameraParams;
// TODO(nathan) forward declare opencv types if they get removed from vio_types

class DepthFrame : public PipelinePayload {
 public:
  // KIMERA_DELETE_COPY_CONSTRUCTORS(DepthFrame);
  KIMERA_POINTER_TYPEDEFS(DepthFrame);

  DepthFrame(const FrameId& id,
             const Timestamp& timestamp,
             const cv::Mat& depth_img);

  DepthFrame(const DepthFrame& other);

  float getDepthAtPoint(const KeypointCV& point) const;

  cv::Mat getDetectionMask(const CameraParams& params) const;

  void registerDepth(const CameraParams& params) const;

 public:
  const FrameId id_;
  const cv::Mat depth_img_;
  mutable bool is_registered_;
  mutable cv::Mat registered_img_;
};

}  // namespace VIO
