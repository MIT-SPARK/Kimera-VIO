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

  /**
   * @brief Get depth (in meters) at a image point
   * @param[in] params Camera params for RGBD camera
   * @param[in] point Image point to extract depth for
   * @returns depth in meters or NaN if depth is not valid
   */
  float getDepthAtPoint(const CameraParams& params,
                        const KeypointCV& point) const;

  /**
   * @brief Get cv::Mat mask for invalid feature extraction regions
   * @param[in] params Camera params for RGBD camera
   * @returns cv::Mat with 255u values for image regions with valid depth
   */
  cv::Mat getDetectionMask(const CameraParams& params) const;

  /**
   * @brief Register depth to be in the same frame as the RGB image
   *
   * Note that this caches the registered image after the first call;
   * repeated calls will just return registered_img_;
   *
   * @param[in] params Camera params for RGBD camera
   */
  void registerDepth(const CameraParams& params) const;

 public:
  const FrameId id_;
  const cv::Mat depth_img_;
  mutable bool is_registered_;
  mutable cv::Mat registered_img_;
};

}  // namespace VIO
