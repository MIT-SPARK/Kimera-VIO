/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DepthFrame.cpp
 * @brief  Class describing a single Depth image
 * @author Antoni Rosinol
 */
#include "kimera-vio/frontend/DepthFrame.h"

#include <opencv2/core/core.hpp>
#include <opencv2/rgbd.hpp>

#include "kimera-vio/frontend/CameraParams.h"

namespace VIO {

DepthFrame::DepthFrame(const FrameId& id,
                       const Timestamp& timestamp,
                       const cv::Mat& depth_img)
    : PipelinePayload(timestamp),
      id_(id),
      depth_img_(depth_img),
      is_registered_(false) {
  CHECK(depth_img_.type() == CV_32FC1 || depth_img_.type() == CV_16UC1);
}

DepthFrame::DepthFrame(const DepthFrame& other)
    : PipelinePayload(other.timestamp_),
      id_(other.id_),
      depth_img_(other.depth_img_),
      is_registered_(other.is_registered_),
      registered_img_(other.registered_img_) {}

float DepthFrame::getDepthAtPoint(const CameraParams& params,
                                  const KeypointCV& point) const {
  const auto x = static_cast<int>(point.x);
  const auto y = static_cast<int>(point.y);

  float depth = std::numeric_limits<float>::quiet_NaN();
  if (x < 0 || x >= depth_img_.cols || y < 0 || y >= depth_img_.rows) {
    VLOG(10) << "Found feature (" << point.x << ", " << point.y
             << ") outside image bounds: [" << depth_img_.cols << " x "
             << depth_img_.rows << "]";
    return depth;
  }

  const cv::Mat& img = is_registered_ ? registered_img_ : depth_img_;
  switch (depth_img_.type()) {
    case CV_32FC1:
      depth = img.at<float>(y, x);
      break;
    case CV_16UC1:
      depth = img.at<uint16_t>(y, x);
      break;
    default:
      LOG(FATAL) << "Invalid depth datatype: " << depth_img_.type();
      return depth;
  }

  depth *= params.depth.depth_to_meters_;
  if (depth < params.depth.min_depth_) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  // TODO(nathan) optionally filter by max depth as well

  return depth;
}

cv::Mat DepthFrame::getDetectionMask(const CameraParams& params) const {
  float min = params.depth.min_depth_ * 1.0f / params.depth.depth_to_meters_;
  float max = params.depth.max_depth_ * 1.0f / params.depth.depth_to_meters_;
  const cv::Mat& img_to_use = is_registered_ ? registered_img_ : depth_img_;
  cv::Mat mask;
  switch (depth_img_.type()) {
    case CV_32FC1:
      cv::inRange(img_to_use, min, max, mask);
      break;
    case CV_16UC1:
      cv::inRange(img_to_use,
                  static_cast<uint16_t>(min),
                  static_cast<uint16_t>(max),
                  mask);
      break;
    default:
      LOG(FATAL) << "Invalid depth datatype: " << img_to_use.type();
      break;
  }

  return mask;
}

void DepthFrame::registerDepth(const CameraParams& params) const {
  if (is_registered_) {
    return;
  }

  cv::rgbd::registerDepth(params.depth.K_,
                          params.K_,
                          params.distortion_coeff_mat_,
                          params.depth.T_color_depth_,
                          depth_img_,
                          params.image_size_,
                          registered_img_);
  is_registered_ = true;
}

}  // namespace VIO
