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

#include <opencv2/core/eigen.hpp>
#include <opencv2/rgbd.hpp>

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

cv::Mat DepthFrame::getDetectionMask(const DepthCameraParams& params) const {
  float min = params.min_depth_ * 1.0f / params.depth_to_meters_;
  float max = params.max_depth_ * 1.0f / params.depth_to_meters_;
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

void DepthFrame::registerDepth(const cv::Mat& depth_camera_matrix,
                               const cv::Mat& color_camera_matrix,
                               const cv::Mat& color_distortion_params,
                               const gtsam::Pose3& T_color_depth,
                               const cv::Size& color_size) const {
  if (is_registered_) {
    return;
  }

  cv::Mat Tcv_color_depth(4, 4, CV_64F);
  cv::eigen2cv(T_color_depth.matrix(), Tcv_color_depth);

  cv::rgbd::registerDepth(depth_camera_matrix,
                          color_camera_matrix,
                          color_distortion_params,
                          Tcv_color_depth,
                          depth_img_,
                          color_size,
                          registered_img_);
  is_registered_ = true;
}

}  // namespace VIO
