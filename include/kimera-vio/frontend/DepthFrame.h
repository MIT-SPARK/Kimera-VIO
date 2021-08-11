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

#include "kimera-vio/frontend/DepthCameraParams.h"
#include "kimera-vio/pipeline/PipelinePayload.h"

namespace VIO {

class DepthFrame : public PipelinePayload {
 public:
  // KIMERA_DELETE_COPY_CONSTRUCTORS(DepthFrame);
  KIMERA_POINTER_TYPEDEFS(DepthFrame);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DepthFrame(const FrameId& id,
             const Timestamp& timestamp,
             const cv::Mat& depth_img);

  DepthFrame(const DepthFrame& other);

  inline float getDepthAtPoint(const KeypointCV& point) const {
    const auto x = static_cast<int>(point.x);
    const auto y = static_cast<int>(point.y);
    // lk tracking produces keypoints outside of the image...
    if (x < 0 || x >= depth_img_.cols || y < 0 || y >= depth_img_.rows) {
      VLOG(5) << "Found feature (" << point.x << ", " << point.y
              << " outside image bounds: [" << depth_img_.cols << " x "
              << depth_img_.rows << "]";
      return std::numeric_limits<float>::quiet_NaN();
    }

    const cv::Mat& img = is_registered_ ? registered_img_ : depth_img_;
    switch (depth_img_.type()) {
      case CV_32FC1:
        return img.at<float>(y, x);
      case CV_16UC1:
        return img.at<uint16_t>(y, x);
      default:
        LOG(FATAL) << "Invalid depth datatype: " << depth_img_.type();
        return std::numeric_limits<float>::quiet_NaN();
    }
  }

  cv::Mat getDetectionMask(const DepthCameraParams& params) const;

  void registerDepth(const cv::Mat& depth_camera_matrix,
                     const cv::Mat& color_camera_matrix,
                     const cv::Mat& color_distortion_params,
                     const gtsam::Pose3& T_color_depth,
                     const cv::Size& color_size) const;

 public:
  const FrameId id_;
  const cv::Mat depth_img_;
  mutable bool is_registered_;
  mutable cv::Mat registered_img_;
};

}  // namespace VIO
