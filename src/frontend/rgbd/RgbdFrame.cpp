/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdFrame.cpp
 * @brief  Class describing a single image
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/rgbd/RgbdFrame.h"

namespace VIO {

RgbdFrame::RgbdFrame(const FrameId& id,
                     const Timestamp& timestamp,
                     Frame::UniquePtr intensity_img,
                     DepthFrame::UniquePtr depth_img)
    : PipelinePayload(timestamp),
      id_(id),
      intensity_img_(std::move(intensity_img)),
      depth_img_(std::move(depth_img)) {
  CHECK(intensity_img_);
  CHECK(depth_img_);
  CHECK_EQ(intensity_img_->img_.type(), CV_8UC1)
      << "The provided left image is not grayscale...";
  CHECK(depth_img_->depth_img_.type() == CV_16UC1 ||
        depth_img_->depth_img_.type() == CV_32FC1)
      << "The provided depth image is not in the expected format...";
  CHECK_EQ(intensity_img_->img_.size, depth_img_->depth_img_.size);
}

}  // namespace VIO
