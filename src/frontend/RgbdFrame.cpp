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

#include "kimera-vio/frontend/RgbdFrame.h"

#include "kimera-vio/frontend/RgbdCamera.h"

namespace VIO {

RgbdFrame::RgbdFrame(const FrameId& id,
                     const Timestamp& timestamp,
                     const Frame& intensity_img,
                     const DepthFrame& depth_img)
    : PipelinePayload(timestamp),
      id_(id),
      intensity_img_(intensity_img),
      depth_img_(depth_img) {
  CHECK_EQ(intensity_img_.img_.type(), CV_8UC1)
      << "The provided left image is not grayscale...";
  CHECK(depth_img_.depth_img_.type() == CV_16UC1 ||
        depth_img_.depth_img_.type() == CV_32FC1)
      << "The provided depth image is not in the expected format...";
  CHECK_EQ(intensity_img_.img_.size, depth_img_.depth_img_.size);
}

RgbdFrame::RgbdFrame(const RgbdFrame& other)
    : PipelinePayload(other.timestamp_),
      id_(other.id_),
      intensity_img_(other.intensity_img_),
      depth_img_(other.depth_img_) {}

StereoFrame::Ptr RgbdFrame::getStereoFrame() const {
  Frame rframe(id_, timestamp_, intensity_img_.cam_param_, cv::Mat());
  return std::make_shared<StereoFrame>(id_, timestamp_, intensity_img_, rframe);
}

void RgbdFrame::fillStereoFrame(const RgbdCamera& camera,
                                StereoFrame& stereo_frame) const {
  const auto& params = camera.getCamParams();
  if (!params.depth.is_registered_) {
    depth_img_.registerDepth(params);
  }

  if (stereo_frame.left_frame_.keypoints_.size() >
      stereo_frame.left_keypoints_rectified_.size()) {
    camera.undistortKeypoints(stereo_frame.left_frame_.keypoints_,
                              &stereo_frame.left_keypoints_rectified_);
  }

  const auto& left_keypoints_rect = stereo_frame.left_keypoints_rectified_;
  auto& right_keypoints_rect = stereo_frame.right_keypoints_rectified_;
  auto& keypoint_depths = stereo_frame.keypoints_depth_;

  const double fx_b = params.intrinsics_[0] * params.depth.virtual_baseline_;
  const KeypointCV null_point(0.0, 0.0);

  // we could try and track matches / removed features, but this is easier
  // and likely only marginally less efficient
  right_keypoints_rect.clear();
  keypoint_depths.clear();
  stereo_frame.keypoints_3d_.clear();
  right_keypoints_rect.reserve(left_keypoints_rect.size());
  keypoint_depths.reserve(left_keypoints_rect.size());
  stereo_frame.keypoints_3d_.reserve(left_keypoints_rect.size());

  for (size_t i = 0; i < left_keypoints_rect.size(); ++i) {
    const auto& status_keypoint_pair = left_keypoints_rect[i];
    if (status_keypoint_pair.first != KeypointStatus::VALID) {
      right_keypoints_rect.push_back({status_keypoint_pair.first, null_point});
      keypoint_depths.push_back(0.0);
      stereo_frame.keypoints_3d_.push_back(Vector3::Zero());
      continue;
    }

    const float keypoint_depth = depth_img_.getDepthAtPoint(
        params, stereo_frame.left_frame_.keypoints_[i]);

    if (!std::isfinite(keypoint_depth)) {
      right_keypoints_rect.push_back({KeypointStatus::NO_DEPTH, null_point});
      keypoint_depths.push_back(0.0);
      stereo_frame.keypoints_3d_.push_back(Vector3::Zero());
      continue;
    }

    const float disparity = fx_b / keypoint_depth;
    const float uR = status_keypoint_pair.second.x - disparity;
    if (uR < 0.0f) {
      right_keypoints_rect.push_back({KeypointStatus::NO_DEPTH, null_point});
      keypoint_depths.push_back(0.0);
      stereo_frame.keypoints_3d_.push_back(Vector3::Zero());
      continue;
    }

    const Vector3& versor = stereo_frame.left_frame_.versors_[i];
    const KeypointCV right_point(uR, status_keypoint_pair.second.y);
    right_keypoints_rect.push_back({KeypointStatus::VALID, right_point});
    keypoint_depths.push_back(keypoint_depth);
    stereo_frame.keypoints_3d_.push_back(versor * keypoint_depth / versor(2));
  }

  // TODO(nathan) consider dropping so we can use a generic camera
  camera.distortKeypoints(right_keypoints_rect,
                          &(stereo_frame.right_frame_.keypoints_));
}

}  // namespace VIO
