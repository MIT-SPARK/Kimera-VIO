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

namespace VIO {

RgbdFrame::RgbdFrame(const FrameId& id,
                     const Timestamp& timestamp,
                     Frame::UniquePtr intensity_img,
                     DepthFrame::UniquePtr depth_img)
    : is_keyframe_(false),
      id_(id),
      timestamp_(timestamp),
      intensity_img_(std::move(intensity_img)),
      depth_img_(std::move(depth_img)) {
  CHECK(intensity_img_);
  CHECK(depth_img_);
  CHECK_EQ(intensity_img_->img_.type(), CV_8UC1)
      << "The provided left image is not grayscale...";
  CHECK(depth_img_->depth_img_.type() == CV_16UC1 ||
        depth_img_->depth_img_.type() == CV_32FC1)
      << "The provided depth image is not in the expected format...";
  CHECK_EQ(id_, intensity_img_.id_);
  CHECK_EQ(id_, depth_img_.id_);
  CHECK_EQ(timestamp_, intensity_img_.timestamp_);
  CHECK_EQ(intensity_img_->img_.size, depth_img_->depth_img_.size);
}

// RgbdFrame::RgbdFrame(const RgbdFrame& other_frame)
//     : is_keyframe_(other_frame.is_keyframe_),
//       id_(other_frame.id_),
//       timestamp_(other_frame.timestamp_)
//  {
//   intensity_img_ = std::move(other_frame.intensity_img_);
//   depth_img_ = std::move(other_frame.depth_img_);
//   CHECK(intensity_img_);
//   CHECK(depth_img_);
//   CHECK_EQ(intensity_img_.img_.type(), CV_8UC1)
//       << "The provided left image is not grayscale...";
//   CHECK(depth_img_.depth_img_.type() == CV_16UC1 ||
//         depth_img_.depth_img_.type() == CV_32FC1)
//       << "The provided depth image is not in the expected format...";
//   CHECK_EQ(id_, intensity_img_.id_);
//   CHECK_EQ(id_, depth_img_.id_);
//   CHECK_EQ(timestamp_, intensity_img_.timestamp_);
//   CHECK_EQ(intensity_img_.img_.size, depth_img_.depth_img_.size);
// }

void RgbdFrame::calculate3dKeypoints(){
  CHECK_GT(intensity_img_->versors_.size(), 0u)
      << "Versors are empty to calculate the 3D keypoints...";

  CHECK(depth_img_->depth_img_.type() == CV_16UC1 ||
        depth_img_->depth_img_.type() == CV_32FC1)

  for (size_t i = 0; i < intensity_img_->versors_.size(); ++i) {
    if (intensity_img_->keypoints_undistorted_[i].first == 
          KeypointStatus::VALID) {
        // NOTE: versors are already in the rectified frame.
        Vector3 versor = intensity_img_->versors_[i];
        CHECK_GE(versor(2), 1e-3)
            << "calculate3dKeypoints: found point with nonpositive depth!";
        KeypointCV index = intensity_img_->keypoints_[i];
        Vector3 keypoint_3d;
        // DepthMap is not the norm of the vector, it is the z component.
        if (depth_img_->depth_img_.at<uint16_t>(std::round(index.y), std::round(index.x)) != 0){
          if (depth_img_->depth_img_.type() == CV_16UC1) {
            keypoint_3d = versor * depth_img_->depth_img_.at<uint16_t>(std::round(index.y), std::round(index.x)) / (versor(2) * 1000);
          }
          else{
            keypoint_3d = versor * depth_img_->depth_img_.at<uint16_t>(std::round(index.y), std::round(index.x)) / versor(2);
          }
          keypoints_3d_.push_back(keypoint_3d);
        }
        else{
          // TODO(Saching): Maybe move this to tracker.cpp so that we can use the depth threshold too ? 
          // We could also add a depth threshold on-device so maybe not ?
          intensity_img_->keypoints_undistorted_[i].first = KeypointStatus::NO_DEPTH;
          keypoints_3d_.push_back(Vector3::Zero());
        }
    } else {
        keypoints_3d_.push_back(Vector3::Zero());
    }
  }
}

void RgbdFrame::checkRgbdFrame() const{
  intensity_img_->checkFrame();
  const size_t nrKeypoints = intensity_img_->keypoints_.size();
  CHECK_EQ(keypoints_3d_.size(), nrKeypoints) << "checkRgbdFrame: number of keypoints does not match with size of keypoints_3d_";

}

}  // namespace VIO
