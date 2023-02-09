/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdFrame.h
 * @brief  Class describing a combination of an RGB image + a Depth image
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/DepthFrame.h"
#include "kimera-vio/pipeline/PipelinePayload.h"

namespace VIO {

class RgbdFrame {
  public:
  // KIMERA_DELETE_COPY_CONSTRUCTORS(RgbdFrame);
  KIMERA_POINTER_TYPEDEFS(RgbdFrame);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RgbdFrame(const FrameId& id,
            const Timestamp& timestamp,
            Frame::UniquePtr intensity_img,
            DepthFrame::UniquePtr depth_img);

  RgbdFrame(const RgbdFrame& rgbd_frame);

  inline void setIsKeyframe(bool is_kf) {
    is_keyframe_ = is_kf;
    intensity_img_->isKeyframe_ = is_kf;
  }

  inline bool isKeyframe() const { return is_keyframe_; }

  void calculate3dKeypoints();

  void checkRgbdFrame() const;
  private:
    bool is_keyframe_;


  public:
    const FrameId id_;
    //! Timestamp of this stereo frame.
    const Timestamp timestamp_;
    Frame::UniquePtr intensity_img_;
    DepthFrame::UniquePtr depth_img_;
    std::vector<gtsam::Vector3> keypoints_3d_;

};

}  // namespace VIO
