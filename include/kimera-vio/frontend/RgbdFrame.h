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
 * @author Nathan Hughes
 */

#pragma once

#include "kimera-vio/frontend/DepthFrame.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/pipeline/PipelinePayload.h"

namespace VIO {

// forward declarations to break dependency on RgbdFrame in RgbdCamera
class RgbdCamera;

class RgbdFrame : public PipelinePayload {
 public:
  // KIMERA_DELETE_COPY_CONSTRUCTORS(RgbdFrame);
  KIMERA_POINTER_TYPEDEFS(RgbdFrame);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RgbdFrame(const FrameId& id,
            const Timestamp& timestamp,
            const Frame& intensity_img,
            const DepthFrame& depth_img);

  RgbdFrame(const RgbdFrame& other);

  /**
   * @brief get a fake stereo frame from the RGB image
   * @returns StereoFrame with a valid left frame and a right frame with the
   * correct timestamp
   */
  StereoFrame::Ptr getStereoFrame() const;

  /**
   * @brief Fill fake right features based on the depth frame
   * @param[in] camera RGBD camera for the frames
   * @param[out] stereo_frame Stereo frame to fill with features
   */
  void fillStereoFrame(const RgbdCamera& camera,
                       StereoFrame& stereo_frame) const;

 public:
  const FrameId id_;
  Frame intensity_img_;
  DepthFrame depth_img_;
};

}  // namespace VIO
