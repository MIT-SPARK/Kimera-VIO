/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdImuPipeline.h
 * @brief  Implements RgbdVIO pipeline workflow.
 * @author Nathan Hughes
 */

#pragma once

#include "kimera-vio/dataprovider/RgbdDataProviderModule.h"
#include "kimera-vio/frontend/RgbdCamera.h"
#include "kimera-vio/pipeline/Pipeline.h"

namespace VIO {

class RgbdImuPipeline : public Pipeline {
 public:
  KIMERA_POINTER_TYPEDEFS(RgbdImuPipeline);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RgbdImuPipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  RgbdImuPipeline(const VioParams& params,
                  Visualizer3D::UniquePtr&& visualizer = nullptr,
                  DisplayBase::UniquePtr&& displayer = nullptr,
                  PreloadedVocab::Ptr&& preloaded_vocab = nullptr);

  ~RgbdImuPipeline() = default;

  void fillDepthFrameQueue(DepthFrame::UniquePtr frame);

 protected:
  RgbdCamera::ConstPtr camera_;
};

}  // namespace VIO
