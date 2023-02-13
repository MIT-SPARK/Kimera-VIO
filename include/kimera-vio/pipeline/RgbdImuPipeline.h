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
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#pragma once

#include "kimera-vio/dataprovider/RgbdDataProviderModule.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/pipeline/Pipeline.h"

namespace VIO {

class RgbdImuPipeline : public Pipeline {
 public:
  KIMERA_POINTER_TYPEDEFS(RgbdImuPipeline);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RgbdImuPipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
     * @brief RgbdImuPipeline
     * @param params Vio parameters
     * @param visualizer Optional visualizer for visualizing 3D results
     * @param displayer Optional displayer for visualizing 2D results
     */
  RgbdImuPipeline(const VioParams& params,
                 Visualizer3D::UniquePtr&& visualizer = nullptr,
                 DisplayBase::UniquePtr&& displayer = nullptr);

  ~RgbdImuPipeline() = default;

 public:
  inline void fillDepthFrameQueue(DepthFrame::UniquePtr depth_frame) {
    CHECK(data_provider_module_);
    CHECK(depth_frame);
    // StereoDataProviderModule::UniquePtr stereo_dataprovider =
    //     VIO::safeCast<MonoDataProviderModule, StereoDataProviderModule>(
    //         std::move(data_provider_module_));
    // stereo_dataprovider->fillRightFrameQueue(std::move(right_frame));
    // data_provider_module_ =
    //     VIO::safeCast<StereoDataProviderModule, MonoDataProviderModule>(
    //         std::move(stereo_dataprovider));

    // TODO(marcus): this is not a good solution. The problem is the above code
    // doesn't work in online/parallel because other threads are accessing 
    // data_provider_module_ when it's been temporarily released to the stereo
    // version. Checks fail for that reason.
    // This fix is really bad because it totally bypasses the rules of 
    // unique_ptr
    dynamic_cast<RgbdDataProviderModule*>(data_provider_module_.get())
        ->fillDepthFrameQueue(std::move(depth_frame));
  }

  inline void fillDepthFrameQueueBlockingIfFull(DepthFrame::UniquePtr depth_frame) {
    CHECK(data_provider_module_);
    CHECK(depth_frame);
    // StereoDataProviderModule::UniquePtr stereo_dataprovider =
    //     VIO::safeCast<MonoDataProviderModule, StereoDataProviderModule>(
    //         std::move(data_provider_module_));
    // stereo_dataprovider->fillRightFrameQueueBlockingIfFull(
    //     std::move(right_frame));
    // data_provider_module_ =
    //     VIO::safeCast<StereoDataProviderModule, MonoDataProviderModule>(
    //         std::move(stereo_dataprovider));

    // TODO(marcus): this is not a good solution. The problem is the above code
    // doesn't work in online/parallel because other threads are accessing
    // data_provider_module_ when it's been temporarily released to the stereo
    // version. Checks fail for that reason.
    // This fix is really bad because it totally bypasses the rules of
    // unique_ptr
    dynamic_cast<RgbdDataProviderModule*>(data_provider_module_.get())
        ->fillDepthFrameQueueBlockingIfFull(std::move(depth_frame));
  }

 protected:
  //! Definition of sensor rig used
  StereoCamera::ConstPtr stereo_camera_;
};

}  // namespace VIO
