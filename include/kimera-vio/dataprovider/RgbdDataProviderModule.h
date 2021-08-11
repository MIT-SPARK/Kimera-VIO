/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdDataProviderModule.h
 * @brief  Pipeline module that provides RGBD data to the VIO pipeline.
 * @details Collects camera and IMU data, publishes RgbdFrames via callback
 *          getInputPacket processes one rgbd pair at a time, attempting to
 *          gather IMU data between the current rgbd frame and the previous
 *          rgbd frame
 * output_queue is unused-- the resulting bundle (IMU + rgbd, called a
 *          RgbdImuSyncPacket) is published via registerVioPipelineCallback.
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <string>
#include <utility>  // for move

#include <glog/logging.h>

#include "kimera-vio/dataprovider/MonoDataProviderModule.h"
#include "kimera-vio/frontend/RgbdFrame.h"
#include "kimera-vio/frontend/RgbdImuSyncPacket.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class RgbdDataProviderModule : public MonoDataProviderModule {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RgbdDataProviderModule);
  KIMERA_POINTER_TYPEDEFS(RgbdDataProviderModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RgbdDataProviderModule(OutputQueue* output_queue,
                         const std::string& name_id,
                         const bool& parallel_run);

  virtual ~RgbdDataProviderModule() = default;

  inline OutputUniquePtr spinOnce(InputUniquePtr input) override {
    // Called by spin(), which also calls getInputPacket().
    // Data provider syncs and publishes input sensor information, which
    // is done at the level of getInputPacket. No other action needed.
    return input;
  }

  //! Callbacks to fill queues: they should be all lighting fast.
  inline void fillDepthFrameQueue(DepthFrame::UniquePtr depth_frame) {
    CHECK(depth_frame);
    depth_frame_queue_.push(std::move(depth_frame));
  }

  //! Callbacks to fill queues but they block if queues are getting full.
  //! Blocking call in case you want to avoid overfilling the input queues.
  //! This is useful when parsing datasets files, since parsing is much faster
  //! than the pipeline. But this is not suitable for online scenarios where
  //! you should not block the sensor processing.
  inline void fillDepthFrameQueueBlockingIfFull(
      DepthFrame::UniquePtr depth_frame) {
    CHECK(depth_frame);
    depth_frame_queue_.pushBlockingIfFull(std::move(depth_frame), 5u);
  }

 protected:
  // Spin the dataset: processes the input data and constructs a Rgbd Imu
  // Synchronized Packet (Rgbd pair + IMU measurements), the minimum data
  // needed for the VIO pipeline to do one processing iteration.
  // Any Rgbd frames that appear before the first IMU packet will be discarded.
  // If a rgbd frames appears after another rgbd pair with no IMU packets in
  // between, it will be discarded.
  // The first valid pair is used as a timing fencepost and is not published.
  InputUniquePtr getInputPacket() override;

  //! Called when general shutdown of PipelineModule is triggered.
  void shutdownQueues() override;

  //! Checks if the module has work to do (should check input queues are empty)
  inline bool hasWork() const override {
    return MonoDataProviderModule::hasWork() || !depth_frame_queue_.empty();
  }

 private:
  //! Input data
  ThreadsafeQueue<DepthFrame::UniquePtr> depth_frame_queue_;
  cv::Mat depth_mask_;
};

}  // namespace VIO
