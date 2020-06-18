/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoDataProviderModule.h
 * @brief  Pipeline module that provides stereo data to the VIO pipeline.
 * @details Collects camera and IMU data, publishes StereoFrames via callback
 *          getInputPacket processes one stereo pair at a time, attempting to
 *          gather IMU data between the current stereo pair and the previous
 *          stereo pair.
 * output_queue is unused-- the resulting bundle (IMU + stereo, called a
 *          StereoImuSyncPacket) is published via registerVioPipelineCallback.
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <string>
#include <utility>  // for move

#include "kimera-vio/dataprovider/DataProviderModule.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class StereoDataProviderModule
    : public DataProviderModule<StereoImuSyncPacket, StereoImuSyncPacket> {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoDataProviderModule);
  KIMERA_POINTER_TYPEDEFS(StereoDataProviderModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using StereoVioPipelineCallback =
      std::function<void(StereoImuSyncPacket::UniquePtr)>;

  StereoDataProviderModule(OutputQueue* output_queue,
                           const std::string& name_id,
                           const bool& parallel_run,
                           const StereoMatchingParams& stereo_matching_params);

  ~StereoDataProviderModule() override = default;

  inline OutputUniquePtr spinOnce(
      StereoImuSyncPacket::UniquePtr input) override {
    // Called by spin(), which also calls getInputPacket().
    // Data provider syncs and publishes input sensor information, which
    // is done at the level of getInputPacket. No other action needed.
    return input;
  }

  //! Callbacks to fill queues: they should be all lighting fast.
  inline void fillRightFrameQueue(Frame::UniquePtr right_frame) {
    CHECK(right_frame);
    right_frame_queue_.push(std::move(right_frame));
  }
  inline void fillRightFrameQueueBlockingIfFull(Frame::UniquePtr right_frame) {
    CHECK(right_frame);
    right_frame_queue_.pushBlockingIfFull(std::move(right_frame), 5u);
  }
  // TODO(Toni): remove, register at ctor level.
  inline void registerVioPipelineCallback(const StereoVioPipelineCallback& cb) {
    vio_pipeline_callback_ = cb;
  }

 protected:
  //! The data synchronization function
  InputUniquePtr getInputPacket() override;

  //! Called when general shutdown of PipelineModule is triggered.
  void shutdownQueues() override;

  //! Checks if the module has work to do (should check input queues are empty)
  inline bool hasWork() const override {
    return DataProviderModule::hasWork() || !right_frame_queue_.empty();
  }

 private:
  //! Input data
  ThreadsafeQueue<Frame::UniquePtr> right_frame_queue_;
  // TODO(Toni): remove these below
  StereoMatchingParams stereo_matching_params_;
  StereoVioPipelineCallback vio_pipeline_callback_;
};

}  // namespace VIO
