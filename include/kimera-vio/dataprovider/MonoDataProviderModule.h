/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoDataProviderModule.h
 * @brief  Pipeline module that provides data to the VIO pipeline.
 * @details Collects camera and IMU data, publishes monocular images via
 * callback
 *          getInputPacket processes one monocular images at a time, attempting
 * to
 *          gather IMU data between the current monocular images and the
 * previous
 *          monocular images.
 * output_queue is unused-- the resulting bundle (IMU + monocular images, called
 * a
 *          monocular imagesImuSyncPacket) is published via
 * registerVioPipelineCallback.
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <string>
#include <utility>  // for move

#include <glog/logging.h>

#include "kimera-vio/dataprovider/DataProviderModule.h"
#include "kimera-vio/frontend/MonoImuSyncPacket.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class MonoDataProviderModule : public DataProviderModule {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoDataProviderModule);
  KIMERA_POINTER_TYPEDEFS(MonoDataProviderModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MonoDataProviderModule(OutputQueue* output_queue,
                         const std::string& name_id,
                         const bool& parallel_run);

  virtual ~MonoDataProviderModule() = default;

  // Called by spin(), which also calls getInputPacket().
  // Data provider syncs and publishes input sensor information, which
  // is done at the level of getInputPacket. No other action needed.
  inline OutputUniquePtr spinOnce(InputUniquePtr input) override {
    return input;
  }

  inline void fillLeftFrameQueue(Frame::UniquePtr left_frame) {
    CHECK(left_frame);
    left_frame_queue_.push(std::move(left_frame));
  }
  
  //! Callbacks to fill queues but they block if queues are getting full.
  //! Blocking call in case you want to avoid overfilling the input queues.
  //! This is useful when parsing datasets files, since parsing is much faster
  //! than the pipeline. But this is not suitable for online scenarios where
  //! you should not block the sensor processing.
  inline void fillLeftFrameQueueBlockingIfFull(Frame::UniquePtr left_frame) {
    CHECK(left_frame);
    left_frame_queue_.pushBlockingIfFull(std::move(left_frame), 5u);
  }

 protected:
  /**
   * @brief getMonoImuSyncPacket Convenience function to return synced
   * mono+imu data.
   * @return Synced Monocular image and IMU data.
   */
  MonoImuSyncPacket::UniquePtr getMonoImuSyncPacket();

  /**
   * @brief getLeftFramePayload from the left_frame_queue
   * @return
   */
  Frame::UniquePtr getLeftFramePayload();

  // Spin the dataset: processes the input data and constructs a Mono+Imu
  // Synchronized Packet (mono + IMU measurements), the minimum data
  // needed for the VIO pipeline to do one processing iteration.
  // Any image that appear before the first IMU packet will be discarded.
  // If an image appears after another image with no IMU packets in
  // between, it will be discarded.
  // The first valid pair is used as a timing fencepost and is not published.
  InputUniquePtr getInputPacket() override;

  virtual void shutdownQueues() override;

  //! Checks if the module has work to do (should check input queues are empty)
  virtual inline bool hasWork() const { return !left_frame_queue_.empty(); }

 protected:
  //! Input data
  ThreadsafeQueue<Frame::UniquePtr> left_frame_queue_;
};

}  // namespace VIO
