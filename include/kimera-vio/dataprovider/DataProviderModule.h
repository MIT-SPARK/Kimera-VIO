/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DataProviderModule.h
 * @brief  Pipeline module that provides data to the VIO pipeline.
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

#include <glog/logging.h>

#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class DataProviderModule
    : public MISOPipelineModule<StereoImuSyncPacket, StereoImuSyncPacket> {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(DataProviderModule);
  KIMERA_POINTER_TYPEDEFS(DataProviderModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using VioPipelineCallback =
      std::function<void(StereoImuSyncPacket::UniquePtr)>;

  DataProviderModule(OutputQueue* output_queue,
                     const std::string& name_id,
                     const bool& parallel_run,
                     const StereoMatchingParams& stereo_matching_params);

  virtual ~DataProviderModule() = default;

  inline OutputUniquePtr spinOnce(
      StereoImuSyncPacket::UniquePtr input) override {
    // Called by spin(), which also calls getInputPacket().
    // Data provider syncs and publishes input sensor information, which
    // is done at the level of getInputPacket. No other action needed.
    return input;
  }

  //! Callbacks to fill queues: they should be all lighting fast.
  inline void fillLeftFrameQueue(Frame::UniquePtr left_frame) {
    CHECK(left_frame);
    left_frame_queue_.push(std::move(left_frame));
  }
  inline void fillRightFrameQueue(Frame::UniquePtr right_frame) {
    CHECK(right_frame);
    right_frame_queue_.push(std::move(right_frame));
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
  inline void fillRightFrameQueueBlockingIfFull(Frame::UniquePtr right_frame) {
    CHECK(right_frame);
    right_frame_queue_.pushBlockingIfFull(std::move(right_frame), 5u);
  }
  //! Fill multiple IMU measurements at once
  inline void fillImuQueue(const ImuMeasurements& imu_measurements) {
    imu_data_.imu_buffer_.addMeasurements(imu_measurements.timestamps_,
                                          imu_measurements.acc_gyr_);
  }
  //! Fill one IMU measurement only
  inline void fillImuQueue(const ImuMeasurement& imu_measurement) {
    imu_data_.imu_buffer_.addMeasurement(imu_measurement.timestamp_,
                                         imu_measurement.acc_gyr_);
  }

  // TODO(Toni): remove, register at ctor level.
  inline void registerVioPipelineCallback(const VioPipelineCallback& cb) {
    vio_pipeline_callback_ = cb;
  }

 protected:
  // Spin the dataset: processes the input data and constructs a Stereo Imu
  // Synchronized Packet (stereo pair + IMU measurements), the minimum data
  // needed for the VIO pipeline to do one processing iteration.
  // Any stereo pairs that appear before the first IMU packet will be discarded.
  // If a stereo pair appears after another stereo pair with no IMU packets in
  // between, it will be discarded.
  // The first valid pair is used as a timing fencepost and is not published.
  InputUniquePtr getInputPacket() override;

  //! Called when general shutdown of PipelineModule is triggered.
  void shutdownQueues() override;

  //! Checks if the module has work to do (should check input queues are empty)
  inline bool hasWork() const override {
    return !left_frame_queue_.empty() || !right_frame_queue_.empty();
  }

 private:
  //! Input data
  ImuData imu_data_;
  ThreadsafeQueue<Frame::UniquePtr> left_frame_queue_;
  ThreadsafeQueue<Frame::UniquePtr> right_frame_queue_;
  const Timestamp kNoFrameYet = 0;
  Timestamp timestamp_last_frame_;
  // TODO(Toni): remove these below
  StereoMatchingParams stereo_matching_params_;
  VioPipelineCallback vio_pipeline_callback_;
};

}  // namespace VIO
