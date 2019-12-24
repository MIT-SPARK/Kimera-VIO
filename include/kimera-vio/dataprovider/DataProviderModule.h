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
    // Data provider is only syncing input sensor information, which
    // is done at the level of getInputPacket, therefore here we h
    return input;
  }

  //! Callbacks to fill queues: they should be all lighting fast.
  inline void fillLeftFrameQueue(Frame::UniquePtr&& left_frame) {
    CHECK(left_frame);
    left_frame_queue_.push(std::move(left_frame));
  }
  inline void fillRightFrameQueue(Frame::UniquePtr&& right_frame) {
    CHECK(right_frame);
    right_frame_queue_.push(std::move(right_frame));
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
  // Synchronized Packet which contains the minimum amount of information
  // for the VIO pipeline to do one processing iteration.
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
  // TODO(Toni): remove these below
  StereoMatchingParams stereo_matching_params_;
  VioPipelineCallback vio_pipeline_callback_;
};

}  // namespace VIO
