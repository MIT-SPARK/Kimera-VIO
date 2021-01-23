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

#include "kimera-vio/frontend/FrontendInputPacketBase.h"
#include "kimera-vio/frontend/MonoImuSyncPacket.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class DataProviderModule : public MISOPipelineModule<FrontendInputPacketBase,
                                                     FrontendInputPacketBase> {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(DataProviderModule);
  KIMERA_POINTER_TYPEDEFS(DataProviderModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using MISO =
      MISOPipelineModule<FrontendInputPacketBase, FrontendInputPacketBase>;
  using OutputQueue = typename MISO::OutputQueue;
  using PipelineOutputCallback =
      std::function<void(FrontendInputPacketBase::UniquePtr)>;

  DataProviderModule(OutputQueue* output_queue,
                     const std::string& name_id,
                     const bool& parallel_run);

  virtual ~DataProviderModule() = default;

  //! Callbacks to fill queues: they should be all lighting fast.
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
  inline void registerVioPipelineCallback(const PipelineOutputCallback& cb) {
    vio_pipeline_callback_ = cb;
  }

 protected:
  // THE USER NEEDS TO IMPLEMENT getInputPacket()!
  // Spin the dataset: processes the input data and constructs a Stereo Imu
  // Synchronized Packet (stereo pair + IMU measurements), the minimum data
  // needed for the VIO pipeline to do one processing iteration.
  // Any stereo pairs that appear before the first IMU packet will be discarded.
  // If a stereo pair appears after another stereo pair with no IMU packets in
  // between, it will be discarded.
  // The first valid pair is used as a timing fencepost and is not published.
  virtual MISO::InputUniquePtr getInputPacket() = 0;

  /**
   * @brief getTimeSyncedImuMeasurements Time synchronizes the IMU buffer
   * with the given timestamp (this is typically the timestamp of a left img)
   * @param[in] timestamp Timestamp for the IMU data to query since
   * timestamp_last_frame_.
   * @param[out] imu_meas IMU measurements to be populated and returned
   * @return False if synchronization failed, true otherwise.
   */
  bool getTimeSyncedImuMeasurements(const Timestamp& timestamp,
                                    ImuMeasurements* imu_meas);

  //! Called when general shutdown of PipelineModule is triggered.
  virtual void shutdownQueues();

 protected:
  //! Input data
  ImuData imu_data_;
  static const Timestamp kNoFrameYet = 0;
  Timestamp timestamp_last_frame_;
  PipelineOutputCallback vio_pipeline_callback_;
};

}  // namespace VIO
