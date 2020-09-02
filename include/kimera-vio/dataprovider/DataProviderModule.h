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

#include "kimera-vio/frontend/MonoImuSyncPacket.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

template <class Input, class Output>
class DataProviderModule : public MISOPipelineModule<Input, Output> {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(DataProviderModule);
  KIMERA_POINTER_TYPEDEFS(DataProviderModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using MISO = MISOPipelineModule<Input, Output>;
  using OutputQueue = typename MISO::OutputQueue;
  using PipelineOutputCallback =
      std::function<void(std::unique_ptr<Output>)>;

  DataProviderModule(OutputQueue* output_queue,
                     const std::string& name_id,
                     const bool& parallel_run)
      : MISO(output_queue, name_id, parallel_run),
        imu_data_(),
        left_frame_queue_("data_provider_left_frame_queue"),
        // not super nice to init a member with another member in ctor...
        timestamp_last_frame_(kNoFrameYet) {}
  virtual ~DataProviderModule() = default;

  //! Callbacks to fill queues: they should be all lighting fast.
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
  // virtual typename MISO::InputUniquePtr getInputPacket();

  /**
   * @brief getMonoImuSyncPacket Convenience function to return synced
   * mono+imu data.
   * @return Synced Monocular image and IMU data.
   */
  MonoImuSyncPacket::UniquePtr getMonoImuSyncPacket() {
    //! Retrieve left frame data.
    Frame::UniquePtr left_frame_payload = getLeftFramePayload();
    if (!left_frame_payload) {
      return nullptr;
    }

    //! Retrieve IMU data.
    const Timestamp& timestamp = left_frame_payload->timestamp_;
    ImuMeasurements imu_meas;
    if (!getTimeSyncedImuMeasurements(timestamp, &imu_meas)) {
      return nullptr;
    }

    //! Send synchronized left frame and IMU data.
    return VIO::make_unique<MonoImuSyncPacket>(
        std::move(left_frame_payload), imu_meas.timestamps_, imu_meas.acc_gyr_);
  }

  /**
   * @brief getLeftFramePayload from the left_frame_queue
   * @return
   */
  Frame::UniquePtr getLeftFramePayload() {
    bool queue_state = false;
    Frame::UniquePtr left_frame_payload = nullptr;
    if (MISO::parallel_run_) {
      queue_state = left_frame_queue_.popBlocking(left_frame_payload);
    } else {
      queue_state = left_frame_queue_.pop(left_frame_payload);
    }

    if (!queue_state) {
      LOG_IF(WARNING, MISO::parallel_run_) << "Module: " << MISO::name_id_
                                           << " - queue is down";
      VLOG_IF(1, !MISO::parallel_run_) << "Module: " << MISO::name_id_
                                       << " - queue is empty or down";
      return nullptr;
    }
    CHECK(left_frame_payload);

    return left_frame_payload;
  }

  /**
   * @brief getTimeSyncedImuMeasurements Time synchronizes the IMU buffer
   * with the given timestamp (this is typically the timestamp of a left img)
   * @param[in] timestamp Timestamp for the IMU data to query since
   * timestamp_last_frame_.
   * @param[out] imu_meas IMU measurements to be populated and returned
   * @return False if synchronization failed, true otherwise.
   */
  bool getTimeSyncedImuMeasurements(const Timestamp& timestamp,
                                    ImuMeasurements* imu_meas) {
    CHECK_NOTNULL(imu_meas);
    CHECK_LT(timestamp_last_frame_, timestamp)
        << "Timestamps out of order:\n"
        << " - Last Frame Timestamp = " << timestamp_last_frame_ << '\n'
        << " - Current Timestamp = " << timestamp;

    if (imu_data_.imu_buffer_.size() == 0) {
      VLOG(1) << "No IMU measurements available yet, dropping this frame.";
      return false;
    }

    // Extract imu measurements between consecutive frames.
    if (timestamp_last_frame_ == kNoFrameYet) {
      // TODO(Toni): wouldn't it be better to get all IMU measurements up to
      // this
      // timestamp? We should add a method to the IMU buffer for that.
      VLOG(1) << "Skipping first frame, because we do not have a concept of "
                 "a previous frame timestamp otherwise.";
      timestamp_last_frame_ = timestamp;
      return false;
    }

    utils::ThreadsafeImuBuffer::QueryResult query_result =
        utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable;
    bool log_error_once = true;
    while (
        !MISO::shutdown_ &&
        (query_result = imu_data_.imu_buffer_.getImuDataInterpolatedUpperBorder(
             timestamp_last_frame_,
             timestamp,
             &imu_meas->timestamps_,
             &imu_meas->acc_gyr_)) !=
            utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable) {
      VLOG(1) << "No IMU data available. Reason:\n";
      switch (query_result) {
        case utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable: {
          if (log_error_once) {
            LOG(WARNING) << "Waiting for IMU data...";
            log_error_once = false;
          }
          continue;
        }
        case utils::ThreadsafeImuBuffer::QueryResult::kQueueShutdown: {
          LOG(WARNING)
              << "IMU buffer was shutdown. Shutting down DataProviderModule.";
          MISO::shutdown();
          return false;
        }
        case utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable: {
          LOG(WARNING)
              << "Asking for data before start of IMU stream, from timestamp: "
              << timestamp_last_frame_ << " to timestamp: " << timestamp;
          // Ignore frames that happened before the earliest imu data
          timestamp_last_frame_ = timestamp;
          return false;
        }
        case utils::ThreadsafeImuBuffer::QueryResult::
            kTooFewMeasurementsAvailable: {
          LOG(WARNING)
              << "No IMU measurements here, and IMU data stream already "
                 "passed this time region"
              << "from timestamp: " << timestamp_last_frame_
              << " to timestamp: " << timestamp;
          return false;
        }
        case utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable: {
          LOG(FATAL)
              << "We should not be inside this while loop if IMU data is "
                 "available...";
          return false;
        }
      }
    }
    timestamp_last_frame_ = timestamp;

    VLOG(10) << "////////////////////////////////////////// Creating packet!\n"
             << "STAMPS IMU rows : \n"
             << imu_meas->timestamps_.rows() << '\n'
             << "STAMPS IMU cols : \n"
             << imu_meas->timestamps_.cols() << '\n'
             << "STAMPS IMU: \n"
             << imu_meas->timestamps_ << '\n'
             << "ACCGYR IMU rows : \n"
             << imu_meas->acc_gyr_.rows() << '\n'
             << "ACCGYR IMU cols : \n"
             << imu_meas->acc_gyr_.cols() << '\n'
             << "ACCGYR IMU: \n"
             << imu_meas->acc_gyr_;

    return true;
  }

  //! Called when general shutdown of PipelineModule is triggered.
  virtual void shutdownQueues() {
    left_frame_queue_.shutdown();
    imu_data_.imu_buffer_.shutdown();
    MISO::shutdownQueues();
  }

  //! Checks if the module has work to do (should check input queues are empty)
  virtual inline bool hasWork() const { return !left_frame_queue_.empty(); }

 protected:
  //! Input data
  ImuData imu_data_;
  ThreadsafeQueue<Frame::UniquePtr> left_frame_queue_;
  static const Timestamp kNoFrameYet = 0;
  Timestamp timestamp_last_frame_;
  PipelineOutputCallback vio_pipeline_callback_;
};

}  // namespace VIO
