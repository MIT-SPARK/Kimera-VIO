/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DataProviderModule.cpp
 * @brief  Pipeline Module that takes care of providing data to the VIO
 * pipeline.
 * @author Antoni Rosinol
 */

#include "kimera-vio/dataprovider/DataProviderModule.h"

namespace VIO {

DataProviderModule::DataProviderModule(
    OutputQueue* output_queue,
    const std::string& name_id,
    const bool& parallel_run,
    const StereoMatchingParams& stereo_matching_params)
    : MISOPipelineModule<StereoImuSyncPacket, StereoImuSyncPacket>(
          output_queue,
          name_id,
          parallel_run),
      imu_data_(),
      left_frame_queue_("data_provider_left_frame_queue"),
      right_frame_queue_("data_provider_right_frame_queue"),
      stereo_matching_params_(stereo_matching_params),
      timestamp_last_frame_(kNoFrameYet) {}

DataProviderModule::InputUniquePtr DataProviderModule::getInputPacket() {
  // Look for a left frame inside the queue.
  bool queue_state = false;
  Frame::UniquePtr left_frame_payload = nullptr;
  if (PIO::parallel_run_) {
    queue_state = left_frame_queue_.popBlocking(left_frame_payload);
  } else {
    queue_state = left_frame_queue_.pop(left_frame_payload);
  }

  if (!queue_state) {
    LOG_IF(WARNING, PIO::parallel_run_) << "Module: " << name_id_
                                        << " - queue is down";
    VLOG_IF(1, !PIO::parallel_run_) << "Module: " << name_id_
                                    << " - queue is empty or down";
    return nullptr;
  }

  CHECK(left_frame_payload);
  const Timestamp& timestamp = left_frame_payload->timestamp_;

  // Look for the synchronized right frame inside the queue.
  // This search might not be successful if the data_provider did not push to
  // the right queue (perhaps fast enough).
  Frame::UniquePtr right_frame_payload = nullptr;
  if (!PIO::syncQueue(timestamp, &right_frame_queue_, &right_frame_payload)) {
    // Dropping this message because of missing left/right stereo synced frames.
    LOG(ERROR) << "Missing right frame for left frame with id "
               << left_frame_payload->id_ << ", dropping this frame.";
    return nullptr;
  }
  CHECK(right_frame_payload);

  if (imu_data_.imu_buffer_.size() == 0) {
    VLOG(1) << "No IMU measurements available yet, dropping this frame.";
    return nullptr;
  }

  // Extract imu measurements between consecutive frames.
  if (timestamp_last_frame_ == kNoFrameYet) {
    // TODO(Toni): wouldn't it be better to get all IMU measurements up to this
    // timestamp? We should add a method to the IMU buffer for that.
    VLOG(1) << "Skipping first frame, because we do not have a concept of "
               "a previous frame timestamp otherwise.";
    timestamp_last_frame_ = timestamp;
    return nullptr;
  }

  ImuMeasurements imu_meas;
  CHECK_LT(timestamp_last_frame_, timestamp)
      << "Timestamps out of order:\n"
      << " - Last Frame Timestamp = " << timestamp_last_frame_ << '\n'
      << " - Current Timestamp = " << timestamp;
  utils::ThreadsafeImuBuffer::QueryResult query_result =
      utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable;
  bool log_error_once = true;
  while (
      !shutdown_ &&
      (query_result = imu_data_.imu_buffer_.getImuDataInterpolatedUpperBorder(
           timestamp_last_frame_,
           timestamp,
           &imu_meas.timestamps_,
           &imu_meas.acc_gyr_)) !=
          utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable) {
    VLOG(1) << "No IMU data available btw: \n"
            << " - From timestamp: " << timestamp_last_frame_ << '\n'
            << " - To timestamp: " << timestamp << '\n'
            << "Reason:\n";
    switch (query_result) {
      case utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable: {
        if (log_error_once || VLOG_IS_ON(1)) {
          LOG(WARNING) << "Waiting for IMU data...";
          log_error_once = false;
        }
        continue;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::kQueueShutdown: {
        LOG(WARNING)
            << "IMU buffer was shutdown. Shutting down DataProviderModule.";
        shutdown();
        return nullptr;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable: {
        LOG(WARNING)
            << "Asking for data before start of IMU stream, from timestamp: "
            << timestamp_last_frame_ << " to timestamp: " << timestamp;
        // Ignore frames that happened before the earliest imu data
        timestamp_last_frame_ = timestamp;
        return nullptr;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::
          kTooFewMeasurementsAvailable: {
        LOG(WARNING) << "No IMU measurements here, and IMU data stream already "
                        "passed this time region"
                     << "from timestamp: " << timestamp_last_frame_
                     << " to timestamp: " << timestamp;
        return nullptr;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable: {
        LOG(FATAL) << "We should not be inside this while loop if IMU data is "
                      "available...";
        return nullptr;
      }
    }
  }
  timestamp_last_frame_ = timestamp;

  VLOG(10) << "////////////////////////////////////////// Creating packet!\n"
           << "STAMPS IMU rows : \n"
           << imu_meas.timestamps_.rows() << '\n'
           << "STAMPS IMU cols : \n"
           << imu_meas.timestamps_.cols() << '\n'
           << "STAMPS IMU: \n"
           << imu_meas.timestamps_ << '\n'
           << "ACCGYR IMU rows : \n"
           << imu_meas.acc_gyr_.rows() << '\n'
           << "ACCGYR IMU cols : \n"
           << imu_meas.acc_gyr_.cols() << '\n'
           << "ACCGYR IMU: \n"
           << imu_meas.acc_gyr_;

  if (!shutdown_) {
    CHECK(vio_pipeline_callback_);
    vio_pipeline_callback_(VIO::make_unique<StereoImuSyncPacket>(
        StereoFrame(
            left_frame_payload->id_,
            timestamp,
            *left_frame_payload,
            *right_frame_payload,
            stereo_matching_params_),  // TODO(Toni): these params should
        // be given in PipelineParams.
        imu_meas.timestamps_,
        imu_meas.acc_gyr_));
  }

  // Push the synced messages to the frontend's input queue
  // TODO(Toni): should be a return like that, so that we pass the info to the
  // queue... Right now we use a callback bcs otw I need to fix all
  // initialization which is a lot to be fixed.
  // return VIO::make_unique<StereoImuSyncPacket>(
  //    StereoFrame(
  //        left_frame_payload->id_,
  //        timestamp,
  //        *left_frame_payload,
  //        *right_frame_payload,
  //        stereo_matching_params_),  // TODO(Toni): these params should
  //                                   // be given in PipelineParams.
  //    imu_meas.timestamps_,
  //    imu_meas.acc_gyr_);
  return nullptr;
}

void DataProviderModule::shutdownQueues() {
  left_frame_queue_.shutdown();
  right_frame_queue_.shutdown();
  imu_data_.imu_buffer_.shutdown();
  MISOPipelineModule::shutdownQueues();
}

}  // namespace VIO
