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
      stereo_matching_params_(stereo_matching_params) {}

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
    LOG_IF(WARNING, PIO::parallel_run_)
        << "Module: " << name_id_ << " - queue is down";
    VLOG_IF(1, !PIO::parallel_run_)
        << "Module: " << name_id_ << " - queue is empty or down";
    return nullptr;
  }

  CHECK(left_frame_payload);
  const Timestamp& timestamp = left_frame_payload->timestamp_;

  // Look for the synchronized right frame inside the queue.
  Frame::UniquePtr right_frame_payload = nullptr;
  PIO::syncQueue(timestamp, &right_frame_queue_, &right_frame_payload);
  CHECK(right_frame_payload);

  // Extract imu measurements between consecutive frames.
  static Timestamp timestamp_last_frame = 0;
  if (timestamp_last_frame == 0) {
    // TODO(Toni): wouldn't it be better to get all IMU measurements up to this
    // timestamp? We should add a method to the IMU buffer for that.
    VLOG(1) << "Skipping first frame, because we do not have a concept of "
               "a previous frame timestamp otherwise.";
    timestamp_last_frame = timestamp;
    return nullptr;
  }

  ImuMeasurements imu_meas;
  CHECK_LT(timestamp_last_frame, timestamp);
  utils::ThreadsafeImuBuffer::QueryResult query_result =
      utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable;
  bool log_error_once = true;
  while (
      (query_result = imu_data_.imu_buffer_.getImuDataInterpolatedUpperBorder(
           timestamp_last_frame,
           timestamp,
           &imu_meas.timestamps_,
           &imu_meas.acc_gyr_)) !=
      utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable) {
    VLOG(1) << "No IMU data available. Reason:\n";
    switch (query_result) {
      case utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable: {
        LOG(FATAL) << "We should not be inside this while loop if IMU data is "
                      "available...";
        break;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::kQueueShutdown: {
        LOG(INFO)
            << "IMU buffer was shutdown. Shutting down DataProviderModule.";
        shutdown();
        return nullptr;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable: {
        LOG_EVERY_N(WARNING, 100)
            << "No IMU data from last frame timestamp: " << timestamp_last_frame
            << " to timestamp: " << timestamp;
        continue;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable: {
        if (log_error_once) {
          LOG(WARNING) << "Waiting for IMU data...";
          log_error_once = false;
        }
        continue;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::
          kTooFewMeasurementsAvailable: {
        LOG_EVERY_N(WARNING, 100)
            << "Too few IMU measurements from last frame timestamp: "
            << timestamp_last_frame << " to timestamp: " << timestamp;
        continue;
      }
    }
  }
  timestamp_last_frame = timestamp;

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

  CHECK(vio_pipeline_callback_);
  vio_pipeline_callback_(VIO::make_unique<StereoImuSyncPacket>(
      StereoFrame(left_frame_payload->id_,
                  timestamp,
                  *left_frame_payload,
                  *right_frame_payload,
                  stereo_matching_params_),  // TODO(Toni): these params should
      // be given in PipelineParams.
      imu_meas.timestamps_,
      imu_meas.acc_gyr_));

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
  MISOPipelineModule::shutdownQueues();
}

}  // namespace VIO
