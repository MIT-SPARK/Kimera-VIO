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

DataProviderModule::DataProviderModule(OutputQueue* output_queue,
                                       const std::string& name_id,
                                       const bool& parallel_run)
    : MISO(output_queue, name_id, parallel_run),
      imu_data_(),
      // not super nice to init a member with another member in ctor...
      timestamp_last_frame_(kNoFrameYet) {}

bool DataProviderModule::getTimeSyncedImuMeasurements(
    const Timestamp& timestamp,
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
        LOG(WARNING) << "No IMU measurements here, and IMU data stream already "
                        "passed this time region"
                     << "from timestamp: " << timestamp_last_frame_
                     << " to timestamp: " << timestamp;
        return false;
      }
      case utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable: {
        LOG(FATAL) << "We should not be inside this while loop if IMU data is "
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

void DataProviderModule::shutdownQueues() {
  imu_data_.imu_buffer_.shutdown();
  MISO::shutdownQueues();
}

}  // namespace VIO
