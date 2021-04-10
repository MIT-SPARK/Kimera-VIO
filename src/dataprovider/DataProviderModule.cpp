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

using utils::ThreadsafeImuBuffer;

DataProviderModule::DataProviderModule(OutputQueue* output_queue,
                                       const std::string& name_id,
                                       const bool& parallel_run)
    : MISO(output_queue, name_id, parallel_run),
      imu_data_(),
      repeated_frame_(false),
      timestamp_last_frame_(kNoFrameYet),
      do_initial_imu_timestamp_correction_(false),
      imu_timestamp_correction_(0),
      imu_time_shift_ns_(0) {}

void DataProviderModule::logQueryResult(
    const Timestamp& timestamp,
    ThreadsafeImuBuffer::QueryResult result) const {
  switch (result) {
    case ThreadsafeImuBuffer::QueryResult::kDataAvailable:
      break;
    case ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable:
      if (!repeated_frame_) {
        LOG(WARNING) << "Waiting for IMU data...";
      }
      break;
    case ThreadsafeImuBuffer::QueryResult::kQueueShutdown:
      LOG(WARNING)
          << "IMU buffer was shutdown. Shutting down DataProviderModule.";
      break;
    case ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable:
      LOG(WARNING) << "Asking for data before start of IMU stream:  "
                   << UtilsNumerical::NsecToSec(timestamp_last_frame_)
                   << " to  " << UtilsNumerical::NsecToSec(timestamp);
      break;
    case ThreadsafeImuBuffer::QueryResult::kTooFewMeasurementsAvailable:
      LOG(WARNING) << "IMU data stream does not contain measurements from "
                   << UtilsNumerical::NsecToSec(timestamp_last_frame_) << " to "
                   << UtilsNumerical::NsecToSec(timestamp);
      break;
    default:
      LOG(ERROR) << "Imu buffer returned unexpected query result";
      break;
  }
}

DataProviderModule::FrameAction
DataProviderModule::getTimeSyncedImuMeasurements(const Timestamp& timestamp,
                                                 ImuMeasurements* imu_meas) {
  if (MISO::shutdown_) {
    return FrameAction::Drop;
  }

  CHECK_NOTNULL(imu_meas);
  CHECK_LT(timestamp_last_frame_, timestamp)
      << "Image timestamps out of order: "
      << UtilsNumerical::NsecToSec(timestamp_last_frame_)
      << " (last) >= " << UtilsNumerical::NsecToSec(timestamp) << " (curr)";

  if (imu_data_.imu_buffer_.size() == 0) {
    VLOG(1) << "No IMU measurements available yet, dropping this frame.";
    return FrameAction::Drop;
  }

  if (timestamp_last_frame_ == kNoFrameYet) {
    // TODO(Toni): wouldn't it be better to get all IMU measurements up to
    // this
    // timestamp? We should add a method to the IMU buffer for that.
    VLOG(1) << "Skipping first frame, because we do not have a concept of "
               "a previous frame timestamp otherwise.";
    timestamp_last_frame_ = timestamp;
    return FrameAction::Drop;
  }

  // Do a very coarse timestamp correction to make sure that the IMU data
  // is aligned enough to send packets to the front-end. This is assumed
  // to be very inaccurate and should not be enabled without some other
  // actual time alignment in the frontend
  if (do_initial_imu_timestamp_correction_) {
    ImuMeasurement newest_imu;
    imu_data_.imu_buffer_.getNewestImuMeasurement(&newest_imu);
    // this is delta = imu.timestamp - frame.timestamp so that when querying,
    // we get query = new_frame.timestamp + delta = frame_delta + imu.timestamp
    imu_timestamp_correction_ = newest_imu.timestamp_ - timestamp;
    do_initial_imu_timestamp_correction_ = false;
    LOG(WARNING) << "Computed intial coarse time alignment of "
                 << UtilsNumerical::NsecToSec(imu_timestamp_correction_);
  }

  // imu_time_shift_ can be externally, asynchronously modified.
  // Caching here prevents a nasty race condition and avoids locking
  const Timestamp curr_imu_time_shift = imu_time_shift_ns_;
  const Timestamp imu_timestamp_last_frame =
      timestamp_last_frame_ + imu_timestamp_correction_ + curr_imu_time_shift;
  const Timestamp imu_timestamp_curr_frame =
      timestamp + imu_timestamp_correction_ + curr_imu_time_shift;

  ThreadsafeImuBuffer::QueryResult query_result =
      imu_data_.imu_buffer_.getImuDataInterpolatedUpperBorder(
          imu_timestamp_last_frame,
          imu_timestamp_curr_frame,
          &imu_meas->timestamps_,
          &imu_meas->acc_gyr_);
  logQueryResult(timestamp, query_result);

  switch (query_result) {
    case ThreadsafeImuBuffer::QueryResult::kDataAvailable:
      break;  // handle this below
    case ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable:
      return FrameAction::Wait;
    case ThreadsafeImuBuffer::QueryResult::kQueueShutdown:
      MISO::shutdown();
      return FrameAction::Drop;
    case ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable:
      timestamp_last_frame_ = timestamp;
      return FrameAction::Drop;
    case ThreadsafeImuBuffer::QueryResult::kTooFewMeasurementsAvailable:
    default:
      return FrameAction::Drop;
  }

  timestamp_last_frame_ = timestamp;
  // adjust the timestamps for the frontend
  // TODO(nathan) may also need to apply imu_time_shift_ here
  imu_meas->timestamps_.array() -= imu_timestamp_correction_;
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
  return FrameAction::Use;
}

void DataProviderModule::shutdownQueues() {
  imu_data_.imu_buffer_.shutdown();
  MISO::shutdownQueues();
}

}  // namespace VIO
