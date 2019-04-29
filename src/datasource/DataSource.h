/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DataSource.h
 * @brief  Base implementation of a data provider for the VIO pipeline.
 * @author Antoni Rosinol
 */

#pragma once

#include <string>
#include <functional>
#include "StereoImuSyncPacket.h"

//########### SPARK_VIO_ROS ############################################
namespace VIO {

// Struct to deal with getting values out of the spin
struct SpinOutputContainer {
  SpinOutputContainer(const Timestamp& timestamp_kf,
                      const gtsam::Pose3& W_Pose_Blkf,
                      const Vector3& W_Vel_Blkf,
                      const ImuBias& imu_bias_lkf)
    : timestamp_kf_(timestamp_kf),
      W_Pose_Blkf_(W_Pose_Blkf),
      W_Vel_Blkf_(W_Vel_Blkf),
      imu_bias_lkf_(imu_bias_lkf) {}

  Timestamp timestamp_kf_;
  gtsam::Pose3 W_Pose_Blkf_;
  Vector3 W_Vel_Blkf_;
  ImuBias imu_bias_lkf_;

  SpinOutputContainer& operator=(SpinOutputContainer other)
    {
        timestamp_kf_ = other.timestamp_kf_;
        W_Pose_Blkf_ = other.W_Pose_Blkf_;
        W_Vel_Blkf_ = other.W_Vel_Blkf_;
        imu_bias_lkf_ = other.imu_bias_lkf_;
        return *this;
    }

};

class DataProvider {
public:
  DataProvider() = default;
  virtual ~DataProvider();

  // The derived classes need to implement this function!
  // Spin the dataset: processes the input data and constructs a Stereo Imu
  // Synchronized Packet which contains the minimum amount of information
  // for the VIO pipeline to do one processing iteration.
  // A Dummy example is provided as an implementation.
  virtual bool spin();

  // Register a callback function that will be called once a StereoImu Synchro-
  // nized packet is available for processing.
  void
  registerVioCallback(std::function<SpinOutputContainer(const StereoImuSyncPacket&)> callback);

protected:
  // Vio callback. This function should be called once a StereoImuSyncPacket
  // is available for processing.
  std::function<SpinOutputContainer(const StereoImuSyncPacket&)> vio_callback_;
};

} // End of VIO namespace.
