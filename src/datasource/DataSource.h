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

#include <functional>
#include <string>
#include "StereoImuSyncPacket.h"
#include "Tracker.h"

//########### SPARK_VIO_ROS ############################################
namespace VIO {

// Struct to deal with getting values out of the spin
struct SpinOutputContainer {
  // Default constructor
  SpinOutputContainer(
      const Timestamp& timestamp_kf, const gtsam::Pose3& W_Pose_Blkf,
      const Vector3& W_Vel_Blkf, const ImuBias& imu_bias_lkf,
      const gtsam::Matrix State_Covariance_lkf = gtsam::zeros(15, 15),
      const DebugTrackerInfo debug_tracker_info = DebugTrackerInfo())
      : timestamp_kf_(timestamp_kf),
        W_Pose_Blkf_(W_Pose_Blkf),
        W_Vel_Blkf_(W_Vel_Blkf),
        imu_bias_lkf_(imu_bias_lkf),
        debug_tracker_info_(debug_tracker_info) {
    // TODO: Create a better assert for this covariance matrix
    CHECK_EQ(State_Covariance_lkf.rows(), 15);
    CHECK_EQ(State_Covariance_lkf.cols(), 15);
    State_Covariance_lkf_ = State_Covariance_lkf;
  }

  // Trivial constructor (do not publish)
  SpinOutputContainer()
    : timestamp_kf_(-1),
      W_Pose_Blkf_(gtsam::Pose3()),
      W_Vel_Blkf_(gtsam::Vector3()),
      imu_bias_lkf_(gtsam::imuBias::ConstantBias()),
      State_Covariance_lkf_(gtsam::zeros(15,15)),
      debug_tracker_info_(DebugTrackerInfo()) {}

  Timestamp timestamp_kf_;
  gtsam::Pose3 W_Pose_Blkf_;
  Vector3 W_Vel_Blkf_;
  ImuBias imu_bias_lkf_;
  gtsam::Matrix State_Covariance_lkf_;
  DebugTrackerInfo debug_tracker_info_;

  inline const Timestamp getTimestamp() { return timestamp_kf_; }

  inline const gtsam::Pose3 getEstimatedPose() { return W_Pose_Blkf_; }

  inline const Vector3 getEstimatedVelocity() { return W_Vel_Blkf_; }

  inline const gtsam::Matrix6 getEstimatedPoseCov() {
    return gtsam::sub(State_Covariance_lkf_, 0, 6, 0, 6);
  }

  inline const gtsam::Matrix3 getEstimatedVelCov() {
    return gtsam::sub(State_Covariance_lkf_, 6, 9, 6, 9);
  }

  inline const ImuBias getEstimatedBias() { return imu_bias_lkf_; }

  inline const gtsam::Matrix6 getEstimatedBiasCov() {
    return gtsam::sub(State_Covariance_lkf_, 9, 15, 9, 15);
  }

  inline const DebugTrackerInfo getTrackerInfo() { return debug_tracker_info_; }
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
  void registerVioCallback(
      std::function<SpinOutputContainer(const StereoImuSyncPacket&)> callback);

 protected:
  // Vio callback. This function should be called once a StereoImuSyncPacket
  // is available for processing.
  std::function<SpinOutputContainer(const StereoImuSyncPacket&)> vio_callback_;
};

}  // namespace VIO
