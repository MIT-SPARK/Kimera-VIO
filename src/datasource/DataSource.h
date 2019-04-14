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

//######################### This is an example of main ##################
// SparkVio vio = VIO()
// DataProvider* dat_prov = RosDataProvider()

//########### SPARK_VIO_ROS ############################################
namespace VIO {

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
  registerVioCallback(std::function<bool(const StereoImuSyncPacket&)> callback);

protected:
  // Vio callback. This function should be called once a StereoImuSyncPacket
  // is available for processing.
  std::function<bool(const StereoImuSyncPacket&)> vio_callback_;
};

} // End of VIO namespace.
