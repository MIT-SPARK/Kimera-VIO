/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DataSource.cpp
 * @brief  Base implementation of a data provider for the VIO pipeline.
 * @author Antoni Rosinol
 */
#include "datasource/DataSource.h"

namespace VIO {

DataProvider::~DataProvider() {
  LOG(INFO) << "Data provider destructor called.";
}

void DataProvider::registerVioCallback(
    std::function<SpinOutputContainer(const StereoImuSyncPacket&)> callback) {
  vio_callback_ = std::move(callback);
}

bool DataProvider::spin() {
  // Dummy example:
  // 1) Check that the vio_callback_ has been registered, aka that the user has
  // called the function registerVioCallback, in order to store the callback
  // function.
  CHECK(vio_callback_);

  // 2) Loop over the dataset and:
  //  a) Create StereoImuSyncPacket packets out of the data.
  //  This one is dummy since it is filled with empty images, parameters,
  //  imu data, etc.
  StereoImuSyncPacket vio_packet(
      StereoFrame(1, 1, cv::Mat(), CameraParams(), cv::Mat(), CameraParams(),
                  gtsam::Pose3(), StereoMatchingParams()),
      ImuStampS(), ImuAccGyrS());
  //  b) Call the vio callback in order to start processing the packet.
  vio_callback_(vio_packet);

  // 3) Once the dataset spin has finished, exit.
  // You can return false if something went wrong.
  return true;
}

}  // namespace VIO
