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

// Here is a very good example of how this should be implemented:
// https://github.com/ethz-asl/maplab/blob/master/applications/rovioli/src/datasource-rosbag.cc
class RosbagDataProvider: public DataProvider {
public:
  RosbagDataProvider(const std::string& rosbag_path);
  virtual ~RosbagDataProvider();
  virtual bool spin();
private:
};

//class RosbagDataProviderRGBD: RosbagDataProvider {
//private:
//std::function<void(RGBD image)> rgbd_callback_;
//std::function<void(IMU)> imu_callback_;
//
//public:
//RosbagDataProviderRGBD(rgbd_cb, imu_cb, rosbag_path)
//  : rgbd_callback_(rgbd_cb),
//    imu_callback_(imu_cb),
//    RosbagDataProvider(rosbag_path) {}
//
//virtual bool spin() {
//  database = rosbag.parse();
//  for (data: database) {
//    if (data.img_available) {
//      rgbd_callback_(data.imgs);
//    }
//    if (data.imu_available) {
//      imu_callback_(data.imu);
//    }
//  }
//}
//};
//
//class RostopicDataProvider: DataProvider {
//public:
//  RostopicDataProvider(): nh_(), DataProvider() {};
//
//protected:
//  ros::NodeHandle nh_;
//
//public:
//  virtual spin() {
//    ros.spin()
//  }
//};
//
//class RostopicDataProviderRGBD: RostopicDataProvider {
//private:
//  std::function<void(Image)> image_callback_;
//  std::function<void(IMU)> imu_callback_;
//
//public:
//  RostopicDataProviderRGBD(maybe names of topics):
//    RostopicDataProvider() {
//    nh_.subscribe("rgb_topic", image_callback); // I don't remember how to create a subscriber.
//    nh_.subscribe("dpeth_topic", depth_callback);
//    nh_.subscribe("imu_topic", imu_callback);
//  }
//
//private:
//
//  void image_callback(ros::ImageMsg rgb_msg) {
//    image_callback_(rgb_msg, d_msg);
//  }
//  void imu_callback(ros::ImageMsg rgb_msg) {
//    imu_callback_(rgb_msg, d_msg);
//  }
//};

//int main {
//  SparkVio vio = VIO();
//  DataProvider* dataset_prov = RosbagDataProviderRGBD(vio.trackRGBD());
//  dataset_prov->spin()
//}

} // End of VIO namespace.
