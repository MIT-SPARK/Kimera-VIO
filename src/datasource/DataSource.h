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

//######################### This is an example of main ##################
// SparkVio vio = VIO()
// DataProvider* dat_prov = RosDataProvider()

//########### SPARK_VIO_ROS ############################################

namespace VIO {

/* /////////////////////////////////////////////////////////////////////////////
class DataProvider {
public:
  DataProvider() = default;
  virtual ~DataProvider();
  virtual bool spin() = 0; // The derived class needs to implement this function!
private:
};

class RosbagDataProvider: DataProvider {
private:
  const std::string rosbag_path_;

public:
  RosbagDataProvider(const std::string& rosbag_path)
    : rosbag_path_(rosbag_path) {}

  virtual bool spin() = 0;// Needs to be implemented by derived classes!!
};

class RosbagDataProviderRGBD: RosbagDataProvider {
private:
std::function<void(RGBD image)> rgbd_callback_;
std::function<void(IMU)> imu_callback_;

public:
RosbagDataProviderRGBD(rgbd_cb, imu_cb, rosbag_path)
  : rgbd_callback_(rgbd_cb),
    imu_callback_(imu_cb),
    RosbagDataProvider(rosbag_path) {}

virtual bool spin() {
  database = rosbag.parse();
  for (data: database) {
    if (data.img_available) {
      rgbd_callback_(data.imgs);
    }
    if (data.imu_available) {
      imu_callback_(data.imu);
    }
  }
}
};

class RostopicDataProvider: DataProvider {
public:
  RostopicDataProvider(): nh_(), DataProvider() {};

protected:
  ros::NodeHandle nh_;

public:
  virtual spin() {
    ros.spin()
  }
};

class RostopicDataProviderRGBD: RostopicDataProvider {
private:
  std::function<void(Image)> image_callback_;
  std::function<void(IMU)> imu_callback_;

public:
  RostopicDataProviderRGBD(maybe names of topics):
    RostopicDataProvider() {
    nh_.subscribe("rgb_topic", image_callback); // I don't remember how to create a subscriber.
    nh_.subscribe("dpeth_topic", depth_callback);
    nh_.subscribe("imu_topic", imu_callback);
  }

private:

  void image_callback(ros::ImageMsg rgb_msg) {
    image_callback_(rgb_msg, d_msg);
  }
  void imu_callback(ros::ImageMsg rgb_msg) {
    imu_callback_(rgb_msg, d_msg);
  }
};

class EurocDataProvider: DataProvider {
public:
  ...
};

int main {
  SparkVio vio = VIO();
  DataProvider* dataset_prov = RosbagDataProviderRGBD(vio.trackRGBD());
  dataset_prov->spin()
}

*///////////////////////////////////////////////////////////////////////////////

} // End of VIO namespace.
