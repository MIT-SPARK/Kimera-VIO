/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   KittiDataSource.h
 * @brief  Kitti dataset parser.
 * @author Antoni Rosinol
 */

#pragma once

#include <string>
#include <functional>
#include "datasource/DataSource.h"
#include "StereoImuSyncPacket.h"
#include "ImuFrontEnd.h"

namespace VIO {

class KittiDataProvider: public DataProvider {
public:
  KittiDataProvider(const std::string& kitti_dataset_path);
  virtual ~KittiDataProvider();
  virtual bool spin();

private:
  // NOTE TO ASK: why this struct?
  struct KittiData {
    inline size_t getNumberOfImages() const {return left_img_names_.size();}
    // This matches the names of the folders in the dataset
    std::vector<std::string> camera_names_;
    // Map from camera name to its parameters
    std::map<std::string, CameraParams> camera_info_;
    // The image names of the images from left camera 
    std::vector<std::string> left_img_names_;
    // The image names of the images from right camera
    std::vector<std::string> right_img_names_;
    // Vector of timestamps see issue in .cpp file 
    std::vector<Timestamp> timestamps_;
    //IMU data 
    ImuData imuData_;
    // Dummy check to ensure data is correctly parsed.
    explicit operator bool() const;
  };

private:
  cv::Mat readKittiImage(const std::string& img_name);
  void parseData(const std::string& kitti_sequence_path,
                 KittiData* kitti_data);

  // Parse the timestamps of a particular device of given dataset 
  bool parseTimestamps(const std::string& timestamps_file,
                       std::vector<Timestamp>& timestamps_list) const;

  // Parse camera info of given dataset 
  bool parseCameraData(const std::string& input_dataset_path,
                       const std::string& left_cam_name,
                       const std::string& right_cam_name,
                       KittiData* kitti_data);

  // Parse IMU data of a given dataset 
  bool parseImuData(const std::string& input_dataset_path, 
                    KittiData* kitti_data_);

  // Get R and T matrix from calibration file 
  bool parseRT(const std::string& input_dataset_path, 
               const std::string& calibration_filename, 
               cv::Mat& R, cv::Mat& T) const;
private:
  KittiData kitti_data_;
};

} // End of VIO namespace.
