/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   KittiDataSource.cpp
 * @brief  Kitti dataset parser.
 * @author Antoni Rosinol
 */
#include "datasource/KittiDataSource.h"

#include <opencv2/core/core.hpp>

#include "StereoImuSyncPacket.h"
#include "StereoFrame.h"

namespace VIO {

KittiDataProvider::KittiData::operator bool() const {
  bool empty_data = timestamps_.empty() ||
                    left_img_names_.empty() ||
                    right_img_names_.empty();
  LOG_IF(ERROR, empty_data) << "Kitti Data is empty!";
  bool missing_data = timestamps_.size() ==
                      left_img_names_.size() ==
                      right_img_names_.size();
  LOG_IF(ERROR, missing_data) << "Missing Kitti Data!";
  LOG_IF(ERROR, empty_data || missing_data)
      << "# of timestamps: " << timestamps_.size() << '\n'
      << "# of left img names: " << left_img_names_.size() << '\n'
      << "# of right img names: " << right_img_names_.size();
  return empty_data || missing_data;
}

KittiDataProvider::KittiDataProvider(const std::string& kitti_dataset_path)
  : kitti_data_(),
    DataProvider() {
  // Parse Kitti dataset.
  parseData(kitti_dataset_path, &kitti_data_);
}

KittiDataProvider::~KittiDataProvider() {}

cv::Mat KittiDataProvider::readKittiImage(const std::string& img_name) {
    cv::Mat img = cv::imread(img_name, CV_LOAD_IMAGE_UNCHANGED);
    LOG_IF(FATAL, img.empty()) << "Failed to load image: " << img_name;
    return img;
}

bool KittiDataProvider::spin() {
  // Loop over the messages and call vio callback.
  // while(!end_of_dataset) {
  //  vio_callback_(your_stereo_imu_sync_packet);

  // Main loop
  Timestamp timestamp_frame_k;
  const size_t number_of_images = kitti_data_.getNumberOfImages();
  for(size_t k = 0; k < number_of_images; k++) {
    timestamp_frame_k = kitti_data_.timestamps_.at(k);

    // Call VIO Pipeline.
    VLOG(10) << "Call VIO processing for frame k: " << k
             << " with timestamp: " << timestamp_frame_k;
    //vio_callback_(StereoImuSyncPacket(
    //                StereoFrame(k, timestamp_frame_k,
    //                            readKittiImage(kitti_data_.left_img_names_.at(k));
    //                            left_cam_info,
    //                            readKittiImage(kitti_data_.right_img_names_.at(k));
    //                            right_cam_info,
    //                            camL_pose_camR,
    //                            stereo_matching_params),
    //                imu_meas.timestamps_,
    //                imu_meas.measurements_));
    /*
    TODO:
    https://github.com/yanii/kitti-pcl/blob/master/KITTI_README.TXT
      - Check the ETHDatasetParser::parseCameraData (prob next step)
      - need stereo_matching_params whatever that is 
      - How does the imu data come in? (check)
      - getImuDataInterpolatedUpperBorder
    */
    VLOG(10) << "Finished VIO processing for frame k = " << k;
  }

  return true;
}

void KittiDataProvider::parseData(const std::string& kitti_sequence_path,
                                  KittiData* kitti_data) const {
  // TODO: start here with example kitti data 
  // Images in Kitti dataset: datapath/image_02/data gives all (left?) images in order 
  // datapath/image_02/timestamps.txt gives the timestamps in order 
  // same for image_3 (right? images)
  CHECK_NOTNULL(kitti_data);

  std::string left_prefix = kitti_sequence_path + "/image_02";
  std::string right_prefix = kitti_sequence_path + "/image_03";

  // parse timestamps (left /image_02)
  // NOTE the timestamps for left and right cam not sychronized
  // TODO investigate of error accumulates 
  std::ifstream left_cam_times_stream, right_cam_times_stream; 
  std::string left_cam_times_path = left_prefix + "/timestamps.txt";
  std::string right_cam_times_path = right_prefix + "/timestamps.txt";

  left_cam_times_stream.open(left_cam_times_path.c_str());
  right_cam_times_stream.open(right_cam_times_path.c_str());
  CHECK(left_cam_times_stream.is_open());
  CHECK(right_cam_times_stream.is_open());
  while(!left_cam_times_stream.eof() && !right_cam_times_stream.eof()) {
    std::string line_lft, line_rht;
    getline(left_cam_times_stream, line_lft);
    getline(right_cam_times_stream, line_rht);
    // (for now take the later timestamp)
    // left timestamp
    double left_timestamp = -1;
    double right_timestamp = -1;
    if(!line_lft.empty()) {
      std::stringstream ss;
      std::replace(line_lft.begin(), line_lft.end(), ':', ' ');
      ss << line_lft;
      std::string date; 
      double hr, min, sec; 
      ss >> date >> hr >> min >> sec; 
      // formate time into double (nano seconds)
      double left_timestamp = (hr*3600 + min*60 + sec)*10E9;
    }
    // right timestamp
    if(!line_rht.empty()) {
      std::stringstream ss;
      std::replace(line_rht.begin(), line_rht.end(), ':', ' ');
      ss << line_rht;
      std::string date; 
      double hr, min, sec; 
      ss >> date >> hr >> min >> sec; 
      // formate time into double (nano seconds)
      double right_timestamp = (hr*3600 + min*60 + sec)*10E9;
    }
    if (left_timestamp != -1 || right_timestamp != -1){
      if (left_timestamp > right_timestamp){
        kitti_data->timestamps_.push_back(left_timestamp)
      }else{
        kitti_data->timestamps_.push_back(right_timestamp);
      }
    }
  }
  
  const size_t timestamps_size = kitti_data->timestamps_.size();
  kitti_data->left_img_names_.resize(timestamps_size);
  kitti_data->right_img_names_.resize(timestamps_size);

  for(size_t i = 0; i < timestamps_size; i++) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(10) << i;
    kitti_data->left_img_names_[i] = left_prefix + "/data/" + ss.str() + ".png";
    kitti_data->right_img_names_[i] = right_prefix + "/data/" + ss.str() + ".png";
  }

  // Check data is parsed correctly.
  CHECK(*kitti_data);
}

bool KittiDataProvider::parseCameraData(const std::string& input_dataset_path,
                                        const std::string& left_cam_name,
                                        const std::string& right_cam_name) {
  return true;
}

} // End of VIO namespace.
