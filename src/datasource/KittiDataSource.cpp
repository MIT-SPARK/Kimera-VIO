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
    VLOG(10) << "Finished VIO processing for frame k = " << k;
  }

  return true;
}

void KittiDataProvider::parseData(const std::string& kitti_sequence_path,
                                  KittiData* kitti_data) const {
  CHECK_NOTNULL(kitti_data);
  std::ifstream times_stream;
  std::string times_path = kitti_sequence_path + "/times.txt";
  times_stream.open(times_path.c_str());
  CHECK(times_stream.is_open());
  while(!times_stream.eof()) {
    std::string line;
    getline(times_stream, line);
    if(!line.empty()) {
      std::stringstream ss;
      ss << line;
      double timestamp;
      ss >> timestamp;
      kitti_data->timestamps_.push_back(timestamp);
    }
  }

  std::string left_prefix = kitti_sequence_path + "/image_0/";
  std::string right_prefix = kitti_sequence_path + "/image_1/";

  const size_t timestamps_size = kitti_data->timestamps_.size();
  kitti_data->left_img_names_.resize(timestamps_size);
  kitti_data->right_img_names_.resize(timestamps_size);

  for(size_t i = 0; i < timestamps_size; i++) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << i;
    kitti_data->left_img_names_[i] = left_prefix + ss.str() + ".png";
    kitti_data->right_img_names_[i] = right_prefix + ss.str() + ".png";
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
