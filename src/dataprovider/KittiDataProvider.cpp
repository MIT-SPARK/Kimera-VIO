/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   KittiDataProvider.cpp
 * @brief  Kitti dataset parser.
 * @author Antoni Rosinol
 * @author Yun Chang
 */
#include "kimera-vio/dataprovider/KittiDataProvider.h"

#include <opencv2/core/core.hpp>

#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"

namespace VIO {

KittiDataProvider::KittiData::operator bool() const {
  bool empty_data = timestamps_.empty() || left_img_names_.empty() ||
                    right_img_names_.empty();
  LOG_IF(ERROR, empty_data) << "Kitti Data is empty!";
  bool missing_data =
      timestamps_.size() == left_img_names_.size() == right_img_names_.size();
  LOG_IF(ERROR, missing_data) << "Missing Kitti Data!";
  LOG_IF(ERROR, empty_data || missing_data)
      << "# of timestamps: " << timestamps_.size() << '\n'
      << "# of left img names: " << left_img_names_.size() << '\n'
      << "# of right img names: " << right_img_names_.size();
  return !empty_data && !missing_data;
}

KittiDataProvider::KittiDataProvider()
    : DataProviderInterface(), kitti_data_() {
  // Parse Kitti dataset.
  std::string dataset_path = "Not implemented...";
  parseKittiData(dataset_path, &kitti_data_);
}

cv::Mat KittiDataProvider::readKittiImage(const std::string& img_name) {
  cv::Mat img = cv::imread(img_name, cv::ImreadModes::IMREAD_UNCHANGED);
  LOG_IF(FATAL, img.empty()) << "Failed to load image: " << img_name;
  // cv::imshow("check", img);
  // cv::waitKey(0);
  return img;
}

bool KittiDataProvider::spin() {
  LOG(FATAL) << "Not implemented.";
}

bool Earlier_time(std::pair<Timestamp, std::string>& a,
                  std::pair<Timestamp, std::string>& b) {
  // for sorting below
  return a.first < b.first;
}

void KittiDataProvider::parseKittiData(const std::string& kitti_sequence_path,
                                       KittiData* kitti_data) {
  LOG(FATAL) << "Not implemented";
}

bool KittiDataProvider::parseTimestamps(
    const std::string& timestamps_file,
    std::vector<Timestamp>& timestamps_list) const {
  std::ifstream times_stream;
  times_stream.open(timestamps_file.c_str());
  CHECK(times_stream.is_open())
      << "Could not open timestamps file: " << timestamps_file;
  timestamps_list.clear();
  static constexpr int seconds_per_hour = 3600u;
  static constexpr int seconds_per_minute = 60u;
  static constexpr long int seconds_to_nanoseconds = 1e9;
  // Loop through timestamps text file
  while (!times_stream.eof()) {
    std::string line;
    getline(times_stream, line);
    if (!line.empty()) {
      std::stringstream ss;
      std::replace(line.begin(), line.end(), ':', ' ');
      ss << line;
      std::string date;
      double hr, min, sec;
      ss >> date >> hr >> min >> sec;
      // formate time into Timestamp (in nanosecs)
      Timestamp timestamp =
          (hr * seconds_per_hour + min * seconds_per_minute + sec) *
          seconds_to_nanoseconds;
      timestamps_list.push_back(timestamp);
    }
  }
  return true;
}

bool KittiDataProvider::parseCameraData(const std::string& input_dataset_path,
                                        const std::string& left_cam_id,
                                        const std::string& right_cam_id,
                                        KittiData* kitti_data) {
  // note that the stamps and images were parsed in parseKittiData method
  // perhaps move all that into this method?
  // for now write parse camera info here
  // Default names: match names of the corresponding folders.
  kitti_data->left_camera_name_ = left_cam_id;
  kitti_data->right_camera_name_ = right_cam_id;

  // Read camera info and list of images.
  std::vector<std::string> camera_names;
  camera_names.push_back(left_cam_id);
  camera_names.push_back(right_cam_id);

  // First get R_imu2velo and T_imu2velo
  cv::Mat R_imu2velo, T_imu2velo;
  std::string imu_to_velo_filename = "/../calib_imu_to_velo.txt";
  parsePose(input_dataset_path, imu_to_velo_filename, R_imu2velo, T_imu2velo);

  // Then get R_velo2cam and T_velo2cam
  cv::Mat R_velo2cam, T_velo2cam;
  std::string velo_to_cam_filename = "/../calib_velo_to_cam.txt";
  parsePose(input_dataset_path, velo_to_cam_filename, R_velo2cam, T_velo2cam);

  // Then form the rotation matrix R_imu2body
  cv::Mat R_imu2body;  // In case the convention is different
  // R_imu2body = cv::Mat::zeros(3, 3, CV_64F);
  // R_imu2body.at<double>(0,2) = -1;
  // R_imu2body.at<double>(1,1) = 1;
  // R_imu2body.at<double>(2,0) = 1;
  R_imu2body = cv::Mat::eye(3, 3, CV_64F);

  // The find transformation from camera to imu frame (since that will be body
  // frame)
  cv::Mat R_cam2body, T_cam2body;
  R_cam2body = R_imu2body * R_imu2velo.t() * R_velo2cam.t();
  T_cam2body =
      -R_imu2body * T_imu2velo - R_imu2body * R_imu2velo.t() * T_velo2cam;

  for (const std::string& cam_name : camera_names) {
    LOG(FATAL) << "Parsing Kitti files directly is NOT supported";
    LOG(INFO) << "reading camera: " << cam_name;
    // CameraParams cam_info_i(cam_name);
    // cam_info_i.parseKITTICalib(input_dataset_path +
    // "/../calib_cam_to_cam.txt",
    //                            R_cam2body, T_cam2body, cam_name);
    // pipeline_params_.camera_params_.push_back(cam_info_i);
    LOG(INFO) << "parsed camera: " << cam_name;
  }

  return true;
}

bool KittiDataProvider::parsePose(
                const std::string& input_dataset_path,
                const std::string& calibration_filename,
                cv::Mat& R, cv::Mat& T) const {
  std::ifstream calib_file;
  std::string calibration_file_path = input_dataset_path + calibration_filename;
  calib_file.open(calibration_file_path.c_str());
  // Read calibration file
  CHECK(calib_file.is_open()) << "Could not open calibration files located at: "
                              << calibration_file_path;
  // Read loop
  while (!calib_file.eof()) {
    std::string line;
    getline(calib_file, line);
    if (!line.empty()) {
      std::stringstream ss;
      ss << line;
      std::string label;
      ss >> label;

      if (label == "R:") {
        // store in R matrix
        R = cv::Mat::zeros(3, 3, CV_64F);
        double value;
        for (int i = 0; i < 9; i++) {
          ss >> value;
          int row = i / 3;
          int col = i % 3;
          R.at<double>(row, col) = value;
        }

      } else if (label == "T:") {
        // store in T vector
        T = cv::Mat::zeros(3, 1, CV_64F);
        double value;
        for (int i = 0; i < 3; i++) {
          ss >> value;
          T.at<double>(i, 0) = value;
        }
      }
    }
  }
  return true;
}

bool Earlier_time_imu(std::pair<Timestamp, int>& a,
                      std::pair<Timestamp, int>& b) {
  // for sorting below
  return a.first < b.first;
}

bool KittiDataProvider::parseImuData(const std::string& input_dataset_path,
                                     KittiData* kitti_data) {
  return false;
}

/* -------------------------------------------------------------------------- */
void KittiDataProvider::print() const {
}

}  // namespace VIO
