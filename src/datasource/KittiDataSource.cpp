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
  // Images in Kitti dataset: datapath/image_02/data gives all (left) images in order 
  // datapath/image_02/timestamps.txt gives the timestamps in order 
  // same for image_3 (right images)
  CHECK_NOTNULL(kitti_data);

  std::string left_prefix = kitti_sequence_path + "/image_02";
  std::string right_prefix = kitti_sequence_path + "/image_03";

  // parse timestamps (left /image_02)
  // NOTE the timestamps for left and right cam not sychronized
  // TODO investigate if error accumulates 
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
    Timestamp left_timestamp = 0;
    Timestamp right_timestamp = 0;
    static constexpr int seconds_per_hour = 3600u; 
    static constexpr int seconds_per_minute = 60u; 
    static constexpr long int seconds_to_nanoseconds = 10e9; // overflow 

    // left timestamp
    if(!line_lft.empty()) {
      std::stringstream ss;
      std::replace(line_lft.begin(), line_lft.end(), ':', ' ');
      ss << line_lft;
      std::string date; 
      double hr, min, sec; 
      ss >> date >> hr >> min >> sec; 
      // formate time into double (nano seconds)
      left_timestamp = (hr * seconds_per_hour 
                        + min * seconds_per_minute 
                        + sec) * seconds_to_nanoseconds; 
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
      right_timestamp = (hr * seconds_per_hour 
                         + min * seconds_per_minute 
                         + sec) * seconds_to_nanoseconds; 
    }
    if (left_timestamp != 0 || right_timestamp != 0){
      if (left_timestamp > right_timestamp){
        kitti_data->timestamps_.push_back(left_timestamp);
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

bool KittiDataProvider::parseTimestamps(const std::string& input_dataset_path, 
                                        const std::string& timestamps_file, 
                                        std::vector<Timestamp>& timestamps_list) {
  std::ifstream times_stream; 
  times_stream.open((input_dataset_path + timestamps_file).c_str());
  CHECK(times_stream.is_open());
  timestamps_list.clear(); 
  static constexpr int seconds_per_hour = 3600u; 
  static constexpr int seconds_per_minute = 60u; 
  static constexpr long int seconds_to_nanoseconds = 10e9;
  // Loop through timestamps text file
  while(!times_stream.eof()){
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
      Timestamp timestamp = (hr * seconds_per_hour
                             + min * seconds_per_minute
                             + sec) * seconds_to_nanoseconds;
      timestamps_list.push_back(timestamp);
    }
  }
  return true; 
}

bool KittiDataProvider::parseCameraData(const std::string& input_dataset_path, 
                                        const std::string& left_cam_id,
                                        const std::string& right_cam_id, 
                                        KittiData* kitti_data) {
  // note that the stamps and images were parsed in parseData method 
  // perhaps move all that into this method? 
  // for now write parse camera info here 
  // Default names: match names of the corresponding folders.
  kitti_data->camera_names_.resize(2);
  kitti_data->camera_names_[0] = left_cam_id;
  kitti_data->camera_names_[1] = right_cam_id;

  // Read camera info and list of images.
  kitti_data->camera_info_.clear();
  for (const std::string& cam_name: kitti_data->camera_names_) {
    LOG(INFO) << "reading camera: " << cam_name;
    CameraParams cam_info_i;
    cam_info_i.parseKITTICalib(input_dataset_path + "calib_cam_to_cam.txt", cam_name);
    kitti_data->camera_info_[cam_name] = cam_info_i;
  }

  return true;
}

bool KittiDataProvider::parseRT(
                const std::string& input_dataset_path, 
                const std::string& calibration_filename, 
                cv::Mat& R, cv::Mat& T) {
  std::ifstream calib_file; 
  calib_file.open((input_dataset_path + calibration_filename).c_str());
  // Read calibratio file
  std::ifstream calib_velo_to_cam; 
  calib_velo_to_cam.open((input_dataset_path + "/calib_velo_to_cam.txt").c_str());
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
        //store in R matrix 
        R = cv::Mat::zeros(3, 3, CV_64F);
        double value; 
        for (int i = 0; i < 9; i++){
          ss >> value; 
          int row = i/3; int col = i%3; 
          R.at<double>(row, col) = value; 
        }

      }else if (label == "T:") {
        // store in T vector
        T = cv::Mat::zeros(3, 1, CV_64F);
        double value;
        for (int i = 0; i < 3; i++){
          ss >> value;  
          T.at<double>(i, 0) = value; 
        }

      }
    }
  }
  return true; 
}

bool KittiDataProvider::parseImuData(
                    const std::string& input_dataset_path, 
                    KittiData* kitti_data_) {
  ///////////////// PARSE IMU PARAMETERS ///////////////////////////////////////
  std::string filename_sensor = input_dataset_path + "/oxts/data/";
  // measurement at each timestep is in a text file starting from 
  // dataset_path/oxts/data/0000000000.txt

  // body_Pose_cam_: sensor to body transformation
  // calib_velo_to_cam R|T transform velodyne coords to left vid cam frame 
  // calib_imu_to_velo R|T transform imu coord to velodyne frame 

  // First get R_imu2velo and T_imu2velo 
  cv::Mat R_imu2velo, T_imu2velo;
  std::string imu_to_velo_filename = "/calib_imu_to_velo.txt"; 
  parseRT(input_dataset_path, imu_to_velo_filename, R_imu2velo, T_imu2velo);
  
  // Then get R_velo2cam and T_velo2cam
  cv::Mat R_velo2cam, T_velo2cam;
  std::string velo_to_cam_filename = "/calib_velo_to_cam.txt";
  parseRT(input_dataset_path, velo_to_cam_filename, R_velo2cam, T_velo2cam);
  
  // Calculate R_imu2cam and T_imu2cam
  cv::Mat R_imu2cam, T_imu2cam; 
  R_imu2cam = R_velo2cam * R_imu2velo;
  T_imu2cam = T_velo2cam + T_imu2velo;

  kitti_data_->imuData_.body_Pose_cam_ = UtilsOpenCV::Cvmats2pose(R_imu2cam, T_imu2cam);

  // Note that in this case left grayscale stereo camera is chosen as body frame 

  int rate = 100; // According to KITTI Readme 
  kitti_data_->imuData_.nominal_imu_rate_ = 1 / double(rate);
  // NOTE gyro noise/walk and acc noise/walk not given in KITTI data 

  // ///////////////// PARSE ACTUAL DATA //////////////////////////////////////////
  // //#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],
  // // a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
  // std::string filename_data = input_dataset_path + "/mav0/" + imuName + "/data.csv";
  // std::ifstream fin (filename_data.c_str());
  // LOG_IF(FATAL, !fin.is_open()) << "Cannot open file: " << filename_data;

  // // Skip the first line, containing the header.
  // std::string line;
  // std::getline(fin, line);

  // size_t deltaCount = 0u;
  // Timestamp sumOfDelta = 0;
  // double stdDelta = 0;
  // double imu_rate_maxMismatch = 0;
  // double maxNormAcc = 0, maxNormRotRate = 0; // only for debugging
  // Timestamp previous_timestamp = -1;

  // // Read/store imu measurements, line by line.
  // while (std::getline(fin, line)) {
  //   Timestamp timestamp = 0;
  //   gtsam::Vector6 gyroAccData;
  //   for (size_t i = 0; i < 7; i++) {
  //     int idx = line.find_first_of(',');
  //     if (i == 0) {
  //       timestamp = std::stoll(line.substr(0, idx));
  //     } else {
  //       gyroAccData(i-1) = std::stod(line.substr(0, idx));
  //     }
  //     line = line.substr(idx+1);
  //   }
  //   Vector6 imu_accgyr;
  //   // Acceleration first!
  //   imu_accgyr << gyroAccData.tail(3), gyroAccData.head(3);

  //   double normAcc = gyroAccData.tail(3).norm();
  //   if(normAcc > maxNormAcc) maxNormAcc = normAcc;

  //   double normRotRate = gyroAccData.head(3).norm();
  //   if(normRotRate > maxNormRotRate) maxNormRotRate = normRotRate;

  //   imuData_.imu_buffer_.addMeasurement(timestamp, imu_accgyr);
  //   if (previous_timestamp == -1) {
  //     // Do nothing.
  //     previous_timestamp = timestamp;
  //   } else {
  //     sumOfDelta += (timestamp - previous_timestamp);
  //     double deltaMismatch = std::fabs(double(timestamp - previous_timestamp -
  //                                             imuData_.nominal_imu_rate_) * 1e-9);
  //     stdDelta += std::pow(deltaMismatch, 2);
  //     imu_rate_maxMismatch = std::max(imu_rate_maxMismatch, deltaMismatch);
  //     deltaCount += 1u;
  //     previous_timestamp = timestamp;
  //   }
  // }

  // LOG_IF(FATAL, deltaCount != imuData_.imu_buffer_.size() - 1u)
  //     << "parseImuData: wrong nr of deltaCount: deltaCount "
  //     << deltaCount << " nr lines "
  //     << imuData_.imu_buffer_.size();

  // // Converted to seconds.
  // imuData_.imu_rate_ = (double(sumOfDelta) / double(deltaCount)) * 1e-9;
  // imuData_.imu_rate_std_ = std::sqrt(stdDelta / double(deltaCount-1u));
  // imuData_.imu_rate_maxMismatch_ = imu_rate_maxMismatch;
  // fin.close();

  // LOG(INFO) << "Maximum measured rotation rate (norm):" << maxNormRotRate << '-'
  //           << "Maximum measured acceleration (norm): " << maxNormAcc;
  // return true;
}

} // End of VIO namespace.
