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

DEFINE_int32(initial_frame, 50, "Initial frame to start processing dataset, "
                            "previous frames will not be used.");
DEFINE_int32(final_frame, 10000, "Final frame to finish processing dataset, "
                             "subsequent frames will not be used.");

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
  return !empty_data && !missing_data;
}

KittiDataProvider::KittiDataProvider(const std::string& kitti_dataset_path)
  : kitti_data_(),
    DataProvider() {
  dataset_path_ = kitti_dataset_path;
  // Parse Kitti dataset.
  parseData(kitti_dataset_path, &kitti_data_);
}

KittiDataProvider::~KittiDataProvider() {}

cv::Mat KittiDataProvider::readKittiImage(const std::string& img_name) {
    cv::Mat img = cv::imread(img_name, CV_LOAD_IMAGE_UNCHANGED);
    LOG_IF(FATAL, img.empty()) << "Failed to load image: " << img_name;
    // cv::imshow("check", img); 
    // cv::waitKey(0);
    return img;
}

bool KittiDataProvider::spin() {
  // Loop over the messages and call vio callback.
  // Timestamp 10 frames before the first (for imu calibration)
  static constexpr size_t frame_offset_for_imu_calib = 10;
  Timestamp timestamp_last_frame = 
      kitti_data_.timestamps_.at(kitti_data_.initial_k_ - frame_offset_for_imu_calib);
  Timestamp timestamp_frame_k;

  const size_t number_of_images = kitti_data_.getNumberOfImages();

  const StereoMatchingParams& stereo_matching_params = 
      frontend_params_.getStereoMatchingParams();

  // Store camera info 
  const CameraParams& left_cam_info = 
          kitti_data_.camera_info_.at(kitti_data_.left_camera_name_);
  const CameraParams& right_cam_info = 
          kitti_data_.camera_info_.at(kitti_data_.right_camera_name_);
  const gtsam::Pose3& camL_pose_camR = 
          kitti_data_.camL_Pose_camR_; 

  // Main loop
  for(size_t k = kitti_data_.initial_k_; k < kitti_data_.final_k_; k++) {
    timestamp_frame_k = kitti_data_.timestamps_.at(k);
    ImuMeasurements imu_meas; 
    CHECK(utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable ==
          kitti_data_.imuData_.imu_buffer_.getImuDataInterpolatedUpperBorder(
          timestamp_last_frame,
          timestamp_frame_k,
          &imu_meas.timestamps_,
          &imu_meas.measurements_)) 
          << "Make sure queried timestamp lies before the first IMU sample in the buffer";
    // Call VIO Pipeline.
    VLOG(10) << "Call VIO processing for frame k: " << k
             << " with timestamp: " << timestamp_frame_k
             << "////////////////////////////////////////// Creating packet!\n"
             << "STAMPS IMU rows : \n" << imu_meas.timestamps_.rows() << '\n'
             << "STAMPS IMU cols : \n" << imu_meas.timestamps_.cols() << '\n'
             << "STAMPS IMU: \n" << imu_meas.timestamps_ << '\n'
             << "ACCGYR IMU rows : \n" << imu_meas.measurements_.rows() << '\n'
             << "ACCGYR IMU cols : \n" << imu_meas.measurements_.cols() << '\n'
             << "ACCGYR IMU: \n" << imu_meas.measurements_;
    vio_callback_(StereoImuSyncPacket(
                   StereoFrame(k, timestamp_frame_k,
                               readKittiImage(kitti_data_.left_img_names_.at(k)),
                               left_cam_info,
                               readKittiImage(kitti_data_.right_img_names_.at(k)),
                               right_cam_info,
                               camL_pose_camR,
                               stereo_matching_params),
                               imu_meas.timestamps_,
                               imu_meas.measurements_));

    VLOG(10) << "Finished VIO processing for frame k = " << k;
  }

  timestamp_last_frame = timestamp_frame_k; 

  return true;
}

void KittiDataProvider::parseData(const std::string& kitti_sequence_path,
                                  KittiData* kitti_data) {
  // Images in Kitti dataset: datapath/image_02/data gives all (left) images in order 
  // datapath/image_02/timestamps.txt gives the timestamps in order 
  // same for image_3 (right images)
  CHECK_NOTNULL(kitti_data);

  std::string left_cam = "00"; 
  std::string right_cam = "01";

  std::string left_prefix = kitti_sequence_path + "/image_" + left_cam;
  std::string right_prefix = kitti_sequence_path + "/image_" + right_cam;

  // parse timestamps (left /image_02)
  // NOTE the timestamps for left and right cam not sychronized
  // TODO investigate if error accumulates 
  // Parse time stamps
  std::string left_timestamp_file = left_prefix + "/timestamps.txt"; 
  std::string right_timestamp_file = right_prefix + "/timestamps.txt";
  std::vector<Timestamp> left_timestamps; 
  std::vector<Timestamp> right_timestamps; 

  parseTimestamps(left_timestamp_file, left_timestamps);
  parseTimestamps(right_timestamp_file, right_timestamps);

  LOG_IF(FATAL, left_timestamps.size() < 1 || right_timestamps.size() < 1)
      << "ParseTimestamps: zero timestamps parsed for image data...";

  // for now take the later one 
  if (left_timestamps[0] > right_timestamps[0]) {
    kitti_data->timestamps_ = left_timestamps;
  }else{
    kitti_data->timestamps_ = right_timestamps;
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

  // Parse camera info and imu data 
  parseCameraData(kitti_sequence_path, left_cam, right_cam, kitti_data);
  parseImuData(kitti_sequence_path, kitti_data);

  // Start processing dataset from frame initial_k (neccessary on convinient)
  kitti_data->initial_k_ = FLAGS_initial_frame; 
  CHECK_GE(kitti_data->initial_k_, 10) << "initial_k should be >= 10 for imu bias initialization";
  kitti_data->final_k_ = FLAGS_final_frame; 
  const size_t& nr_images = kitti_data->getNumberOfImages();
  if (kitti_data->final_k_ > nr_images) {
    LOG(WARNING) << "Value for final_k, " << kitti_data->final_k_ << " is larger than total"
                 << " number of frames in dataset " << nr_images;
    // Skip last frames which are typically problematic
    // (IMU bumps, FOV occluded)...
    static constexpr size_t skip_n_end_frames = 100;
    kitti_data->final_k_ = nr_images - skip_n_end_frames;
    LOG(WARNING) << "Using final_k = " << kitti_data->final_k_ << ", where we removed "
                 << skip_n_end_frames << " frames to avoid bad IMU readings.";
  }
  CHECK(kitti_data->final_k_ > kitti_data->initial_k_)
      << "Value for final_k (" << kitti_data->final_k_ << ") is smaller than value for"
      << " initial_k (" << kitti_data->initial_k_ << ").";

  LOG(INFO) << "Running dataset between frame " << kitti_data->initial_k_
            << " and frame " <<  kitti_data->final_k_;

  // Check data is parsed correctly.
  CHECK(*kitti_data);
  print(); 
}

bool KittiDataProvider::parseTimestamps(const std::string& timestamps_file, 
                                        std::vector<Timestamp>& timestamps_list) const {
  std::ifstream times_stream; 
  times_stream.open(timestamps_file.c_str());
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
  kitti_data->left_camera_name_ = left_cam_id;
  kitti_data->right_camera_name_ = right_cam_id;

  // Read camera info and list of images.
  std::vector<std::string> camera_names;
  camera_names.push_back(left_cam_id);
  camera_names.push_back(right_cam_id);
  kitti_data->camera_info_.clear();

  // First get R_imu2velo and T_imu2velo 
  cv::Mat R_imu2velo, T_imu2velo;
  std::string imu_to_velo_filename = "calib_imu_to_velo.txt"; 
  parseRT(input_dataset_path, imu_to_velo_filename, R_imu2velo, T_imu2velo);
  
  // Then get R_velo2cam and T_velo2cam
  cv::Mat R_velo2cam, T_velo2cam;
  std::string velo_to_cam_filename = "calib_velo_to_cam.txt";
  parseRT(input_dataset_path, velo_to_cam_filename, R_velo2cam, T_velo2cam);

  // Then form the rotation matrix R_imu2body
  cv::Mat R_imu2body;  // In case the convention is different
  // R_imu2body = cv::Mat::zeros(3, 3, CV_64F);
  // R_imu2body.at<double>(0,2) = -1; 
  // R_imu2body.at<double>(1,1) = 1;
  // R_imu2body.at<double>(2,0) = 1; 
  R_imu2body = cv::Mat::eye(3, 3, CV_64F); 

  // The find transformation from camera to imu frame (since that will be body frame)
  cv::Mat R_cam2body, T_cam2body; 
  R_cam2body = R_imu2body * R_imu2velo.t() * R_velo2cam.t();
  T_cam2body = -R_imu2body * T_imu2velo - R_imu2body * R_imu2velo.t() * T_velo2cam; 

  for (const std::string& cam_name: camera_names) {
    LOG(INFO) << "reading camera: " << cam_name;
    CameraParams cam_info_i;
    cam_info_i.parseKITTICalib(input_dataset_path + "calib_cam_to_cam.txt", 
                               R_cam2body, T_cam2body, cam_name);
    kitti_data->camera_info_[cam_name] = cam_info_i;
  }

  // Set extrinsic for the stereo.
  const CameraParams& left_camera_info = kitti_data->camera_info_.at(left_cam_id);
  const CameraParams& right_camera_info = kitti_data->camera_info_.at(right_cam_id);

  // Extrinsics of the stereo (not rectified)
  // relative pose between cameras
  kitti_data->camL_Pose_camR_ = (left_camera_info.body_Pose_cam_).between(
                      right_camera_info.body_Pose_cam_);

  return true;
}

bool KittiDataProvider::parseRT(
                const std::string& input_dataset_path, 
                const std::string& calibration_filename, 
                cv::Mat& R, cv::Mat& T) const {
  std::ifstream calib_file; 
  calib_file.open((input_dataset_path + calibration_filename).c_str());
  // Read calibratio file
  CHECK(calib_file.is_open());
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
                    KittiData* kitti_data) {
  ///////////////// PARSE IMU PARAMETERS ///////////////////////////////////////

  // body_Pose_cam_: sensor pose w respect to body 
  // IMU chosen as body frame 
  cv::Mat R_body, T_body; 

  // Then form the rotation matrix R_imu2body
  cv::Mat R_imu2body; 
  // R_imu2body = cv::Mat::zeros(3, 3, CV_64F);
  // R_imu2body.at<double>(0,2) = -1; 
  // R_imu2body.at<double>(1,1) = 1;
  // R_imu2body.at<double>(2,0) = 1; 
  R_imu2body = cv::Mat::eye(3, 3, CV_64F); 

  R_body = R_imu2body;
  T_body = cv::Mat::zeros(3, 1, CV_64F);

  kitti_data->imuData_.body_Pose_cam_ = UtilsOpenCV::Cvmats2pose(R_body, T_body);

  // NOTE that in this case left grayscale stereo camera is chosen as body frame (ok?) 

  int rate = 100; // According to KITTI Readme 
  kitti_data->imuData_.nominal_imu_rate_ = 1 / double(rate);
  // NOTE gyro noise/walk and acc noise/walk not given in KITTI data 

  ///////////////// GET TIMESTAMP //////////////////////////////////////////////
  std::string oxts_timestamps_filename = input_dataset_path + "/oxts/timestamps.txt";
  std::vector<Timestamp> oxts_timestamps; 
  parseTimestamps(oxts_timestamps_filename, oxts_timestamps);
  LOG_IF(FATAL, oxts_timestamps.size() < 1) 
              << "ParseTimestamps: zero timestamps parsed for imu data...";

  ///////////////// PARSE ACTUAL DATA //////////////////////////////////////////
  std::string oxtsdata_filename = input_dataset_path + "/oxts/data/";
  // measurement at each timestep is in a text file starting from 
  // dataset_path/oxts/data/0000000000.txt extract imu data according to kitti readme
  
  size_t deltaCount = 0u;
  Timestamp sumOfDelta = 0;
  double stdDelta = 0;
  double imu_rate_maxMismatch = 0;
  double maxNormAcc = 0, maxNormRotRate = 0; // only for debugging

  for (size_t i = 0; i < oxts_timestamps.size(); i++) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(10) << i;
    std::string oxts_file_i = oxtsdata_filename + ss.str() + ".txt";
    std::ifstream oxts_data_i; 
    oxts_data_i.open(oxts_file_i.c_str());
    LOG_IF(FATAL, !oxts_data_i.is_open()) << "Cannot open oxts file: " << oxts_file_i;

    // All information should be on one line (according example file)
    // Store words in vector for easy access by index 
    std::vector<std::string> oxts_data_str; 
    std::string item; 
    while (getline(oxts_data_i, item, ' ')){
      oxts_data_str.push_back(item);
    }
    
    // Read/store imu measurements 
    gtsam::Vector6 gyroAccData; 
    // terms 17~19 (starting from 0) are wx, wy, wz
    for (int k = 0; k < 3; k++) {
      gyroAccData(k) = std::stod(oxts_data_str[k + 17]);
    }
    // terms 11~13 (starting from 0) are ax, ay, az
    for (int k = 3; k < 6; k++) {
      gyroAccData(k) = std::stod(oxts_data_str[k + 8]);
    }

    Vector6 imu_accgyr;
    // Acceleration first! 
    imu_accgyr << gyroAccData.tail(3), gyroAccData.head(3);

    double normAcc = gyroAccData.tail(3).norm();
    if(normAcc > maxNormAcc) maxNormAcc = normAcc;

    double normRotRate = gyroAccData.head(3).norm();
    if(normRotRate > maxNormRotRate) maxNormRotRate = normRotRate;

    kitti_data->imuData_.imu_buffer_.addMeasurement(oxts_timestamps[i], imu_accgyr);
    if (i != 0){
      sumOfDelta += (oxts_timestamps[i] - oxts_timestamps[i-1]);
      double deltaMismatch = std::fabs(double(oxts_timestamps[i] - 
                                              oxts_timestamps[i-1] -
                                              kitti_data->imuData_.nominal_imu_rate_) * 1e-9);
      stdDelta += std::pow(deltaMismatch, 2);
      imu_rate_maxMismatch = std::max(imu_rate_maxMismatch, deltaMismatch);
      deltaCount += 1u;
    }
    oxts_data_i.close(); 
  }

  LOG_IF(FATAL, deltaCount != kitti_data->imuData_.imu_buffer_.size() - 1u)
      << "parseImuData: wrong nr of deltaCount: deltaCount "
      << deltaCount << " nr lines "
      << kitti_data->imuData_.imu_buffer_.size();

  // Converted to seconds.
  kitti_data->imuData_.imu_rate_ = (double(sumOfDelta) / double(deltaCount)) * 1e-9;
  kitti_data->imuData_.imu_rate_std_ = std::sqrt(stdDelta / double(deltaCount-1u));
  kitti_data->imuData_.imu_rate_maxMismatch_ = imu_rate_maxMismatch;

  LOG(INFO) << "Maximum measured rotation rate (norm):" << maxNormRotRate << '-'
            << "Maximum measured acceleration (norm): " << maxNormAcc;
  return true;
}

/* -------------------------------------------------------------------------- */
void KittiDataProvider::print() const {
  LOG(INFO) << "-------------------------------------------------------------\n"
            << "------------------ KittiDataProvider::print ------------------\n"
            << "-------------------------------------------------------------\n"
            << "Displaying info for dataset: " << dataset_path_;
  kitti_data_.camL_Pose_camR_.print("camL_Pose_calR \n");
  // For each of the 2 cameras.
  LOG(INFO) << "Left camera name: " << kitti_data_.left_camera_name_ << ", with params:\n";
  kitti_data_.camera_info_.at(kitti_data_.left_camera_name_).print();
  LOG(INFO) << "Left camera name: " << kitti_data_.left_camera_name_ << ", with params:\n";
  kitti_data_.camera_info_.at(kitti_data_.left_camera_name_).print();
  kitti_data_.imuData_.print();
  LOG(INFO) << "-------------------------------------------------------------\n"
            << "-------------------------------------------------------------\n"
            << "-------------------------------------------------------------";
}


} // End of VIO namespace.
