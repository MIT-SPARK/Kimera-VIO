/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   CameraParams.cpp
 * @brief  Parameters describing a monocular camera.
 * @author Antoni Rosinol, Luca Carlone
 */

#include "CameraParams.h"

namespace VIO {

/* -------------------------------------------------------------------------- */
// Parse YAML file describing camera parameters.
bool CameraParams::parseYAML(const std::string& filepath) {
  // Make sure that each YAML file has %YAML:1.0 as first line.
  cv::FileStorage fs;
  UtilsOpenCV::safeOpenCVFileStorage(&fs, filepath);

  // Intrinsics.
  intrinsics_.clear();
  fs["intrinsics"] >> intrinsics_;

  // 4 parameters (read from file): distortion_model: radial-tangential
  // TODO allow for different distortion models, at least equidistant as well!
  std::vector<double> distortion_coeff4_;
  distortion_coeff4_.clear();
  fs["distortion_coefficients"] >> distortion_coeff4_;
  // Convert distortion coefficients to OpenCV Format
  distortion_coeff_ = cv::Mat::zeros(1, 5, CV_64F);
  for (int k = 0; k < 4; k++) {
    distortion_coeff_.at<double>(0, k) = distortion_coeff4_[k];
  }

  // Camera resolution.
  std::vector<int> resolution;
  resolution.clear();
  fs["resolution"] >> resolution;
  image_size_ = cv::Size(resolution[0], resolution[1]);

  // Camera frame rate.
  int rate = fs["rate_hz"];
  frame_rate_ = 1 / double(rate);

  // Cam pose wrt to body.
  int n_rows = (int)fs["T_BS"]["rows"];
  int n_cols = (int)fs["T_BS"]["cols"];
  std::vector<double> vec;
  fs["T_BS"]["data"] >> vec;
  body_Pose_cam_ = UtilsOpenCV::Vec2pose(vec,n_rows,n_cols);

  fs.release();

  // Convert intrinsics to OpenCV Format.
  camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix_.at<double>(0, 0) = intrinsics_[0];
  camera_matrix_.at<double>(1, 1) = intrinsics_[1];
  camera_matrix_.at<double>(0, 2) = intrinsics_[2];
  camera_matrix_.at<double>(1, 2) = intrinsics_[3];

  calibration_ = gtsam::Cal3DS2(intrinsics_[0], // fx
      intrinsics_[1], // fy
      0.0,           // skew
      intrinsics_[2], // u0
      intrinsics_[3], // v0
      distortion_coeff4_[0],  //  k1
      distortion_coeff4_[1],  //  k2
      distortion_coeff4_[2],  //  p1
      distortion_coeff4_[3]); //  p2

  return true;
}

/* -------------------------------------------------------------------------- */
// Parse KITTI calibration txt file describing camera parameters.
bool CameraParams::parseKITTICalib(const std::string& filepath, const std::string& cam_id) {
  // rate is approx 10 hz as given by the README 
  frame_rate_ = 1 / 10.0; 

  // set up R and T matrices 
  cv::Mat cvR = cv::Mat::zeros(3, 3, CV_64F);
  cv::Mat cvT = cv::Mat::zeros(3, 1, CV_64F);

  // Set up to read the text file 
  std::ifstream calib_file; 
  calib_file.open(filepath.c_str());
  bool read_all_cam_info = false;

  // read loop
  while(!calib_file.eof() && !read_all_cam_info) {
    std::string line; 
    getline(calib_file, line);
    if (!line.empty()) {
      std::stringstream ss; 
      ss << line; 
      std::string label; 
      ss >> label; 
      std::cout << "label: " << label << std::endl; 
      if (label == "S_" + cam_id + ":") {
        // this entry gives image size 
        double width, height; 
        ss >> width >> height; 
        std::cout << "w: " << width << " h: " << height << std::endl; 
        image_size_ = cv::Size(width, height);
      }else if (label == "K_" + cam_id + ":") {
        // this entry gives the camera matrix 
        std::vector<double> K_vector; 
        double value; // store values in Kvector 
        while (ss >> value) K_vector.push_back(value);
        intrinsics_.clear(); 
        intrinsics_.push_back(K_vector[0]); 
        intrinsics_.push_back(K_vector[4]);
        intrinsics_.push_back(K_vector[2]);
        intrinsics_.push_back(K_vector[5]);
        // Convert intrinsics to OpenCV Format.
        camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix_.at<double>(0, 0) = intrinsics_[0];
        camera_matrix_.at<double>(1, 1) = intrinsics_[1];
        camera_matrix_.at<double>(0, 2) = intrinsics_[2];
        camera_matrix_.at<double>(1, 2) = intrinsics_[3];
      }else if (label == "D_" + cam_id + ":") {
        // this entry gives the distortion coeffs 
        distortion_coeff_ = cv::Mat::zeros(1, 5, CV_64F);
        std::vector<double> D_vector; 
        double value; 
        while (ss >> value) D_vector.push_back(value); 
        for (int k = 0; k < 5; k++) {
          distortion_coeff_.at<double>(0,k) = D_vector[k]; 
        }

      }else if (label == "R_" + cam_id + ":") {
        // this entry gives the rotation matrix 
        std::vector<double> Rvect; 
        double value; 
        while (ss >> value) Rvect.push_back(value); 
        cvR.at<double>(0, 0) = Rvect[0];
        cvR.at<double>(0, 1) = Rvect[1];
        cvR.at<double>(0, 2) = Rvect[2];
        cvR.at<double>(1, 0) = Rvect[3];
        cvR.at<double>(1, 1) = Rvect[4];
        cvR.at<double>(1, 2) = Rvect[5];
        cvR.at<double>(2, 0) = Rvect[6];
        cvR.at<double>(2, 1) = Rvect[7];
        cvR.at<double>(2, 2) = Rvect[8];

      }else if (label == "T_" + cam_id + ":") {
        // this entry gives the translation 
        std::vector<double> Tvect; 
        double value; 
        while (ss >> value) Tvect.push_back(value); 
        cvT.at<double>(0, 0) = Tvect[0];
        cvT.at<double>(1, 0) = Tvect[1];
        cvT.at<double>(2, 0) = Tvect[2];
      }
    }
  }

  // Cam pose wrt to body.
  body_Pose_cam_ = UtilsOpenCV::Vec2pose(cvR, cvT);

  calibration_ = gtsam::Cal3DS2(intrinsics_[0], // fx
      intrinsics_[1], // fy
      0.0,           // skew
      intrinsics_[2], // u0
      intrinsics_[3], // v0
      distortion_coeff_[0],  //  k1
      distortion_coeff_[1],  //  k2
      distortion_coeff_[2],  //  p1
      distortion_coeff_[3]); //  p2

  return true;
}

/* -------------------------------------------------------------------------- */
// Display all params.
void CameraParams::print() const {
  LOG(INFO) << "------------ CameraParams::print -------------\n"
            << "intrinsics_: ";
  for(size_t i = 0; i < intrinsics_.size(); i++) {
    std::cout <<  intrinsics_.at(i) << " , ";
  }
  std::cout << std::endl;

  LOG(INFO) << "body_Pose_cam_: \n" << body_Pose_cam_ << std::endl;

  calibration_.print("gtsam calibration:\n");

  LOG(INFO) << "frame_rate_: " << frame_rate_ << '\n'
            << "image_size_: width= " << image_size_.width
            << " height= " << image_size_.height << '\n'
            << "camera_matrix_: \n" << camera_matrix_ << '\n'
            << "distortion_coeff_: \n" << distortion_coeff_ << '\n'
            << "R_rectify_: \n" << R_rectify_ << '\n'
            << "undistRect_map_y_ too large to display (only created in "
            << "StereoFrame)" << '\n'
            << "P_: \n" << P_ << '\n';
}

/* -------------------------------------------------------------------------- */
// Assert equality up to a tolerance.
bool CameraParams::equals(const CameraParams& camPar, const double& tol) const {
  bool areIntrinsicEqual = true;
  for (size_t i = 0; i < intrinsics_.size(); i++) {
    if (std::fabs(intrinsics_[i] - camPar.intrinsics_[i]) > tol) {
      areIntrinsicEqual = false;
      break;
    }
  }
  return areIntrinsicEqual &&
      body_Pose_cam_.equals(camPar.body_Pose_cam_,tol) &&
      (std::fabs(frame_rate_ - camPar.frame_rate_) < tol) &&
      (image_size_.width == camPar.image_size_.width)  &&
      (image_size_.height == camPar.image_size_.height)  &&
      calibration_.equals(camPar.calibration_, tol) &&
      UtilsOpenCV::CvMatCmp(camera_matrix_, camPar.camera_matrix_) &&
      UtilsOpenCV::CvMatCmp(distortion_coeff_, camPar.distortion_coeff_) &&
      UtilsOpenCV::CvMatCmp(undistRect_map_x_, camPar.undistRect_map_x_) &&
      UtilsOpenCV::CvMatCmp(undistRect_map_y_, camPar.undistRect_map_y_) &&
      UtilsOpenCV::CvMatCmp(R_rectify_, camPar.R_rectify_) &&
      UtilsOpenCV::CvMatCmp(P_, camPar.P_);
}

} // End of VIO namespace.
