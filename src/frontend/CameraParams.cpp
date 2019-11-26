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

#include "kimera-vio/frontend/CameraParams.h"

#include <iostream>
#include <fstream>

#include <gtsam/navigation/ImuBias.h>

namespace VIO {

/* -------------------------------------------------------------------------- */
// Parse YAML file describing camera parameters.
bool CameraParams::parseYAML(const std::string& filepath) {
  // Make sure that each YAML file has %YAML:1.0 as first line.
  cv::FileStorage fs;
  UtilsOpenCV::safeOpenCVFileStorage(&fs, filepath);

  // Distortion parameters.
  std::vector<double> distortion_;
  parseDistortion(fs, filepath,
                  &distortion_model_,
                  &distortion_);
  convertDistortionVectorToMatrix(distortion_, &distortion_coeff_);

  // Camera resolution.
  parseImgSize(fs, &image_size_);

  // Camera frame rate.
  parseFrameRate(fs, &frame_rate_);

  // Camera pose wrt body.
  parseBodyPoseCam(fs, &body_Pose_cam_);

  // Intrinsics.
  parseCameraIntrinsics(fs, &intrinsics_);

  // Convert intrinsics to cv::Mat format.
  convertIntrinsicsVectorToMatrix(intrinsics_, &camera_matrix_);

  // Create gtsam calibration object.
  // Calibration of a camera with radial distortion that also supports
  createGtsamCalibration(distortion_, intrinsics_, &calibration_);

  // P_ = R_rectify_ * camera_matrix_;
  fs.release();
  return true;
}

/* -------------------------------------------------------------------------- */
// TODO(Toni): DEPRECATE. Move this code into a Kitti->Euroc converter or
// something, and re-use parseYAML.
// Parse KITTI calibration txt file describing camera parameters.
bool CameraParams::parseKITTICalib(const std::string& filepath,
                                   cv::Mat R_cam_to_body, cv::Mat T_cam_to_body,
                                   const std::string& cam_id) {
  // rate is approx 10 hz as given by the README
  frame_rate_ = 1 / 10.0;

  // rate is approx 10 hz as given by the README
  frame_rate_ = 1 / 10.0;

  // set up R and T matrices
  cv::Mat rotation = cv::Mat::zeros(3, 3, CV_64F);
  cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);
  distortion_model_ = "radial-tangential";
  // Set up to read the text file
  std::ifstream calib_file;
  calib_file.open(filepath.c_str());
  CHECK(calib_file.is_open()) << "Could not open file at: " << filepath.c_str();
  std::vector<double> distortion_coeff5_;
  // read loop
  while (!calib_file.eof()) {
    std::string line;
    getline(calib_file, line);

    if (!line.empty()) {
      std::stringstream ss;
      ss << line;
      std::string label;
      ss >> label;
      if (label == "S_" + cam_id + ":") {
        // this entry gives image size
        double width, height;
        ss >> width >> height;
        image_size_ = cv::Size(width, height);
      } else if (label == "K_" + cam_id + ":") {
        // this entry gives the camera matrix
        std::vector<double> K_vector;
        double value;  // store values in Kvector
        while (ss >> value) K_vector.push_back(value);
        intrinsics_[0] = K_vector[0];
        intrinsics_[1] = K_vector[4];
        intrinsics_[1] = K_vector[2];
        intrinsics_[1] = K_vector[5];
        // Convert intrinsics to OpenCV Format.
        camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix_.at<double>(0, 0) = intrinsics_[0];
        camera_matrix_.at<double>(1, 1) = intrinsics_[1];
        camera_matrix_.at<double>(0, 2) = intrinsics_[2];
        camera_matrix_.at<double>(1, 2) = intrinsics_[3];
      } else if (label == "D_" + cam_id + ":") {
        // this entry gives the distortion coeffs
        distortion_coeff_ = cv::Mat::zeros(1, 5, CV_64F);
        double value;
        while (ss >> value) distortion_coeff5_.push_back(value);
        for (int k = 0; k < 5; k++) {
          distortion_coeff_.at<double>(0, k) = distortion_coeff5_[k];
        }

      }else if (label == "R_" + cam_id + ":") {
        // this entry gives the rotation matrix
        double value;
        for (int i = 0; i < 9; i++){
          ss >> value;
          int row = i/3; int col = i%3;
          rotation.at<double>(row, col) = value;
        }

      }else if (label == "T_" + cam_id + ":") {
        // this entry gives the translation
        double value;
        for (int i = 0; i < 3; i++){
          ss >> value;
          translation.at<double>(i, 0) = value;
        }

      } else if (label == "R_rect_" + cam_id + ":") {
        // this entry gives the rotation resulting from rectification.
        R_rectify_ = cv::Mat::zeros(3, 3, CV_64F);
        double value;
        for (int i = 0; i < 9; i++) {
          ss >> value;
          int row = i / 3;
          int col = i % 3;
          R_rectify_.at<double>(row, col) = value;
        }
      } else if (label == "P_rect_" + cam_id + ":") {
        // this entry gives the camera matric from rectification
        P_ = cv::Mat::zeros(3, 4, CV_64F);
        double value;
        for (int i = 0; i < 12; i++) {
          ss >> value;
          int row = i / 4;
          int col = i % 4;
          P_.at<double>(row, col) = value;
        }
      }
    }
  }

  // Cam pose wrt to body
  rotation = R_cam_to_body * rotation.t();
  translation = T_cam_to_body - R_cam_to_body * translation;

  body_Pose_cam_ = UtilsOpenCV::cvMatsToGtsamPose3(rotation, translation);

  calibration_ = gtsam::Cal3DS2(intrinsics_[0],          // fx
                                intrinsics_[1],          // fy
                                0.0,                     // skew
                                intrinsics_[2],          // u0
                                intrinsics_[3],          // v0
                                distortion_coeff5_[0],   //  k1
                                distortion_coeff5_[1],   //  k2
                                distortion_coeff5_[3],   //  p1
                                distortion_coeff5_[4]);  //  p2
  // NOTE check if ignorting the 3rd distortion coeff is correct

  return true;
}

/* -------------------------------------------------------------------------- */
void CameraParams::parseDistortion(
    const cv::FileStorage& fs,
    const std::string& filepath,
    std::string* distortion_model,
    std::vector<double>* distortion_coeff) {
  CHECK_NOTNULL(distortion_model);
  CHECK_NOTNULL(distortion_coeff)->clear();
  // 4 parameters (read from file)
  fs["distortion_model"] >> *distortion_model;
  CHECK(*distortion_model == "radtan" ||
        *distortion_model == "radial-tangential" ||
        *distortion_model == "equidistant")
      << "Unsupported distortion model. Expected: radtan or equidistant,"
         "but got instead: " << distortion_model->c_str();
  fs["distortion_coefficients"] >> *distortion_coeff;
  CHECK_EQ(distortion_coeff->size(), 4);
}

/* -------------------------------------------------------------------------- */
// Convert distortion coefficients to OpenCV Format
void CameraParams::convertDistortionVectorToMatrix(
    const std::vector<double>& distortion_coeffs,
    cv::Mat* distortion_coeffs_mat) {
  CHECK_NOTNULL(distortion_coeffs_mat);
  CHECK_EQ(distortion_coeffs.size(), 4);
  *distortion_coeffs_mat = cv::Mat::zeros(1, distortion_coeffs.size(), CV_64F);
  for (int k = 0; k < distortion_coeffs_mat->cols; k++) {
    distortion_coeffs_mat->at<double>(0, k) = distortion_coeffs[k];
  }
}

/* -------------------------------------------------------------------------- */
void CameraParams::parseImgSize(const cv::FileStorage& fs,
                                cv::Size* image_size) {
  CHECK_NOTNULL(image_size);
  std::vector<int> resolution;
  resolution.clear();
  fs["resolution"] >> resolution;
  CHECK_EQ(resolution.size(), 2);
  *image_size = cv::Size(resolution[0], resolution[1]);
}

/* -------------------------------------------------------------------------- */
void CameraParams::parseFrameRate(const cv::FileStorage& fs,
                                  double* frame_rate) {
  CHECK_NOTNULL(frame_rate);
  int rate = fs["rate_hz"];
  CHECK_GT(rate, 0u);
  *frame_rate = 1 / static_cast<double>(rate);
}

/* -------------------------------------------------------------------------- */
void CameraParams::parseBodyPoseCam(const cv::FileStorage& fs,
                                    gtsam::Pose3* body_Pose_cam) {
  CHECK_NOTNULL(body_Pose_cam);
  int n_rows = static_cast<int>(fs["T_BS"]["rows"]);
  int n_cols = static_cast<int>(fs["T_BS"]["cols"]);
  CHECK_GT(n_rows, 0u);
  CHECK_GT(n_cols, 0u);
  std::vector<double> vector_pose;
  fs["T_BS"]["data"] >> vector_pose;
  *body_Pose_cam = UtilsOpenCV::poseVectorToGtsamPose3(vector_pose);
}

/* -------------------------------------------------------------------------- */
void CameraParams::parseCameraIntrinsics(const cv::FileStorage& fs,
                                         Intrinsics* intrinsics_) {
  CHECK_NOTNULL(intrinsics_);
  std::vector<double> intrinsics;
  fs["intrinsics"] >> intrinsics;
  CHECK_EQ(intrinsics.size(), intrinsics_->size());
  // Move elements from one to the other.
  std::copy_n(std::make_move_iterator(intrinsics.begin()),
              intrinsics_->size(),
              intrinsics_->begin());
}

/* -------------------------------------------------------------------------- */
void CameraParams::convertIntrinsicsVectorToMatrix(const Intrinsics& intrinsics,
                                                   cv::Mat* camera_matrix) {
  CHECK_NOTNULL(camera_matrix);
  DCHECK_EQ(intrinsics.size(), 4);
  *camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix->at<double>(0, 0) = intrinsics[0];
  camera_matrix->at<double>(1, 1) = intrinsics[1];
  camera_matrix->at<double>(0, 2) = intrinsics[2];
  camera_matrix->at<double>(1, 2) = intrinsics[3];
}

/* -------------------------------------------------------------------------- */
// TODO(Toni) : Check if equidistant distortion is supported as well in gtsam.
void CameraParams::createGtsamCalibration(const std::vector<double>& distortion,
                                          const Intrinsics& intrinsics,
                                          gtsam::Cal3DS2* calibration) {
  CHECK_NOTNULL(calibration);
  CHECK_GE(intrinsics.size(), 4);
  CHECK_GE(distortion.size(), 4);
  *calibration = gtsam::Cal3DS2(
      intrinsics[0],   // fx
      intrinsics[1],   // fy
      0.0,             // skew
      intrinsics[2],   // u0
      intrinsics[3],   // v0
      distortion[0],   // k1
      distortion[1],   // k2
      distortion[2],   // p1 (k3)
      distortion[3]);  // p2 (k4)
}

/* -------------------------------------------------------------------------- */
// Display all params.
void CameraParams::print() const {
  std::string output;
  for(size_t i = 0; i < intrinsics_.size(); i++) {
    output += std::to_string(intrinsics_.at(i)) + " , ";
  }
  LOG(INFO) << "------------ Camera ID: " << camera_id_ << " -------------\n"
            << "intrinsics_: " << output;

  LOG(INFO) << "body_Pose_cam_: \n" << body_Pose_cam_ << std::endl;

  if (FLAGS_minloglevel < 1)
    calibration_.print("\n gtsam calibration:\n");

  LOG(INFO) << "frame_rate_: " << frame_rate_ << '\n'
            << "image_size_: width= " << image_size_.width
            << " height= " << image_size_.height << '\n'
            << "camera_matrix_: \n"
            << camera_matrix_ << '\n'
            << "distortion_model_: " << distortion_model_ << '\n'
            << "distortion_coeff_: \n"
            << distortion_coeff_ << '\n'
            << "R_rectify_: \n"
            << R_rectify_ << '\n'
            << "undistRect_map_y_ too large to display (only created in "
            << "StereoFrame)" << '\n'
            << "P_: \n"
            << P_ << '\n';
}

/* -------------------------------------------------------------------------- */
// Assert equality up to a tolerance.
bool CameraParams::equals(const CameraParams& cam_par, const double& tol) const {
  bool areIntrinsicEqual = true;
  for (size_t i = 0; i < intrinsics_.size(); i++) {
    if (std::fabs(intrinsics_[i] - cam_par.intrinsics_[i]) > tol) {
      areIntrinsicEqual = false;
      break;
    }
  }
  return camera_id_ == cam_par.camera_id_ && areIntrinsicEqual &&
         body_Pose_cam_.equals(cam_par.body_Pose_cam_, tol) &&
         (std::fabs(frame_rate_ - cam_par.frame_rate_) < tol) &&
         (image_size_.width == cam_par.image_size_.width) &&
         (image_size_.height == cam_par.image_size_.height) &&
         calibration_.equals(cam_par.calibration_, tol) &&
         UtilsOpenCV::compareCvMatsUpToTol(camera_matrix_,
                                           cam_par.camera_matrix_) &&
         UtilsOpenCV::compareCvMatsUpToTol(distortion_coeff_,
                                           cam_par.distortion_coeff_) &&
         UtilsOpenCV::compareCvMatsUpToTol(undistRect_map_x_,
                                           cam_par.undistRect_map_x_) &&
         UtilsOpenCV::compareCvMatsUpToTol(undistRect_map_y_,
                                           cam_par.undistRect_map_y_) &&
         UtilsOpenCV::compareCvMatsUpToTol(R_rectify_, cam_par.R_rectify_) &&
         UtilsOpenCV::compareCvMatsUpToTol(P_, cam_par.P_);
}

}  // namespace VIO
