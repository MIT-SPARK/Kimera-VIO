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

#include <gtsam/navigation/ImuBias.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace VIO {

/* -------------------------------------------------------------------------- */
// Parse YAML file describing camera parameters.
bool CameraParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);

  yaml_parser.getYamlParam("camera_id", &camera_id_);
  CHECK(!camera_id_.empty()) << "Camera id cannot be empty.";
  LOG(INFO) << "Parsing camera parameters for: " << camera_id_;

  // Distortion parameters.
  parseDistortion(yaml_parser);

  // Camera resolution.
  parseImgSize(yaml_parser, &image_size_);

  // Camera frame rate.
  parseFrameRate(yaml_parser, &frame_rate_);

  // Camera pose wrt body.
  parseBodyPoseCam(yaml_parser, &body_Pose_cam_);

  // Intrinsics.
  parseCameraIntrinsics(yaml_parser, &intrinsics_);

  // Convert intrinsics to cv::Mat format.
  convertIntrinsicsVectorToMatrix(intrinsics_, &camera_matrix_);

  // Create gtsam calibration object.
  // Calibration of a camera with radial distortion that also supports
  createGtsamCalibration(
      distortion_model_, distortion_coeff_, intrinsics_, &distortion_);

  // P_ = R_rectify_ * camera_matrix_;
  return true;
}

/* -------------------------------------------------------------------------- */
void CameraParams::parseDistortion(const YamlParser& yaml_parser) {
  yaml_parser.getYamlParam("distortion_model", &distortion_model_);
  // 4 parameters (read from file)
  CHECK(distortion_model_ == "radtan" ||
        distortion_model_ == "radial-tangential" ||
        distortion_model_ == "equidistant")
      << "Unsupported distortion model. Expected: radtan or equidistant,"
         "but got instead: "
      << distortion_model_.c_str();
  std::vector<double> distortion_coeff;
  yaml_parser.getYamlParam("distortion_coefficients", &distortion_coeff);
  CHECK_EQ(distortion_coeff.size(), 4);
  convertDistortionVectorToMatrix(distortion_coeff, &distortion_coeff_);
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
void CameraParams::parseImgSize(const YamlParser& yaml_parser,
                                cv::Size* image_size) {
  CHECK_NOTNULL(image_size);
  std::vector<int> resolution;
  yaml_parser.getYamlParam("resolution", &resolution);
  CHECK_EQ(resolution.size(), 2);
  *image_size = cv::Size(resolution[0], resolution[1]);
}

/* -------------------------------------------------------------------------- */
void CameraParams::parseFrameRate(const YamlParser& yaml_parser,
                                  double* frame_rate) {
  CHECK_NOTNULL(frame_rate);
  int rate = 0;
  yaml_parser.getYamlParam("rate_hz", &rate);
  CHECK_GT(rate, 0u);
  *frame_rate = 1 / static_cast<double>(rate);
}

/* -------------------------------------------------------------------------- */
void CameraParams::parseBodyPoseCam(const YamlParser& yaml_parser,
                                    gtsam::Pose3* body_Pose_cam) {
  CHECK_NOTNULL(body_Pose_cam);
  // int n_rows = 0;
  // yaml_parser.getNestedYamlParam("T_BS", "rows", &n_rows);
  // CHECK_EQ(n_rows, 4);
  // int n_cols = 0;
  // yaml_parser.getNestedYamlParam("T_BS", "cols", &n_cols);
  // CHECK_EQ(n_cols, 4);
  std::vector<double> vector_pose;
  yaml_parser.getNestedYamlParam("T_BS", "data", &vector_pose);
  *body_Pose_cam = UtilsOpenCV::poseVectorToGtsamPose3(vector_pose);
}

/* -------------------------------------------------------------------------- */
void CameraParams::parseCameraIntrinsics(const YamlParser& yaml_parser,
                                         Intrinsics* intrinsics_) {
  CHECK_NOTNULL(intrinsics_);
  std::vector<double> intrinsics;
  yaml_parser.getYamlParam("intrinsics", &intrinsics);
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
void CameraParams::createGtsamCalibration(const std::string& model,
                                          const cv::Mat& distortion,
                                          const Intrinsics& intrinsics,
                                          DistortionModelConstPtr* dm) {
  CHECK_NOTNULL(dm);
  CHECK_GE(intrinsics.size(), 4);
  std::vector<double> dist_vec;
  for (size_t i = 0; i < distortion.total(); i++) {
    dist_vec.push_back(distortion.at<double>(i));
  }
  if (DistortionModel::is_valid(model, dist_vec.size())) {
    *dm = DistortionModel::make(model, intrinsics, dist_vec);
  }
}

/* -------------------------------------------------------------------------- */
// Display all params.
void CameraParams::print() const {
  std::string output;
  for (size_t i = 0; i < intrinsics_.size(); i++) {
    output += std::to_string(intrinsics_.at(i)) + " , ";
  }
  LOG(INFO) << "------------ Camera ID: " << camera_id_ << " -------------\n"
            << "intrinsics_: " << output;

  LOG(INFO) << "body_Pose_cam_: \n" << body_Pose_cam_ << std::endl;

  if (FLAGS_minloglevel < 1) {
    if (distortion_) {
      distortion_->print();
    } else {
      LOG(INFO) << "no distortion model present yet!" << std::endl;
    }
  }

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
bool CameraParams::equals(const CameraParams& cam_par,
                          const double& tol) const {
  bool areIntrinsicEqual = true;
  for (size_t i = 0; i < intrinsics_.size(); i++) {
    if (std::fabs(intrinsics_[i] - cam_par.intrinsics_[i]) > tol) {
      areIntrinsicEqual = false;
      break;
    }
  }
  bool distortionMatches = (distortion_ && cam_par.distortion_ &&
                            distortion_->equals(cam_par.distortion_)) ||
                           (!distortion_ && !cam_par.distortion_);

  return camera_id_ == cam_par.camera_id_ && areIntrinsicEqual &&
         body_Pose_cam_.equals(cam_par.body_Pose_cam_, tol) &&
         (std::fabs(frame_rate_ - cam_par.frame_rate_) < tol) &&
         (image_size_.width == cam_par.image_size_.width) &&
         (image_size_.height == cam_par.image_size_.height) &&
         distortionMatches &&
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
