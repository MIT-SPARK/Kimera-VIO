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
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/CameraParams.h"

#include <fstream>
#include <iostream>

#include <gtsam/navigation/ImuBias.h>

namespace VIO {

bool CameraParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);

  yaml_parser.getYamlParam("camera_id", &camera_id_);
  CHECK(!camera_id_.empty()) << "Camera id cannot be empty.";
  VLOG(1) << "Parsing camera parameters for: " << camera_id_;

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
  convertIntrinsicsVectorToMatrix(intrinsics_, &K_);

  // P_ = R_rectify_ * camera_matrix_;
  return true;
}

void CameraParams::parseDistortion(const YamlParser& yaml_parser) {
  std::string distortion_model;
  yaml_parser.getYamlParam("distortion_model", &distortion_model);
  yaml_parser.getYamlParam("camera_model", &camera_model_);
  distortion_model_ = stringToDistortion(distortion_model, camera_model_);
  CHECK(distortion_model_ == DistortionModel::RADTAN ||
        distortion_model_ == DistortionModel::EQUIDISTANT)
      << "Unsupported distortion model. Expected: radtan or equidistant.";
  yaml_parser.getYamlParam("distortion_coefficients", &distortion_coeff_);
  convertDistortionVectorToMatrix(distortion_coeff_, &distortion_coeff_mat_);
}

const DistortionModel CameraParams::stringToDistortion(
    const std::string& distortion_model,
    const std::string& camera_model) {
  std::string lower_case_distortion_model = distortion_model;
  std::string lower_case_camera_model = camera_model;

  std::transform(lower_case_distortion_model.begin(),
                 lower_case_distortion_model.end(),
                 lower_case_distortion_model.begin(),
                 ::tolower);
  std::transform(lower_case_camera_model.begin(),
                 lower_case_camera_model.end(),
                 lower_case_camera_model.begin(),
                 ::tolower);

  if (lower_case_camera_model == "pinhole") {
    if (lower_case_camera_model == std::string("none")) {
      return DistortionModel::NONE;
    } else if ((lower_case_distortion_model == std::string("plumb_bob")) ||
               (lower_case_distortion_model ==
                std::string("radial-tangential")) ||
               (lower_case_distortion_model == std::string("radtan"))) {
      return DistortionModel::RADTAN;
    } else if (lower_case_distortion_model == std::string("equidistant")) {
      return DistortionModel::EQUIDISTANT;
    } else {
      LOG(FATAL)
          << "Unrecognized distortion model for pinhole camera. Valid "
             "pinhole "
             "distortion model options are 'none', 'radtan', 'equidistant'.";
    }
  } else {
    LOG(FATAL)
        << "Unrecognized camera model. Valid camera models are 'pinhole'";
  }
}

void CameraParams::convertDistortionVectorToMatrix(
    const std::vector<double>& distortion_coeffs,
    cv::Mat* distortion_coeffs_mat) {
  CHECK_NOTNULL(distortion_coeffs_mat);
  CHECK_GE(distortion_coeffs.size(), 4u);
  *distortion_coeffs_mat = cv::Mat::zeros(1, distortion_coeffs.size(), CV_64F);
  for (int k = 0; k < distortion_coeffs_mat->cols; k++) {
    distortion_coeffs_mat->at<double>(0, k) = distortion_coeffs[k];
  }
}

void CameraParams::parseImgSize(const YamlParser& yaml_parser,
                                cv::Size* image_size) {
  CHECK_NOTNULL(image_size);
  std::vector<int> resolution;
  yaml_parser.getYamlParam("resolution", &resolution);
  CHECK_EQ(resolution.size(), 2);
  *image_size = cv::Size(resolution[0], resolution[1]);
}

void CameraParams::parseFrameRate(const YamlParser& yaml_parser,
                                  double* frame_rate) {
  CHECK_NOTNULL(frame_rate);
  int rate = 0;
  yaml_parser.getYamlParam("rate_hz", &rate);
  CHECK_GT(rate, 0u);
  *frame_rate = 1 / static_cast<double>(rate);
}

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

// TODO(Toni): Check if equidistant distortion is supported as well in gtsam.
// TODO(Toni): rather remove this function as it is only used in tests for
// uncalibrating the keypoints.. Use instead opencv.
void CameraParams::createGtsamCalibration(const cv::Mat& distortion,
                                          const Intrinsics& intrinsics,
                                          gtsam::Cal3DS2* calibration) {
  CHECK_NOTNULL(calibration);
  CHECK_GE(intrinsics.size(), 4);
  CHECK_GE(distortion.cols, 4);
  CHECK_EQ(distortion.rows, 1);
  *calibration = gtsam::Cal3DS2(intrinsics[0],                 // fx
                                intrinsics[1],                 // fy
                                0.0,                           // skew
                                intrinsics[2],                 // u0
                                intrinsics[3],                 // v0
                                distortion.at<double>(0, 0),   // k1
                                distortion.at<double>(0, 1),   // k2
                                distortion.at<double>(0, 2),   // p1 (k3)
                                distortion.at<double>(0, 3));  // p2 (k4)
}

//! Display all params.
void CameraParams::print() const {
  std::stringstream out;
  PipelineParams::print(out,
                        "Camera ID ",
                        camera_id_,
                        "Intrinsics: \n- fx",
                        intrinsics_[0],
                        "- fy",
                        intrinsics_[1],
                        "- cu",
                        intrinsics_[2],
                        "- cv",
                        intrinsics_[3],
                        "frame_rate_: ",
                        frame_rate_,
                        "image_size_: \n - width",
                        image_size_.width,
                        "- height",
                        image_size_.height);
  LOG(INFO) << out.str();
  LOG(INFO) << "- body_Pose_cam_: " << body_Pose_cam_ << '\n'
            << "- K: " << K_ << '\n'
            << "- Distortion Model:" << to_underlying(distortion_model_) << '\n'
            << "- Distortion Coeff:" << distortion_coeff_mat_;
}

//! Assert equality up to a tolerance.
bool CameraParams::equals(const CameraParams& cam_par,
                          const double& tol) const {
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
         UtilsOpenCV::compareCvMatsUpToTol(K_, cam_par.K_) &&
         UtilsOpenCV::compareCvMatsUpToTol(distortion_coeff_mat_,
                                           cam_par.distortion_coeff_mat_);
}

}  // namespace VIO
