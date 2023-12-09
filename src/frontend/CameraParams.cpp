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
 * @author Marcus Abate
 */

#include "kimera-vio/frontend/CameraParams.h"

#include <gtsam/navigation/ImuBias.h>

#include <fstream>
#include <iostream>
#include <opencv2/core/eigen.hpp>

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

  // use required rgbd field to toggle whether or not to parse depth params
  if (yaml_parser.hasParam("virtual_baseline")) {
    parseDepthParams(yaml_parser);
  } else {
    depth.valid = false;
  }

  // P_ = R_rectify_ * camera_matrix_;
  return true;
}

void CameraParams::parseDistortion(const YamlParser& yaml_parser) {
  std::string camera_model, distortion_model;
  yaml_parser.getYamlParam("distortion_model", &distortion_model);
  yaml_parser.getYamlParam("camera_model", &camera_model);
  camera_model_ = stringToCameraModel(camera_model);
  distortion_model_ = stringToDistortionModel(distortion_model, camera_model_);
  yaml_parser.getYamlParam("distortion_coefficients", &distortion_coeff_);
  convertDistortionVectorToMatrix(distortion_coeff_, &distortion_coeff_mat_);

  // For omni cam only
  if (camera_model_ == CameraModel::OMNI) {
    std::vector<double> omni_distortion_center, omni_affine;
    // std::vector<double> omni_pol_inv;  // Not required until omni point
    // projection is supported
    yaml_parser.getYamlParam("omni_distortion_center", &omni_distortion_center);
    CHECK_EQ(omni_distortion_center.size(), 2);
    omni_distortion_center_ << omni_distortion_center.at(0),
        omni_distortion_center.at(1);

    // Not required until omni point projection is supported
    // yaml_parser.getYamlParam("omni_pol_inv", &omni_pol_inv);
    // CHECK_EQ(omni_pol_inv.size(), 12);
    // // Assume invpol is of order 12:
    // for (size_t i = 0; i < 12; i++) {
    //   omni_pol_inv_(i) = omni_pol_inv.at(i);
    // }

    yaml_parser.getYamlParam("omni_affine", &omni_affine);
    CHECK_EQ(omni_affine.size(), 3);  // c, d, and e only
    omni_affine_ << 1.0, omni_affine.at(0), omni_affine.at(1),
        omni_affine.at(2);
    omni_affine_inv_ = omni_affine_.inverse();
  }
}

CameraModel CameraParams::stringToCameraModel(const std::string& camera_model) {
  std::string lower_case_camera_model = camera_model;
  std::transform(lower_case_camera_model.begin(),
                 lower_case_camera_model.end(),
                 lower_case_camera_model.begin(),
                 ::tolower);

  if (lower_case_camera_model == "pinhole") {
    return CameraModel::PINHOLE;
  } else if (lower_case_camera_model == "omni") {
    return CameraModel::OMNI;
  } else {
    LOG(FATAL) << "Unrecognized camera model. "
               << "Valid camera models are 'pinhole' and 'omni'";
  }
}

DistortionModel CameraParams::stringToDistortionModel(
    const std::string& distortion_model,
    const CameraModel& camera_model) {
  std::string lower_case_distortion_model = distortion_model;

  std::transform(lower_case_distortion_model.begin(),
                 lower_case_distortion_model.end(),
                 lower_case_distortion_model.begin(),
                 ::tolower);

  if (camera_model == CameraModel::PINHOLE) {
    if (lower_case_distortion_model == std::string("none")) {
      return DistortionModel::NONE;
    } else if ((lower_case_distortion_model == std::string("plumb_bob")) ||
               (lower_case_distortion_model ==
                std::string("radial-tangential")) ||
               (lower_case_distortion_model == std::string("radtan"))) {
      return DistortionModel::RADTAN;
    } else if (lower_case_distortion_model == std::string("equidistant")) {
      return DistortionModel::EQUIDISTANT;
    } else {
      LOG(FATAL) << "Unrecognized distortion model for pinhole camera: "
                 << lower_case_distortion_model
                 << " Valid pinhole distortion model options are 'none', "
                    "'radtan', 'equidistant'.";
    }
  } else if (camera_model == CameraModel::OMNI) {
    if (lower_case_distortion_model == std::string("omni")) {
      return DistortionModel::OMNI;
    } else {
      LOG(FATAL) << "Unrecognized distortion model for omni camera: "
                 << lower_case_distortion_model
                 << " Valid omni distortion model options are 'omni'.";
    }
  } else {
    LOG(FATAL) << "Unrecognized camera model. "
               << "Valid camera models are 'pinhole' and 'omni'.";
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
                                         Intrinsics* _intrinsics) {
  CHECK_NOTNULL(_intrinsics);
  std::vector<double> intrinsics;
  yaml_parser.getYamlParam("intrinsics", &intrinsics);
  if (intrinsics.size() == 4) {
    CHECK_EQ(intrinsics.size(), _intrinsics->size());
  } else {
    if (camera_model_ == CameraModel::OMNI) {
      CHECK_EQ(intrinsics.size(), 0);
      intrinsics.resize(4);
      // Use a more ideal pinhole camera model: based on Matlab impl: see
      // undistortFisheyePoints from Vision Toolbox
      // This only occurs if intrinsics are not provided in yaml file
      // focalLength = min(imageSize) / 2
      // kpt.x = focalLength * lmk.x + imageSize.y / 2 + 0.5
      // kpt.y = focalLength * lmk.y + imageSize.x / 2 + 0.5
      intrinsics.at(0) = std::min(image_size_.width, image_size_.height) / 2.0;
      intrinsics.at(1) = intrinsics.at(0);
      intrinsics.at(2) = (image_size_.width / 2.0) + 0.5;
      intrinsics.at(3) = (image_size_.height / 2.0) + 0.5;
    } else {
      LOG(FATAL) << "CameraParams: must have intrinsics specified for non-omni "
                    "camera!";
    }
  }

  // Move elements from one to the other.
  std::copy_n(std::make_move_iterator(intrinsics.begin()),
              _intrinsics->size(),
              _intrinsics->begin());
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
                        "image_size_: \n- width",
                        image_size_.width,
                        "- height",
                        image_size_.height,
                        "depth_: \n- virtual_baseline",
                        depth.virtual_baseline_,
                        "- depth_to_meters",
                        depth.depth_to_meters_,
                        "- min_depth",
                        depth.min_depth_,
                        "- max_depth",
                        depth.max_depth_,
                        "- is_registered",
                        depth.is_registered_);

  LOG(INFO) << out.str();
  LOG(INFO) << "- body_Pose_cam_: " << body_Pose_cam_ << '\n'
            << "- K: " << K_ << '\n'
            << "- Distortion Model:" << to_underlying(distortion_model_) << '\n'
            << "- Distortion Coeff:" << distortion_coeff_mat_;
}

template <typename T>
inline bool floatWithinTol(const T& v1, const T& v2, double tolerance) {
  return std::abs(v2 - v1) < static_cast<T>(tolerance);
}

//! Assert equality up to a tolerance.
bool CameraParams::equals(const CameraParams& cam_par,
                          const double& tol) const {
  bool areIntrinsicEqual = true;
  for (size_t i = 0; i < intrinsics_.size(); i++) {
    if (!floatWithinTol(intrinsics_[i], cam_par.intrinsics_[i], tol)) {
      areIntrinsicEqual = false;
      break;
    }
  }

  bool depth_params_equal;
  if (depth.valid && !cam_par.depth.valid) {
    depth_params_equal = true;
  } else if (depth.valid != cam_par.depth.valid) {
    depth_params_equal = false;
  } else {
    depth_params_equal =
        floatWithinTol(
            depth.virtual_baseline_, cam_par.depth.virtual_baseline_, tol) &&
        floatWithinTol(
            depth.depth_to_meters_, cam_par.depth.depth_to_meters_, tol) &&
        floatWithinTol(depth.min_depth_, cam_par.depth.min_depth_, tol) &&
        floatWithinTol(depth.max_depth_, cam_par.depth.max_depth_, tol);
  }

  return camera_id_ == cam_par.camera_id_ && areIntrinsicEqual &&
         depth_params_equal &&
         body_Pose_cam_.equals(cam_par.body_Pose_cam_, tol) &&
         floatWithinTol(frame_rate_, cam_par.frame_rate_, tol) &&
         (image_size_.width == cam_par.image_size_.width) &&
         (image_size_.height == cam_par.image_size_.height) &&
         UtilsOpenCV::compareCvMatsUpToTol(K_, cam_par.K_) &&
         UtilsOpenCV::compareCvMatsUpToTol(distortion_coeff_mat_,
                                           cam_par.distortion_coeff_mat_);
}

void CameraParams::parseDepthParams(const YamlParser& yaml_parser) {
  yaml_parser.getYamlParam("virtual_baseline", &depth.virtual_baseline_);
  CHECK_GT(depth.virtual_baseline_, 0.0) << "Baseline must be positive";
  yaml_parser.getYamlParam("depth_to_meters", &depth.depth_to_meters_);
  yaml_parser.getYamlParam("min_depth", &depth.min_depth_);
  yaml_parser.getYamlParam("max_depth", &depth.max_depth_);
  yaml_parser.getYamlParam("is_registered", &depth.is_registered_);

  if (!depth.is_registered_) {
    std::vector<double> pose_elements;
    yaml_parser.getNestedYamlParam("T_color_depth", "data", &pose_elements);
    gtsam::Pose3 T_CD = UtilsOpenCV::poseVectorToGtsamPose3(pose_elements);

    // convert to cv::Mat for more efficient registration
    depth.T_color_depth_ = cv::Mat(4, 4, CV_64F);
    cv::eigen2cv(T_CD.matrix(), depth.T_color_depth_);

    std::vector<double> intrinsics;
    yaml_parser.getYamlParam("depth_intrinsics", &intrinsics);
    depth.K_ = cv::Mat::eye(3, 3, CV_64F);
    depth.K_.at<double>(0, 0) = intrinsics[0];
    depth.K_.at<double>(1, 1) = intrinsics[1];
    depth.K_.at<double>(0, 2) = intrinsics[2];
    depth.K_.at<double>(1, 2) = intrinsics[3];
  }

  depth.valid = true;
}

}  // namespace VIO
