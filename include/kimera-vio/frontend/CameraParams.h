/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   CameraParams.h
 * @brief  Parameters describing a monocular camera.
 * @author Antoni Rosinol, Luca Carlone
 */

#pragma once

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/UtilsOpenCV.h"

namespace VIO {

// TODO(Toni): leaving this here is a bit ugly...
using Calibration = gtsam::Cal3DS2;

/*
 * Class describing camera parameters.
 */
class CameraParams {
 public:
  /* ------------------------------------------------------------------------ */
  // Parse YAML file describing camera parameters.
  bool parseYAML(const std::string& filepath);

  /* ------------------------------------------------------------------------ */
  static void parseDistortion(const cv::FileStorage& fs,
                              const std::string& filepath,
                              std::string* distortion_model,
                              std::vector<double>* distortion_coeff);
  static void parseImgSize(const cv::FileStorage& fs, cv::Size* image_size);
  static void parseFrameRate(const cv::FileStorage& fs, double* frame_rate);
  static void parseBodyPoseCam(const cv::FileStorage& fs,
                               gtsam::Pose3* body_Pose_cam);
  static void parseCameraIntrinsics(const cv::FileStorage& fs,
                                    std::vector<double>* intrinsics_);
  // Convert distortion coefficients to OpenCV Format
  static void convertDistortionVectorToMatrix(
      const std::vector<double>& distortion_coeffs,
      cv::Mat* distortion_coeffs_mat);
  static void convertIntrinsicsVectorToMatrix(
      const std::vector<double>& intrinsics,
      cv::Mat* camera_matrix);
  static void createGtsamCalibration(const std::vector<double>& distortion,
                                     const std::vector<double>& intrinsics,
                                     gtsam::Cal3DS2* calibration);

  /* ------------------------------------------------------------------------ */
  // Parse KITTI calib file describing camera parameters.
  bool parseKITTICalib(const std::string& filepath,
                       cv::Mat R_cam_to_imu,
                       cv::Mat T_cam_to_imu,
                       const std::string& cam_id);

  /* ------------------------------------------------------------------------ */
  // Display all params.
  void print() const;

  /* ------------------------------------------------------------------------ */
  // Assert equality up to a tolerance.
  bool equals(const CameraParams& cam_par, const double& tol = 1e-9) const;

 public:
  // fu, fv, cu, cv
  std::vector<double> intrinsics_;

  // Sensor extrinsics wrt. body-frame
  gtsam::Pose3 body_Pose_cam_;

  // Image info.
  double frame_rate_;
  cv::Size image_size_;

  // GTSAM structures to calibrate points.
  Calibration calibration_;

  // OpenCV structures: For radial distortion and rectification.
  // needed to compute the undistorsion map.
  cv::Mat camera_matrix_;
  // 5 parameters (last is zero): distortion_model: radial-tangential.
  std::string distortion_model_;  // define default
  cv::Mat distortion_coeff_;

  cv::Mat undistRect_map_x_;
  cv::Mat undistRect_map_y_;

  // Rotation resulting from rectification.
  cv::Mat R_rectify_;

  // Camera matrix after rectification.
  cv::Mat P_;
};

}  // namespace VIO
