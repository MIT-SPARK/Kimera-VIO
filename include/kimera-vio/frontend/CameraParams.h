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

#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace VIO {

/*
 * Class describing camera parameters.
 */
class CameraParams {
 public:
  KIMERA_POINTER_TYPEDEFS(CameraParams);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using CameraId = std::string;

  CameraParams(const CameraId& cam_id = "cam") : camera_id_(cam_id) {}
  virtual ~CameraParams() = default;

  /* ------------------------------------------------------------------------ */
  // Parse YAML file describing camera parameters.
  bool parseYAML(const std::string& filepath);

  /* ------------------------------------------------------------------------ */
  // TODO(Toni): remove this, make calib to be converted to Euroc format.
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
  // Id of the camera
  CameraId camera_id_;

  // fu, fv, cu, cv
  std::vector<double> intrinsics_;

  // Sensor extrinsics wrt. body-frame
  gtsam::Pose3 body_Pose_cam_;

  // Image info.
  double frame_rate_;
  cv::Size image_size_;

  // GTSAM structures to calibrate points.
  gtsam::Cal3DS2 calibration_;

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

  //! List of Cameras which share field of view with this one: i.e. stereo.
  std::vector<CameraId> is_stereo_with_camera_ids_;

 private:
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
};
// TODO(Toni): this should be a base class, so that stereo camera is a specific
// type of a multi camera sensor rig, or something along these lines.
typedef std::vector<CameraParams> MultiCameraParams;

}  // namespace VIO
