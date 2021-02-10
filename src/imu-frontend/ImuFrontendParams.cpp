/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontendParams.cpp
 * @brief  Params for ImuFrontend.
 * @author Antoni Rosinol
 */

#include "kimera-vio/imu-frontend/ImuFrontendParams.h"

#include <glog/logging.h>

#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/utils/UtilsOpenCV.h"
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

ImuParams::ImuParams() : PipelineParams("IMU params") {}

bool ImuParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);

  int imu_preintegration_type;
  yaml_parser.getYamlParam("imu_preintegration_type", &imu_preintegration_type);
  imu_preintegration_type_ =
      static_cast<ImuPreintegrationType>(imu_preintegration_type);

  // Rows and cols are redundant info, since the pose 4x4, but we parse just
  // to check we are all on the same page.
  // int n_rows = 0;
  // yaml_parser.getNestedYamlParam("T_BS", "rows", &n_rows);
  // CHECK_EQ(n_rows, 4u);
  // int n_cols = 0;
  // yaml_parser.getNestedYamlParam("T_BS", "cols", &n_cols);
  // CHECK_EQ(n_cols, 4u);
  std::vector<double> vector_pose;
  yaml_parser.getNestedYamlParam("T_BS", "data", &vector_pose);
  const gtsam::Pose3& body_Pose_cam =
      UtilsOpenCV::poseVectorToGtsamPose3(vector_pose);

  // Sanity check: IMU is usually chosen as the body frame.
  LOG_IF(FATAL, !body_Pose_cam.equals(gtsam::Pose3::identity()))
      << "parseImuData: we expected identity body_Pose_cam_: is everything ok?";

  double rate_hz = 0.0;
  yaml_parser.getYamlParam("rate_hz", &rate_hz);
  CHECK_GT(rate_hz, 0.0);
  nominal_sampling_time_s_ = 1.0 / rate_hz;

  // IMU PARAMS
  yaml_parser.getYamlParam("gyroscope_noise_density", &gyro_noise_density_);
  yaml_parser.getYamlParam("accelerometer_noise_density", &acc_noise_density_);
  yaml_parser.getYamlParam("gyroscope_random_walk", &gyro_random_walk_);
  yaml_parser.getYamlParam("accelerometer_random_walk", &acc_random_walk_);
  yaml_parser.getYamlParam("imu_integration_sigma", &imu_integration_sigma_);
  yaml_parser.getYamlParam("imu_time_shift", &imu_time_shift_);
  std::vector<double> n_gravity;
  yaml_parser.getYamlParam("n_gravity", &n_gravity);
  CHECK_EQ(n_gravity.size(), 3);
  for (int k = 0; k < 3; k++) n_gravity_(k) = n_gravity[k];

  return true;
}

void ImuParams::print() const {
  std::stringstream out;
  PipelineParams::print(out,
                        "gyroscope_noise_density: ",
                        gyro_noise_density_,
                        "gyroscope_random_walk: ",
                        gyro_random_walk_,
                        "accelerometer_noise_density: ",
                        acc_noise_density_,
                        "accelerometer_random_walk: ",
                        acc_random_walk_,
                        "imu_integration_sigma: ",
                        imu_integration_sigma_,
                        "imu_time_shift: ",
                        imu_time_shift_,
                        "n_gravity: ",
                        n_gravity_);
  LOG(INFO) << out.str();
}

bool ImuParams::equals(const PipelineParams& obj) const {
  const auto& rhs = static_cast<const ImuParams&>(obj);
  // clang-format off
  return imu_preintegration_type_ == rhs.imu_preintegration_type_ &&
      gyro_noise_density_ == rhs.gyro_noise_density_ &&
      gyro_random_walk_ == rhs.gyro_random_walk_ &&
      acc_noise_density_ == rhs.acc_noise_density_ &&
      acc_random_walk_ == rhs.acc_random_walk_ &&
      imu_time_shift_ == rhs.imu_time_shift_ &&
      nominal_sampling_time_s_ == rhs.nominal_sampling_time_s_ &&
      imu_integration_sigma_ == rhs.imu_integration_sigma_ &&
      n_gravity_ == rhs.n_gravity_;
  // clang-format on
}

}  // namespace VIO
