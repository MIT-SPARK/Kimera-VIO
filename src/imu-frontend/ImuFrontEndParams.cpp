/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontEndParams.cpp
 * @brief  Params for ImuFrontEnd.
 * @author Antoni Rosinol
 */

#include "kimera-vio/imu-frontend/ImuFrontEndParams.h"

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
  switch (imu_preintegration_type) {
    case to_underlying(
        ImuPreintegrationType::kPreintegratedCombinedMeasurements): {
      imu_preintegration_type_ =
          ImuPreintegrationType::kPreintegratedCombinedMeasurements;
      break;
    }
    case to_underlying(ImuPreintegrationType::kPreintegratedImuMeasurements): {
      imu_preintegration_type_ =
          ImuPreintegrationType::kPreintegratedImuMeasurements;
      break;
    }
    default: {
      LOG(FATAL) << "Unknown Imu Preintegration Type in IMU parameters.";
      break;
    }
  }

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
  LOG_IF(FATAL, !body_Pose_cam.equals(gtsam::Pose3()))
      << "parseImuData: we expected identity body_Pose_cam_: is everything ok?";

  int rate = 0;
  yaml_parser.getYamlParam("rate_hz", &rate);
  CHECK_GT(rate, 0u);
  nominal_rate_ = 1 / static_cast<double>(rate);

  // IMU PARAMS
  yaml_parser.getYamlParam("gyroscope_noise_density", &gyro_noise_);
  yaml_parser.getYamlParam("accelerometer_noise_density", &acc_noise_);
  yaml_parser.getYamlParam("gyroscope_random_walk", &gyro_walk_);
  yaml_parser.getYamlParam("accelerometer_random_walk", &acc_walk_);
  yaml_parser.getYamlParam("imu_integration_sigma", &imu_integration_sigma_);
  yaml_parser.getYamlParam("imu_time_shift", &imu_shift_);
  std::vector<double> n_gravity;
  yaml_parser.getYamlParam("n_gravity", &n_gravity);
  CHECK_EQ(n_gravity.size(), 3);
  for (int k = 0; k < 3; k++) n_gravity_(k) = n_gravity[k];

  return true;
}

void ImuParams::print() const {
  LOG(INFO) << "------------ ImuParams::print -------------\n"
            << "gyroscope_noise_density: " << gyro_noise_ << '\n'
            << "gyroscope_random_walk: " << gyro_walk_ << '\n'
            << "accelerometer_noise_density: " << acc_noise_ << '\n'
            << "accelerometer_random_walk: " << acc_walk_ << '\n'
            << "imu_integration_sigma: " << imu_integration_sigma_ << '\n'
            << "imu_time_shift: " << imu_shift_ << '\n'
            << "n_gravity: " << n_gravity_;
}

}  // namespace VIO
