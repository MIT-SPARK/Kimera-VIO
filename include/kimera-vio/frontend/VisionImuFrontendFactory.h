/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionImuFrontendFactory.h
 * @brief  Factory of vision Frontends.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/MonoVisionImuFrontend.h"
#include "kimera-vio/frontend/StereoVisionImuFrontend.h"
#include "kimera-vio/frontend/VisionImuFrontend.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"

namespace VIO {

class VisionImuFrontendFactory {
 public:
  KIMERA_POINTER_TYPEDEFS(VisionImuFrontendFactory);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisionImuFrontendFactory);
  VisionImuFrontendFactory() = delete;
  virtual ~VisionImuFrontendFactory() = default;

  // Mono version: feed a VIO::Camera
  static VisionImuFrontend::UniquePtr createFrontend(
      const FrontendType& frontend_type,
      const ImuParams& imu_params,
      const ImuBias& imu_initial_bias,
      const FrontendParams& frontend_params,
      const Camera::ConstPtr& camera,
      DisplayQueue* display_queue,
      bool log_output,
      std::optional<OdometryParams> odom_params) {
    switch (frontend_type) {
      case FrontendType::kMonoImu: {
        return std::make_unique<MonoVisionImuFrontend>(frontend_params,
                                                       imu_params,
                                                       imu_initial_bias,
                                                       camera,
                                                       display_queue,
                                                       log_output,
                                                       odom_params);
      }
      case FrontendType::kStereoImu: {
        LOG(FATAL) << "Tried to create a StereoVisionFrontEnd"
                   << "with a Mono Camera!";
      }
      default: {
        LOG(FATAL) << "Requested frontend type is not supported.\n"
                   << "Currently supported frontend types:\n"
                   << "0: Mono + IMU \n"
                   << "1: Stereo + IMU \n"
                   << " but requested frontend: "
                   << static_cast<int>(frontend_type);
      }
    }
  }

  // Stereo version: feed a VIO::StereoCamera
  static VisionImuFrontend::UniquePtr createFrontend(
      const FrontendType& frontend_type,
      const ImuParams& imu_params,
      const ImuBias& imu_initial_bias,
      const FrontendParams& frontend_params,
      const StereoCamera::ConstPtr& stereo_camera,
      DisplayQueue* display_queue,
      bool log_output,
      std::optional<OdometryParams> odom_params) {
    switch (frontend_type) {
      case FrontendType::kMonoImu: {
        LOG(FATAL) << "Tried to create a MonoVisionFrontEnd"
                   << "with a StereoCamera!";
      }
      case FrontendType::kStereoImu: {
        return std::make_unique<StereoVisionImuFrontend>(frontend_params,
                                                         imu_params,
                                                         imu_initial_bias,
                                                         stereo_camera,
                                                         display_queue,
                                                         log_output,
                                                         odom_params);
      }
      default: {
        LOG(FATAL) << "Requested frontend type is not supported.\n"
                   << "Currently supported frontend types:\n"
                   << "0: Mono + IMU \n"
                   << "1: Stereo + IMU \n"
                   << " but requested frontend: "
                   << static_cast<int>(frontend_type);
      }
    }
  }
};

}  // namespace VIO
