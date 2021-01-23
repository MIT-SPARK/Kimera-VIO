/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionFrontEndFactory.cpp
 * @brief  Factory of vision frontends.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/VisionFrontEndFactory.h"

namespace VIO {

VisionFrontEnd::UniquePtr VisionFrontEndFactory::createFrontend(
    const FrontendType& frontend_type,
    const ImuParams& imu_params,
    const ImuBias& imu_initial_bias,
    const FrontendParams& frontend_params,
    const Camera::ConstPtr& camera,
    DisplayQueue* display_queue,
    bool log_output) {
  switch (frontend_type) {
    case FrontendType::kMonoImu: {
      return VIO::make_unique<MonoVisionFrontEnd>(imu_params,
                                                  imu_initial_bias,
                                                  frontend_params,
                                                  camera,
                                                  display_queue,
                                                  log_output);
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

VisionFrontEnd::UniquePtr VisionFrontEndFactory::createFrontend(
    const FrontendType& frontend_type,
    const ImuParams& imu_params,
    const ImuBias& imu_initial_bias,
    const FrontendParams& frontend_params,
    const StereoCamera::ConstPtr& stereo_camera,
    DisplayQueue* display_queue,
    bool log_output) {
  switch (frontend_type) {
    case FrontendType::kMonoImu: {
      LOG(FATAL) << "Tried to create a MonoVisionFrontEnd"
                 << "with a StereoCamera!";
    }
    case FrontendType::kStereoImu: {
      return VIO::make_unique<StereoVisionFrontEnd>(imu_params,
                                                    imu_initial_bias,
                                                    frontend_params,
                                                    stereo_camera,
                                                    display_queue,
                                                    log_output);
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

}  // namespace VIO
