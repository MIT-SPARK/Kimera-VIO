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

#include <glog/logging.h>

namespace VIO {

StereoVisionFrontEnd::UniquePtr VisionFrontEndFactory::createFrontend(
    const FrontendType& frontend_type,
    const ImuParams& imu_params,
    const ImuBias& imu_initial_bias,
    const FrontendParams& frontend_params,
    const CameraParams& camera_params,
    DisplayQueue* display_queue,
    bool log_output) {
  switch (frontend_type) {
    case FrontendType::kStereoImu: {
      return VIO::make_unique<StereoVisionFrontEnd>(imu_params,
                                                    imu_initial_bias,
                                                    frontend_params,
                                                    camera_params,
                                                    display_queue,
                                                    log_output);
    }
    default: {
      LOG(FATAL) << "Requested fronetnd type is not supported.\n"
                 << "Currently supported frontend types:\n"
                 << "0: Stereo + IMU \n"
                 << " but requested frontend: "
                 << static_cast<int>(frontend_type);
    }
  }
}

}  // namespace VIO
