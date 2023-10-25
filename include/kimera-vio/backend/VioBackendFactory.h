/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackendModule.h
 * @brief  Pipeline module for the Backend.
 *
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/backend/RegularVioBackend.h"
#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/backend/VioBackend.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class BackendFactory {
 public:
  KIMERA_POINTER_TYPEDEFS(BackendFactory);
  KIMERA_DELETE_COPY_CONSTRUCTORS(BackendFactory);
  BackendFactory() = delete;
  virtual ~BackendFactory() = default;

  static VioBackend::UniquePtr createBackend(
      const BackendType& backend_type,
      const Pose3& B_Pose_leftCamRect,
      const StereoCalibPtr& stereo_calibration,
      const BackendParams& backend_params,
      const ImuParams& imu_params,
      const BackendOutputParams& backend_output_params,
      bool log_output,
      std::optional<OdometryParams> odom_params) {
    switch (backend_type) {
      case BackendType::kStereoImu: {
        return std::make_unique<VioBackend>(B_Pose_leftCamRect,
                                            stereo_calibration,
                                            backend_params,
                                            imu_params,
                                            backend_output_params,
                                            log_output,
                                            odom_params);
      }
      case BackendType::kStructuralRegularities: {
        return std::make_unique<RegularVioBackend>(B_Pose_leftCamRect,
                                                   stereo_calibration,
                                                   backend_params,
                                                   imu_params,
                                                   backend_output_params,
                                                   log_output,
                                                   odom_params);
      }
      default: {
        LOG(FATAL) << "Requested Backend type is not supported.\n"
                   << "Currently supported Backend types:\n"
                   << "0: normal VIO\n 1: regular VIO\n"
                   << " but requested Backend: "
                   << static_cast<int>(backend_type);
        return nullptr;
      }
    }
  }
};

}  // namespace VIO
