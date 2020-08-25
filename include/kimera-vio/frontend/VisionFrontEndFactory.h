/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionFrontEndFactory.h
 * @brief  Factory of vision frontends.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/StereoVisionFrontEnd-definitions.h"
#include "kimera-vio/frontend/VisionFrontEnd.h"
#include "kimera-vio/frontend/MonoVisionFrontEnd.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd-definitions.h"

namespace VIO {

class VisionFrontEndFactory {
 public:
  KIMERA_POINTER_TYPEDEFS(VisionFrontEndFactory);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisionFrontEndFactory);
  VisionFrontEndFactory() = delete;
  virtual ~VisionFrontEndFactory() = default;

  template <class... Args>
  static StereoVisionFrontEnd::UniquePtr createFrontend(
      const FrontendType& frontend_type,
      Args&&... args) {
    switch (frontend_type) {
      // case FrontendType::kMonoImu: {
      //   return VIO::make_unique<MonoVisionFrontEnd>(
      //     std::forward<Args>(args)...);
      // }
      case FrontendType::kStereoImu: {
        return VIO::make_unique<StereoVisionFrontEnd>(
            std::forward<Args>(args)...);
      }
      default: {
        LOG(FATAL) << "Requested fronetnd type is not supported.\n"
                   << "Currently supported frontend types:\n"
                   << "0: Stereo + IMU \n"
                   << "1: Mono + IMU \n"
                   << " but requested frontend: "
                   << static_cast<int>(frontend_type);
      }
    }
  }
};

}  // namespace VIO
