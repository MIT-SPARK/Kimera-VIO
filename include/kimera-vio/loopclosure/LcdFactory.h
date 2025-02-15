/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LcdFactory.h
 * @brief  Factory for LoopClosureDetector
 * @author Marcus Abate
 * @author Luca Carlone
 */

#pragma once

#include "kimera-vio/loopclosure/LoopClosureDetector.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"

namespace VIO {

class LcdFactory {
 public:
  KIMERA_POINTER_TYPEDEFS(LcdFactory);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LcdFactory);
  LcdFactory() = delete;
  virtual ~LcdFactory() = default;

  static LoopClosureDetector::UniquePtr createLcd(
      const LoopClosureDetectorType& lcd_type,
      const LoopClosureDetectorParams& lcd_params,
      const CameraParams& tracker_cam_params,
      const gtsam::Pose3& B_Pose_Cam,
      const std::optional<StereoCamera::ConstPtr>& stereo_camera,
      const std::optional<StereoMatchingParams>& stereo_matching_params,
      const std::optional<RgbdCamera::ConstPtr>& rgbd_camera,
      bool log_output,
      PreloadedVocab::Ptr&& preloaded_vocab = nullptr) {
    switch (lcd_type) {
      case LoopClosureDetectorType::BoW: {
        return std::make_unique<LoopClosureDetector>(lcd_params,
                                                     tracker_cam_params,
                                                     B_Pose_Cam,
                                                     stereo_camera,
                                                     stereo_matching_params,
                                                     rgbd_camera,
                                                     log_output,
                                                     std::move(preloaded_vocab));
      }
      default: {
        LOG(FATAL) << "Requested loop closure detector type is not supported.\n"
                   << "Currently supported loop closure detector types:\n"
                   << "0: BoW \n but requested loop closure detector: "
                   << static_cast<int>(lcd_type);
      }
    }
  }
};

}  // namespace VIO
