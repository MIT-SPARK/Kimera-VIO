/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OpticalFlowPredictor.h
 * @brief  Class that predicts optical flow between two images. This is
 * helpful for the tracker to have a good initial guess for feature tracking.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/optical-flow/OpticalFlowPredictor-definitions.h"
#include "kimera-vio/frontend/optical-flow/OpticalFlowPredictor.h"

namespace VIO {

class OpticalFlowPredictorFactory {
 public:
  template <class... Args>
  static OpticalFlowPredictor::UniquePtr makeOpticalFlowPredictor(
      const OpticalFlowPredictorType& optical_flow_predictor_type,
      Args&&... args) {
    switch (optical_flow_predictor_type) {
      case OpticalFlowPredictorType::kNoPrediction: {
        return std::make_unique<NoOpticalFlowPredictor>();
      }
      case OpticalFlowPredictorType::kRotational: {
        return std::make_unique<RotationalOpticalFlowPredictor>(
            std::forward<Args>(args)...);
      }
      default: {
        LOG(FATAL) << "Unknown OpticalFlowPredictorType: "
                   << static_cast<int>(optical_flow_predictor_type);
        return nullptr;
      }
    }
  }
};

}  // namespace VIO
