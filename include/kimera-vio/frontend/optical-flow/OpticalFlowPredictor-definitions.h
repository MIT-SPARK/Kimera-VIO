/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OpticalFlowPredictor.h
 * @brief  Useful definitions for the optical flow predictor class.
 * @author Antoni Rosinol
 */

#pragma once

namespace VIO {

enum class OpticalFlowPredictorType {
  kNoPrediction = 0,
  kRotational = 1,
};

}  // namespace VIO
