/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontEndParams.h
 * @brief  Params for ImuFrontEnd
 * @author Antoni Rosinol
 */

#pragma once

#include <gtsam/base/Vector.h>

#include "kimera-vio/imu-frontend/ImuFrontEnd-definitions.h"
#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

struct ImuParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(ImuParams);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuParams();
  virtual ~ImuParams() = default;

 public:
  virtual bool parseYAML(const std::string& filepath) override;
  virtual void print() const override;

 public:
  ImuPreintegrationType imu_preintegration_type_ =
      ImuPreintegrationType::kPreintegratedCombinedMeasurements;

  double gyro_noise_ = 0.0;
  double gyro_walk_ = 0.0;
  double acc_noise_ = 0.0;
  double acc_walk_ = 0.0;
  double imu_shift_ = 0.0;  // Defined as t_imu = t_cam + imu_shift

  double nominal_rate_ = 0.0;
  double imu_integration_sigma_ = 0.0;

  gtsam::Vector3 n_gravity_ = gtsam::Vector3();
};

}  // namespace VIO
