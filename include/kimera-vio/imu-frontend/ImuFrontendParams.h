/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontendParams.h
 * @brief  Params for ImuFrontend
 * @author Antoni Rosinol
 */

#pragma once

#include <gtsam/base/Vector.h>

#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
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
  bool parseYAML(const std::string& filepath) override;
  void print() const override;

protected:
  bool equals(const PipelineParams& obj) const override;

 public:
  ImuPreintegrationType imu_preintegration_type_ =
      ImuPreintegrationType::kPreintegratedCombinedMeasurements;

  double init_bias_sigma_ = 0.0;
  double gyro_noise_density_ = 0.0;
  double gyro_random_walk_ = 0.0;
  double acc_noise_density_ = 0.0;
  double acc_random_walk_ = 0.0;
  double imu_time_shift_ = 0.0;  // Defined as t_imu = t_cam + imu_shift

  //! estimate fine alignment at IMU rate instead of image rate
  bool do_imu_rate_time_alignment_ = true;
  //! approximate size of time alignment window
  double time_alignment_window_size_s_ = 10.0;
  //! additional scaling to apply to nominal threshold of  imu_gryo_variance
  double time_alignment_variance_threshold_scaling_ = 100.0;

  double nominal_sampling_time_s_ = 0.0;
  double imu_integration_sigma_ = 0.0;

  gtsam::Vector3 n_gravity_ = gtsam::Vector3::Zero();
};

}  // namespace VIO
