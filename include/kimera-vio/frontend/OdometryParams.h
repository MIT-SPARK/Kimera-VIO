/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OdometryParams.h
 * @brief  Params for Odometry Sensor
 * @author Marcus Abate
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

struct OdometryParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(OdometryParams);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryParams();
  virtual ~OdometryParams() = default;

 public:
  bool parseYAML(const std::string& filepath) override;
  void print() const override;

protected:
  bool equals(const PipelineParams& obj) const override;

 public:
  gtsam::Pose3 body_Pose_ext_odom_ = gtsam::Pose3();

  double betweenRotationPrecision_ = 0.0;
  double betweenTranslationPrecision_ = 100.0;
  double velocityPrecision_ = 0.0;

  double nominal_sampling_time_s_ = 0.0;
  double time_shift_s_ = 0.0;
};

}  // namespace VIO
