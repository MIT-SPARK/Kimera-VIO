/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontEnd-definitions.h
 * @brief  Definitions for ImuFrontEnd.h
 * @author Antoni Rosinol
 */

#pragma once

#include <Eigen/Dense>

#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/ImuBias.h>

#include "common/vio_types.h"

namespace VIO {

// Inertial containers.
using ImuStamp = Timestamp;
using ImuStampS = Eigen::Matrix<ImuStamp, 1, Eigen::Dynamic>;
// First 3 elements correspond to acceleration data [m/s^2]
// while the 3 last correspond to angular velocities [rad/s].
using ImuAccGyr = Eigen::Matrix<double, 6, 1>;
using ImuAcc = Eigen::Matrix<double, 3, 1>;
using ImuGyr = Eigen::Matrix<double, 3, 1>;
using ImuAccGyrS = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using ImuBias = gtsam::imuBias::ConstantBias;

struct ImuMeasurement {
  ImuMeasurement() = default;
  ImuMeasurement(const ImuStamp& timestamp,
                 const ImuAccGyr& imu_data)
    : timestamp_(timestamp),
      imu_data_(imu_data) {}
  ImuMeasurement(ImuStamp&& timestamp,
                 ImuAccGyr&& imu_data)
    : timestamp_(std::move(timestamp)),
      imu_data_(std::move(imu_data)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuStamp timestamp_;
  ImuAccGyr imu_data_;
};

// Multiple Imu measurements, bundled in dynamic matrices.
struct ImuMeasurements {
 public:
  ImuMeasurements() = default;
  ImuMeasurements(const ImuStampS& timestamps,
                  const ImuAccGyrS& measurements)
    : timestamps_(timestamps),
      measurements_(measurements) {}
  ImuMeasurements(ImuStampS&& timestamps,
                  ImuAccGyrS&& measurements)
    : timestamps_(std::move(timestamps)),
      measurements_(std::move(measurements)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuStampS timestamps_;
  ImuAccGyrS measurements_;
};

}  // namespace VIO
