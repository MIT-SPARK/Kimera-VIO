/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontend-definitions.h
 * @brief  Definitions for ImuFrontend.h
 * @author Antoni Rosinol
 */

#pragma once

#include <Eigen/Dense>

#include <glog/logging.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>

#include "kimera-vio/common/vio_types.h"

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
  ImuMeasurement(const ImuStamp& timestamp, const ImuAccGyr& imu_data)
      : timestamp_(timestamp), acc_gyr_(imu_data) {}
  ImuMeasurement(ImuStamp&& timestamp, ImuAccGyr&& imu_data)
      : timestamp_(std::move(timestamp)), acc_gyr_(std::move(imu_data)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuStamp timestamp_;
  ImuAccGyr acc_gyr_;
};

// Multiple Imu measurements, bundled in dynamic matrices.
struct ImuMeasurements {
 public:
  ImuMeasurements() = default;
  ImuMeasurements(const ImuStampS& timestamps, const ImuAccGyrS& measurements)
      : timestamps_(timestamps), acc_gyr_(measurements) {}
  ImuMeasurements(ImuStampS&& timestamps, ImuAccGyrS&& measurements)
      : timestamps_(std::move(timestamps)), acc_gyr_(std::move(measurements)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuStampS timestamps_;
  ImuAccGyrS acc_gyr_;
};

enum class ImuPreintegrationType {
  kPreintegratedCombinedMeasurements = 0,
  kPreintegratedImuMeasurements = 1
};

/* -------------------------------------------------------------------------- */
inline const gtsam::PreintegratedImuMeasurements&
safeCastToPreintegratedImuMeasurements(const gtsam::PreintegrationType& pim) {
  try {
    return dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(pim);
  } catch (const std::bad_cast& e) {
    LOG(ERROR) << "Seems that you are casting PreintegratedType to "
                  "PreintegratedImuMeasurements, but this object is not "
                  "a PreintegratedImuMeasurements!";
    LOG(FATAL) << e.what();
  } catch (...) {
    LOG(FATAL) << "Exception caught when casting to "
                  "PreintegratedImuMeasurements.";
  }
}

/* -------------------------------------------------------------------------- */
inline const gtsam::PreintegratedCombinedMeasurements&
safeCastToPreintegratedCombinedImuMeasurements(
    const gtsam::PreintegrationType& pim) {
  try {
    return dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(pim);
  } catch (const std::bad_cast& e) {
    LOG(ERROR) << "Seems that you are casting PreintegratedType to "
                  "PreintegratedCombinedMeasurements, but this object is not "
                  "a PreintegratedCombinedMeasurements!";
    LOG(FATAL) << e.what();
  } catch (...) {
    LOG(FATAL) << "Exception caught when casting to "
                  "PreintegratedCombinedMeasurements.";
  }
}

}  // namespace VIO
