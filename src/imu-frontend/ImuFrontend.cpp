/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontend.cpp
 * @brief  Class managing sequences of IMU measurements.
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include "kimera-vio/imu-frontend/ImuFrontend.h"

#include <glog/logging.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/utils/UtilsNumerical.h"

namespace VIO {

/* -------------------------------------------------------------------------- */
void ImuData::print() const {
  LOG(INFO) << "------------ ImuData::print -------------\n"
            << "\nnominal_imu_rate: " << nominal_imu_rate_ << '\n'
            << "imu_rate: " << imu_rate_ << '\n'
            << "imu_rate_std: " << imu_rate_std_ << '\n'
            << "imu_rate_maxMismatch: " << imu_rate_maxMismatch_ << '\n'
            << "nr of imu measurements: " << imu_buffer_.size();
}

ImuFrontend::ImuFrontend(const ImuParams& imu_params, const ImuBias& imu_bias)
    : imu_params_(imu_params) {
  CHECK_GT(imu_params.acc_noise_density_, 0.0);
  CHECK_GT(imu_params.acc_random_walk_, 0.0);
  CHECK_GT(imu_params.gyro_noise_density_, 0.0);
  CHECK_GT(imu_params.gyro_random_walk_, 0.0);
  CHECK_GT(imu_params.imu_integration_sigma_, 0.0);
  LOG_IF(WARNING, imu_params.imu_time_shift_ != 0.0)
      << "Applying IMU timestamp shift of: " << imu_params.imu_time_shift_
      << "ns.";
  initializeImuFrontend(imu_bias);
}

template <typename R, typename... A>
struct FunctionSignature;

template <typename R, typename... A>
struct FunctionSignature<auto(A...)->R> {
  using ReturnType = R;
  using ArgTuple = std::tuple<A...>;
};

using RegularParams = gtsam::PreintegratedImuMeasurements::Params;
using RegularParamSignature =
    FunctionSignature<decltype(RegularParams::MakeSharedD)>;
using RegularParamPtr = RegularParamSignature::ReturnType;

using CombinedParams = gtsam::PreintegratedCombinedMeasurements::Params;
using CombinedParamSignature =
    FunctionSignature<decltype(CombinedParams::MakeSharedD)>;
using CombinedParamPtr = CombinedParamSignature::ReturnType;

ImuFrontend::PimUniquePtr generateRegularPim(const ImuParams& info,
                                             const ImuBias& imu_bias) {
  RegularParamPtr pim_params;
  pim_params.reset(new RegularParams(info.n_gravity_));

  const auto gtsam_params = ImuFrontend::convertVioImuParamsToGtsam(info);
  if (gtsam_params.body_P_sensor) {
    pim_params->setBodyPSensor(*gtsam_params.getBodyPSensor());
  }

  if (gtsam_params.omegaCoriolis) {
    pim_params->setOmegaCoriolis(*gtsam_params.getOmegaCoriolis());
  }

  pim_params->setGyroscopeCovariance(gtsam_params.getGyroscopeCovariance());
  pim_params->setUse2ndOrderCoriolis(gtsam_params.getUse2ndOrderCoriolis());
  pim_params->setIntegrationCovariance(gtsam_params.getIntegrationCovariance());
  pim_params->setAccelerometerCovariance(
      gtsam_params.getAccelerometerCovariance());

  return std::make_unique<gtsam::PreintegratedImuMeasurements>(pim_params,
                                                               imu_bias);
}

ImuFrontend::PimUniquePtr generateCombinedPim(const ImuParams& imu_params,
                                              const ImuBias& imu_bias) {
  CombinedParamPtr pim_params;
  pim_params.reset(new CombinedParams(imu_params.n_gravity_));

  const auto gtsam_params = ImuFrontend::convertVioImuParamsToGtsam(imu_params);
  if (gtsam_params.body_P_sensor) {
    pim_params->setBodyPSensor(*gtsam_params.getBodyPSensor());
  }

  if (gtsam_params.omegaCoriolis) {
    pim_params->setOmegaCoriolis(*gtsam_params.getOmegaCoriolis());
  }

  pim_params->setGyroscopeCovariance(gtsam_params.getGyroscopeCovariance());
  pim_params->setUse2ndOrderCoriolis(gtsam_params.getUse2ndOrderCoriolis());
  pim_params->setIntegrationCovariance(gtsam_params.getIntegrationCovariance());
  pim_params->setAccelerometerCovariance(
      gtsam_params.getAccelerometerCovariance());
  // TODO(Toni): how come we are initializing like this?
  // We should parametrize perhaps this as well.
  pim_params->biasAccOmegaInt = gtsam::I_6x6 * imu_params.init_bias_sigma_;
  pim_params->biasAccCovariance =
      std::pow(imu_params.acc_random_walk_, 2.0) * Eigen::Matrix3d::Identity();
  pim_params->biasOmegaCovariance =
      std::pow(imu_params.gyro_random_walk_, 2.0) * Eigen::Matrix3d::Identity();

  return std::make_unique<gtsam::PreintegratedCombinedMeasurements>(pim_params,
                                                                    imu_bias);
}

void ImuFrontend::initializeImuFrontend(const ImuBias& imu_bias) {
  switch (imu_params_.imu_preintegration_type_) {
    case ImuPreintegrationType::kPreintegratedCombinedMeasurements:
      pim_ = generateCombinedPim(imu_params_, imu_bias);
      break;
    case ImuPreintegrationType::kPreintegratedImuMeasurements:
      pim_ = generateRegularPim(imu_params_, imu_bias);
      break;
    default:
      LOG(ERROR) << "Unknown Imu Frontend type.";
      break;
  }

  CHECK(pim_);

  {
    std::lock_guard<std::mutex> lock(imu_bias_mutex_);
    latest_imu_bias_ = imu_bias;
  }

  if (VLOG_IS_ON(10)) {
    LOG(ERROR) << "IMU PREINTEGRATION PARAMS GIVEN TO IMU Frontend.";
    imu_params_.print();
    LOG(ERROR) << "IMU BIAS GIVEN TO IMU Frontend AT CONSTRUCTION:\n "
               << getCurrentImuBias();
    LOG(ERROR) << "IMU PREINTEGRATION COVARIANCE: ";
    pim_->print("PIM type: " + std::to_string(VIO::to_underlying(
                                   imu_params_.imu_preintegration_type_)));
  }
}

/* -------------------------------------------------------------------------- */
// NOT THREAD-SAFE
// What happens if someone updates the bias in the middle of the
// preintegration??
ImuFrontend::PimPtr ImuFrontend::preintegrateImuMeasurements(
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_accgyr) {
  CHECK(pim_) << "Pim not initialized.";
  CHECK(imu_stamps.cols() >= 2) << "No Imu data found.";
  CHECK(imu_accgyr.cols() >= 2) << "No Imu data found.";
  for (int i = 0; i < imu_stamps.cols() - 1; ++i) {
    const gtsam::Vector3& measured_acc = imu_accgyr.block<3, 1>(0, i);
    const gtsam::Vector3& measured_omega = imu_accgyr.block<3, 1>(3, i);
    const double& delta_t =
        UtilsNumerical::NsecToSec(imu_stamps(i + 1) - imu_stamps(i));
    CHECK_GT(delta_t, 0.0) << "Imu delta is 0!";
    // TODO Shouldn't we use pim_->integrateMeasurements(); for less code
    // and efficiency??
    pim_->integrateMeasurement(measured_acc, measured_omega, delta_t);
  }
  if (VLOG_IS_ON(10)) {
    LOG(INFO) << "Finished preintegration: ";
    pim_->print("PIM type: " + std::to_string(VIO::to_underlying(
                                   imu_params_.imu_preintegration_type_)));
  }

  // Create a copy of the current pim, because the ImuFrontend pim will be
  // reused over and over. Avoid object slicing by using the derived type of
  // pim. All of this is to deal with gtsam's idiosyncracies with base classes.
  // TODO(Toni): this copies might be quite expensive...
  switch (imu_params_.imu_preintegration_type_) {
    case ImuPreintegrationType::kPreintegratedCombinedMeasurements: {
      return std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
          safeCastToPreintegratedCombinedImuMeasurements(*pim_));
    }
    case ImuPreintegrationType::kPreintegratedImuMeasurements: {
      return std::make_unique<gtsam::PreintegratedImuMeasurements>(
          safeCastToPreintegratedImuMeasurements(*pim_));
    }
    default: {
      LOG(FATAL) << "Unknown IMU Preintegration Type.";
    }
  }
}

/* -------------------------------------------------------------------------- */
gtsam::Rot3 ImuFrontend::preintegrateGyroMeasurements(
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_accgyr) {
  CHECK(imu_stamps.cols() >= 2) << "No Imu data found.";
  CHECK(imu_accgyr.cols() >= 2) << "No Imu data found.";
  std::lock_guard<std::mutex> lock(imu_bias_mutex_);
  gtsam::PreintegratedAhrsMeasurements pim_rot(latest_imu_bias_.gyroscope(),
                                               gtsam::Matrix3::Identity());
  for (int i = 0; i < imu_stamps.cols() - 1; ++i) {
    const gtsam::Vector3& measured_omega = imu_accgyr.block<3, 1>(3, i);
    const double& delta_t =
        UtilsNumerical::NsecToSec(imu_stamps(i + 1) - imu_stamps(i));
    CHECK_GT(delta_t, 0.0) << "Imu delta is 0!";
    pim_rot.integrateMeasurement(measured_omega, delta_t);
  }
  if (VLOG_IS_ON(10)) {
    LOG(INFO) << "Finished preintegration for gyro aided: ";
    pim_rot.print();
  }
  return pim_rot.deltaRij();
}

/* -------------------------------------------------------------------------- */
gtsam::PreintegrationBase::Params ImuFrontend::convertVioImuParamsToGtsam(
    const ImuParams& imu_params) {
  CHECK_GT(imu_params.acc_noise_density_, 0.0);
  CHECK_GT(imu_params.gyro_noise_density_, 0.0);
  CHECK_GT(imu_params.imu_integration_sigma_, 0.0);
  gtsam::PreintegrationBase::Params preint_imu_params(imu_params.n_gravity_);
  preint_imu_params.gyroscopeCovariance =
      std::pow(imu_params.gyro_noise_density_, 2.0) *
      Eigen::Matrix3d::Identity();
  preint_imu_params.accelerometerCovariance =
      std::pow(imu_params.acc_noise_density_, 2.0) *
      Eigen::Matrix3d::Identity();
  preint_imu_params.integrationCovariance =
      std::pow(imu_params.imu_integration_sigma_, 2.0) *
      Eigen::Matrix3d::Identity();
  // TODO(Toni): REMOVE HARDCODED
  preint_imu_params.use2ndOrderCoriolis = false;

  return preint_imu_params;
}

}  // namespace VIO
