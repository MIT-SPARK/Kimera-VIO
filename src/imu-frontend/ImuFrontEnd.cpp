/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontEnd.cpp
 * @brief  Class managing sequences of IMU measurements.
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include "kimera-vio/imu-frontend/ImuFrontEnd.h"

#include <glog/logging.h>

#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>  // Used if IMU combined is off.
#include <gtsam/navigation/ImuFactor.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd-definitions.h"
#include "kimera-vio/utils/UtilsNumerical.h"

namespace VIO {

ImuFrontEnd::ImuFrontEnd(const ImuParams& imu_params, const ImuBias& imu_bias)
    : imu_params_(imu_params) {
  CHECK_GT(imu_params.acc_noise_, 0.0);
  CHECK_GT(imu_params.acc_walk_, 0.0);
  CHECK_GT(imu_params.gyro_noise_, 0.0);
  CHECK_GT(imu_params.gyro_walk_, 0.0);
  CHECK_GT(imu_params.imu_integration_sigma_, 0.0);
  LOG_IF(WARNING, imu_params.imu_shift_ != 0.0)
      << "Applying IMU timestamp shift of: " << imu_params.imu_shift_ << "ns.";
  initializeImuFrontEnd(imu_bias);
}

void ImuFrontEnd::initializeImuFrontEnd(const ImuBias& imu_bias) {
  switch (imu_params_.imu_preintegration_type_) {
    case ImuPreintegrationType::kPreintegratedCombinedMeasurements: {
      pim_ = VIO::make_unique<gtsam::PreintegratedCombinedMeasurements>(
          generateCombinedImuParams(imu_params_), imu_bias);
      break;
    }
    case ImuPreintegrationType::kPreintegratedImuMeasurements: {
      pim_ = VIO::make_unique<gtsam::PreintegratedImuMeasurements>(
          generateRegularImuParams(imu_params_), imu_bias);
      break;
    }
    default: {
      LOG(ERROR) << "Unknown Imu frontend type.";
      break;
    }
  }
  CHECK(pim_);
  {
    std::lock_guard<std::mutex> lock(imu_bias_mutex_);
    latest_imu_bias_ = imu_bias;
  }
  if (VLOG_IS_ON(10)) {
    LOG(ERROR) << "IMU PREINTEGRATION PARAMS GIVEN TO IMU FRONTEND.";
    imu_params_.print();
    LOG(ERROR) << "IMU BIAS GIVEN TO IMU FRONTEND AT CONSTRUCTION:\n "
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
ImuFrontEnd::PimPtr ImuFrontEnd::preintegrateImuMeasurements(
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_accgyr) {
  CHECK(pim_) << "Pim not initialized.";
  CHECK(imu_stamps.cols() >= 2) << "No Imu data found.";
  CHECK(imu_accgyr.cols() >= 2) << "No Imu data found.";
  // TODO why are we not using the last measurement??
  // Just because we do not have a future imu_stamp??
  // This can be a feature instead of a bug, in the sense that the user can then
  // either integrate or not the last measurement (typically fake/interpolated)
  // measurement. Nevertheless the imu_stamps, should be shifted one step back
  // I would say.
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

  // Create a copy of the current pim, because the ImuFrontEnd pim will be
  // reused over and over. Avoid object slicing by using the derived type of
  // pim. All of this is to deal with gtsam's idiosyncracies with base classes.
  // TODO(Toni): this copies might be quite expensive...
  switch (imu_params_.imu_preintegration_type_) {
    case ImuPreintegrationType::kPreintegratedCombinedMeasurements: {
      return VIO::make_unique<gtsam::PreintegratedCombinedMeasurements>(
          safeCastToPreintegratedCombinedImuMeasurements(*pim_));
    }
    case ImuPreintegrationType::kPreintegratedImuMeasurements: {
      return VIO::make_unique<gtsam::PreintegratedImuMeasurements>(
          safeCastToPreintegratedImuMeasurements(*pim_));
    }
    default: {
      LOG(FATAL) << "Unknown IMU Preintegration Type.";
    }
  }
}

/* -------------------------------------------------------------------------- */
gtsam::Rot3 ImuFrontEnd::preintegrateGyroMeasurements(
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_accgyr) {
  CHECK(imu_stamps.cols() >= 2) << "No Imu data found.";
  CHECK(imu_accgyr.cols() >= 2) << "No Imu data found.";
  std::lock_guard<std::mutex> lock(imu_bias_mutex_);
  gtsam::PreintegratedAhrsMeasurements pimRot(latest_imu_bias_.gyroscope(),
                                              gtsam::Matrix3::Identity());
  for (int i = 0; i < imu_stamps.cols() - 1; ++i) {
    const gtsam::Vector3& measured_omega = imu_accgyr.block<3, 1>(3, i);
    const double& delta_t =
        UtilsNumerical::NsecToSec(imu_stamps(i + 1) - imu_stamps(i));
    CHECK_GT(delta_t, 0.0) << "Imu delta is 0!";
    pimRot.integrateMeasurement(measured_omega, delta_t);
  }
  if (VLOG_IS_ON(10)) {
    LOG(INFO) << "Finished preintegration for gyro aided: ";
    pimRot.print();
  }
  return pimRot.deltaRij();
}

/* -------------------------------------------------------------------------- */
gtsam::PreintegrationBase::Params ImuFrontEnd::convertVioImuParamsToGtsam(
    const ImuParams& imu_params) {
  CHECK_GT(imu_params.acc_noise_, 0.0);
  CHECK_GT(imu_params.gyro_noise_, 0.0);
  CHECK_GT(imu_params.imu_integration_sigma_, 0.0);
  gtsam::PreintegrationBase::Params preint_imu_params(imu_params.n_gravity_);
  preint_imu_params.gyroscopeCovariance =
      std::pow(imu_params.gyro_noise_, 2.0) * Eigen::Matrix3d::Identity();
  preint_imu_params.accelerometerCovariance =
      std::pow(imu_params.acc_noise_, 2.0) * Eigen::Matrix3d::Identity();
  preint_imu_params.integrationCovariance =
      std::pow(imu_params.imu_integration_sigma_, 2.0) *
      Eigen::Matrix3d::Identity();
  // TODO(Toni): REMOVE HARDCODED
  preint_imu_params.use2ndOrderCoriolis = false;

  return preint_imu_params;
}

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
ImuFrontEnd::generateCombinedImuParams(const ImuParams& imu_params) {
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
      combined_imu_params =
          boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
              imu_params.n_gravity_);
  gtsam::PreintegrationParams gtsam_imu_params =
      ImuFrontEnd::convertVioImuParamsToGtsam(imu_params);
  if (gtsam_imu_params.body_P_sensor) {
    combined_imu_params->setBodyPSensor(*gtsam_imu_params.getBodyPSensor());
  }
  if (gtsam_imu_params.omegaCoriolis) {
    combined_imu_params->setOmegaCoriolis(*gtsam_imu_params.getOmegaCoriolis());
  }
  combined_imu_params->setGyroscopeCovariance(
      gtsam_imu_params.getGyroscopeCovariance());
  combined_imu_params->setUse2ndOrderCoriolis(
      gtsam_imu_params.getUse2ndOrderCoriolis());
  combined_imu_params->setIntegrationCovariance(
      gtsam_imu_params.getIntegrationCovariance());
  combined_imu_params->setAccelerometerCovariance(
      gtsam_imu_params.getAccelerometerCovariance());
  ///< covariance of bias used for pre-integration
  // TODO(Toni): how come we are initializing like this?
  // We should parametrize perhaps this as well.
  combined_imu_params->biasAccOmegaInt = gtsam::I_6x6;
  ///< continuous-time "Covariance" describing
  ///< accelerometer bias random walk
  combined_imu_params->biasAccCovariance =
      std::pow(imu_params.acc_walk_, 2.0) * Eigen::Matrix3d::Identity();
  ///< continuous-time "Covariance" describing gyroscope bias random walk
  combined_imu_params->biasOmegaCovariance =
      std::pow(imu_params.gyro_walk_, 2.0) * Eigen::Matrix3d::Identity();
  return combined_imu_params;
}

boost::shared_ptr<gtsam::PreintegratedImuMeasurements::Params>
ImuFrontEnd::generateRegularImuParams(const ImuParams& imu_params) {
  boost::shared_ptr<gtsam::PreintegratedImuMeasurements::Params>
      regular_imu_params =
          boost::make_shared<gtsam::PreintegratedImuMeasurements::Params>(
              imu_params.n_gravity_);
  gtsam::PreintegrationParams gtsam_imu_params =
      ImuFrontEnd::convertVioImuParamsToGtsam(imu_params);
  if (gtsam_imu_params.body_P_sensor) {
    regular_imu_params->setBodyPSensor(*gtsam_imu_params.getBodyPSensor());
  }
  if (gtsam_imu_params.omegaCoriolis) {
    regular_imu_params->setOmegaCoriolis(*gtsam_imu_params.getOmegaCoriolis());
  }
  regular_imu_params->setGyroscopeCovariance(
      gtsam_imu_params.getGyroscopeCovariance());
  regular_imu_params->setUse2ndOrderCoriolis(
      gtsam_imu_params.getUse2ndOrderCoriolis());
  regular_imu_params->setIntegrationCovariance(
      gtsam_imu_params.getIntegrationCovariance());
  regular_imu_params->setAccelerometerCovariance(
      gtsam_imu_params.getAccelerometerCovariance());
  return regular_imu_params;
}

}  // namespace VIO
