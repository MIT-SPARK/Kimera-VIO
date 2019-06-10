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
 * @author Antoni Rosinol, Luca Carlone
 */

#include <glog/logging.h>

#include "ImuFrontEnd.h"
#include "UtilsOpenCV.h"
#include "common/vio_types.h"

namespace VIO {

////////////////////////////////////////////////////////////////////////////////
//////////////// FUNCTIONS OF THE CLASS ImuData                       //////////
////////////////////////////////////////////////////////////////////////////////
/* -------------------------------------------------------------------------- */
void ImuData::print() const {
  LOG(INFO) << "------------ ImuData::print -------------\n"
            << "\nnominal_imu_rate: " << nominal_imu_rate_ << '\n'
            << "imu_rate: " << imu_rate_ << '\n'
            << "imu_rate_std: " << imu_rate_std_ << '\n'
            << "imu_rate_maxMismatch: " << imu_rate_maxMismatch_ << '\n'
            << "nr of imu measurements: " << imu_buffer_.size();
}

/* -------------------------------------------------------------------------- */
void ImuParams::print() const {
  LOG(INFO) << "------------ ImuParams::print -------------";
  LOG(INFO) << "\ngyroscope_noise_density: " << gyro_noise_ << '\n'
            << "gyroscope_random_walk: " << gyro_walk_ << '\n'
            << "accelerometer_noise_density: " << acc_noise_ << '\n'
            << "accelerometer_random_walk: " << acc_walk_ << '\n'
            << "n_gravity: " << n_gravity_ << '\n'
            << "imu_integration_sigma: " << imu_integration_sigma_;
}

/* -------------------------------------------------------------------------- */
// NOT THREAD-SAFE
// What happens if someone updates the bias in the middle of the
// preintegration??
gtsam::PreintegratedImuMeasurements ImuFrontEnd::preintegrateImuMeasurements(
    const ImuStampS& imu_stamps, const ImuAccGyrS& imu_accgyr) {
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
        UtilsOpenCV::NsecToSec(imu_stamps(i + 1) - imu_stamps(i));
    CHECK_GT(delta_t, 0.0) << "Imu delta is 0!";
    // TODO Shouldn't we use pim_->integrateMeasurements(); for less code
    // and efficiency??
    pim_->integrateMeasurement(measured_acc, measured_omega, delta_t);
  }
  if (VLOG_IS_ON(10)) {
    LOG(INFO) << "Finished preintegration: ";
    pim_->print();
  }

  return *pim_;
}

/* -------------------------------------------------------------------------- */
gtsam::Rot3 ImuFrontEnd::preintegrateGyroMeasurements(
    const ImuStampS& imu_stamps, const ImuAccGyrS& imu_accgyr) {
  CHECK(imu_stamps.cols() >= 2) << "No Imu data found.";
  CHECK(imu_accgyr.cols() >= 2) << "No Imu data found.";
  std::lock_guard<std::mutex> lock(imu_bias_mutex_);
  gtsam::PreintegratedAhrsMeasurements pimRot(latest_imu_bias_.gyroscope(),
                                              gtsam::Matrix3::Identity());
  for (int i = 0; i < imu_stamps.cols() - 1; ++i) {
    const gtsam::Vector3& measured_omega = imu_accgyr.block<3, 1>(3, i);
    const double& delta_t =
        UtilsOpenCV::NsecToSec(imu_stamps(i + 1) - imu_stamps(i));
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
// Set parameters for imu factors.
gtsam::PreintegrationBase::Params ImuFrontEnd::setImuParams(
    const ImuParams& imu_params) {
  PreintegratedImuMeasurements::Params preint_imu_params =
      PreintegratedImuMeasurements::Params(imu_params.n_gravity_);
  preint_imu_params.gyroscopeCovariance =
      std::pow(imu_params.gyro_noise_, 2.0) * Eigen::Matrix3d::Identity();
  preint_imu_params.accelerometerCovariance =
      std::pow(imu_params.acc_noise_, 2.0) * Eigen::Matrix3d::Identity();
  preint_imu_params.integrationCovariance =
      std::pow(imu_params.imu_integration_sigma_, 2.0) *
      Eigen::Matrix3d::Identity();
  preint_imu_params.use2ndOrderCoriolis = false;  // TODO: expose this parameter

#ifdef USE_COMBINED_IMU_FACTOR
  preint_imu_params.biasAccCovariance =
      std::pow(vioParams.accBiasSigma_, 2.0) * Eigen::Matrix3d::Identity();
  preint_imu_params.biasOmegaCovariance =
      std::pow(vioParams.gyroBiasSigma_, 2.0) * Eigen::Matrix3d::Identity();
#endif

  return preint_imu_params;
}

}  // namespace VIO
