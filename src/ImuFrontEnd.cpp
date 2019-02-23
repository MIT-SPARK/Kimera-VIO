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
#include "ImuFrontEnd.h"

#include <glog/logging.h>

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
            << "accelerometer_random_walk: " << acc_walk_;
}

/* -------------------------------------------------------------------------- */
// NOT THREAD-SAFE
// And this function is called outside VioBackEnd thread... for the
// frontend to have a rotation prior for RANSAC.
gtsam::PreintegratedImuMeasurements ImuFrontEnd::preintegrateImuMeasurements(
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_accgyr) {
  CHECK(pim_) << "Pim not initialized.";
  CHECK(imu_stamps.cols() >= 2) << "No Imu data found.";
  CHECK(imu_accgyr.cols() >= 2) << "No Imu data found.";
  for (int i = 0; i < imu_stamps.cols() - 1; ++i) {
    const Vector3& measured_omega = imu_accgyr.block<3,1>(3, i);
    const Vector3& measured_acc = imu_accgyr.block<3,1>(0, i);
    const double& delta_t = UtilsOpenCV::NsecToSec(imu_stamps(i + 1) -
                                                   imu_stamps(i));
    CHECK_GT(delta_t, 0.0) << "Imu delta is 0!";
    pim_->integrateMeasurement(measured_acc, measured_omega, delta_t);
  }
  return *pim_;
}

/* -------------------------------------------------------------------------- */
// Set parameters for imu factors.
boost::shared_ptr<ImuFrontEnd::PreintegratedImuMeasurements::Params>
ImuFrontEnd::setImuParams(const ImuParams& imu_params) {
  boost::shared_ptr<PreintegratedImuMeasurements::Params> preint_imu_params =
      boost::make_shared<PreintegratedImuMeasurements::Params>(imu_params.n_gravity_);
  preint_imu_params->gyroscopeCovariance =
      std::pow(imu_params.gyro_noise_, 2.0) * Eigen::Matrix3d::Identity();
  preint_imu_params->accelerometerCovariance =
      std::pow(imu_params.acc_noise_, 2.0) * Eigen::Matrix3d::Identity();
  preint_imu_params->integrationCovariance =
      std::pow(imu_params.imu_integration_sigma_, 2.0) * Eigen::Matrix3d::Identity();
  preint_imu_params->use2ndOrderCoriolis = false; // TODO: expose this parameter
  return preint_imu_params;
}

} // End of VIO namespace.
