/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontEnd.h
 * @brief  Class managing sequences of IMU measurements.
 * @author Antoni Rosinol, Luca Carlone
 */

#pragma once

#include <map>
#include <string>
#include <tuple>
#include <thread>
#include <utility>
#include <mutex>

#include <Eigen/Dense>

#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h> // Used if IMU combined is off.
#include <gtsam/navigation/ImuFactor.h>

#include "ImuFrontEnd-definitions.h"

#include "utils/ThreadsafeImuBuffer.h"

namespace VIO {

/*
 * Class describing the imu data.
 */
class ImuData {
public:
  // Imu buffer with virtually infinite memory.
  ImuData()
    : imu_buffer_(-1) {}

  // Checks for statistics..
  double imu_rate_;
  double nominal_imu_rate_;
  double imu_rate_std_;
  double imu_rate_maxMismatch_;

  // Imu data.
  utils::ThreadsafeImuBuffer imu_buffer_;

public:
  void print() const;
};

/*
 * Struct describing the imu parameters.
 */
struct ImuParams {
public:
  double gyro_noise_;
  double gyro_walk_;
  double acc_noise_;
  double acc_walk_;

  gtsam::Vector3 n_gravity_;
  double imu_integration_sigma_;

public:
  void print() const;
};

/*
 * Class implementing the Imu Front End preintegration.
 * Construct using imu_params, and optionally an initial ImuBias.
 * Call preintegrateImuMeasurements to progressively integrate a bunch of Imu
 * measurements.
 * Eventually, call resetIntegrationAndSetBias when you would like to update
 * the Imu bias.
 */
class ImuFrontEnd {
public:
  #ifdef USE_COMBINED_IMU_FACTOR
    using PreintegratedImuMeasurements = gtsam::PreintegratedCombinedMeasurements;
  #else
    using PreintegratedImuMeasurements = gtsam::PreintegratedImuMeasurements;
  #endif
public:
  /* ------------------------------------------------------------------------ */
  ImuFrontEnd(const ImuParams& imu_params,
              const ImuBias& imu_bias_prev_kf = ImuBias())
    : imu_params_(setImuParams(imu_params)) {
    pim_ = VIO::make_unique<PreintegratedImuMeasurements>(imu_params_,
                                                          imu_bias_prev_kf);
  }

  /* ------------------------------------------------------------------------ */
  // NOT THREAD-SAFE
  // DO the actual preint inside,
  PreintegratedImuMeasurements preintegrateImuMeasurements(
      const ImuStampS& imu_stamps,
      const ImuAccGyrS& imu_accgyr);

  /* ------------------------------------------------------------------------ */
  inline void resetIntegrationAndSetBias(
      const ImuBias& imu_bias_prev_kf) {
    pim_->resetIntegrationAndSetBias(imu_bias_prev_kf);
  }

private:
  // boost::shared_ptr because IMU preintegration needs it....
  boost::shared_ptr<PreintegratedImuMeasurements::Params> imu_params_;
  std::unique_ptr<PreintegratedImuMeasurements> pim_ = nullptr;

  /* ------------------------------------------------------------------------ */
  // Set parameters for imu preintegration from the given ImuParams.
  boost::shared_ptr<PreintegratedImuMeasurements::Params> setImuParams(
      const ImuParams& imu_params);
};

} // End of VIO namespace.
