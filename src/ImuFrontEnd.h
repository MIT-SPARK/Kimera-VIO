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
#include <gtsam/navigation/AHRSFactor.h>

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

  // TODO: n_gravity_ should not be in ImuParams!!!
  gtsam::Vector3 n_gravity_ = gtsam::Vector3(0.0, 0.0, -9.81); // Added default value
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
  /* ------------------------------------------------------------------------
   * Class to do IMU preintegration.
   * [in] imu_params: IMU parameters used for the preintegration.
   * [in] imu_bias: IMU bias used to initialize PreintegratedImuMeasurements
   * !! The user of this class must update the bias and reset the integration
   * manually in order to preintegrate the IMU with the latest IMU bias coming
   * from the backend optimization.
   */
  ImuFrontEnd(const ImuParams& imu_params,
              const ImuBias& imu_bias)
    : imu_params_(setImuParams(imu_params)) {
    pim_ = VIO::make_unique<PreintegratedImuMeasurements>(
          boost::make_shared<PreintegratedImuMeasurements::Params>(imu_params_),
          imu_bias);
    CHECK(pim_);
    {
      std::lock_guard<std::mutex> lock(imu_bias_mutex_);
      latest_imu_bias_ = imu_bias;
    }
    if (VLOG_IS_ON(10)) {
      LOG(ERROR) << "IMU PREINTEGRATION PARAMS GIVEN TO IMU FRONTEND.";
      imu_params_.print("");
      LOG(ERROR) << "IMU BIAS GIVEN TO IMU FRONTEND AT CONSTRUCTION:\n "
                 << getCurrentImuBias();
      LOG(ERROR) << "IMU PREINTEGRATION COVARIANCE: ";
      pim_->print();
    }
  }

  /* ------------------------------------------------------------------------ */
  PreintegratedImuMeasurements preintegrateImuMeasurements(
      const ImuStampS& imu_stamps,
      const ImuAccGyrS& imu_accgyr);
  PreintegratedImuMeasurements preintegrateImuMeasurements(
      const ImuStampS& imu_stamps,
      const ImuAccGyr& imu_accgyr) = delete;
  PreintegratedImuMeasurements preintegrateImuMeasurements(
      const ImuStamp& imu_stamps,
      const ImuAccGyrS& imu_accgyr) = delete;
  PreintegratedImuMeasurements preintegrateImuMeasurements(
      const ImuStamp& imu_stamps,
      const ImuAccGyr& imu_accgyr) = delete;

  /* -------------------------------------------------------------------------- */
  gtsam::Rot3 preintegrateGyroMeasurements(
      const ImuStampS& imu_stamps,
      const ImuAccGyrS& imu_accgyr);
  gtsam::Rot3 preintegrateGyroMeasurements(
      const ImuStampS& imu_stamps,
      const ImuAccGyr& imu_accgyr) = delete;
  gtsam::Rot3 preintegrateGyroMeasurements(
      const ImuStamp& imu_stamps,
      const ImuAccGyrS& imu_accgyr) = delete;
  gtsam::Rot3 preintegrateGyroMeasurements(
      const ImuStamp& imu_stamps,
      const ImuAccGyr& imu_accgyr) = delete;

  /* ------------------------------------------------------------------------ */
  // This should be called by the backend, whenever there
  // is a new imu bias estimate. Note that we only store the new
  // bias but we do not attempt to reset pre-integration as we
  // might have already started preintegrating measurements from
  // latest keyframe to the current frame using the previous bias.
  inline void updateBias(const ImuBias& imu_bias_prev_kf) {
    std::lock_guard<std::mutex> lock(imu_bias_mutex_);
    latest_imu_bias_ = imu_bias_prev_kf;
    // Debug output.
    if (VLOG_IS_ON(10)) {
      LOG(INFO) << "Updating Preintegration imu bias (needs to reset "
                   "integration for the bias to have effect):";
      latest_imu_bias_.print();
    }
  }

  /* ------------------------------------------------------------------------ */
  // This should be called by the stereo frontend, whenever there
  // is a new keyframe and we want to reset the integration to
  // use the latest imu bias.
  // THIS IS NOT THREAD-SAFE: pim_ is not protected.
  inline void resetIntegrationWithCachedBias() {
    std::lock_guard<std::mutex> lock(imu_bias_mutex_);
    pim_->resetIntegrationAndSetBias(latest_imu_bias_);
    // For debugging.
    if (VLOG_IS_ON(10)) {
      LOG(ERROR) << "Reset preintegration with new bias:";
      latest_imu_bias_.print();
    }
  }

  /* ------------------------------------------------------------------------ */
  // THREAD-SAFE.
  inline ImuBias getCurrentImuBias() const {
    std::lock_guard<std::mutex> lock(imu_bias_mutex_);
    return latest_imu_bias_;
  }

  /* ------------------------------------------------------------------------ */
  // THIS IS NOT THREAD-SAFE.
  inline PreintegratedImuMeasurements getCurrentPIM() const {
    return *pim_;
  }

  /* ------------------------------------------------------------------------ */
  inline PreintegratedImuMeasurements::Params getImuParams() const {
    return imu_params_;
  }

private:
  const PreintegratedImuMeasurements::Params imu_params_;
  std::unique_ptr<PreintegratedImuMeasurements> pim_ = nullptr;
  ImuBias latest_imu_bias_;
  mutable std::mutex imu_bias_mutex_;

  /* ------------------------------------------------------------------------ */
  // Set parameters for imu preintegration from the given ImuParams.
  PreintegratedImuMeasurements::Params setImuParams(
      const ImuParams& imu_params);
};

} // End of VIO namespace.
