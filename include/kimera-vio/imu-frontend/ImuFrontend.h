/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontend.h
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
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>  // Used if IMU combined is off.
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>

#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontendParams.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/ThreadsafeImuBuffer.h"

namespace VIO {

class ImuData {
public:
 KIMERA_POINTER_TYPEDEFS(ImuData);
 KIMERA_DELETE_COPY_CONSTRUCTORS(ImuData);
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 // Imu buffer with virtually infinite memory.
 ImuData() : imu_buffer_(-1) {}

 // Checks for statistics..
 // TODO(Toni): remove these and put in params.
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
 * Class implementing the Imu Front End preintegration.
 * Construct using imu_params, and optionally an initial ImuBias.
 * Call preintegrateImuMeasurements to progressively integrate a bunch of Imu
 * measurements.
 * Eventually, call resetIntegrationAndSetBias when you would like to update
 * the Imu bias.
 */
class ImuFrontend {
public:
 using PimPtr = std::shared_ptr<gtsam::PreintegrationType>;
 using PimUniquePtr = std::unique_ptr<gtsam::PreintegrationType>;

public:
 KIMERA_POINTER_TYPEDEFS(ImuFrontend);
 KIMERA_DELETE_COPY_CONSTRUCTORS(ImuFrontend);
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 /* ------------------------------------------------------------------------
  * Class to do IMU preintegration.
  * [in] imu_params: IMU parameters used for the preintegration.
  * [in] imu_bias: IMU bias used to initialize PreintegratedImuMeasurements
  * !! The user of this class must update the bias and reset the integration
  * manually in order to preintegrate the IMU with the latest IMU bias coming
  * from the Backend optimization.
  */
 ImuFrontend(const ImuParams& imu_params, const ImuBias& imu_bias);
 ~ImuFrontend() = default;

 /* ------------------------------------------------------------------------ */
 PimPtr preintegrateImuMeasurements(const ImuStampS& imu_stamps,
                                    const ImuAccGyrS& imu_accgyr);
 PimPtr preintegrateImuMeasurements(const ImuStampS& imu_stamps,
                                    const ImuAccGyr& imu_accgyr) = delete;
 PimPtr preintegrateImuMeasurements(const ImuStamp& imu_stamps,
                                    const ImuAccGyrS& imu_accgyr) = delete;
 PimPtr preintegrateImuMeasurements(const ImuStamp& imu_stamps,
                                    const ImuAccGyr& imu_accgyr) = delete;

 /* ------------------------------------------------------------------------- */
 gtsam::Rot3 preintegrateGyroMeasurements(const ImuStampS& imu_stamps,
                                          const ImuAccGyrS& imu_accgyr);
 gtsam::Rot3 preintegrateGyroMeasurements(const ImuStampS& imu_stamps,
                                          const ImuAccGyr& imu_accgyr) = delete;
 gtsam::Rot3 preintegrateGyroMeasurements(const ImuStamp& imu_stamps,
                                          const ImuAccGyrS& imu_accgyr) =
     delete;
 gtsam::Rot3 preintegrateGyroMeasurements(const ImuStamp& imu_stamps,
                                          const ImuAccGyr& imu_accgyr) = delete;

 /* ------------------------------------------------------------------------ */
 // This should be called by the Backend, whenever there
 // is a new imu bias estimate. Note that we only store the new
 // bias, but we don't reset the pre-integration.This is because we
 // might have already started preintegrating measurements from
 // latest keyframe to the current frame using the previous bias,
 // which at the moment was the best last estimate.
 // The pre-integration is updated with the correct bias
 // in the Backend.
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
  // This should be called by the stereo Frontend, whenever there
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
  // Reset gravity value in pre-integration.
  // This is needed for the online initialization.
  // THREAD-SAFE.
  inline void resetPreintegrationGravity(const gtsam::Vector3& reset_value) {
    LOG(WARNING) << "Resetting value of gravity in ImuFrontend to: "
                 << reset_value;
    std::lock_guard<std::mutex> lock(imu_bias_mutex_);
    pim_->params()->n_gravity = reset_value;
    CHECK(gtsam::assert_equal(pim_->params()->getGravity(), reset_value));
    // TODO(Toni): should we update imu_params n_gravity for consistency?
    // imu_params_.n_gravity_ = reset_value;
  }

  /* ------------------------------------------------------------------------ */
  // THREAD-SAFE.
  inline ImuPreintegrationType getImuPreintegrationType() const {
    return imu_params_.imu_preintegration_type_;
  }

  inline gtsam::Vector3 getPreintegrationGravity() const {
    // TODO(Toni): why are we locking the imu_bias_mutex here???
    std::lock_guard<std::mutex> lock(imu_bias_mutex_);
    return imu_params_.n_gravity_;
  }

  /* ------------------------------------------------------------------------ */
  inline gtsam::PreintegrationType::Params getGtsamImuParams() const {
    return *(pim_->params());
  }

  /* ------------------------------------------------------------------------ */
  // Convert parameters for imu preintegration from the given ImuParams.
  static gtsam::PreintegrationType::Params convertVioImuParamsToGtsam(
      const ImuParams& imu_params);

  static boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
  generateCombinedImuParams(const ImuParams& imu_params);

  static boost::shared_ptr<gtsam::PreintegratedImuMeasurements::Params>
  generateRegularImuParams(const ImuParams& imu_params);

 private:
  void initializeImuFrontend(const ImuBias& imu_bias);

 private:
  ImuParams imu_params_;
  PimUniquePtr pim_ = nullptr;
  ImuBias latest_imu_bias_;
  mutable std::mutex imu_bias_mutex_;
};

} // End of VIO namespace.
