/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GnssVisionImuFrontendModule.h
 * @brief  Pipeline module for the gnss vision Frontend.
 * @author Elizaveta Karaseva
 */

#pragma once

#include "kimera-vio/frontend/GnssStereoVisionImuFrontend.h"
#include "kimera-vio/pipeline/PipelineModule.h"

namespace VIO {

class GnssVisionImuFrontendModule
    : public SIMOPipelineModule<FrontendInputPacketBase,
                                FrontendOutputPacketBase> {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(GnssVisionImuFrontendModule);
  KIMERA_POINTER_TYPEDEFS(GnssVisionImuFrontendModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using SIMO =
      SIMOPipelineModule<FrontendInputPacketBase, FrontendOutputPacketBase>;
  using InputQueue = ThreadsafeQueue<typename SIMO::InputUniquePtr>;

  /**
   * @brief VisionImuFrontendModule
   * @param input_queue
   * @param output_queue
   * @param parallel_run
   * @param vio_frontend
   */
  explicit GnssVisionImuFrontendModule(
      InputQueue* input_queue,
      bool parallel_run,
      VisionImuFrontend::UniquePtr vio_frontend);

  virtual ~GnssVisionImuFrontendModule() = default;

 public:
  virtual FrontendOutputPacketBase::UniquePtr spinOnce(
      FrontendInputPacketBase::UniquePtr input);

  inline bool isInitialized() const { return vio_frontend_->isInitialized(); }

  //! Imu related
  inline void updateAndResetImuBias(const ImuBias& imu_bias) const {
    vio_frontend_->updateAndResetImuBias(imu_bias);
  }

  inline ImuBias getCurrentImuBias() const {
    return vio_frontend_->getCurrentImuBias();
  }

  //! Callbacks
  inline void updateImuBias(const ImuBias& imu_bias) const {
    vio_frontend_->updateImuBias(imu_bias);
  }

  inline void updateMap(const LandmarksMap& map) const {
    vio_frontend_->updateMap(map);
  }

  inline void registerImuTimeShiftUpdateCallback(
      const VisionImuFrontend::ImuTimeShiftCallback& callback) {
    vio_frontend_->registerImuTimeShiftUpdateCallback(callback);
  }

  inline void registerGnssTimeShiftUpdateCallback(
      const VisionImuFrontend::GnssTimeShiftCallback& callback) {
    vio_frontend_->registerGnssTimeShiftUpdateCallback(callback);
  }

 private:
  VisionImuFrontend::UniquePtr vio_frontend_;
};

}  // namespace VIO
