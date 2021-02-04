/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionImuFrontendModule.h
 * @brief  Pipeline module for the vision Frontend.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/VisionImuFrontend.h"
#include "kimera-vio/pipeline/PipelineModule.h"

namespace VIO {

class VisionImuFrontendModule : public 
    SIMOPipelineModule<FrontendInputPacketBase, FrontendOutputPacketBase> {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisionImuFrontendModule);
  KIMERA_POINTER_TYPEDEFS(VisionImuFrontendModule);
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
  explicit VisionImuFrontendModule(
      InputQueue* input_queue,
      bool parallel_run,
      VisionImuFrontend::UniquePtr vio_frontend);

  virtual ~VisionImuFrontendModule() = default;

 public:
  virtual FrontendOutputPacketBase::UniquePtr
      spinOnce(FrontendInputPacketBase::UniquePtr input);

  inline bool isInitialized() const {
    return vio_frontend_->isInitialized();
  }

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

 private:
  VisionImuFrontend::UniquePtr vio_frontend_;
};

}  // namespace VIO
