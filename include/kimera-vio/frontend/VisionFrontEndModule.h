/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionFrontEndModule.h
 * @brief  Pipeline module for the vision frontend.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/VisionFrontEnd.h"
#include "kimera-vio/pipeline/PipelineModule.h"

namespace VIO {

class VisionFrontEndModule : public 
    SIMOPipelineModule<FrontendInputPacketBase, FrontendOutputPacketBase> {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisionFrontEndModule);
  KIMERA_POINTER_TYPEDEFS(VisionFrontEndModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using SIMO =
      SIMOPipelineModule<FrontendInputPacketBase, FrontendOutputPacketBase>;
  using InputQueue = ThreadsafeQueue<typename SIMO::InputUniquePtr>;

  /**
   * @brief VisionFrontEndModule
   * @param input_queue
   * @param output_queue
   * @param parallel_run
   * @param vio_frontend
   */
  explicit VisionFrontEndModule(
      InputQueue* input_queue,
      bool parallel_run,
      VisionFrontEnd::UniquePtr vio_frontend);

  virtual ~VisionFrontEndModule() = default;

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
  VisionFrontEnd::UniquePtr vio_frontend_;
};

}  // namespace VIO
