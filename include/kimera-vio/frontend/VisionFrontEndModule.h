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

#include "kimera-vio/frontend/StereoVisionFrontEnd-definitions.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd.h"
#include "kimera-vio/pipeline/PipelineModule.h"

namespace VIO {

class StereoVisionFrontEndModule
    : public SIMOPipelineModule<StereoImuSyncPacket, FrontendOutput> {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoVisionFrontEndModule);
  KIMERA_POINTER_TYPEDEFS(StereoVisionFrontEndModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using SIMO = SIMOPipelineModule<StereoImuSyncPacket, FrontendOutput>;
  using InputQueue = ThreadsafeQueue<typename SIMO::InputUniquePtr>;

  /**
   * @brief StereoVisionFrontEndModule
   * @param input_queue
   * @param output_queue
   * @param parallel_run
   * @param vio_frontend
   */
  explicit StereoVisionFrontEndModule(
      InputQueue* input_queue,
      bool parallel_run,
      StereoVisionFrontEnd::UniquePtr vio_frontend);
  virtual ~StereoVisionFrontEndModule() = default;

 public:
  virtual OutputUniquePtr spinOnce(StereoImuSyncPacket::UniquePtr input);

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
  StereoVisionFrontEnd::UniquePtr vio_frontend_;
};

}  // namespace VIO
