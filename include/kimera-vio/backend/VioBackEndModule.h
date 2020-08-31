/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackEndModule.h
 * @brief  Pipeline module for the backend.
 *
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/backend/RegularVioBackEnd.h"
#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/backend/VioBackEnd.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/Statistics.h"

namespace VIO {

class VioBackEndModule
    : public SIMOPipelineModule<BackendInput, BackendOutput> {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(VioBackEndModule);
  KIMERA_POINTER_TYPEDEFS(VioBackEndModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using SIMO = SIMOPipelineModule<BackendInput, BackendOutput>;
  using InputQueue = ThreadsafeQueue<typename PIO::InputUniquePtr>;

  /**
   * @brief VioBackEndModule
   * @param input_queue
   * @param output_queue
   * @param parallel_run
   * @param vio_backend
   */
  VioBackEndModule(InputQueue* input_queue,
                   bool parallel_run,
                   VioBackEnd::UniquePtr vio_backend);
  virtual ~VioBackEndModule() = default;

  /**
   * @brief spinOnce
   * @param input
   * @return
   */
  virtual OutputUniquePtr spinOnce(BackendInput::UniquePtr input);

 public:
  inline bool isInitialized() const { return vio_backend_->isInitialized(); }

  /**
   * @brief registerImuBiasUpdateCallback Register callback to be called
   * whenever the backend has a new estimate of the IMU bias.
   * The frontend needs to register this function for
   * the IMU preintegration to be done wrt the latest IMU bias estimate.
   * @param imu_bias_update_callback function that will be called on a new
   * IMU bias update.
   */
  void registerImuBiasUpdateCallback(
      const VioBackEnd::ImuBiasCallback& imu_bias_update_callback);

 protected:
  const VioBackEnd::UniquePtr vio_backend_;
};

}  // namespace VIO
