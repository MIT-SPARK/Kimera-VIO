/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoPipeline.h
 * @brief  Implements MonoVIO pipeline workflow.
 * @author Marcus Abate
 */

#pragma once

#include "kimera-vio/dataprovider/MonoDataProviderModule.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/MonoImuSyncPacket.h"
#include "kimera-vio/frontend/MonoVisionFrontEnd-definitions.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/visualizer/DisplayFactory.h"
#include "kimera-vio/visualizer/Visualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3DFactory.h"

namespace VIO {

class MonoPipeline : public Pipeline<MonoImuSyncPacket, MonoFrontendOutput> {
 public:
  KIMERA_POINTER_TYPEDEFS(MonoPipeline);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoPipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  MonoPipeline(const VioParams& params,
               Visualizer3D::UniquePtr&& visualizer = nullptr,
               DisplayBase::UniquePtr&& displayer = nullptr);

  virtual ~MonoPipeline();

 public:
  bool spin() override {
    CHECK(data_provider_module_);
    LOG(INFO) << "Spinning Kimera-VIO.";
    return data_provider_module_->spin();
  }

  bool shutdownWhenFinished(const int& sleep_time_ms) override;

  inline void fillLeftFrameQueue(Frame::UniquePtr left_frame) {
    CHECK(data_provider_module_);
    CHECK(left_frame);
    data_provider_module_->fillLeftFrameQueue(std::move(left_frame));
  }

  inline void fillLeftFrameQueueBlockingIfFull(Frame::UniquePtr left_frame) {
    CHECK(data_provider_module_);
    CHECK(left_frame);
    data_provider_module_->fillLeftFrameQueueBlockingIfFull(
        std::move(left_frame));
  }

  inline void fillSingleImuQueue(const ImuMeasurement& imu_measurement) {
    CHECK(data_provider_module_);
    data_provider_module_->fillImuQueue(imu_measurement);
  }

  inline void fillMultiImuQueue(const ImuMeasurements& imu_measurements) {
    CHECK(data_provider_module_);
    data_provider_module_->fillImuQueue(imu_measurements);
  }

  void shutdown() override;

 protected:
  void spinSequential() override;

 protected:
  Camera::Ptr camera_;
  MonoDataProviderModule::UniquePtr data_provider_module_;
};

}  // namespace VIO
