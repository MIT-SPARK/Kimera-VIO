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

  bool shutdownWhenFinished(const int& sleep_time_ms,
                            const bool& print_stats = false) override;

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

  inline bool isInitialized() const override {
    return vio_frontend_module_->isInitialized() &&
          // TODO(marcus): enable!
          //  vio_backend_module_->isInitialized();
          true;
  }

  void launchThreads() override {
    if (parallel_run_) {
      frontend_thread_ = VIO::make_unique<std::thread>(
          &MonoVisionFrontEndModule::spin,
          CHECK_NOTNULL(vio_frontend_module_.get()));

      // TODO(marcus): remove this function, use the base version
      // backend_thread_ = VIO::make_unique<std::thread>(
      //     &VioBackEndModule::spin, CHECK_NOTNULL(vio_backend_module_.get()));

      if (mesher_module_) {
        mesher_thread_ = VIO::make_unique<std::thread>(
            &MesherModule::spin, CHECK_NOTNULL(mesher_module_.get()));
      }

      if (lcd_module_) {
        lcd_thread_ = VIO::make_unique<std::thread>(
            &LcdModule::spin, CHECK_NOTNULL(lcd_module_.get()));
      }

      if (visualizer_module_) {
        visualizer_thread_ = VIO::make_unique<std::thread>(
            &VisualizerModule::spin, CHECK_NOTNULL(visualizer_module_.get()));
      }
      LOG(INFO) << "Pipeline Modules launched (parallel_run set to "
                << parallel_run_ << ").";
    } else {
      LOG(INFO) << "Pipeline Modules running in sequential mode"
                << " (parallel_run set to " << parallel_run_ << ").";
    }
  }

 protected:
  Camera::Ptr camera_;
  MonoDataProviderModule::UniquePtr data_provider_module_;
};

}  // namespace VIO
