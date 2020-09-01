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

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/backend/VioBackEndModule.h"
#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/dataprovider/MonoDataProviderModule.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/MonoImuSyncPacket.h"
#include "kimera-vio/frontend/VisionFrontEndModule.h"
#include "kimera-vio/loopclosure/LoopClosureDetector.h"
#include "kimera-vio/mesh/MesherModule.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/DisplayModule.h"
#include "kimera-vio/visualizer/Visualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3DModule.h"

namespace VIO {

class MonoPipeline : public Pipeline<MonoImuSyncPacket> {
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

 public:
  bool spinViz() override;

  bool shutdownWhenFinished(const int& sleep_time_ms = 500) override;

  void shutdown() override;

  void resume() override;

  inline void registerBackendOutputCallback(
      const VioBackEndModule::OutputCallback& callback) {
    CHECK(vio_backend_module_);
    vio_backend_module_->registerOutputCallback(callback);
  }

  inline void registerFrontendOutputCallback(
      const MonoVisionFrontEndModule::OutputCallback& callback) {
    CHECK(vio_frontend_module_);
    vio_frontend_module_->registerOutputCallback(callback);
  }

  inline void registerMesherOutputCallback(
      const MesherModule::OutputCallback& callback) {
    if (mesher_module_) {
      mesher_module_->registerOutputCallback(callback);
    } else {
      LOG(ERROR) << "Attempt to register Mesher output callback, but no "
                 << "Mesher member is active in pipeline.";
    }
  }

  inline void registerLcdOutputCallback(
      const LcdModule::OutputCallback& callback) {
    if (lcd_module_) {
      lcd_module_->registerOutputCallback(callback);
    } else {
      LOG(ERROR) << "Attempt to register LCD/PGO callback, but no "
                 << "LoopClosureDetector member is active in pipeline.";
    }
  }

 protected:
  void spinOnce(MonoImuSyncPacket::UniquePtr input) override;

  void spinSequential() override;

  inline bool isInitialized() const override {
    return vio_frontend_module_->isInitialized() &&
          // TODO(marcus): enable!
          //  vio_backend_module_->isInitialized();
          true;
  }

  void launchThreads() override;

  void stopThreads() override;

  void joinThreads() override;

  void joinThread(const std::string& thread_name, std::thread* thread) override;

  void signalBackendFailure() {
    VLOG(1) << "Backend failure signal received.";
    is_backend_ok_ = false;
  }

 protected:
  BackendParams::ConstPtr backend_params_;
  MonoFrontendParams frontend_params_;
  ImuParams imu_params_;

  Camera::Ptr camera_;

  MonoDataProviderModule::UniquePtr data_provider_module_;
  MonoVisionFrontEndModule::UniquePtr vio_frontend_module_;
  VioBackEndModule::UniquePtr vio_backend_module_;
  MesherModule::UniquePtr mesher_module_;
  LcdModule::UniquePtr lcd_module_;
  VisualizerModule::UniquePtr visualizer_module_;
  DisplayModule::UniquePtr display_module_;

  MonoVisionFrontEndModule::InputQueue frontend_input_queue_;
  VioBackEndModule::InputQueue backend_input_queue_;
  DisplayModule::InputQueue display_input_queue_;

  std::atomic_bool is_backend_ok_ = {true};

  std::unique_ptr<std::thread> frontend_thread_ = {nullptr};
  std::unique_ptr<std::thread> backend_thread_ = {nullptr};
  std::unique_ptr<std::thread> mesher_thread_ = {nullptr};
  std::unique_ptr<std::thread> lcd_thread_ = {nullptr};
  std::unique_ptr<std::thread> visualizer_thread_ = {nullptr};
};

}  // namespace VIO
