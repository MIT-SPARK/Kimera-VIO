/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoPipeline.h
 * @brief  Implements StereoVIO pipeline workflow.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#pragma once

#include <stddef.h>
#include <atomic>
#include <cstdlib>  // for srand()
#include <memory>
#include <thread>
#include <utility>  // for make_pair
#include <vector>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/backend/VioBackEndModule.h"
#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/dataprovider/StereoDataProviderModule.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/VisionFrontEndModule.h"
#include "kimera-vio/loopclosure/LoopClosureDetector.h"
#include "kimera-vio/mesh/MesherModule.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/DisplayModule.h"
#include "kimera-vio/visualizer/Visualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3DModule.h"

namespace VIO {

class StereoPipeline : public Pipeline<StereoImuSyncPacket> {
 public:
  KIMERA_POINTER_TYPEDEFS(StereoPipeline);
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoPipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  /**
     * @brief StereoPipeline
     * @param params Vio parameters
     * @param visualizer Optional visualizer for visualizing 3D results
     * @param displayer Optional displayer for visualizing 2D results
     */
  StereoPipeline(const VioParams& params,
                 Visualizer3D::UniquePtr&& visualizer = nullptr,
                 DisplayBase::UniquePtr&& displayer = nullptr);

  virtual ~StereoPipeline();

 public:
  bool spin() override {
    // Feed data to the pipeline
    CHECK(data_provider_module_);
    // TODO(marcus): assert data-provider is derived from base class
    LOG(INFO) << "Spinning Kimera-VIO.";
    return data_provider_module_->spin();
  }

  //! Callbacks to fill stereo frames
  inline void fillLeftFrameQueue(Frame::UniquePtr left_frame) {
    CHECK(data_provider_module_);
    CHECK(left_frame);
    data_provider_module_->fillLeftFrameQueue(std::move(left_frame));
  }
  inline void fillRightFrameQueue(Frame::UniquePtr right_frame) {
    CHECK(data_provider_module_);
    CHECK(right_frame);
    data_provider_module_->fillRightFrameQueue(std::move(right_frame));
  }
  //! Callbacks to fill queues but they block if queues are getting full.
  //! Useful when parsing datasets, don't use with real sensors.
  inline void fillLeftFrameQueueBlockingIfFull(Frame::UniquePtr left_frame) {
    CHECK(data_provider_module_);
    CHECK(left_frame);
    data_provider_module_->fillLeftFrameQueueBlockingIfFull(
        std::move(left_frame));
  }
  inline void fillRightFrameQueueBlockingIfFull(Frame::UniquePtr right_frame) {
    CHECK(data_provider_module_);
    CHECK(right_frame);
    data_provider_module_->fillRightFrameQueueBlockingIfFull(
        std::move(right_frame));
  }
  //! Fill one IMU measurement at a time.
  inline void fillSingleImuQueue(const ImuMeasurement& imu_measurement) {
    CHECK(data_provider_module_);
    data_provider_module_->fillImuQueue(imu_measurement);
  }
  //! Fill multiple IMU measurements in batch
  inline void fillMultiImuQueue(const ImuMeasurements& imu_measurements) {
    CHECK(data_provider_module_);
    data_provider_module_->fillImuQueue(imu_measurements);
  }

 public:
  /**
   * @brief spinViz Run an endless loop until shutdown to visualize.
   * @return Returns whether the visualizer_ is running or not. While in
   * parallel mode, it does not return unless shutdown.
   */
  bool spinViz() override;

  /**
   * @brief shutdownWhenFinished
   * Shutdown the pipeline once all data has been consumed, or if the backend
   * has died unexpectedly.
   * @param sleep_time_ms period of time between checks of vio status.
   * @return true if shutdown succesful, false otherwise (never returns
   * unless successful shutdown).
   */
  bool shutdownWhenFinished(const int& sleep_time_ms = 500) override;

  void shutdown() override;

  /**
   * @brief resume Resumes all queues.
   */
  void resume() override;

  //! Register external callback to output the VIO backend results.
  inline void registerBackendOutputCallback(
      const VioBackEndModule::OutputCallback& callback) {
    CHECK(vio_backend_module_);
    vio_backend_module_->registerOutputCallback(callback);
  }

  //! Register external callback to output the VIO frontend results.
  inline void registerFrontendOutputCallback(
      const StereoVisionFrontEndModule::OutputCallback& callback) {
    CHECK(vio_frontend_module_);
    vio_frontend_module_->registerOutputCallback(callback);
  }

  //! Register external callback to output mesher results.
  inline void registerMesherOutputCallback(
      const MesherModule::OutputCallback& callback) {
    if (mesher_module_) {
      mesher_module_->registerOutputCallback(callback);
    } else {
      LOG(ERROR) << "Attempt to register Mesher output callback, but no "
                 << "Mesher member is active in pipeline.";
    }
  }

  //! Register external callback to output the LoopClosureDetector's results.
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
  // Spin the pipeline only once.
  void spinOnce(StereoImuSyncPacket::UniquePtr input) override;

  // A parallel pipeline should always be able to run sequentially...
  void spinSequential() override;

 protected:
  inline bool isInitialized() const override {
    return vio_frontend_module_->isInitialized() &&
           vio_backend_module_->isInitialized();
  }

  void launchThreads() override;

  void stopThreads() override;

  void joinThreads() override;

  void joinThread(const std::string& thread_name, std::thread* thread) override;

  //! Shutdown for in case backend fails (this is done for a graceful shutdown).
  void signalBackendFailure() {
    VLOG(1) << "Backend failure signal received.";
    is_backend_ok_ = false;
  }

  // VIO parameters
  //! Mind that the backend params is shared with the dataprovider which might
  //! modify them to add the ground truth initial 3d pose
  BackendParams::ConstPtr backend_params_;
  FrontendParams frontend_params_;
  ImuParams imu_params_;
  BackendType backend_type_;

  //! Definition of sensor rig used
  StereoCamera::Ptr stereo_camera_;

  // Pipeline Modules
  //! Data provider.
  // TODO(marcus): is shadowing sufficient? will base functions work properly?
  StereoDataProviderModule::UniquePtr data_provider_module_;

  // TODO(Toni) this should go to another class to avoid not having copy-ctor...
  //! Frontend.
  StereoVisionFrontEndModule::UniquePtr vio_frontend_module_;

  //! Stereo vision frontend payloads.
  StereoVisionFrontEndModule::InputQueue stereo_frontend_input_queue_;

  //! Backend
  VioBackEndModule::UniquePtr vio_backend_module_;

  //! Thread-safe queue for the backend.
  VioBackEndModule::InputQueue backend_input_queue_;

  //! Mesher
  MesherModule::UniquePtr mesher_module_;

  //! Loop Closure Detector
  LcdModule::UniquePtr lcd_module_;

  //! Visualizer: builds images to be displayed
  VisualizerModule::UniquePtr visualizer_module_;

  //! Thread-safe queue for the input to the display module
  DisplayModule::InputQueue display_input_queue_;

  //! Displays actual images and 3D visualization
  DisplayModule::UniquePtr display_module_;

  // Atomic Flags
  std::atomic_bool is_backend_ok_ = {true};

  // Pipeline Threads.
  std::unique_ptr<std::thread> frontend_thread_ = {nullptr};
  std::unique_ptr<std::thread> backend_thread_ = {nullptr};
  std::unique_ptr<std::thread> mesher_thread_ = {nullptr};
  std::unique_ptr<std::thread> lcd_thread_ = {nullptr};
  std::unique_ptr<std::thread> visualizer_thread_ = {nullptr};
};

}  // namespace VIO
