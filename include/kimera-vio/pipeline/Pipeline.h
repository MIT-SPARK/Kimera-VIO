/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Pipeline.h
 * @brief  Implements abstract VIO pipeline workflow.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#pragma once

#include <gflags/gflags.h>
#include <stddef.h>

#include <atomic>
#include <cstdlib>  // for srand()
#include <memory>
#include <thread>
#include <vector>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/backend/VioBackendModule.h"
#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/dataprovider/MonoDataProviderModule.h"
#include "kimera-vio/frontend/VisionImuFrontendModule.h"
#include "kimera-vio/loopclosure/LcdModule.h"
#include "kimera-vio/mesh/MesherModule.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/DisplayModule.h"
#include "kimera-vio/visualizer/Visualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3DModule.h"

DECLARE_bool(log_output);
DECLARE_bool(extract_planes_from_the_scene);
DECLARE_bool(visualize);
DECLARE_bool(visualize_lmk_type);
DECLARE_int32(viz_type);
DECLARE_bool(deterministic_random_number_generator);
DECLARE_int32(min_num_obs_for_mesher_points);
DECLARE_bool(use_lcd);

namespace VIO {

class Pipeline {
 public:
  KIMERA_POINTER_TYPEDEFS(Pipeline);
  KIMERA_DELETE_COPY_CONSTRUCTORS(Pipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  Pipeline(const VioParams& params);

  virtual ~Pipeline();

 public:
  //! Callbacks to fill input queues.
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

  inline void fillExternalOdomQueue(
      const ExternalOdomMeasurement& odom_measurement) {
    CHECK(data_provider_module_);
    data_provider_module_->fillExternalOdometryQueue(odom_measurement);
  }

  inline LcdModule* getLcdModule() const { return lcd_module_.get(); }

 public:
  /**
   * @brief spin Spin the whole pipeline by spinning the data provider
   * If in sequential mode, it will return for each spin.
   * If in parallel mode, it will not return until the pipeline is shutdown.
   * @return Data provider module state: false if finished or shutdown, true
   * if working nominally (it does not return unless shutdown in parallel mode).
   */
  virtual bool spin();

  /**
   * @brief spinViz Run an endless loop until shutdown to visualize.
   * @return Returns whether the visualizer_ is running or not. While in
   * parallel mode, it does not return unless shutdown.
   */
  virtual bool spinViz();

  /**
   * @brief printStatus Returns a string with useful information to monitor the
   * status of the pipeline, in particular, whether the pipeline's modules are
   * working and if their queues are filled.
   * @return String with pipeline status information
   */
  virtual std::string printStatus() const;

  /**
   * @brief hasFinished
   * @return Whether the pipeline has finished working or not.
   */
  virtual bool hasFinished() const;

  /**
   * @brief shutdownWhenFinished
   * Shutdown the pipeline once all data has been consumed, or if the Backend
   * has died unexpectedly.
   * @param sleep_time_ms period of time between checks of vio status.
   * @return true if shutdown succesful, false otherwise (only returns
   * if running in sequential mode, or if shutdown happens).
   */
  virtual bool shutdownWhenFinished(const int& sleep_time_ms = 500,
                                    const bool& print_stats = false);

  /**
   * @brief spin pipeline until all data has been processed
   * Shutdown the pipeline once all data has been consumed, or if the Backend
   * has died unexpectedly.
   * @param data_done_cb threadsafe callback to check if the data
   * provider has finished queueing all avaiable data
   * @param sleep_time_ms period of time between checks of vio status.
   * @return true if shutdown succesful, false otherwise (only returns
   * if running in sequential mode, or if shutdown happens).
   */
  virtual bool waitForShutdown(const std::function<bool()>& data_done_cb,
                               const int& sleep_time_ms = 501,
                               const bool& print_stats = false);

  /**
   * @brief shutdown Shutdown processing pipeline: stops and joins threads,
   * stops queues. And closes logfiles.
   */
  virtual void shutdown();

  inline bool isShutdown() const { return shutdown_; }

  /**
   * @brief resume Resumes all queues.
   */
  virtual void resume();

  //! Register external callback to be called when the VIO pipeline shuts down.
  virtual void registerShutdownCallback(
      const ShutdownPipelineCallback& callback) {
    shutdown_pipeline_cb_ = callback;
  }

  /**
   * @brief printStatistics Prints timing statistics of each VIO module.
   * @return A table of the timing statistics that can be printed to console.
   */
  inline std::string printStatistics() const {
    return utils::Statistics::Print();
  }

 protected:
  // Spin the pipeline only once.
  virtual void spinOnce(FrontendInputPacketBase::UniquePtr input);

  /**
   * @brief Sequential pipeline runner.
   * Must be written in the derived class because it references the data'
   * provider module.
   */
  virtual void spinSequential();

 protected:
  //! Initialize random seed for repeatability (only on the same machine).
  //! Still does not make RANSAC repeatable across different machines.
  virtual void setDeterministicPipeline() const { srand(0); }

  virtual bool isInitialized() const {
    return vio_frontend_module_->isInitialized() &&
           vio_backend_module_->isInitialized();
  }

  //! Shutdown for in case Backend fails (this is done for a graceful shutdown).
  virtual void signalBackendFailure() {
    VLOG(1) << "Backend failure signal received.";
    is_backend_ok_ = false;
  }

  inline void registerBackendOutputCallback(
      const VioBackendModule::OutputCallback& callback) {
    CHECK(vio_backend_module_);
    vio_backend_module_->registerOutputCallback(callback);
  }

  inline void registerFrontendOutputCallback(
      const typename VisionImuFrontendModule::OutputCallback& callback) {
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

  /// Launch threads for each pipeline module.
  virtual void launchThreads();

  /// Shutdown processes and queues.
  virtual void stopThreads();

  /// Join threads to do a clean shutdown.
  virtual void joinThreads();

  /// Join a single thread.
  virtual void joinThread(const std::string& thread_name, std::thread* thread);

 protected:
  // VIO parameters
  //! Mind that the Backend params is shared with the dataprovider which might
  //! modify them to add the ground truth initial 3d pose
  BackendParams::ConstPtr backend_params_;
  FrontendParams frontend_params_;
  ImuParams imu_params_;
  bool parallel_run_;

  //! Shutdown switch to stop pipeline, threads, and queues.
  std::atomic_bool shutdown_ = {false};

  // Pipeline Modules
  // TODO(Toni) this should go to another class to avoid not having copy-ctor...
  //! Frontend.
  MonoDataProviderModule::UniquePtr data_provider_module_;

  // Vision Frontend
  VisionImuFrontendModule::UniquePtr vio_frontend_module_;

  //! Vision Frontend payloads.
  VisionImuFrontendModule::InputQueue frontend_input_queue_;

  //! Backend
  VioBackendModule::UniquePtr vio_backend_module_;

  //! Thread-safe queue for the Backend.
  VioBackendModule::InputQueue backend_input_queue_;

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

  // Pipeline Callbacks
  //! Callback called when the VIO pipeline has shut down.
  ShutdownPipelineCallback shutdown_pipeline_cb_;

  // Pipeline Threads.
  std::unique_ptr<std::thread> frontend_thread_ = {nullptr};
  std::unique_ptr<std::thread> backend_thread_ = {nullptr};
  std::unique_ptr<std::thread> mesher_thread_ = {nullptr};
  std::unique_ptr<std::thread> lcd_thread_ = {nullptr};
  std::unique_ptr<std::thread> visualizer_thread_ = {nullptr};
};

}  // namespace VIO
