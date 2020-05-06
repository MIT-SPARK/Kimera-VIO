/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Pipeline.h
 * @brief  Implements VIO pipeline workflow.
 * @author Antoni Rosinol
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
#include "kimera-vio/dataprovider/DataProviderModule.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/VisionFrontEndModule.h"
#include "kimera-vio/initial/InitializationBackEnd-definitions.h"
#include "kimera-vio/loopclosure/LoopClosureDetector.h"
#include "kimera-vio/mesh/MesherModule.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/visualizer/DisplayModule.h"
#include "kimera-vio/visualizer/Visualizer3DModule.h"

namespace VIO {

class Pipeline {
 public:
  KIMERA_POINTER_TYPEDEFS(Pipeline);
  KIMERA_DELETE_COPY_CONSTRUCTORS(Pipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  explicit Pipeline(const VioParams& params);

  virtual ~Pipeline();

 public:
  // Callbacks to fill input queues.
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
  bool spinViz();

  /**
   * @brief shutdownWhenFinished
   * Shutdown the pipeline once all data has been consumed, or if the backend
   * has died unexpectedly.
   * @param sleep_time_ms period of time between checks of vio status.
   * @return true if shutdown succesful, false otherwise (never returns
   * unless successful shutdown).
   */
  bool shutdownWhenFinished(const int& sleep_time_ms = 500);

  /**
   * @brief shutdown Shutdown processing pipeline: stops and joins threads,
   * stops queues. And closes logfiles.
   */
  void shutdown();

  /**
   * @brief resume Resumes all queues.
   */
  void resume();

  // Output Callbacks
  //! Register external callback to output the VIO backend results.
  inline void registerBackendOutputCallback(
      const VioBackEndModule::OutputCallback& callback) {
    CHECK(vio_backend_module_);
    vio_backend_module_->registerOutputCallback(callback);
  }

  // TODO(marcus): once we have a base class for StereoVisionFrontend, we need
  // that type to go here instead.
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

  //! Register external callback to be called when the VIO pipeline shuts down.
  inline void registerShutdownCallback(
      const ShutdownPipelineCallback& callback) {
    shutdown_pipeline_cb_ = callback;
  }

  /**
   * @brief spin Spin the whole pipeline by spinning the data provider
   * If in sequential mode, it will return for each spin.
   * If in parallel mode, it will not return until the pipeline is shutdown.
   * @return Data provider module state: false if finished or shutdown, true
   * if working nominally (it does not return unless shutdown in parallel mode).
   */
  bool spin() {
    // Feed data to the pipeline
    CHECK(data_provider_module_);
    LOG(INFO) << "Spinning Kimera-VIO.";
    return data_provider_module_->spin();
  }

  /**
   * @brief printStatistics Prints timing statistics of each VIO module.
   * @return A table of the timing statistics that can be printed to console.
   */
  inline std::string printStatistics() const {
    return utils::Statistics::Print();
  }

 private:
  // Spin the pipeline only once.
  void spinOnce(StereoImuSyncPacket::UniquePtr stereo_imu_sync_packet);

  // A parallel pipeline should always be able to run sequentially...
  void spinSequential();

 private:
  // TODO(Toni) Still does not make RANSAC repeatable across different machines.
  //! Initialize random seed for repeatability (only on the same machine).
  inline void setDeterministicPipeline() const { srand(0); }

  // Initialization functions
  /// Initialize pipeline with desired option (flag).
  bool initialize(const StereoImuSyncPacket& stereo_imu_sync_packet);

  /// Check if necessary to re-initialize pipeline.
  void checkReInitialize(const StereoImuSyncPacket& stereo_imu_sync_packet);

  /// Initialize pipeline from ground truth pose.
  bool initializeFromGroundTruth(
      const StereoImuSyncPacket& stereo_imu_sync_packet,
      const VioNavState& initial_ground_truth_state);

  /// Initialize pipeline from IMU readings only:
  ///  - Guesses initial state assuming zero velocity.
  ///  - Guesses IMU bias assuming steady upright vehicle.
  bool initializeFromIMU(const StereoImuSyncPacket& stereo_imu_sync_packet);

  /// Initialize pipeline from online gravity alignment.
  bool initializeOnline(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Thread Managing
  /// Launch frontend thread with process.
  void launchFrontendThread();

  /// Launch remaining threads with processes.
  void launchRemainingThreads();

  /// Shutdown processes and queues.
  void stopThreads();

  /// Join threads to do a clean shutdown.
  void joinThreads();

  /// Join a single thread.
  void joinThread(const std::string& thread_name, std::thread* thread);

  // Shutdown for in case backend fails (this is done for a graceful shutdown).
  void signalBackendFailure() {
    VLOG(1) << "Backend failure signal received.";
    is_backend_ok_ = false;
  }

 public:
  //! VIO parameters
  VioParams vio_params_;

  //! TODO(TONI): remove this
  FrameId init_frame_id_;

  //! Definition of sensor rig used
  StereoCamera::UniquePtr stereo_camera_;

  // Pipeline Modules
  //! Data provider.
  DataProviderModule::UniquePtr data_provider_module_;

  // TODO(Toni) this should go to another class to avoid not having copy-ctor...
  //! Frontend.
  StereoVisionFrontEndModule::UniquePtr vio_frontend_module_;

  //! Stereo vision frontend payloads.
  StereoVisionFrontEndModule::InputQueue stereo_frontend_input_queue_;

  // Online initialization frontend queue.
  ThreadsafeQueue<InitializationInputPayload::UniquePtr>
      initialization_frontend_output_queue_;

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
  //! Shutdown switch to stop pipeline, threads, and queues.
  std::atomic_bool shutdown_ = {false};
  std::atomic_bool is_initialized_ = {false};
  std::atomic_bool is_launched_ = {false};
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
