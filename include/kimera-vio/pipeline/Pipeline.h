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

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/backend/VioBackEndModule.h"
#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/frontend/VisionFrontEndModule.h"
#include "kimera-vio/loopclosure/LoopClosureDetector.h"
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

template <class FInput, class FOutput>
class Pipeline {
 public:
  KIMERA_POINTER_TYPEDEFS(Pipeline);
  KIMERA_DELETE_COPY_CONSTRUCTORS(Pipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  Pipeline(const VioParams& params)
      : backend_params_(params.backend_params_),
        frontend_params_(params.frontend_params_),
        imu_params_(params.imu_params_),
        parallel_run_(params.parallel_run_),
        shutdown_pipeline_cb_(nullptr),
        vio_frontend_module_(nullptr),
        vio_backend_module_(nullptr),
        mesher_module_(nullptr),
        lcd_module_(nullptr),
        visualizer_module_(nullptr),
        display_module_(nullptr),
        frontend_input_queue_("frontend_input_queue"),
        backend_input_queue_("backend_input_queue"),
        display_input_queue_("display_input_queue"),
        frontend_thread_(nullptr),
        backend_thread_(nullptr),
        mesher_thread_(nullptr),
        lcd_thread_(nullptr),
        visualizer_thread_(nullptr) {
    if (FLAGS_deterministic_random_number_generator) {
      setDeterministicPipeline();
    }
  }

  virtual ~Pipeline() {}

 public:
  /**
   * @brief spin Spin the whole pipeline by spinning the data provider
   * If in sequential mode, it will return for each spin.
   * If in parallel mode, it will not return until the pipeline is shutdown.
   * @return Data provider module state: false if finished or shutdown, true
   * if working nominally (it does not return unless shutdown in parallel mode).
   */
  virtual bool spin() = 0;
  /**
   * @brief spinViz Run an endless loop until shutdown to visualize.
   * @return Returns whether the visualizer_ is running or not. While in
   * parallel mode, it does not return unless shutdown.
   */
  virtual bool spinViz() {
    if (display_module_) {
      return display_module_->spin();
    }
    return true;
  }

  /**
   * @brief shutdownWhenFinished//! Callback called when the VIO pipeline has shut down.
   * This must be specified in derived classes because it must reference the data 
   * provider module.
   */
  virtual bool shutdownWhenFinished(const int& sleep_time_ms) = 0;

  /**
   * @brief shutdown Shutdown processing pipeline: stops and joins threads,
   * stops queues. And closes logfiles.
   */
  virtual void shutdown() {
    LOG_IF(ERROR, shutdown_) << "Shutdown requested, but Pipeline was already "
                                "shutdown.";
    LOG(INFO) << "Shutting down VIO pipeline.";
    shutdown_ = true;

    // First: call registered shutdown callbacks, these are typically to signal
    // data providers that they should now die.
    if (shutdown_pipeline_cb_) {
      LOG(INFO) << "Calling registered shutdown callbacks...";
      // Mind that this will raise a SIGSEGV seg fault if the callee is
      // destroyed.
      shutdown_pipeline_cb_();
    }
  }

  /**
   * @brief resume Resumes all queues.
   */
  virtual void resume() {
    LOG(INFO) << "Restarting frontend workers and queues...";
    frontend_input_queue_.resume();

    LOG(INFO) << "Restarting backend workers and queues...";
    backend_input_queue_.resume();
  }

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
  virtual void spinOnce(std::unique_ptr<FInput> input) {
    CHECK(input);
    if (!shutdown_) {
      // Push to frontend input queue.
      VLOG(2) << "Push input payload to Frontend.";
      frontend_input_queue_.pushBlockingIfFull(std::move(input), 5u);

      if (!parallel_run_) {
        // Run the pipeline sequentially.
        spinSequential();
      }
    } else {
      LOG(WARNING) << "Not spinning pipeline as it's been shutdown.";
    }
  }

  /**
   * @brief Sequential pipeline runner.
   * Must be written in the derived class because it references the data'
   * provider module.
  */
  virtual void spinSequential() = 0;

 protected:
  //! Initialize random seed for repeatability (only on the same machine).
  //! Still does not make RANSAC repeatable across different machines.
  virtual void setDeterministicPipeline() const { srand(0); }

  virtual bool isInitialized() const {
    return vio_frontend_module_->isInitialized() &&
            vio_backend_module_->isInitialized();
  }

  //! Shutdown for in case backend fails (this is done for a graceful shutdown).
  virtual void signalBackendFailure() {
    VLOG(1) << "Backend failure signal received.";
    is_backend_ok_ = false;
  }

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

  /// Launch threads for each pipeline module.
  virtual void launchThreads() {
    if (parallel_run_) {
      frontend_thread_ = VIO::make_unique<std::thread>(
          &VisionFrontEndModule<FInput, FOutput>::spin,
          CHECK_NOTNULL(vio_frontend_module_.get()));

      backend_thread_ = VIO::make_unique<std::thread>(
          &VioBackEndModule::spin, CHECK_NOTNULL(vio_backend_module_.get()));

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
  
  /// Shutdown processes and queues.
  virtual void stopThreads() {
    VLOG(1) << "Stopping workers and queues...";

    backend_input_queue_.shutdown();
    // TODO(marcus): enable:
    // CHECK(vio_backend_module_);
    // vio_backend_module_->shutdown();

    frontend_input_queue_.shutdown();
    CHECK(vio_frontend_module_);
    vio_frontend_module_->shutdown();

    if (mesher_module_) mesher_module_->shutdown();
    if (lcd_module_) lcd_module_->shutdown();
    if (visualizer_module_) visualizer_module_->shutdown();
    if (display_module_) {
      display_input_queue_.shutdown();
      display_module_->shutdown();
    }

    VLOG(1) << "Sent stop flag to all module and queues...";
  }

  /// Join threads to do a clean shutdown.
  virtual void joinThreads() {
    LOG_IF(WARNING, !parallel_run_)
        << "Asked to join threads while in sequential mode, this is ok, but "
        << "should not happen.";
    VLOG(1) << "Joining threads...";

    joinThread("backend", backend_thread_.get());
    joinThread("frontend", frontend_thread_.get());
    joinThread("mesher", mesher_thread_.get());
    joinThread("lcd", lcd_thread_.get());
    joinThread("visualizer", visualizer_thread_.get());

    VLOG(1) << "All threads joined.";
  }

  /// Join a single thread.
  virtual void joinThread(const std::string& thread_name,
                          std::thread* thread) {
    if (thread) {
      VLOG(1) << "Joining " << thread_name.c_str() << " thread...";
      if (thread->joinable()) {
        thread->join();
        VLOG(1) << "Joined " << thread_name.c_str() << " thread...";
      } else {
        LOG_IF(ERROR, parallel_run_)
            << thread_name.c_str() << " thread is not joinable...";
      }
    } else {
      LOG(WARNING) << "No " << thread_name.c_str() << " thread, not joining.";
    }
  }

 protected:
  // VIO parameters
  //! Mind that the backend params is shared with the dataprovider which might
  //! modify them to add the ground truth initial 3d pose
  BackendParams::ConstPtr backend_params_;
  FrontendParams frontend_params_;
  ImuParams imu_params_;
  bool parallel_run_;

  //! Shutdown switch to stop pipeline, threads, and queues.
  std::atomic_bool shutdown_ = {false};

  //! Callback called when the VIO pipeline has shut down.
  ShutdownPipelineCallback shutdown_pipeline_cb_;

  // TODO(Toni) this should go to another class to avoid not having copy-ctor...
  //! Frontend.
  typename VisionFrontEndModule<FInput, FOutput>::UniquePtr vio_frontend_module_;

  //! Vision frontend payloads.
  typename VisionFrontEndModule<FInput, FOutput>::InputQueue frontend_input_queue_;

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
