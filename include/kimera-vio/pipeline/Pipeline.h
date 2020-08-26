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

#include <stddef.h>
#include <atomic>
#include <cstdlib>  // for srand()
#include <memory>
#include <thread>
#include <utility>  // for make_pair
#include <vector>

#include "kimera-vio/dataprovider/DataProviderModule.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"

namespace VIO {

template <class Input>
class Pipeline {
 public:
  KIMERA_POINTER_TYPEDEFS(Pipeline);
  KIMERA_DELETE_COPY_CONSTRUCTORS(Pipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  Pipeline(bool parallel_run)
      : shutdown_pipeline_cb_(nullptr),
        parallel_run_(parallel_run) {}

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
  virtual bool spinViz() = 0;

  /**
   * @brief shutdownWhenFinished//! Callback called when the VIO pipeline has shut down.
  ShutdownPipelineCallback shutdown_pipeline_cb_;
   */
  virtual bool shutdownWhenFinished(const int& sleep_time_ms = 500) = 0;

  /**
   * @brief shutdown Shutdown processing pipeline: stops and joins threads,
   * stops queues. And closes logfiles.
   */
  virtual void shutdown() {
    LOG_IF(ERROR, shutdown_) << "Shutdown requested, but Pipeline was already "
                                "shutdown.";
    LOG(INFO) << "Shutting down VIO pipeline.";
    shutdown_ = true;
  }

  /**
   * @brief resume Resumes all queues.
   */
  virtual void resume() = 0;

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
  virtual void spinOnce(std::unique_ptr<Input> input) = 0;

  // A parallel pipeline should always be able to run sequentially...
  virtual void spinSequential() = 0;

 protected:
  //! Initialize random seed for repeatability (only on the same machine).
  //! Still does not make RANSAC repeatable across different machines.
  virtual void setDeterministicPipeline() const { srand(0); }

  virtual bool isInitialized() const = 0;
  
  /// Launch threads for each pipeline module.
  virtual void launchThreads() = 0;
  
  /// Shutdown processes and queues.
  virtual void stopThreads() = 0;
  
  /// Join threads to do a clean shutdown.
  virtual void joinThreads() = 0;
  
  /// Join a single thread.
  virtual void joinThread(const std::string& thread_name, std::thread* thread) = 0;

 protected:
  bool parallel_run_;

  //! Shutdown switch to stop pipeline, threads, and queues.
  std::atomic_bool shutdown_ = {false};

  //! Callback called when the VIO pipeline has shut down.
  ShutdownPipelineCallback shutdown_pipeline_cb_;
};

}  // namespace VIO
