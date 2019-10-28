/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   PipelineModule.h
 * @brief  Implements a VIO pipeline module.
 * @author Antoni Rosinol
 */

#pragma once

#include <atomic>
#include <memory>

#include <glog/logging.h>

#include <Eigen/Core>

#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/Timer.h"

namespace VIO {

struct PipelinePayload {
  KIMERA_POINTER_TYPEDEFS(PipelinePayload);
  KIMERA_DELETE_COPY_CONSTRUCTORS(PipelinePayload);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PipelinePayload();
  virtual ~PipelinePayload() = default;
};

/**
 * @brief Abstraction of a pipeline module. Templated on the expected input
 * payload. The output payload is a generic PipelinePayload.
 *
 * Provides common algorithmic logic for a pipeline module, in summary:
 * - spin(): runs the pipeline module by pulling and pushing from/to the
 * input/output queues given at construction.
 * - spinOnce():
 * @param myParam1 Description of 1st parameter.
 * @param myParam2 Description of 2nd parameter.
 * @returns Description of returned value.
 */
template <typename InputPayload>
class PipelineModule {
 public:
  KIMERA_POINTER_TYPEDEFS(PipelineModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(PipelineModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::unique_ptr<InputPayload> InputPayloadPtr;
  typedef ThreadsafeQueue<InputPayloadPtr> InputQueue;
  typedef ThreadsafeQueue<PipelinePayload::UniquePtr> OutputQueue;

  PipelineModule();
  virtual ~PipelineModule() { LOG(INFO) << name_id_ + " destructor called."; };

  /**
   * @brief Main spin function.
   * @return True if everything goes well.
   */
  bool spin() {
    LOG(INFO) << "Spinning" << name_id_;
    utils::StatsCollector timing_stats(name_id_ + " [ms]");
    while (!shutdown_) {
      // Get input data from queue by waiting for payload.
      is_thread_working_ = false;
      auto tic_pipeline_overall = utils::Timer::tic();
      InputPayloadPtr input;
      if (input_queue_.popBlocking(input)) {
        is_thread_working_ = true;
        if (input) {
          auto tic = utils::Timer::tic();
          output_queue_.push(spinOnce(*input));
          auto spin_duration = utils::Timer::toc(tic).count();
          LOG(WARNING) << "Module " << name_id_
                       << " - frequency: " << 1000.0 / spin_duration << " Hz. ("
                       << spin_duration << " ms).";
          timing_stats.AddSample(spin_duration);
        } else {
          LOG(WARNING) << "No InputPayload received for module: " << name_id_;
        }
      } else {
        LOG(WARNING) << "Input Queue is down for module: " << name_id_;
      }

      // Break the while loop if we are in sequential mode.
      if (!parallel_run_) return true;
    }
    LOG(INFO) << "Successfully shutdown for module: " << name_id_;
    return true;
  }

  /**
   * @brief Abstract functiuon to process a single input payload.
   * @param[in] input: an input payload for module to work on.
   * @return The output payload from the pipeline module.
   */
  virtual PipelinePayload::UniquePtr spinOnce(const InputPayload& input) = 0;

  /* ------------------------------------------------------------------------ */
  inline void shutdown() {
    LOG_IF(WARNING, shutdown_)
        << "Shutdown requested, but " << name_id_ << " was already shutdown.";
    LOG(INFO) << "Shutting down: " << name_id_;
    shutdown_ = true;
  }

  /* ------------------------------------------------------------------------ */
  inline void restart() {
    LOG(INFO) << "Resetting shutdown flag to false for: " << name_id_;
    shutdown_ = false;
  }

  /* ------------------------------------------------------------------------ */
  //! Checks if the thread is waiting for the input_queue or working.
  inline bool isWorking() const { return is_thread_working_; }

 private:
  std::string name_id_ = {"PipelineModule"};
  bool parallel_run_ = {true};

  //! Queues
  InputQueue input_queue_;
  OutputQueue output_queue_;

  //! Thread related members.
  std::atomic_bool shutdown_ = {false};
  std::atomic_bool is_thread_working_ = {false};
};

}  // namespace VIO
