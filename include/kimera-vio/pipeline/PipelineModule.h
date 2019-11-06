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
#include <string>

#include <glog/logging.h>

#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/Timer.h"

namespace VIO {

/**
 * @brief Abstraction of a pipeline module. Templated on the expected input
 * and output payloads.
 *
 * Provides common algorithmic logic for a pipeline module, in summary:
 * - spin(): runs the pipeline module by pulling and pushing from/to the
 * input/output. The way pushing and pulling input/output depends on the user
 * implementation of the functions getSyncedInputPacket() and
 * pushOutputPacket.
 * - spinOnce(): given a minimal input, computes the output of the module.
 * Returning a nullptr signals that the output should not be pushed as
 * output but just ignored.
 */
template <typename Input, typename Output>
class PipelineModule {
 public:
  KIMERA_POINTER_TYPEDEFS(PipelineModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(PipelineModule);
  using InputPtr = std::unique_ptr<Input>;
  using OutputPtr = std::unique_ptr<Output>;

  // TODO(Toni) In/Output queue should be shared ptr
  /**
   * @brief PipelineModule
   * @param name_id Identifier for the pipeline module
   * @param parallel_run Spin in parallel mode or sequentially (the spin
   * does only one call to spinOnce and returns).
   */
  PipelineModule(const std::string& name_id, const bool& parallel_run)
      : parallel_run_(parallel_run), name_id_(name_id) {}
  virtual ~PipelineModule() { LOG(INFO) << name_id_ + " destructor called."; }

  /**
   * @brief getSyncedInputPacket Retrieves the input packet for processing.
   * The typical usage of this function just pops from a threadsafe queue that
   * contains the input packets to be processed. Alternatively, one may consider
   * synchronizing different queues and generating a custom packet.
   * @param[out] input_packet Parameter to be filled that is then used by the
   * pipeline module's specific spinOnce.
   * @return a boolean indicating whether the generation of the input packet was
   * successful.
   */
  virtual inline bool getInputPacket(InputPtr input_packet) = 0;

  /**
   * @brief pushOutputPacket Sends the output of the module to other interested
   * parties, potentially other pipeline modules.
   * The typical use case would be to just push to a threadsafe output queue
   * the newly generated output. Alternatively, one may override this function
   * to send the output to multiple registered queues or callbacks.
   * @param[out] output_packet  Parameter to be sent to others
   * @return boolean indicating whether the push was successful or not.
   */
  virtual inline bool pushOutputPacket(OutputPtr output_packet) = 0;

  /**
   * @brief Main spin function. Every pipeline module calls this spin, where
   * the input is taken from an input queue and processed into an output packet
   * which is sent to the output queue. If the module returns a nullptr, then
   * we don't push to the output queue to save computation time.
   * @return True if everything goes well.
   */
  bool spin() {
    LOG(INFO) << "Spinning" << name_id_;
    utils::StatsCollector timing_stats(name_id_ + " [ms]");
    while (!shutdown_) {
      // Get input data from queue by waiting for payload.
      is_thread_working_ = false;
      InputPtr input;
      if (getInputPacket(input)) {
        is_thread_working_ = true;
        if (input) {
          auto tic = utils::Timer::tic();
          OutputPtr output = spinOnce(*input);
          if (output) {
            // Received a valid output, send to output queue
            if (!pushOutputPacket(output)) {
              LOG(WARNING) << "Output is down for module: " << name_id_;
            }
          } else {
            VLOG(1) << "Module " << name_id_ << " skipped sending an output.";
          }
          auto spin_duration = utils::Timer::toc(tic).count();
          LOG(WARNING) << "Module " << name_id_
                       << " - frequency: " << 1000.0 / spin_duration << " Hz. ("
                       << spin_duration << " ms).";
          timing_stats.AddSample(spin_duration);
        } else {
          LOG(WARNING) << "No Input received for module: " << name_id_;
        }
      } else {
        LOG(WARNING) << "Input is down for module: " << name_id_;
      }

      // Break the while loop if we are in sequential mode.
      if (!parallel_run_) return true;
    }
    LOG(INFO) << "Successful shutdown of module: " << name_id_;
    return true;
  }

  /**
   * @brief Abstract function to process a single input payload.
   * @param[in] input: an input payload for module to work on.
   * @return The output payload from the pipeline module. Returning a nullptr
   * signals that the output should not be sent to the output queue.
   */
  virtual OutputPtr spinOnce(const Input& input) = 0;

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
  //! Checks if the thread is waiting for the input or if it is working instead.
  inline bool isWorking() const { return is_thread_working_; }

 private:
  //! Thread related members.
  std::atomic_bool shutdown_ = {false};
  std::atomic_bool is_thread_working_ = {false};
  bool parallel_run_ = {true};

  //! Properties
  std::string name_id_ = {"PipelineModule"};
};

/** @brief SISOPipelineModule Single Input Single Output (SISO) pipeline module.
 * Receives Input packets via a threadsafe queue, and sends output packets
 * to a threadsafe output queue.
 * This is the most standard and simplest pipeline module.
 */
template <typename Input, typename Output>
class SISOPipelineModule : public PipelineModule<Input, Output> {
 public:
  KIMERA_POINTER_TYPEDEFS(SISOPipelineModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(SISOPipelineModule);

  using PIO = PipelineModule<Input, Output>;
  using InputQueue = ThreadsafeQueue<typename PIO::InputPtr>;
  using OutputQueue = ThreadsafeQueue<typename PIO::OutputPtr>;

  SISOPipelineModule(InputQueue* input_queue,
                     OutputQueue* output_queue,
                     const std::string& name_id,
                     const bool& parallel_run)
      : PipelineModule<Input, Output>(name_id, parallel_run),
        input_queue_(input_queue),
        output_queue_(output_queue) {
    CHECK(input_queue_);
    CHECK(output_queue_);
  }
  virtual ~SISOPipelineModule() = 0;

  /**
   * @brief getSyncedInputPacket Retrieves the input packet for processing.
   * Just pops from a threadsafe queue that contains the input packets to be
   * processed.
   * @param[out] input_packet Parameter to be filled that is then used by the
   * pipeline module's specific spinOnce.
   * @return a boolean indicating whether the generation of the input packet was
   * successful.
   */
  virtual inline bool getInputPacket(
      typename PIO::InputPtr input_packet) const override {
    return input_queue_->popBlocking(input_packet);
  }

  /**
   * @brief pushOutputPacket Sends the output of the module to other interested
   * parties, potentially other pipeline modules.
   * Just push to a threadsafe output queue the newly generated output.
   * @param[out] output_packet  Parameter to be sent to others
   * @return boolean indicating whether the push was successful or not.
   */
  virtual inline bool pushOutputPacket(
      typename PIO::OutputPtr output_packet) const override {
    return output_queue_->push(std::move(output_packet));
  }

 private:
  //! Queues
  InputQueue* input_queue_;
  OutputQueue* output_queue_;
};

}  // namespace VIO
