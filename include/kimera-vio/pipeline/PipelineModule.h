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
#include <functional>  // for function
#include <memory>
#include <string>

#include <glog/logging.h>

#include "kimera-vio/common/vio_types.h"
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
  //! The input is a unique ptr, as the user should implement getInputPacket
  //! such that it only retrieves an input structure with all data.
  using InputPtr = std::shared_ptr<Input>;  //! should be unique_ptr
  //! The output is instead a shared ptr, since many users might need the output
  using OutputPtr = std::shared_ptr<Output>;

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
   * @brief Main spin function. Every pipeline module calls this spin, where
   * the input is taken from an input queue and processed into an output packet
   * which is sent to the output queue. If the module returns a nullptr, then
   * we don't push to the output queue to save computation time.
   * @return True if everything goes well.
   */
  bool spin() {
    LOG_IF(INFO, parallel_run_) << "Module: " << name_id_ << " - Spinning.";
    utils::StatsCollector timing_stats(name_id_ + " [ms]");
    while (!shutdown_) {
      // Get input data from queue by waiting for payload.
      is_thread_working_ = false;
      InputPtr input = getInputPacket();
      is_thread_working_ = true;
      if (input) {
        auto tic = utils::Timer::tic();
        OutputPtr output = spinOnce(*input);
        if (output) {
          // Received a valid output, send to output queue
          if (!pushOutputPacket(output)) {
            LOG(WARNING) << "Module: " << name_id_ << " - Output push failed.";
          } else {
            VLOG(2) << "Module: " << name_id_ << " - Pushed output.";
          }
        } else {
          VLOG(1) << "Module: " << name_id_ << "  - Skipped sending an output.";
        }
        auto spin_duration = utils::Timer::toc(tic).count();
        LOG(WARNING) << "Module: " << name_id_
                     << " - frequency: " << 1000.0 / spin_duration << " Hz. ("
                     << spin_duration << " ms).";
        timing_stats.AddSample(spin_duration);
      } else {
        LOG(WARNING) << "Module: " << name_id_ << " - No Input received.";
      }

      // Break the while loop if we are in sequential mode.
      if (!parallel_run_) return true;
    }
    LOG(INFO) << "Module: " << name_id_ << " - Successful shutdown.";
    return true;
  }

  /* ------------------------------------------------------------------------ */
  virtual inline void shutdown() {
    LOG_IF(WARNING, shutdown_)
        << "Module: " << name_id_
        << " - Shutdown requested, but was already shutdown.";
    shutdownQueues();
    LOG(INFO) << "Module: " << name_id_ << " - Shutting down.";
    shutdown_ = true;
  }

  /* ------------------------------------------------------------------------ */
  inline void restart() {
    LOG(INFO) << "Module: " << name_id_
              << " - Resetting shutdown flag to false";
    shutdown_ = false;
  }

  /* ------------------------------------------------------------------------ */
  //! Checks if the module is working or if it still has work to do.
  inline bool isWorking() const { return is_thread_working_ && !hasWork(); }

 protected:
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
  // TODO(Toni): given a list of queues, syncronize them and get an output
  // payload. Maybe keep a list of input queues, that the user can provide.
  virtual inline InputPtr getInputPacket() = 0;

  /**
   * @brief pushOutputPacket Sends the output of the module to other interested
   * parties, potentially other pipeline modules.
   * The typical use case would be to just push to a threadsafe output queue
   * the newly generated output. Alternatively, one may override this function
   * to send the output to multiple registered queues or callbacks.
   * @param[out] output_packet  Parameter to be sent to others
   * @return boolean indicating whether the push was successful or not.
   */
  virtual inline bool pushOutputPacket(OutputPtr output_packet) const = 0;

  /**
   * @brief Abstract function to process a single input payload.
   * @param[in] input: an input payload for module to work on.
   * @return The output payload from the pipeline module. Returning a nullptr
   * signals that the output should not be sent to the output queue.
   */
  virtual OutputPtr spinOnce(const Input& input) = 0;

  /**
   * @brief shutdownQueues If the module stores Threadsafe queues, it must
   * shutdown those for a complete shutdown.
   */
  virtual void shutdownQueues() = 0;

  //! Checks if the module has work to do (should check input queues are empty)
  virtual bool hasWork() const = 0;

 protected:
  //! Properties
  std::string name_id_ = {"PipelineModule"};
  bool parallel_run_ = {true};

 private:
  //! Thread related members.
  std::atomic_bool shutdown_ = {false};
  std::atomic_bool is_thread_working_ = {false};
};

/** @brief MIMOPipelineModule Multiple Input Multiple Output (MIMO) pipeline
 * module.
 * This is still an abstract class and the user must implement the
 * getInputPacket function that deals with the input.
 * Potentially one can receive Input packets via a set of callbacks.
 * Alternatively, one can use a threadsafe queue (in which case you can use
 * the class SIMOPipelineModule, a specialization of a MIMO pipeline module).
 * Sends output to a list of registered callbacks with a specific signature.
 * This is the most general pipeline module accepting and dispatching multiple
 * results.
 */
template <typename Input, typename Output>
class MIMOPipelineModule : public PipelineModule<Input, Output> {
 public:
  KIMERA_POINTER_TYPEDEFS(MIMOPipelineModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MIMOPipelineModule);

  using PIO = PipelineModule<Input, Output>;
  using OutputCallback =
      std::function<void(const typename PIO::OutputPtr& output)>;

  MIMOPipelineModule(const std::string& name_id, const bool& parallel_run)
      : PipelineModule<Input, Output>(name_id, parallel_run),
        output_callbacks_() {}
  virtual ~MIMOPipelineModule() = default;

  /**
   * @brief registerOutputCallback Add an extra output callback to the list
   * of callbacks. This will be called every time there is a new output from
   * this module.
   * @param output_callback actual callback to register.
   */
  virtual void registerCallback(const OutputCallback& output_callback) {
    CHECK(output_callback);
    output_callbacks_.push_back(output_callback);
  }

 protected:
  /**
   * @brief pushOutputPacket Sends the output of the module to other interested
   * parties, potentially other pipeline modules.
   * Just push to a threadsafe output queue the newly generated output.
   * @param[out] output_packet  Parameter to be sent to others
   * @return boolean indicating whether the push was successful or not.
   */
  virtual inline bool pushOutputPacket(
      typename PIO::OutputPtr output_packet) const override {
    //! Call all callbacks
    auto tic_callbacks = utils::Timer::tic();
    for (const OutputCallback& callback : output_callbacks_) {
      CHECK(callback);
      callback(output_packet);
    }
    static constexpr auto kTimeLimitCallbacks = std::chrono::milliseconds(10);
    auto callbacks_duration = utils::Timer::toc(tic_callbacks);
    if (callbacks_duration > kTimeLimitCallbacks) {
      LOG(WARNING) << "Callbacks for module: " << this->name_id_
                   << " are taking very long! Current latency: "
                   << callbacks_duration.count() << " ms.";
      return false;
    } else {
      return true;
    }
  }

 private:
  //! Output callbacks
  std::vector<OutputCallback> output_callbacks_;
};

/** @brief SIMOPipelineModule Single Input Multiple Output (SIMO) pipeline
 * module.
 * Receives Input packets via a threadsafe queue, and sends output packets
 * to a list of registered callbacks with a specific signature.
 * This is useful when there are multiple modules expecting results from this
 * module.
 */
template <typename Input, typename Output>
class SIMOPipelineModule : public MIMOPipelineModule<Input, Output> {
 public:
  KIMERA_POINTER_TYPEDEFS(SIMOPipelineModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(SIMOPipelineModule);

  using PIO = PipelineModule<Input, Output>;
  using InputQueue = ThreadsafeQueue<typename PIO::InputPtr>;

  SIMOPipelineModule(InputQueue* input_queue,
                     const std::string& name_id,
                     const bool& parallel_run)
      : MIMOPipelineModule<Input, Output>(name_id, parallel_run),
        input_queue_(input_queue) {
    CHECK(input_queue_);
  }
  virtual ~SIMOPipelineModule() = default;

 protected:
  /**
   * @brief getSyncedInputPacket Retrieves the input packet for processing.
   * Just pops from a threadsafe queue that contains the input packets to be
   * processed.
   * @param[out] input_packet Parameter to be filled that is then used by the
   * pipeline module's specific spinOnce.
   * @return a boolean indicating whether the generation of the input packet was
   * successful.
   */
  virtual inline typename PIO::InputPtr getInputPacket() override {
    typename PIO::InputPtr input = nullptr;
    bool queue_state = false;
    if (PIO::parallel_run_) {
      queue_state = input_queue_->popBlocking(input);
    } else {
      queue_state = input_queue_->pop(input);
    }

    if (queue_state) {
      return input;
    } else {
      LOG(WARNING) << "Module: " << PIO::name_id_ << " - "
                   << "Input queue: " << input_queue_->queue_id_
                   << " didn't return an output.";
      return nullptr;
    }
  }

  //! Called when general shutdown of PipelineModule is triggered.
  virtual void shutdownQueues() override { input_queue_->shutdown(); };

  //! Checks if the module has work to do (should check input queues are empty)
  virtual bool hasWork() const override { return input_queue_->empty(); };

 private:
  //! Input
  InputQueue* input_queue_;
};

/** @brief MISOPipelineModule Single Input Multiple Output (MISO) pipeline
 * module.
 * Receives Input packets via a threadsafe queue, and sends output packets
 * to a list of registered callbacks with a specific signature.
 * This is useful when there are multiple modules expecting results from this
 * module.
 */
template <typename Input, typename Output>
class MISOPipelineModule : public MIMOPipelineModule<Input, Output> {
 public:
  KIMERA_POINTER_TYPEDEFS(MISOPipelineModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MISOPipelineModule);

  using MIMO = MIMOPipelineModule<Input, Output>;
  using OutputQueue = ThreadsafeQueue<typename MIMO::OutputPtr>;

  MISOPipelineModule(OutputQueue* output_queue,
                     const std::string& name_id,
                     const bool& parallel_run)
      : MIMOPipelineModule<Input, Output>(name_id, parallel_run),
        output_queue_(output_queue) {
    CHECK(output_queue_);
  }
  virtual ~MISOPipelineModule() = default;

  //! Override registering of output callbacks since this is only used for
  //! multiple output pipelines.
  virtual void registerCallback(
      const typename MIMO::OutputCallback& output_callback) override {
    LOG(WARNING) << "SISO Pipeline Module does not use callbacks.";
  }

 protected:
  /**
   * @brief pushOutputPacket Sends the output of the module to other interested
   * parties, potentially other pipeline modules.
   * Just push to a threadsafe output queue the newly generated output.
   * @param[out] output_packet  Parameter to be sent to others
   * @return boolean indicating whether the push was successful or not.
   */
  virtual inline bool pushOutputPacket(
      typename MIMO::OutputPtr output_packet) const override {
    return output_queue_->push(std::move(output_packet));
  }

  //! Called when general shutdown of PipelineModule is triggered.
  virtual void shutdownQueues() override { output_queue_->shutdown(); };

  //! Checks if the module has work to do (should check input queues are empty)
  virtual bool hasWork() const override { return output_queue_->empty(); };

 private:
  //! Output
  OutputQueue* output_queue_;
};

// We explictly avoid using multiple inheritance check for excellent reference:
// https://isocpp.org/wiki/faq/multiple-inheritance
// Since we would end in the "Dreaded Diamond of Death" inheritance
// (anti-)pattern...
/** @brief SISOPipelineModule Single Input Single Output (SISO) pipeline module.
 * Receives Input packets via a threadsafe queue, and sends output packets
 * to a threadsafe output queue.
 * This is the most standard and simplest pipeline module.
 */
template <typename Input, typename Output>
class SISOPipelineModule : public MISOPipelineModule<Input, Output> {
 public:
  KIMERA_POINTER_TYPEDEFS(SISOPipelineModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(SISOPipelineModule);

  using MISO = MISOPipelineModule<Input, Output>;
  using InputQueue = ThreadsafeQueue<typename MISO::InputPtr>;
  using OutputQueue = ThreadsafeQueue<typename MISO::OutputPtr>;

  SISOPipelineModule(InputQueue* input_queue,
                     OutputQueue* output_queue,
                     const std::string& name_id,
                     const bool& parallel_run)
      : MISO(input_queue, name_id, parallel_run) {}
  virtual ~SISOPipelineModule() = default;

  //! Override registering of output callbacks since this is only used for
  //! multiple output pipelines.
  virtual void registerCallback(
      const typename MISO::OutputCallback& output_callback) override {
    LOG(WARNING) << "SISO Pipeline Module does not use callbacks.";
  }

 protected:
  /**
   * @brief getSyncedInputPacket Retrieves the input packet for processing.
   * Just pops from a threadsafe queue that contains the input packets to be
   * processed.
   * @param[out] input_packet Parameter to be filled that is then used by the
   * pipeline module's specific spinOnce.
   * @return a boolean indicating whether the generation of the input packet was
   * successful.
   */
  virtual inline typename MISO::InputPtr getInputPacket() override {
    typename MISO::InputPtr input = nullptr;
    bool queue_state = false;
    if (MISO::parallel_run_) {
      queue_state = input_queue_->popBlocking(input);
    } else {
      queue_state = input_queue_->pop(input);
    }

    if (queue_state) {
      return input;
    } else {
      LOG(WARNING) << "Module: " << MISO::name_id_ << " - "
                   << "Input queue: " << input_queue_->queue_id_
                   << " didn't return an output.";
      return nullptr;
    }
  }

  //! Called when general shutdown of PipelineModule is triggered.
  virtual void shutdownQueues() override {
    input_queue_->shutdown() && MISO::shutdownQueues();
  };

  //! Checks if the module has work to do (should check input queues are empty)
  virtual bool hasWork() const override {
    return input_queue_->empty() && MISO::hasWork();
  };

 private:
  //! Input
  InputQueue* input_queue_;
};

}  // namespace VIO
