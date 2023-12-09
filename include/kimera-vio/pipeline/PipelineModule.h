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
#include <utility>  // for move
#include <vector>

#include <glog/logging.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/pipeline/PipelinePayload.h"
#include "kimera-vio/pipeline/QueueSynchronizer.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/Timer.h"

namespace VIO {

/**
 * @brief Abstraction of a pipeline module. Contains non-templated members.
 *
 * Provides common algorithmic logic for a pipeline module base class.
 */
class PipelineModuleBase {
 public:
  KIMERA_POINTER_TYPEDEFS(PipelineModuleBase);
  KIMERA_DELETE_COPY_CONSTRUCTORS(PipelineModuleBase);

  //! Callback used to signal if the pipeline module failed.
  //! TODO(Toni): return an error code perhaps.
  using OnFailureCallback = std::function<void()>;

 public:
  /**
   * @brief PipelineModuleBase
   * @param
   */
  PipelineModuleBase(const std::string& name_id, const bool& parallel_run)
      : name_id_(name_id), parallel_run_(parallel_run) {}

  virtual ~PipelineModuleBase() = default;

  /**
   * @brief Main spin function to be called by a thread and will return only
   * when done (this is typically a while(!shutdown()) loop, see PipelineModule
   * derived class.
   * @return True if everything goes well.
   */
  virtual bool spin() = 0;

  virtual inline void shutdown() {
    LOG_IF(WARNING, shutdown_)
        << "Module: " << name_id_
        << " - Shutdown requested, but was already shutdown.";
    VLOG(1) << "Stopping module " << name_id_ << " and its queues...";
    shutdownQueues();
    VLOG(1) << "Module: " << name_id_ << " - Shutting down.";
    shutdown_ = true;
  }

  inline void restart() {
    VLOG(1) << "Module: " << name_id_ << " - Resetting shutdown flag to false";
    shutdown_ = false;
  }

  inline bool isWorking() const { return is_thread_working_ || hasWork(); }

  /**
   * @brief registerOnFailureCallback Add an extra on-failure callback to the
   * list of callbacks. This will be called every time the module does not
   * return an output, potentially because of a failure.
   * @param on_failure_callback actual callback to register.
   */
  virtual void registerOnFailureCallback(const OnFailureCallback& callback) {
    CHECK(callback);
    on_failure_callbacks_.push_back(callback);
  }

 protected:
  // TODO(Toni) Pass the specific queue synchronizer at the ctor level
  // (kind of like visitor pattern), and use the queue synchronizer base class.
  /**
   * @brief Non-static wrapper around the static queue synchronizer.
   * this->name_id_ is used for the name_id parameter.
   */
  template <class T>
  bool syncQueue(const Timestamp& timestamp,
                 ThreadsafeQueue<T>* queue,
                 T* pipeline_payload,
                 int max_iterations = 10,
                 size_t timeout_ms = 10000u) {
    return SimpleQueueSynchronizer<T>::getInstance().syncQueue(
        timestamp, queue, pipeline_payload, name_id_, max_iterations, timeout_ms);
  }
  /**
   * @brief shutdownQueues If the module stores Threadsafe queues, it must
   * shutdown those for a complete shutdown.
   */
  virtual void shutdownQueues() = 0;

  //! Checks if the module has work to do (should check input queues are empty)
  virtual bool hasWork() const = 0;

  virtual void notifyOnFailure() {
    for (const auto& on_failure_callback : on_failure_callbacks_) {
      if (on_failure_callback) {
        on_failure_callback();
      } else {
        LOG(ERROR) << "Invalid OnFailureCallback for module: " << name_id_;
      }
    }
  }

 protected:
  //! Properties
  std::string name_id_ = {"PipelineModule"};
  bool parallel_run_ = {true};

  //! Callbacks to be called in case module does not return an output.
  std::vector<OnFailureCallback> on_failure_callbacks_;

  //! Thread related members.
  std::atomic_bool shutdown_ = {false};
  std::atomic_bool is_thread_working_ = {false};
};

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
class PipelineModule : public PipelineModuleBase {
 public:
  KIMERA_POINTER_TYPEDEFS(PipelineModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(PipelineModule);
  //! The input is a unique ptr, as the user should implement getInputPacket
  //! such that it only retrieves an input structure with all data.
  using InputUniquePtr = std::unique_ptr<Input>;
  //! The output is instead a shared ptr, since many users might need the output
  using OutputUniquePtr = std::unique_ptr<Output>;
  using OutputSharedPtr = std::shared_ptr<Output>;

  /**
   * @brief PipelineModule
   * @param name_id Identifier for the pipeline module
   * @param parallel_run Spin in parallel mode or sequentially (the spin
   * does only one call to spinOnce and returns).
   */
  PipelineModule(const std::string& name_id, const bool& parallel_run)
      : PipelineModuleBase(name_id, parallel_run) {}

  virtual ~PipelineModule() { VLOG(1) << name_id_ + " destructor called."; }

  /**
   * @brief Main spin function. Every pipeline module calls this spin, where
   * the input is taken from an input queue and processed into an output packet
   * which is sent to the output queue. If the module returns a nullptr, then
   * we don't push to the output queue to save computation time.
   * @return False when shutdown requested, true while working nominally.
   * Note that it only returns true if working in sequential mode,
   * otherwise it will simply not return unless it is shutdown, in which case,
   * it returns false.
   */
  bool spin() override {
    VLOG_IF(1, parallel_run_) << "Module: " << name_id_ << " - Spinning.";
    utils::StatsCollector timing_stats(name_id_ + " [ms]");
    while (!shutdown_) {
      // Get input data from queue by waiting for payload.
      is_thread_working_ = false;
      InputUniquePtr input = getInputPacket();
      is_thread_working_ = true;
      if (input) {
        auto tic = utils::Timer::tic();
        // Transfer the ownership of input to the actual pipeline module.
        // From this point on, you cannot use input, since spinOnce owns it.
        OutputUniquePtr output = spinOnce(std::move(input));
        if (output) {
          // Received a valid output, send to output queue
          if (!pushOutputPacket(std::move(output))) {
            LOG(WARNING) << "Module: " << name_id_ << " - Output push failed.";
          } else {
            VLOG(2) << "Module: " << name_id_ << " - Pushed output.";
          }
        } else {
          VLOG(1) << "Module: " << name_id_ << "  - Skipped sending an output.";
          // Notify interested parties about failure.
          notifyOnFailure();
        }
        auto spin_duration = utils::Timer::toc(tic).count();
        timing_stats.AddSample(spin_duration);
      } else {
        // TODO(nathan) switch to VLOG_IS_ON(1) when we fix how spinning works
        LOG_IF_EVERY_N(WARNING, VLOG_IS_ON(10), 50)
            << "Module: " << name_id_ << " - No Input received.";
      }

      // Break the while loop if we are in sequential mode.
      if (!parallel_run_) {
        is_thread_working_ = false;
        return true;
      }
    }
    is_thread_working_ = false;
    VLOG(1) << "Module: " << name_id_ << " - Successful shutdown.";
    return false;
  }

 protected:
  /**
   * @brief getSyncedInputPacket Retrieves the input packet for processing when
   * spinning. The user must implement this to feed input payloads to the
   * spinOnce.
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
  virtual InputUniquePtr getInputPacket() = 0;

  /**
   * @brief pushOutputPacket Sends the output of the module to other interested
   * parties, potentially other pipeline modules.
   * The typical use case would be to just push to a threadsafe output queue
   * the newly generated output. Alternatively, one may override this function
   * to send the output to multiple registered queues or callbacks.
   * @param[out] output_packet  Parameter to be sent to others
   * @return boolean indicating whether the push was successful or not.
   */
  virtual bool pushOutputPacket(OutputUniquePtr output_packet) const = 0;

  /**
   * @brief Abstract function to process a single input payload.
   * The user must implement this function at a minimum, which
   * is the one doing the actual work of the pipeline module.
   * @param[in] input: an input payload unique pointer, for the module to work
   * on. Mind that we pass a unique_ptr so that one can do perfect forwarding,
   * i.e. if the pipeline module does nothing but forwarding the input to the
   * output (for example the data provider module does this, since it just syncs
   * input and forwards it to the output).
   * @return The output payload from the pipeline module. Returning a nullptr
   * signals that the output should not be sent to the output queue.
   */
  virtual OutputUniquePtr spinOnce(InputUniquePtr input) = 0;
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
  //! Callback used to send data to other pipeline modules, makes use of
  //! shared pointer since the data may be shared between several modules.
  using OutputCallback =
      std::function<void(const typename PIO::OutputSharedPtr& output)>;

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
  virtual void registerOutputCallback(const OutputCallback& output_callback) {
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
  bool pushOutputPacket(
      typename PIO::OutputUniquePtr output_packet) const override {
    auto tic_callbacks = utils::Timer::tic();
    //! We need to make our packet shared in order to send it to multiple
    //! other modules.
    typename PIO::OutputSharedPtr shared_output_packet =
        std::move(output_packet);
    //! Call all callbacks
    for (const OutputCallback& callback : output_callbacks_) {
      CHECK(callback);
      callback(shared_output_packet);
    }
    static constexpr auto kTimeLimitCallbacks = std::chrono::milliseconds(10);
    auto callbacks_duration = utils::Timer::toc(tic_callbacks);
    LOG_IF(WARNING, callbacks_duration > kTimeLimitCallbacks)
        << "Callbacks for module: " << this->name_id_
        << " are taking very long! Current latency: "
        << callbacks_duration.count() << " ms.";
    return true;
  }

 private:
  //! Output callbacks that will be called on each spinOnce if
  //! an output is present.
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
  using InputQueue = ThreadsafeQueue<typename PIO::InputUniquePtr>;

  SIMOPipelineModule(InputQueue* input_queue,
                     const std::string& name_id,
                     const bool& parallel_run)
      : MIMOPipelineModule<Input, Output>(name_id, parallel_run),
        input_queue_(input_queue) {
    CHECK(input_queue_ != nullptr)
        << "Input queue must be non NULL for a SIMO pipeline module.\n"
        << "SIMO Pipeline Module: " << PIO::name_id_;
  }
  virtual ~SIMOPipelineModule() = default;

 protected:
  /**
   * @brief getSyncedInputPacket Retrieves the input packet for processing.
   * Just pops from the input threadsafe queue that contains the input packets
   * to be processed. Since this is a single input pipeline module (SISO),
   * there is no need to sync queues.
   * @return a pointer with the generated input packet. If the generation was
   * unsuccessful, returns a nullptr.
   */
  typename PIO::InputUniquePtr getInputPacket() override {
    typename PIO::InputUniquePtr input = nullptr;
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
  void shutdownQueues() override { input_queue_->shutdown(); }

  //! Checks if the module has work to do (should check input queues are empty)
  bool hasWork() const override {
    return !input_queue_->isShutdown() && !input_queue_->empty();
  }

 private:
  //! Input
  InputQueue* input_queue_;
};

/** @brief MISOPipelineModule Multi Input Single Output (MISO) pipeline
 * module.
 * This is still an abstract class and the user must implement the
 * getInputPacket function that deals with the input.
 * Potentially one can receive Input packets via a set of callbacks.
 *
 * The implementation side of MISO wrt MIMO is that the output is sent to
 * a threadsafe queue, instead of a list of registered callbacks. This makes
 * a clear contract for the behavior of this pipeline module.
 *
 * Note: OutputQueue is now optional, allowing to create MINO modules (aka
 * Multi Input No Output). This is useful for modules like Display Module which
 * receives data from other modules but just displays the data instead of
 * transmitting information to other modules.
 *
 */
template <typename Input, typename Output>
class MISOPipelineModule : public MIMOPipelineModule<Input, Output> {
 public:
  KIMERA_POINTER_TYPEDEFS(MISOPipelineModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MISOPipelineModule);

  using MIMO = MIMOPipelineModule<Input, Output>;
  //! The output queue of a MISO pipeline is a unique pointer instead of a
  //! shared pointer!
  using OutputQueue = ThreadsafeQueue<typename MIMO::OutputUniquePtr>;

  MISOPipelineModule(OutputQueue* output_queue,
                     const std::string& name_id,
                     const bool& parallel_run)
      : MIMOPipelineModule<Input, Output>(name_id, parallel_run),
        output_queue_(output_queue) {
    LOG_IF(INFO, !output_queue_) << "MISO Pipeline Module: " << name_id
                                 << " has no output queue registered.";
  }
  virtual ~MISOPipelineModule() = default;

  //! Override registering of output callbacks since this is only used for
  //! multiple output pipelines.
  void registerOutputCallback(const typename MIMO::OutputCallback&) override {
    LOG(WARNING) << "MISO Pipeline Module does not use callbacks.";
  }

 protected:
  /**
   * @brief pushOutputPacket Sends the output of the module to other interested
   * parties, potentially other pipeline modules.
   * Just push to a threadsafe output queue the newly generated output.
   * @param[out] output_packet  Parameter to be sent to others
   * @return boolean indicating whether the push was successful or not.
   */
  inline bool pushOutputPacket(
      typename MIMO::OutputUniquePtr output_packet) const override {
    return output_queue_ ? output_queue_->push(std::move(output_packet)) : true;
  }

  //! Called when general shutdown of PipelineModule is triggered.
  void shutdownQueues() override {
    if (output_queue_) output_queue_->shutdown();
  }

 private:
  //! Output
  OutputQueue* output_queue_;
};

// We explictly avoid using multiple inheritance (SISO is a MISO and a SIMO)
// check for excellent reference:
// https://isocpp.org/wiki/faq/multiple-inheritance
// Since we would end in the "Dreaded Diamond of Death" inheritance
// (anti-)pattern...
/** @brief SISOPipelineModule Single Input Single Output (SISO) pipeline module.
 * Receives Input packets via a threadsafe queue, and sends output packets
 * to a threadsafe output queue.
 * This is the most standard and simplest pipeline module.
 *
 * Note: the output queue might be optional nullptr as it is optional (this is
 * to allow SINO modules: aka single input no output modules like the Display
 * Module that only consumes data to display it but does not return data.)
 */
template <typename Input, typename Output>
class SISOPipelineModule : public MISOPipelineModule<Input, Output> {
 public:
  KIMERA_POINTER_TYPEDEFS(SISOPipelineModule);
  KIMERA_DELETE_COPY_CONSTRUCTORS(SISOPipelineModule);

  using PIO = PipelineModule<Input, Output>;
  using MISO = MISOPipelineModule<Input, Output>;
  using InputQueue = ThreadsafeQueue<typename PIO::InputUniquePtr>;
  using OutputQueue = typename MISO::OutputQueue;

  SISOPipelineModule(InputQueue* input_queue,
                     OutputQueue* output_queue,
                     const std::string& name_id,
                     const bool& parallel_run)
      : MISOPipelineModule<Input, Output>(output_queue, name_id, parallel_run),
        input_queue_(input_queue) {
    CHECK_NOTNULL(input_queue_);
  }
  virtual ~SISOPipelineModule() = default;

  //! Override registering of output callbacks since this is only used for
  //! multiple output pipelines.
  void registerOutputCallback(const typename MISO::OutputCallback&) override {
    LOG(WARNING) << "SISO Pipeline Module does not use callbacks.";
  }

 protected:
  /**
   * @brief getSyncedInputPacket Retrieves the input packet for processing.
   * Just pops from the input threadsafe queue that contains the input packets
   * to be processed. Since this is a single input pipeline module (SISO),
   * there is no need to sync queues.
   * @return a pointer with the generated input packet. If the generation was
   * unsuccessful, returns a nullptr.
   */
  typename MISO::InputUniquePtr getInputPacket() override {
    typename MISO::InputUniquePtr input = nullptr;
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
  void shutdownQueues() override {
    input_queue_->shutdown();
    MISO::shutdownQueues();
  }

  //! Checks if the module has work to do (should check input queues are empty)
  bool hasWork() const override {
    return !input_queue_->isShutdown() && !input_queue_->empty();
  }

 protected:
  //! Input
  InputQueue* input_queue_;
};

}  // namespace VIO
