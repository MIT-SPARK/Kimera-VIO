/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   QueueSynchronizer.h
 * @brief  Implements temporal synchronization of queues.
 * @author Antoni Rosinol
 */

#pragma once

#include <memory>
#include <numeric>  // for numeric_limits
#include <string>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/pipeline/PipelinePayload.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/Timer.h"

namespace VIO {

/**
 * @brief The QueueSynchronizer class: a singleton class meant to
 * synchronize threadsafe queues (ThreadsafeQueue).
 */
template <class T>
class QueueSynchronizerBase {
 public:
  KIMERA_POINTER_TYPEDEFS(QueueSynchronizerBase);
  /**
   * @brief Utility function to synchronize threadsafe queues.
   *
   * @param[in] timestamp Timestamp of the payload we want to retrieve from the
   * queue
   * @param[in] queue Threadsafe queue templated on a POINTER to a class that
   * is derived from PipelinePayload (otw we cannot query what is its timestamp)
   * @param[out] pipeline_payload Returns payload to be found in the given queue
   * at the given timestamp.
   * @param[in] max_iterations Number of times we try to find the payload at the
   * given timestamp in the given queue.
   * @param[in] callback User defined function to be called on each successful
   * retrieval of a payload in the queue, the callback should be lighting fast!
   * @return a boolean indicating whether the synchronizing was successful (i.e.
   * we found a payload with the requested timestamp) or we failed because a
   * payload with an older timestamp was retrieved.
   */
  virtual bool syncQueue(const Timestamp& timestamp,
                         ThreadsafeQueue<T>* queue,
                         T* pipeline_payload,
                         std::string name_id,
                         int max_iterations = 10,
                         size_t timeout_ms = 10000u,
                         std::function<void(const T&)>* callback = nullptr) = 0;
  virtual ~QueueSynchronizerBase() = default;
};

template <class T>
class SimpleQueueSynchronizer : public QueueSynchronizerBase<T> {
 public:
  KIMERA_POINTER_TYPEDEFS(SimpleQueueSynchronizer);
  KIMERA_DELETE_COPY_CONSTRUCTORS(SimpleQueueSynchronizer);

  /**
   * @brief getInstance of a SimpleQueueSynchronizer
   * @return a unique ptr to a simple queue synchronizer which can be casted
   * to its base class for common interface as a QueueSynchronizer.
   */
  static SimpleQueueSynchronizer<T>& getInstance() {
    static SimpleQueueSynchronizer<T> synchronizer_instance_;
    return synchronizer_instance_;
  }

  /**
   * @brief Utility function to synchronize threadsafe queues.
   * For now we are doing a very simple naive sync approach:
   * Just loop over the messages in the queue until you find the matching
   * timestamp. If we are at a timestamp greater than the queried one
   *
   * @param[in] timestamp Timestamp of the payload we want to retrieve from the
   * queue
   * @param[in] queue Threadsafe queue templated on a POINTER to a class that
   * is derived from PipelinePayload (otw we cannot query what is its timestamp)
   * @param[out] pipeline_payload Returns payload to be found in the given queue
   * at the given timestamp.
   * @param[in] max_iterations Number of times we try to find the payload at the
   * given timestamp in the given queue.
   * @param[in] callback User defined function to be called on each successful
   * retrieval of a payload in the queue, the callback should be lighting fast!
   * @return a boolean indicating whether the synchronizing was successful (i.e.
   * we found a payload with the requested timestamp) or we failed because a
   * payload with an older timestamp was retrieved.
   */
  bool syncQueue(const Timestamp& timestamp,
                 ThreadsafeQueue<T>* queue,
                 T* pipeline_payload,
                 std::string name_id,
                 int max_iterations = 10,
                 size_t timeout_ms = 10000u,
                 std::function<void(const T&)>* callback = nullptr) {
    CHECK_NOTNULL(queue);
    CHECK_NOTNULL(pipeline_payload);
    static_assert(
        std::is_base_of<PipelinePayload,
                        typename std::pointer_traits<T>::element_type>::value,
        "T must be a pointer to a class that derives from PipelinePayload.");
    // Look for the synchronized packet in payload queue
    // Loop over payload timestamps until we reach the query timestamp
    // or we are past the asked timestamp (in which case, we failed).

    Timestamp payload_timestamp = std::numeric_limits<Timestamp>::min();
    
    while (true) {
      std::shared_ptr<T> curr_payload =
          queue->peekBlockingWithTimeout(timeout_ms);
      if (!curr_payload) {
        LOG(ERROR) << "Queue sync failed for module: " << name_id
                   << " with queue: " << queue->queue_id_ << "\n Reason: \n"
                   << "Queue status: "
                   << (queue->isShutdown() ? "Shutdown..." : "Timeout...");
        return false;
      }

      if (!(*curr_payload)) {
        LOG(WARNING)
            << "Payload synchronization failed. Missing payload for Module: "
            << name_id;
        continue;
      }

      // Call any user defined callback at this point (should be fast!!).
      if (callback) {
        (*callback)(*curr_payload);
      }

      payload_timestamp = (*curr_payload)->timestamp_;

      if (payload_timestamp > timestamp) {
        LOG(WARNING)
            << "Syncing queue " << queue->queue_id_ << " in module " << name_id
            << " failed;\n Could not retrieve exact timestamp requested: \n"
            << " - Requested timestamp: " << timestamp << '\n'
            << " - Actual timestamp:    " << payload_timestamp << '\n';
        return false;
      }

      VLOG(5) << "Popping from: " << queue->queue_id_;
      CHECK(queue->pop(*pipeline_payload))
          << "queue somehow lost a measurement";
      if (payload_timestamp == timestamp) {
        return true;  // we found the payload we want
      }
    }
  }

 private:
  // TODO(Toni): don't make this guy a singleton... Rather send it around.
  /// Non-constructible class, so make ctors private (copy ctors are deleted):
  SimpleQueueSynchronizer() = default;
  ~SimpleQueueSynchronizer() = default;
};

}  // namespace VIO
