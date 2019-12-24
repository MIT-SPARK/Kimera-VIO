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

namespace VIO {

/**
 * @brief The QueueSynchronizer class: a singleton class meant to
 * synchronize threadsafe queues (ThreadsafeQueue).
 */
template <class T>
class QueueSynchronizerBase {
 public:
  KIMERA_POINTER_TYPEDEFS(QueueSynchronizerBase);
  virtual bool syncQueue(const Timestamp& timestamp,
                         ThreadsafeQueue<T>* queue,
                         T* pipeline_payload,
                         std::string name_id,
                         int max_iterations = 10) = 0;
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
   * @return a boolean indicating whether the synchronizing was successful (i.e.
   * we found a payload with the requested timestamp) or we failed because a
   * payload with an older timestamp was retrieved.
   */
  bool syncQueue(const Timestamp& timestamp,
                 ThreadsafeQueue<T>* queue,
                 T* pipeline_payload,
                 std::string name_id,
                 int max_iterations = 10) {
    CHECK_NOTNULL(queue);
    CHECK_NOTNULL(pipeline_payload);
    static_assert(
        std::is_base_of<PipelinePayload,
                        typename std::pointer_traits<T>::element_type>::value,
        "T must be a pointer to a class that derives from PipelinePayload.");
    // Look for the synchronized packet in payload queue
    Timestamp payload_timestamp = std::numeric_limits<Timestamp>::min();
    // Loop over payload timestamps until we reach the query timestamp
    // or we are past the asked timestamp (in which case, we failed).
    int i = 0;
    for (; i < max_iterations && timestamp > payload_timestamp; ++i) {
      // TODO(Toni): add a timer to avoid waiting forever...
      if (!queue->popBlocking(*pipeline_payload)) {
        LOG(ERROR) << name_id << "'s " << queue->queue_id_ << " is empty or "
                   << "has been shutdown.";
        return false;
      } else {
        VLOG(5) << "Popping from: " << queue->queue_id_;
      }
      if (*pipeline_payload) {
        payload_timestamp = (*pipeline_payload)->timestamp_;
      } else {
        LOG(WARNING) << "Missing frontend payload for Module: " << name_id;
      }
    }
    CHECK_EQ(timestamp, payload_timestamp)
        << "Syncing queue " << queue->queue_id_ << " in module " << name_id
        << " failed;\n Could not retrieve exact timestamp requested: \n"
        << " - Requested timestamp: " << timestamp << '\n'
        << " - Actual timestamp:    " << payload_timestamp << '\n'
        << (i >= max_iterations ? "Reached max number of sync attempts: " +
                                      std::to_string(max_iterations)
                                : "");
    CHECK(*pipeline_payload);
    return true;
  }

 private:
  // TODO(Toni): don't make this guy a singleton... Rather send it around.
  /// Non-constructible class, so make ctors private (copy ctors are deleted):
  SimpleQueueSynchronizer() = default;
  ~SimpleQueueSynchronizer() = default;
};

}  // namespace VIO
