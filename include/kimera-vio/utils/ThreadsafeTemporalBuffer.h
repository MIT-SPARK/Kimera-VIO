/*******************************************************************************
 Copyright 2017 Autonomous Systems Lab, ETH Zurich, Switzerland

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
********************************************************************************/

/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ThreadsafeTemporalBuffer.h
 * @brief  Thread Safe Buffer with timestamp lookup.
 * @author Antoni Rosinol
 */

#pragma once

#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <utility>

#include "kimera-vio/common/vio_types.h"

namespace VIO {

namespace utils {

template <typename ValueType,
          typename AllocatorType =
              std::allocator<std::pair<const Timestamp, ValueType> > >
class ThreadsafeTemporalBuffer {
 public:
  typedef std::map<Timestamp, ValueType, std::less<Timestamp>, AllocatorType>
      BufferType;

  // Create buffer of infinite length (buffer_length_nanoseconds = -1)
  ThreadsafeTemporalBuffer();

  // Buffer length in nanoseconds defines after which time old entries get
  // dropped. (buffer_length_nanoseconds == -1: infinite length.)
  explicit ThreadsafeTemporalBuffer(Timestamp buffer_length_nanoseconds);

  ThreadsafeTemporalBuffer(const ThreadsafeTemporalBuffer& other);

  void addValue(Timestamp timestamp, const ValueType& value);
  void addValue(const Timestamp timestamp, const ValueType& value,
                const bool emit_warning_on_value_overwrite);
  void insert(const ThreadsafeTemporalBuffer& other);

  inline size_t size() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return values_.size();
  }
  inline bool empty() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return values_.empty();
  }
  void clear() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    values_.clear();
  }

  // Returns false if no value at a given timestamp present.
  bool getValueAtTime(Timestamp timestamp_ns, ValueType* value) const;

  bool deleteValueAtTime(Timestamp timestamp_ns);

  bool getNearestValueToTime(Timestamp timestamp_ns, ValueType* value) const;
  bool getNearestValueToTime(Timestamp timestamp_ns, Timestamp maximum_delta_ns,
                             ValueType* value) const;
  bool getNearestValueToTime(Timestamp timestamp, Timestamp maximum_delta_ns,
                             ValueType* value,
                             Timestamp* timestamp_at_value_ns) const;

  bool getOldestValue(ValueType* value) const;
  bool getNewestValue(ValueType* value) const;

  // These functions return False if there is no valid time.
  bool getValueAtOrBeforeTime(Timestamp timestamp_ns,
                              Timestamp* timestamp_ns_of_value,
                              ValueType* value) const;
  bool getValueAtOrAfterTime(Timestamp timestamp_ns,
                             Timestamp* timestamp_ns_of_value,
                             ValueType* value) const;

  // Get all values between the two specified timestamps excluding the border
  // values.
  // Example: content: 2 3 4 5
  //          getValuesBetweenTimes(2, 5, ...) returns elements at 3, 4.
  // Alternatively, you can ask for the lower bound, such that:
  // Example: content: 2 3 4 5
  //          getValuesBetweenTimes(2, 5, ...) returns elements at 2, 3, 4.
  // by setting the parameter get_lower_bound to true.
  template <typename ValueContainerType>
  bool getValuesBetweenTimes(Timestamp timestamp_lower_ns,
                             Timestamp timestamp_higher_ns,
                             ValueContainerType* values,
                             const bool get_lower_bound = false) const;

  inline void lockContainer() const { mutex_.lock(); }
  inline void unlockContainer() const { mutex_.unlock(); }

  // The container is exposed so we can iterate over the values in a linear
  // fashion. The container is not locked inside this method so call
  // lockContainer()/unlockContainer() when accessing this.
  const BufferType& buffered_values() const { return values_; }

  inline bool operator==(const ThreadsafeTemporalBuffer& other) const {
    return values_ == other.values_ &&
           buffer_length_nanoseconds_ == other.buffer_length_nanoseconds_;
  }

 protected:
  // Remove items that are older than the buffer length.
  void removeOutdatedItems();

  bool getIteratorAtTimeOrEarlier(
      Timestamp timestamp,
      typename BufferType::const_iterator* it_lower_bound) const;

  BufferType values_;
  Timestamp buffer_length_nanoseconds_;
  mutable std::recursive_mutex mutex_;
};

}  // namespace utils

}  // namespace VIO

#include "./ThreadsafeTemporalBuffer-inl.h"
