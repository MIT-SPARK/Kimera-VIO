/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ThreadsafeQueue.h
 * @brief  Thread Safe Queue with shutdown/resume functionality.
 * @author Antoni Rosinol
 */

#pragma once

#include <glog/logging.h>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>

// TODO make specialized queue for unique ptrs. (non-copyable objects).
template <typename T>
class ThreadsafeQueue {
 public:
  typedef std::queue<std::shared_ptr<T>> InternalQueue;
  ThreadsafeQueue(const std::string& queue_id) : queue_id_(queue_id) {}
  ThreadsafeQueue(const ThreadsafeQueue& other) {
    std::unique_lock<std::mutex> lk(other.mutex_);
    data_queue_ = other.data_queue_;
    lk.unlock();
    // No need to lock here, atomic bool.
    shutdown_ = other.shutdown_;
  }

  // Push an lvalue to the queue.
  // Returns false if the queue has been shutdown.
  bool push(const T& new_value) {
    if (shutdown_) return false;  // atomic, no lock needed.
    // This does a copy, so you better use pointers as T.
    std::shared_ptr<T> data(std::make_shared<T>(new_value));
    std::unique_lock<std::mutex> lk(mutex_);
    size_t queue_size = data_queue_.size();
    VLOG_IF(1, queue_size != 0) << "Queue with id: " << queue_id_
                                << " is getting full, size: " << queue_size;
    data_queue_.push(data);
    lk.unlock();  // Unlock before notify.
    data_cond_.notify_one();
    return true;
  }

  // Push rvalue to the queue using move semantics.
  // Returns false if the queue has been shutdown.
  // Since there is no type deduction, T&& is an rvalue, not a universal
  // reference.
  bool push(T&& new_value) {
    if (shutdown_) return false;  // atomic, no lock needed.
    std::shared_ptr<T> data(std::make_shared<T>(std::move(new_value)));
    std::unique_lock<std::mutex> lk(mutex_);
    size_t queue_size = data_queue_.size();
    VLOG_IF(1, queue_size != 0) << "Queue with id: " << queue_id_
                                << " is getting full, size: " << queue_size;
    data_queue_.push(data);
    lk.unlock();  // Unlock before notify.
    data_cond_.notify_one();
    return true;
  }

  // Pop value. Waits for data to be available in the queue.
  // Returns false if the queue has been shutdown.
  bool popBlocking(T& value) {
    std::unique_lock<std::mutex> lk(mutex_);
    // Wait until there is data in the queue or shutdown requested.
    data_cond_.wait(lk, [this] { return !data_queue_.empty() || shutdown_; });
    // Return false in case shutdown is requested.
    if (shutdown_) return false;
    value = std::move(*data_queue_.front());
    data_queue_.pop();
    return true;
  }

  // Pop value. Waits for data to be available in the queue.
  // If the queue has been shutdown, it returns a null shared_ptr.
  std::shared_ptr<T> popBlocking() {
    std::unique_lock<std::mutex> lk(mutex_);
    data_cond_.wait(lk, [this] { return !data_queue_.empty() || shutdown_; });
    if (shutdown_) return std::shared_ptr<T>(nullptr);
    std::shared_ptr<T> result = data_queue_.front();
    data_queue_.pop();
    return result;
  }

  // Pop without blocking, just checks once if the queue is empty.
  // Returns true if the value could be retrieved, false otherwise.
  bool pop(T& value) {
    if (shutdown_) return false;
    std::lock_guard<std::mutex> lk(mutex_);
    if (data_queue_.empty()) return false;
    value = std::move(*data_queue_.front());
    data_queue_.pop();
    return true;
  }

  // Pop without blocking, just checks once if the queue is empty.
  // Returns a shared_ptr to the value retrieved.
  // If the queue is empty or has been shutdown,
  // it returns a null shared_ptr.
  std::shared_ptr<T> pop() {
    if (shutdown_) return std::shared_ptr<T>(nullptr);
    std::lock_guard<std::mutex> lk(mutex_);
    if (data_queue_.empty()) return std::shared_ptr<T>(nullptr);
    std::shared_ptr<T> result = data_queue_.front();
    data_queue_.pop();
    return result;
  }

  // Swap queue with empty queue if not empty.
  // Returns true if values were retrieved.
  // Returns false if values were not retrieved.
  bool batchPop(InternalQueue* output_queue) {
    if (shutdown_)
      return false;
    CHECK_NOTNULL(output_queue);
    CHECK(output_queue->empty());
    //*output_queue = InternalQueue();
    std::lock_guard<std::mutex> lk(mutex_);
    if (data_queue_.empty()) {
      return false;
    } else {
      data_queue_.swap(*output_queue);
      return true;
    }
  }

  void shutdown() {
    std::unique_lock<std::mutex> mlock(mutex_);
    // Even if the shared variable is atomic, it must be modified under the
    // mutex in order to correctly publish the modification to the waiting
    // threads.
    shutdown_ = true;
    mlock.unlock();
    data_cond_.notify_all();
  }

  void resume() {
    std::unique_lock<std::mutex> mlock(mutex_);
    // Even if the shared variable is atomic, it must be modified under the
    // mutex in order to correctly publish the modification to the waiting
    // threads.
    shutdown_ = false;
    mlock.unlock();
    data_cond_.notify_all();
  }

  // Checks if the queue is empty.
  // !! the state of the queue might change right after this query.
  bool empty() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return data_queue_.empty();
  }

 private:
  mutable std::mutex mutex_;  // mutable for empty() and copy-constructor.
  std::string queue_id_;
  InternalQueue data_queue_;
  std::condition_variable data_cond_;
  std::atomic_bool shutdown_ = {false};  // flag for signaling queue shutdown.
};
