#pragma once

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <glog/logging.h>

template <typename T>
class ThreadSafeQueue {
    // Delete assignment operator
    // Specify copy operator
  public:
    T pop() {
      std::unique_lock<std::mutex> mlock(mutex_);
      while (!requested_stop_) {
        LOG(INFO) << "Popping, id: " << std::this_thread::get_id();
        if (queue_.empty()) {
          cond_.wait(mlock);
        }
        if (queue_.empty()) {
          // Sporadic wakeup.
          continue;
        }
        auto item = queue_.front(); // This is not exception safe...
        queue_.pop();
        return item;
      }
      LOG(INFO) << "Shutdown";
    }

    std::shared_ptr<T> popShared() {
      std::unique_lock<std::mutex> mlock(mutex_);
      while (!requested_stop_) {
        LOG(INFO) << "Popping, id: " << std::this_thread::get_id();
        if (queue_.empty()) {
          cond_.wait(mlock);
        }
        if (queue_.empty()) {
          // Sporadic wakeup.
          continue;
        }
        std::shared_ptr<T> item (std::make_shared<T>(queue_.front()));
        queue_.pop();
        return item;
      }
      LOG(INFO) << "Shutdown";
      return std::shared_ptr<T>();
    }

    void pop(T& item) {
      std::unique_lock<std::mutex> mlock(mutex_);
      while (!requested_stop_) {
        LOG(INFO) << "Popping ref, id: " << std::this_thread::get_id();
        if (queue_.empty()) {
          cond_.wait(mlock);
        }
        if (queue_.empty()) {
          // Sporadic wakeup.
          continue;
        }
        item = queue_.front(); // This is exception safe, but inconvenient.
        queue_.pop();
        return;
      }
    }

    // Provide a pop with a shared pointer.
    // std::shared_ptr<T> pop();

    void push(const T& item) {
      std::unique_lock<std::mutex> mlock(mutex_);
      LOG(INFO) << "Pushing, id: " << std::this_thread::get_id();
      queue_.push(item);
      cond_.notify_one();
    }

    void push(T&& item) {
      std::unique_lock<std::mutex> mlock(mutex_);
      LOG(INFO) << "Pushing, id: " << std::this_thread::get_id();
      queue_.push(std::move(item));
      cond_.notify_one();
    }

    void shutdown() {
      std::unique_lock<std::mutex> mlock(mutex_);
      LOG(INFO) << "Shutting down queue." << std::this_thread::get_id();
      requested_stop_ = true;
      cond_.notify_all();
    }

  private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    std::atomic_bool requested_stop_ = {false};
};
