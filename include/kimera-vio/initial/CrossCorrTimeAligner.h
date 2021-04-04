/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   crossCorrTimeAligner.h
 * @brief  Class to estimate IMU to camera time offset by cross-correlation
 * @author Nathan Hughes
 */

#pragma once
#include <Eigen/Dense>
#include <cstddef>
#include <iterator>
#include <vector>

#include "kimera-vio/initial/TimeAlignerBase.h"

namespace VIO {

class RingBuffer {
 public:
  // iterator based on
  // https://internalpointers.com/post/writing-custom-iterators-modern-cpp
  template <typename Value>
  struct Iterator {
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = Value;
    using pointer = value_type*;
    using reference = value_type&;

    Iterator(std::vector<Value>& buffer, pointer ptr)
        : m_ptr_(ptr),
          begin_ptr_(&(buffer.data()[0])),
          end_ptr_(&(buffer.data()[buffer.size() - 1])) {}

    reference operator*() const {
      CHECK_GE(m_ptr_, begin_ptr_);
      CHECK_LE(m_ptr_, end_ptr_);
      return *m_ptr_;
    }

    pointer operator->() { return m_ptr_; }

    Iterator& operator++() {
      m_ptr_++;
      if (m_ptr_ > end_ptr_) {
        m_ptr_ = begin_ptr_;
      }
      return *this;
    }

    Iterator operator++(int) {
      Iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    friend bool operator==(const Iterator& a, const Iterator& b) {
      return a.m_ptr_ == b.m_ptr_;
    }

    friend bool operator!=(const Iterator& a, const Iterator& b) {
      return !(a == b);
    }

    friend std::ostream& operator<<(std::ostream& out,
                                    const Iterator<Value>& iter) {
      out << "iter<(curr=" << iter.m_ptr_ << ", begin=" << iter.begin_ptr_
          << ", end=" << iter.end_ptr_ << ")>";
      return out;
    }

   private:
    pointer m_ptr_;
    value_type* begin_ptr_;
    value_type* end_ptr_;
  };

  explicit RingBuffer(size_t window_size)
      : times(this),
        values(this),
        window_size_(window_size),
        buffer_size_(window_size + 1),
        num_measurements_(0),
        curr_index_(0),
        timestamps_(window_size + 1, 0),
        measurements_(window_size + 1, 0.0) {}

  inline bool full() const { return window_size_ == num_measurements_; }

  inline void push(Timestamp timestamp, double measurement) {
    timestamps_[curr_index_] = timestamp;
    measurements_[curr_index_] = measurement;
    if (curr_index_ == window_size_) {
      curr_index_ = 0;
    } else {
      curr_index_++;
    }
    if (num_measurements_ < window_size_) {
      num_measurements_++;
    }
  }

  struct Timestamps {
    Timestamps(RingBuffer* buffer) : buffer_(buffer) {}

    inline Iterator<Timestamp> begin() const {
      return Iterator<Timestamp>(
          buffer_->timestamps_,
          &(buffer_->timestamps_.data()[buffer_->get_begin_index_()]));
    }

    inline Iterator<Timestamp> end() const {
      return Iterator<Timestamp>(
          buffer_->timestamps_,
          &(buffer_->timestamps_.data()[buffer_->get_end_index_()]));
    }

   private:
    RingBuffer* buffer_;
  } times;

  struct Values {
    Values(RingBuffer* buffer) : buffer_(buffer) {}

    inline Iterator<double> begin() const {
      return Iterator<double>(
          buffer_->measurements_,
          &(buffer_->measurements_.data()[buffer_->get_begin_index_()]));
    }

    inline Iterator<double> end() const {
      return Iterator<double>(
          buffer_->measurements_,
          &(buffer_->measurements_.data()[buffer_->get_end_index_()]));
    }

   private:
    RingBuffer* buffer_;
  } values;

  friend class Timestamps;

  friend std::ostream& operator<<(std::ostream& out, const RingBuffer& buffer) {
    out << "buffer<(curr=" << buffer.curr_index_
        << ", size=" << buffer.num_measurements_
        << ", max_size=" << buffer.window_size_ << ")>";
    return out;
  }

 private:
  inline size_t get_begin_index_() const {
    if (num_measurements_ < window_size_) {
      return 0;
    }

    return (curr_index_ == window_size_) ? 0 : curr_index_ + 1;
  }

  inline size_t get_end_index_() const {
    if (num_measurements_ == 0) {
      return 0;
    }
    return curr_index_;
  }

  size_t window_size_;
  size_t buffer_size_;
  size_t num_measurements_;
  size_t curr_index_;
  std::vector<Timestamp> timestamps_;
  std::vector<double> measurements_;
};

class CrossCorrTimeAligner : public TimeAlignerBase {
 public:
  // TODO(nathan) add other parameters here
  explicit CrossCorrTimeAligner(bool do_imu_rate_estimation,
                                size_t window_size = 100);

  void addNewImuData(const ImuStampS& imu_stamps_,
                     const ImuAccGyrS& imu_acc_gyrs) override;

 protected:
  TimeAlignerBase::Result attemptEstimation(
      const FrontendOutputPacketBase& input) override;

 private:
  bool do_imu_rate_estimation_;
  RingBuffer imu_buffer_;
  RingBuffer vision_buffer_;
};

}  // namespace VIO
