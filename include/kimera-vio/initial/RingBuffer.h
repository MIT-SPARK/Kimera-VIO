#pragma once
#include <glog/logging.h>
#include <cstddef>
#include <iterator>
#include <vector>

namespace VIO {

// based on
// https://internalpointers.com/post/writing-custom-iterators-modern-cpp
template <typename Value>
struct RingBufferIter {
  // using iterator_category = std::random_access_iterator_tag;
  using iterator_category = std::forward_iterator_tag;
  using difference_type = std::ptrdiff_t;
  using value_type = Value;
  using pointer = value_type*;
  using reference = value_type&;

  RingBufferIter(std::vector<Value>& buffer,
                 size_t num_measurements,
                 size_t curr_index,
                 pointer ptr)
      : m_ptr_(ptr),
        curr_index_(curr_index),
        num_values_(num_measurements),
        begin_ptr_(&(buffer.data()[0])),
        end_ptr_(&(buffer.data()[buffer.size() - 1])) {}

  reference operator*() const {
    CHECK_GE(m_ptr_, begin_ptr_);
    CHECK_LE(m_ptr_, end_ptr_);
    return *m_ptr_;
  }

  pointer operator->() { return m_ptr_; }

  // TODO(nathan) sit down and do the math for all of these
  RingBufferIter& operator-=(difference_type diff) { return *this; }

  RingBufferIter& operator+=(difference_type diff) { return *this; }

  RingBufferIter operator-(difference_type diff) const { return *this; }

  RingBufferIter operator+(difference_type diff) const { return *this; }

  difference_type operator-(const RingBufferIter& rhs) {
    CHECK_EQ(begin_ptr_, rhs.begin_ptr_);
    CHECK_EQ(end_ptr_, rhs.end_ptr_);
    CHECK_EQ(curr_index_, rhs.curr_index_);
    CHECK_EQ(num_values_, rhs.num_values_);
    // get the absolute index in the buffer
    difference_type lhs_abs_idx = m_ptr_ - begin_ptr_;
    difference_type rhs_abs_idx = rhs.m_ptr_ - rhs.begin_ptr_;
    // get the relative index compared to curr_index_
    difference_type lhs_rel_idx;
    if (lhs_abs_idx > curr_index_) {
      lhs_rel_idx = lhs_abs_idx - curr_index_;
    } else {
      lhs_rel_idx = num_values_ - (curr_index_ - lhs_abs_idx);
    }
    // do the same for the right side
    difference_type rhs_rel_idx;
    if (rhs_abs_idx > curr_index_) {
      rhs_rel_idx = rhs_abs_idx - curr_index_;
    } else {
      rhs_rel_idx = num_values_ - (curr_index_ - rhs_abs_idx);
    }
    return lhs_rel_idx - rhs_rel_idx;
  }

  RingBufferIter& operator++() {
    m_ptr_++;
    if (m_ptr_ > end_ptr_) {
      m_ptr_ = begin_ptr_;
    }
    return *this;
  }

  RingBufferIter operator++(int) {
    RingBufferIter tmp = *this;
    ++(*this);
    return tmp;
  }

  friend bool operator==(const RingBufferIter& a, const RingBufferIter& b) {
    return a.m_ptr_ == b.m_ptr_;
  }

  friend bool operator!=(const RingBufferIter& a, const RingBufferIter& b) {
    return !(a == b);
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const RingBufferIter<Value>& iter) {
    out << "iter<(curr=" << iter.m_ptr_ << ", begin=" << iter.begin_ptr_
        << ", end=" << iter.end_ptr_ << ")>";
    return out;
  }

 private:
  pointer m_ptr_;
  size_t curr_index_;
  size_t num_values_;
  value_type* begin_ptr_;
  value_type* end_ptr_;
};

template <typename T>
class RingBuffer {
 public:
  typedef RingBufferIter<T> iterator;

  explicit RingBuffer(size_t buffer_size)
      : size_(buffer_size),
        num_values_(0),
        curr_index_(0),
        values_(buffer_size + 1) {}

  inline void clear() {
    values_ = std::vector<T>(size_ + 1);
    num_values_ = 0;
    curr_index_ = 0;
  }

  inline size_t size() const { return num_values_; }

  inline bool full() const { return size_ == num_values_; }

  inline void push(const T& value) {
    values_[curr_index_] = value;
    if (curr_index_ == size_) {
      curr_index_ = 0;
    } else {
      curr_index_++;
    }
    if (num_values_ < size_) {
      num_values_++;
    }
  }

  inline iterator begin() {
    return iterator(values_,
                    num_values_,
                    curr_index_,
                    &(values_.data()[get_begin_index_()]));
  }

  inline iterator end() {
    return iterator(
        values_, num_values_, curr_index_, &(values_.data()[get_end_index_()]));
  }

  inline T front() const { return values_[get_begin_index_()]; }

  inline T back() const { return values_[get_newest_index_()]; }

  friend std::ostream& operator<<(std::ostream& out, const RingBuffer& buffer) {
    out << "buffer<(curr=" << buffer.curr_index_
        << ", size=" << buffer.num_values_
        << ", max_size=" << buffer.size_ << ")>";
    return out;
  }

 private:
  // oldest message in buffer
  inline size_t get_begin_index_() const {
    if (num_values_ < size_) {
      return 0;
    }

    return (curr_index_ == size_) ? 0 : curr_index_ + 1;
  }

  // newest message in buffer
  inline size_t get_newest_index_() const {
    if (num_values_ == 0) {
      return 0;
    }
    return (curr_index_ == 0) ? size_ : curr_index_ - 1;
  }

  // index to stop iteration on
  inline size_t get_end_index_() const {
    if (num_values_ == 0) {
      return 0;
    }
    return curr_index_;
  }

  size_t size_;
  size_t num_values_;
  size_t curr_index_;
  std::vector<T> values_;
};

}  // namespace VIO
