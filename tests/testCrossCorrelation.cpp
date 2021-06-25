/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testCrossCorrelation.cpp
 * @brief  Unit tests for the ring buffer and cross correlation
 * @author Nathan Hughes
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <numeric>

#include "kimera-vio/initial/RingBuffer.h"

namespace VIO {

template <typename T>
double mean(const RingBuffer<T>& buffer) {
  return utils::mean(buffer,
                     [](const T& value) { return static_cast<double>(value); });
}

template <typename T>
double variance(const RingBuffer<T>& buffer) {
  return utils::variance(
      buffer, [](const T& value) { return static_cast<double>(value); });
}

template <typename T>
std::vector<double> crossCorrelation(const RingBuffer<T>& seq_a,
                                     const RingBuffer<T>& seq_b) {
  return utils::crossCorrelation(
      seq_a, seq_b, [](const T& value) { return static_cast<double>(value); });
}

TEST(testCrossCorrelation, ringBufferSimpleTest) {
  RingBuffer<int> buffer(5);
  buffer.push(1);
  buffer.push(2);
  buffer.push(3);

  std::vector<int> values(buffer.begin(), buffer.end());

  EXPECT_FALSE(buffer.full());
  ASSERT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0], 1);
  EXPECT_EQ(values[1], 2);
  EXPECT_EQ(values[2], 3);

  buffer.push(4);
  buffer.push(5);

  EXPECT_TRUE(buffer.full());

  values = std::vector<int>(buffer.begin(), buffer.end());
  std::vector<int> expected{1, 2, 3, 4, 5};
  ASSERT_EQ(values.size(), 5u);
  for (size_t i = 0; i < values.size(); ++i) {
    EXPECT_EQ(expected[i], values[i]);
  }
}

TEST(testCrossCorrelation, ringBufferPushWhileFull) {
  RingBuffer<int> buffer(5);
  for (size_t i = 0; i < 10; ++i) {
    buffer.push(i);
  }

  EXPECT_TRUE(buffer.full());

  std::vector<int> values(buffer.begin(), buffer.end());
  ASSERT_EQ(5u, values.size());

  std::vector<int> expected{5, 6, 7, 8, 9};

  for (size_t i = 0; i < values.size(); ++i) {
    EXPECT_EQ(expected[i], values[i]);
    EXPECT_EQ(values[i], buffer[i]);
  }
}

TEST(testCrossCorrelation, ringBufferClear) {
  RingBuffer<int> buffer(5);
  for (size_t i = 0; i < 10; ++i) {
    buffer.push(-1);
  }

  EXPECT_TRUE(buffer.full());
  int total = std::accumulate(buffer.begin(), buffer.end(), 0);
  EXPECT_EQ(-5, total);

  buffer.clear();
  EXPECT_FALSE(buffer.full());
  buffer.push(1);
  buffer.push(2);
  total = std::accumulate(buffer.begin(), buffer.end(), 0);
  EXPECT_EQ(3, total);
}

TEST(testCrossCorrelation, bufferFrontAndBack) {
  RingBuffer<int> buffer(5);
  for (size_t i = 0; i < 5; ++i) {
    buffer.push(i);
    EXPECT_EQ(buffer.back(), i);
    EXPECT_EQ(buffer.front(), 0);
  }
  // check front and back after dropping values
  for (size_t i = 5; i < 10; ++i) {
    buffer.push(i);
    EXPECT_EQ(buffer.back(), i);
    EXPECT_EQ(buffer.front(), i - 4);
  }
}

TEST(testCrossCorrelation, meanCorrect) {
  RingBuffer<double> buffer(5);
  EXPECT_EQ(0.0, mean(buffer));
  buffer.push(1);
  EXPECT_EQ(1.0, mean(buffer));
  buffer.push(2);
  EXPECT_EQ(1.5, mean(buffer));
  buffer.push(3);
  EXPECT_EQ(2, mean(buffer));
  buffer.push(4);
  EXPECT_EQ(2.5, mean(buffer));
  buffer.push(5);
  EXPECT_EQ(3.0, mean(buffer));
  buffer.push(6);
  EXPECT_EQ(4.0, mean(buffer));
}

TEST(testCrossCorrelation, varianceCorrect) {
  RingBuffer<double> buffer(5);
  EXPECT_EQ(0.0, variance(buffer));
  buffer.push(1);
  EXPECT_EQ(0.0, variance(buffer));
  buffer.push(1);
  EXPECT_EQ(0.0, variance(buffer));
  buffer.push(4);
  EXPECT_EQ(2.0, variance(buffer));
}

TEST(testCrossCorrelation, crossCorrelationIdentityA) {
  RingBuffer<double> seq_a(1);
  seq_a.push(1);

  RingBuffer<double> seq_b(5);
  seq_b.push(1);
  seq_b.push(2);
  seq_b.push(3);
  seq_b.push(4);
  seq_b.push(5);

  std::vector<double> correlation = crossCorrelation(seq_a, seq_b);
  ASSERT_EQ(seq_b.size(), correlation.size());
  for (size_t i = 0; i < seq_b.size(); ++i) {
    EXPECT_EQ(seq_b[i], correlation[seq_b.size() - 1 - i]);
  }
}

TEST(testCrossCorrelation, crossCorrelationIdentityB) {
  RingBuffer<double> seq_b(1);
  seq_b.push(1);

  RingBuffer<double> seq_a(5);
  seq_a.push(1);
  seq_a.push(2);
  seq_a.push(3);
  seq_a.push(4);
  seq_a.push(5);

  std::vector<double> correlation = crossCorrelation(seq_a, seq_b);
  ASSERT_EQ(seq_a.size(), correlation.size());
  for (size_t i = 0; i < seq_a.size(); ++i) {
    EXPECT_EQ(seq_a[i], correlation[i]);
  }
}

TEST(testCrossCorrelation, crossCorrelationNpyExample) {
  RingBuffer<double> seq_a(3);
  seq_a.push(1);
  seq_a.push(2);
  seq_a.push(3);

  RingBuffer<double> seq_b(3);
  seq_b.push(0);
  seq_b.push(1);
  seq_b.push(0.5);

  std::vector<double> correlation = crossCorrelation(seq_a, seq_b);
  std::vector<double> expected{0.5, 2.0, 3.5, 3.0, 0.0};
  ASSERT_EQ(expected.size(), correlation.size());
  for (size_t i = 0; i < seq_a.size(); ++i) {
    EXPECT_EQ(expected[i], correlation[i]);
  }

  std::vector<double> correlation_rev = crossCorrelation(seq_b, seq_a);
  std::vector<double> expected_rev(expected.rbegin(), expected.rend());
  ASSERT_EQ(expected_rev.size(), correlation_rev.size());
  for (size_t i = 0; i < seq_a.size(); ++i) {
    EXPECT_EQ(expected_rev[i], correlation_rev[i]);
  }
}

}  // namespace VIO
