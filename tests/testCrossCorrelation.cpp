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

TEST(testCrossCorrelation, basicPointerDiffCorrect) {
  RingBuffer<int> buffer(5);
  EXPECT_EQ(0, buffer.begin() - buffer.end());
  EXPECT_EQ(0, buffer.end() - buffer.begin());
  buffer.push(1);
  EXPECT_EQ(-1, buffer.begin() - buffer.end());
  EXPECT_EQ(1, buffer.end() - buffer.begin());
  buffer.push(2);
  EXPECT_EQ(-2, buffer.begin() - buffer.end());
  EXPECT_EQ(2, buffer.end() - buffer.begin());
  buffer.push(3);
  EXPECT_EQ(-3, buffer.begin() - buffer.end());
  EXPECT_EQ(3, buffer.end() - buffer.begin());
  buffer.push(4);
  EXPECT_EQ(-4, buffer.begin() - buffer.end());
  EXPECT_EQ(4, buffer.end() - buffer.begin());
}

TEST(testCrossCorrelation, DISABLED_harderPointerDiffCorrect) {
  RingBuffer<int> buffer(5);
  for (size_t i = 0; i < 7; ++i) {
    buffer.push(0);  // contents don't matter
  }
  EXPECT_EQ(-5, buffer.begin() - buffer.end());
  EXPECT_EQ(5, buffer.end() - buffer.begin());

  RingBuffer<int>::iterator next_to_last = buffer.end() - 1;
  EXPECT_EQ(1, buffer.end() - next_to_last);
  EXPECT_EQ(-1, next_to_last - buffer.end());
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

}  // namespace VIO
