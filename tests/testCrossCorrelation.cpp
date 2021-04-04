/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testCrossCorrelation.cpp
 * @brief  Unit tests for cross correlation and time calibration
 * @author Nathan Hughes
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <algorithm>

#include "kimera-vio/initial/CrossCorrTimeAligner.h"

namespace VIO {

TEST(testCrossCorrelation, ringBufferSimpleTest) {
  RingBuffer buffer(5);
  buffer.push(1, 0.0);
  buffer.push(2, 0.0);
  buffer.push(3, 0.0);

  std::vector<Timestamp> times(buffer.times.begin(), buffer.times.end());
  EXPECT_FALSE(buffer.full());
  ASSERT_EQ(times.size(), 3u);
  EXPECT_EQ(times[0], 1);
  EXPECT_EQ(times[1], 2);
  EXPECT_EQ(times[2], 3);

  buffer.push(4, 0.0);
  buffer.push(5, 0.0);

  EXPECT_TRUE(buffer.full());

  times = std::vector<Timestamp>(buffer.times.begin(), buffer.times.end());
  std::vector<Timestamp> expected{1, 2, 3, 4, 5};
  ASSERT_EQ(times.size(), 5u);
  for (size_t i = 0; i < times.size(); ++i) {
    EXPECT_EQ(expected[i], times[i]);
  }
}

TEST(testCrossCorrelation, ringBufferPushWhileFull) {
  RingBuffer buffer(5);
  for (size_t i = 0; i < 10; ++i) {
    buffer.push(i, 0.0);
  }

  EXPECT_TRUE(buffer.full());

  std::vector<Timestamp> times(buffer.times.begin(), buffer.times.end());
  ASSERT_EQ(times.size(), 5u);

  std::vector<Timestamp> expected{5, 6, 7, 8, 9};

  for (size_t i = 0; i < times.size(); ++i) {
    EXPECT_EQ(expected[i], times[i]);
  }

  EXPECT_EQ(std::accumulate(buffer.times.begin(), buffer.times.end(), 0), 35);
}

TEST(testCrossCorrelation, ringBufferValues) {
  RingBuffer buffer(5);
  for (size_t i = 0; i < 10; ++i) {
    buffer.push(0, static_cast<double>(i) * 2.0);
  }

  EXPECT_TRUE(buffer.full());
  EXPECT_EQ(std::accumulate(buffer.times.begin(), buffer.times.end(), 0), 0);
  EXPECT_EQ(std::accumulate(buffer.values.begin(), buffer.values.end(), 0.0), 70.0);
}

}  // namespace VIO
