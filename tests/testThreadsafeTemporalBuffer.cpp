/*******************************************************************************
 * MIT License
 *
 * Copyright (c) 2019 Toni Rosinol
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *******************************************************************************/

/********************************************************************************
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
*********************************************************************************/

#include <chrono>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/utils/ThreadsafeTemporalBuffer.h"

namespace VIO {

namespace utils {

struct TestData {
  explicit TestData(int64_t time) : timestamp(time) {}
  TestData() = default;

  int64_t timestamp;
};

class ThreadsafeTemporalBufferFixture : public ::testing::Test {
 public:
  ThreadsafeTemporalBufferFixture() : buffer_(kBufferLengthNs) {}

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}  //
  void addValue(const TestData& data) {
    buffer_.addValue(data.timestamp, data);
  }

  static constexpr int64_t kBufferLengthNs = 100;
  ThreadsafeTemporalBuffer<TestData> buffer_;
};

TEST_F(ThreadsafeTemporalBufferFixture, SizeEmptyClearWork) {
  EXPECT_TRUE(buffer_.empty());
  EXPECT_EQ(buffer_.size(), 0u);

  addValue(TestData(10));
  addValue(TestData(20));
  EXPECT_TRUE(!buffer_.empty());
  EXPECT_EQ(buffer_.size(), 2u);

  buffer_.clear();
  EXPECT_TRUE(buffer_.empty());
  EXPECT_EQ(buffer_.size(), 0u);
}

TEST_F(ThreadsafeTemporalBufferFixture, GetValueAtTimeWorks) {
  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(40));

  TestData retrieved_item;
  EXPECT_TRUE(buffer_.getValueAtTime(10, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getValueAtTime(20, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 20);

  EXPECT_TRUE(!buffer_.getValueAtTime(15, &retrieved_item));

  EXPECT_TRUE(buffer_.getValueAtTime(30, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 30);
}

TEST_F(ThreadsafeTemporalBufferFixture, GetNearestValueToTimeWorks) {
  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));

  TestData retrieved_item;
  EXPECT_TRUE(buffer_.getNearestValueToTime(10, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(0, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(16, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 20);

  EXPECT_TRUE(buffer_.getNearestValueToTime(26, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 30);

  EXPECT_TRUE(buffer_.getNearestValueToTime(32, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 30);

  EXPECT_TRUE(buffer_.getNearestValueToTime(1232, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 30);
}

TEST_F(ThreadsafeTemporalBufferFixture, GetNearestValueToTimeMaxDeltaWorks) {
  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));

  const int kMaxDelta = 5;

  TestData retrieved_item;
  EXPECT_TRUE(buffer_.getNearestValueToTime(10, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(!buffer_.getNearestValueToTime(0, kMaxDelta, &retrieved_item));

  EXPECT_TRUE(buffer_.getNearestValueToTime(9, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(16, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 20);

  EXPECT_TRUE(buffer_.getNearestValueToTime(26, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 30);

  EXPECT_TRUE(buffer_.getNearestValueToTime(32, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 30);

  EXPECT_TRUE(!buffer_.getNearestValueToTime(36, kMaxDelta, &retrieved_item));

  buffer_.clear();
  addValue(TestData(10));
  addValue(TestData(20));

  EXPECT_TRUE(buffer_.getNearestValueToTime(9, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(12, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(16, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 20);

  EXPECT_TRUE(buffer_.getNearestValueToTime(22, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 20);

  buffer_.clear();
  addValue(TestData(10));

  EXPECT_TRUE(buffer_.getNearestValueToTime(6, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(14, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(!buffer_.getNearestValueToTime(16, kMaxDelta, &retrieved_item));
}

TEST_F(ThreadsafeTemporalBufferFixture, GetValueAtOrBeforeTimeWorks) {
  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(40));

  TestData retrieved_item;
  int64_t timestamp;

  EXPECT_TRUE(buffer_.getValueAtOrBeforeTime(40, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
  EXPECT_EQ(timestamp, 40);

  EXPECT_TRUE(buffer_.getValueAtOrBeforeTime(50, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
  EXPECT_EQ(timestamp, 40);

  EXPECT_TRUE(buffer_.getValueAtOrBeforeTime(15, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);
  EXPECT_EQ(timestamp, 10);

  EXPECT_TRUE(buffer_.getValueAtOrBeforeTime(10, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);
  EXPECT_EQ(timestamp, 10);

  EXPECT_TRUE(!buffer_.getValueAtOrBeforeTime(5, &timestamp, &retrieved_item));
}

TEST_F(ThreadsafeTemporalBufferFixture, GetValueAtOrAfterTimeWorks) {
  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(40));

  TestData retrieved_item;
  int64_t timestamp;

  EXPECT_TRUE(buffer_.getValueAtOrAfterTime(10, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);
  EXPECT_EQ(timestamp, 10);

  EXPECT_TRUE(buffer_.getValueAtOrAfterTime(5, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);
  EXPECT_EQ(timestamp, 10);

  EXPECT_TRUE(buffer_.getValueAtOrAfterTime(35, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
  EXPECT_EQ(timestamp, 40);

  EXPECT_TRUE(buffer_.getValueAtOrAfterTime(40, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
  EXPECT_EQ(timestamp, 40);

  EXPECT_TRUE(!buffer_.getValueAtOrAfterTime(45, &timestamp, &retrieved_item));
}

TEST_F(ThreadsafeTemporalBufferFixture, GetOldestNewestValueWork) {
  TestData retrieved_item;
  EXPECT_TRUE(!buffer_.getOldestValue(&retrieved_item));
  EXPECT_TRUE(!buffer_.getNewestValue(&retrieved_item));

  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(40));

  EXPECT_TRUE(buffer_.getOldestValue(&retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNewestValue(&retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
}

TEST_F(ThreadsafeTemporalBufferFixture, GetValuesBetweenTimesWorks) {
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(30));
  addValue(TestData(40));
  addValue(TestData(50));

  // Test aligned borders.
  /// When the user does not ask for the lower bound.
  /// Implicitly also checks that it is default behaviour.
  std::vector<TestData> values;
  EXPECT_TRUE(buffer_.getValuesBetweenTimes(10, 50, &values));
  EXPECT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0].timestamp, 20);
  EXPECT_EQ(values[1].timestamp, 30);
  EXPECT_EQ(values[2].timestamp, 40);

  // Test aligned borders.
  /// When the user does ask for the lower bound.
  EXPECT_TRUE(buffer_.getValuesBetweenTimes(10, 50, &values, true));
  EXPECT_EQ(values.size(), 4u);
  EXPECT_EQ(values[0].timestamp, 10);
  EXPECT_EQ(values[1].timestamp, 20);
  EXPECT_EQ(values[2].timestamp, 30);
  EXPECT_EQ(values[3].timestamp, 40);

  // Test unaligned borders.
  /// When the user does not ask for the lower bound.
  /// Implicitly also checks that it is default behaviour.
  EXPECT_TRUE(buffer_.getValuesBetweenTimes(15, 45, &values));
  EXPECT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0].timestamp, 20);
  EXPECT_EQ(values[1].timestamp, 30);
  EXPECT_EQ(values[2].timestamp, 40);

  // Test unaligned borders.
  /// When the user does ask for the lower bound.
  EXPECT_TRUE(buffer_.getValuesBetweenTimes(15, 45, &values));
  EXPECT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0].timestamp, 20);
  EXPECT_EQ(values[1].timestamp, 30);
  EXPECT_EQ(values[2].timestamp, 40);

  // Test unsuccessful queries.
  // Lower border oob.
  EXPECT_TRUE(!buffer_.getValuesBetweenTimes(5, 45, &values));
  EXPECT_TRUE(!buffer_.getValuesBetweenTimes(5, 45, &values, true));
  // Higher border oob.
  EXPECT_TRUE(!buffer_.getValuesBetweenTimes(30, 55, &values));
  EXPECT_TRUE(!buffer_.getValuesBetweenTimes(30, 55, &values, true));
  EXPECT_TRUE(values.empty());

  // The method should check-fail when the buffer is empty.
  buffer_.clear();
  EXPECT_TRUE(!buffer_.getValuesBetweenTimes(10, 50, &values));
  EXPECT_TRUE(!buffer_.getValuesBetweenTimes(10, 50, &values, true));
  // EXPECT_DEATH(buffer_.getValuesBetweenTimes(40, 30, &values), "^");
}

TEST_F(ThreadsafeTemporalBufferFixture, MaintaingBufferLengthWorks) {
  addValue(TestData(0));
  addValue(TestData(50));
  addValue(TestData(100));
  EXPECT_EQ(buffer_.size(), 3u);

  addValue(TestData(150));
  EXPECT_EQ(buffer_.size(), 3u);

  TestData retrieved_item;
  EXPECT_TRUE(buffer_.getOldestValue(&retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 50);

  EXPECT_TRUE(buffer_.getNewestValue(&retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 150);
}

}  // namespace utils

}  // namespace VIO
