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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
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

#include "utils/ThreadsafeTemporalBuffer.h"

// Add last, since it redefines CHECK, which is first defined by glog.
#include <CppUnitLite/TestHarness.h>

namespace VIO {

namespace utils {

struct TestData {
  explicit TestData(int64_t time) : timestamp(time) {}
  TestData() = default;

  int64_t timestamp;
};

// Test fixture gtest style.
// class ThreadsafeTemporalBufferFixture : public ::testing::Test {
//  public:
//   ThreadsafeTemporalBufferFixture() : buffer_(kBufferLengthNs) {}
//
//  protected:
//   virtual void SetUp() {}
//   virtual void TearDown() {}
//
//   void addValue(const TestData& data) {
//     buffer_.addValue(data.timestamp, data);
//   }
//
//   static constexpr int64_t kBufferLengthNs = 100;
//   ThreadsafeTemporalBuffer<TestData> buffer_;
// };
class ThreadsafeTemporalBufferFixture {
 public:
  ThreadsafeTemporalBufferFixture() : buffer_(kBufferLengthNs) {}

  void addValue(const TestData& data) {
    buffer_.addValue(data.timestamp, data);
  }

  static constexpr int64_t kBufferLengthNs = 100;
  ThreadsafeTemporalBuffer<TestData> buffer_;
};

TEST(ThreadsafeTemporalBufferFixture, SizeEmptyClearWork) {
  ThreadsafeTemporalBufferFixture fixture;
  EXPECT(fixture.buffer_.empty());
  EXPECT(fixture.buffer_.size() == 0u);

  fixture.addValue(TestData(10));
  fixture.addValue(TestData(20));
  EXPECT(!fixture.buffer_.empty());
  EXPECT(fixture.buffer_.size() == 2u);

  fixture.buffer_.clear();
  EXPECT(fixture.buffer_.empty());
  EXPECT(fixture.buffer_.size() == 0u);
}

TEST(ThreadsafeTemporalBufferFixture, GetValueAtTimeWorks) {
  ThreadsafeTemporalBufferFixture fixture;
  fixture.addValue(TestData(30));
  fixture.addValue(TestData(10));
  fixture.addValue(TestData(20));
  fixture.addValue(TestData(40));

  TestData retrieved_item;
  EXPECT(fixture.buffer_.getValueAtTime(10, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);

  EXPECT(fixture.buffer_.getValueAtTime(20, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 20);

  EXPECT(!fixture.buffer_.getValueAtTime(15, &retrieved_item));

  EXPECT(fixture.buffer_.getValueAtTime(30, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 30);
}

TEST(ThreadsafeTemporalBufferFixture, GetNearestValueToTimeWorks) {
  ThreadsafeTemporalBufferFixture fixture;
  fixture.addValue(TestData(30));
  fixture.addValue(TestData(10));
  fixture.addValue(TestData(20));

  TestData retrieved_item;
  EXPECT(fixture.buffer_.getNearestValueToTime(10, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);

  EXPECT(fixture.buffer_.getNearestValueToTime(0, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);

  EXPECT(fixture.buffer_.getNearestValueToTime(16, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 20);

  EXPECT(fixture.buffer_.getNearestValueToTime(26, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 30);

  EXPECT(fixture.buffer_.getNearestValueToTime(32, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 30);

  EXPECT(fixture.buffer_.getNearestValueToTime(1232, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 30);
}

TEST(ThreadsafeTemporalBufferFixture, GetNearestValueToTimeMaxDeltaWorks) {
  ThreadsafeTemporalBufferFixture fixture;
  fixture.addValue(TestData(30));
  fixture.addValue(TestData(10));
  fixture.addValue(TestData(20));

  const int kMaxDelta = 5;

  TestData retrieved_item;
  EXPECT(fixture.buffer_.getNearestValueToTime(10, kMaxDelta, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);

  EXPECT(!fixture.buffer_.getNearestValueToTime(0, kMaxDelta, &retrieved_item));

  EXPECT(fixture.buffer_.getNearestValueToTime(9, kMaxDelta, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);

  EXPECT(fixture.buffer_.getNearestValueToTime(16, kMaxDelta, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 20);

  EXPECT(fixture.buffer_.getNearestValueToTime(26, kMaxDelta, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 30);

  EXPECT(fixture.buffer_.getNearestValueToTime(32, kMaxDelta, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 30);

  EXPECT(!fixture.buffer_.getNearestValueToTime(36, kMaxDelta, &retrieved_item));

  fixture.buffer_.clear();
  fixture.addValue(TestData(10));
  fixture.addValue(TestData(20));

  EXPECT(fixture.buffer_.getNearestValueToTime(9, kMaxDelta, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);

  EXPECT(fixture.buffer_.getNearestValueToTime(12, kMaxDelta, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);

  EXPECT(fixture.buffer_.getNearestValueToTime(16, kMaxDelta, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 20);

  EXPECT(fixture.buffer_.getNearestValueToTime(22, kMaxDelta, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 20);

  fixture.buffer_.clear();
  fixture.addValue(TestData(10));

  EXPECT(fixture.buffer_.getNearestValueToTime(6, kMaxDelta, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);

  EXPECT(fixture.buffer_.getNearestValueToTime(14, kMaxDelta, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);

  EXPECT(!fixture.buffer_.getNearestValueToTime(16, kMaxDelta, &retrieved_item));
}

TEST(ThreadsafeTemporalBufferFixture, GetValueAtOrBeforeTimeWorks) {
  ThreadsafeTemporalBufferFixture fixture;
  fixture.addValue(TestData(30));
  fixture.addValue(TestData(10));
  fixture.addValue(TestData(20));
  fixture.addValue(TestData(40));

  TestData retrieved_item;
  int64_t timestamp;

  EXPECT(fixture.buffer_.getValueAtOrBeforeTime(40, &timestamp, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 40);
  EXPECT(timestamp == 40);

  EXPECT(fixture.buffer_.getValueAtOrBeforeTime(50, &timestamp, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 40);
  EXPECT(timestamp == 40);

  EXPECT(fixture.buffer_.getValueAtOrBeforeTime(15, &timestamp, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);
  EXPECT(timestamp == 10);

  EXPECT(fixture.buffer_.getValueAtOrBeforeTime(10, &timestamp, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);
  EXPECT(timestamp == 10);

  EXPECT(!fixture.buffer_.getValueAtOrBeforeTime(5, &timestamp, &retrieved_item));
}

TEST(ThreadsafeTemporalBufferFixture, GetValueAtOrAfterTimeWorks) {
  ThreadsafeTemporalBufferFixture fixture;
  fixture.addValue(TestData(30));
  fixture.addValue(TestData(10));
  fixture.addValue(TestData(20));
  fixture.addValue(TestData(40));

  TestData retrieved_item;
  int64_t timestamp;

  EXPECT(fixture.buffer_.getValueAtOrAfterTime(10, &timestamp, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);
  EXPECT(timestamp == 10);

  EXPECT(fixture.buffer_.getValueAtOrAfterTime(5, &timestamp, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);
  EXPECT(timestamp == 10);

  EXPECT(fixture.buffer_.getValueAtOrAfterTime(35, &timestamp, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 40);
  EXPECT(timestamp == 40);

  EXPECT(fixture.buffer_.getValueAtOrAfterTime(40, &timestamp, &retrieved_item));
  EXPECT(retrieved_item.timestamp == 40);
  EXPECT(timestamp == 40);

  EXPECT(!fixture.buffer_.getValueAtOrAfterTime(45, &timestamp, &retrieved_item));
}

TEST(ThreadsafeTemporalBufferFixture, GetOldestNewestValueWork) {
  ThreadsafeTemporalBufferFixture fixture;
  TestData retrieved_item;
  EXPECT(!fixture.buffer_.getOldestValue(&retrieved_item));
  EXPECT(!fixture.buffer_.getNewestValue(&retrieved_item));

  fixture.addValue(TestData(30));
  fixture.addValue(TestData(10));
  fixture.addValue(TestData(20));
  fixture.addValue(TestData(40));

  EXPECT(fixture.buffer_.getOldestValue(&retrieved_item));
  EXPECT(retrieved_item.timestamp == 10);

  EXPECT(fixture.buffer_.getNewestValue(&retrieved_item));
  EXPECT(retrieved_item.timestamp == 40);
}

TEST(ThreadsafeTemporalBufferFixture, GetValuesBetweenTimesWorks) {
  ThreadsafeTemporalBufferFixture fixture;
  fixture.addValue(TestData(10));
  fixture.addValue(TestData(20));
  fixture.addValue(TestData(30));
  fixture.addValue(TestData(40));
  fixture.addValue(TestData(50));

  // Test aligned borders.
  /// When the user does not ask for the lower bound.
  /// Implicitly also checks that it is default behaviour.
  std::vector<TestData> values;
  EXPECT(fixture.buffer_.getValuesBetweenTimes(10, 50, &values));
  EXPECT(values.size() == 3u);
  EXPECT(values[0].timestamp == 20);
  EXPECT(values[1].timestamp == 30);
  EXPECT(values[2].timestamp == 40);

  // Test aligned borders.
  /// When the user does ask for the lower bound.
  EXPECT(fixture.buffer_.getValuesBetweenTimes(10, 50, &values, true));
  EXPECT(values.size() == 4u);
  EXPECT(values[0].timestamp == 10);
  EXPECT(values[1].timestamp == 20);
  EXPECT(values[2].timestamp == 30);
  EXPECT(values[3].timestamp == 40);

  // Test unaligned borders.
  /// When the user does not ask for the lower bound.
  /// Implicitly also checks that it is default behaviour.
  EXPECT(fixture.buffer_.getValuesBetweenTimes(15, 45, &values));
  EXPECT(values.size() == 3u);
  EXPECT(values[0].timestamp == 20);
  EXPECT(values[1].timestamp == 30);
  EXPECT(values[2].timestamp == 40);

  // Test unaligned borders.
  /// When the user does ask for the lower bound.
  EXPECT(fixture.buffer_.getValuesBetweenTimes(15, 45, &values));
  EXPECT(values.size() == 3u);
  EXPECT(values[0].timestamp == 20);
  EXPECT(values[1].timestamp == 30);
  EXPECT(values[2].timestamp == 40);

  // Test unsuccessful queries.
  // Lower border oob.
  EXPECT(!fixture.buffer_.getValuesBetweenTimes(5, 45, &values));
  EXPECT(!fixture.buffer_.getValuesBetweenTimes(5, 45, &values, true));
  // Higher border oob.
  EXPECT(!fixture.buffer_.getValuesBetweenTimes(30, 55, &values));
  EXPECT(!fixture.buffer_.getValuesBetweenTimes(30, 55, &values, true));
  EXPECT(values.empty());

  // The method should check-fail when the buffer is empty.
  fixture.buffer_.clear();
  EXPECT(!fixture.buffer_.getValuesBetweenTimes(10, 50, &values));
  EXPECT(!fixture.buffer_.getValuesBetweenTimes(10, 50, &values, true));
  // EXPECT_DEATH(fixture.buffer_.getValuesBetweenTimes(40, 30, &values), "^");
}

TEST(ThreadsafeTemporalBufferFixture, MaintaingBufferLengthWorks) {
  ThreadsafeTemporalBufferFixture fixture;
  fixture.addValue(TestData(0));
  fixture.addValue(TestData(50));
  fixture.addValue(TestData(100));
  EXPECT(fixture.buffer_.size() == 3u);

  fixture.addValue(TestData(150));
  EXPECT(fixture.buffer_.size() == 3u);

  TestData retrieved_item;
  EXPECT(fixture.buffer_.getOldestValue(&retrieved_item));
  EXPECT(retrieved_item.timestamp == 50);

  EXPECT(fixture.buffer_.getNewestValue(&retrieved_item));
  EXPECT(retrieved_item.timestamp == 150);
}

}  // End of utils namespace.

}  // End of VIO namespace.

/* ************************************************************************* */
int main(int argc, char *argv[]) {
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::INFO);

  TestResult tr; return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
