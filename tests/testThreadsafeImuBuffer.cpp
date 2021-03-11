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

#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontend.h"
#include "kimera-vio/utils/ThreadsafeImuBuffer.h"

#include <glog/logging.h>
#include <gtest/gtest.h>

namespace VIO {

TEST(ThreadsafeImuBuffer, PopFromEmptyBuffer) {
  VIO::utils::ThreadsafeImuBuffer buffer(-1);
  // Pop from empty buffer.
  ImuStampS imu_timestamps(1, 2);
  ImuAccGyrS imu_measurements(6, 2);
  {
    VIO::utils::ThreadsafeImuBuffer::QueryResult success =
        buffer.getImuDataBtwTimestamps(50, 100, &imu_timestamps,
                                       &imu_measurements);
    EXPECT_EQ(success,
              utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
    EXPECT_EQ(0u, imu_timestamps.size());
    EXPECT_EQ(0u, imu_measurements.size());
  }
  {
    VIO::utils::ThreadsafeImuBuffer::QueryResult success =
        buffer.getImuDataBtwTimestamps(50, 100, &imu_timestamps,
                                       &imu_measurements, true);
    EXPECT_EQ(success,
              utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
    EXPECT_EQ(0u, imu_timestamps.size());
    EXPECT_EQ(0u, imu_measurements.size());
  }
  {
    VIO::utils::ThreadsafeImuBuffer::QueryResult success =
        buffer.getImuDataInterpolatedUpperBorder(50, 100, &imu_timestamps,
                                                 &imu_measurements);
    EXPECT_EQ(success,
              utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
    EXPECT_EQ(0u, imu_timestamps.size());
    EXPECT_EQ(0u, imu_measurements.size());
  }
  {
    VIO::utils::ThreadsafeImuBuffer::QueryResult success =
        buffer.getImuDataInterpolatedBorders(50, 100, &imu_timestamps,
                                             &imu_measurements);
    EXPECT_EQ(success,
              utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
    EXPECT_EQ(0u, imu_timestamps.size());
    EXPECT_EQ(0u, imu_measurements.size());
  }
}

TEST(ThreadsafeImuBuffer, LinearInterpolate) {
  ImuAccGyr y;
  VIO::utils::ThreadsafeImuBuffer::linearInterpolate(
      10, ImuAccGyr::Constant(10.0), 20, ImuAccGyr::Constant(50.0), 15, &y);
  EXPECT_EQ(y, ImuAccGyr::Constant(30.0));
}

TEST(ThreadsafeImuBuffer, getImuDataBtwTimestamps) {
  VIO::utils::ThreadsafeImuBuffer buffer(-1);
  buffer.addMeasurement(10, ImuAccGyr::Constant(10.0));
  buffer.addMeasurement(15, ImuAccGyr::Constant(15.0));
  buffer.addMeasurement(20, ImuAccGyr::Constant(20.0));
  buffer.addMeasurement(25, ImuAccGyr::Constant(25.0));
  buffer.addMeasurement(30, ImuAccGyr::Constant(30.0));
  buffer.addMeasurement(40, ImuAccGyr::Constant(40.0));
  buffer.addMeasurement(50, ImuAccGyr::Constant(50.0));

  ImuStampS imu_timestamps;
  ImuAccGyrS imu_measurements;
  VIO::utils::ThreadsafeImuBuffer::QueryResult result;

  // Test aligned getter.
  result = buffer.getImuDataBtwTimestamps(20, 30, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 1);
  EXPECT_EQ(imu_measurements.cols(), 1);
  EXPECT_EQ(imu_timestamps(0), 25);
  EXPECT_EQ(imu_measurements.col(0)(0), 25.0);

  // Test aligned getter, but asking for lower bound
  result = buffer.getImuDataBtwTimestamps(20, 30, &imu_timestamps,
                                          &imu_measurements, true);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 2);
  EXPECT_EQ(imu_measurements.cols(), 2);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);

  // Test unaligned getter (no lower/upper-interpolation).
  result = buffer.getImuDataBtwTimestamps(19, 31, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 3);
  EXPECT_EQ(imu_measurements.cols(), 3);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 30);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 30.0);

  // Test unaligned getter but asking for lower bound.
  result = buffer.getImuDataBtwTimestamps(19, 31, &imu_timestamps,
                                          &imu_measurements, true);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 3);
  EXPECT_EQ(imu_measurements.cols(), 3);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 30);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 30.0);

  // Fail: query out of upper bound.
  result = buffer.getImuDataBtwTimestamps(40, 51, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
  result = buffer.getImuDataBtwTimestamps(60, 61, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);

  // Fail: query out of lower bound.
  result = buffer.getImuDataBtwTimestamps(-1, 20, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable);
  result = buffer.getImuDataBtwTimestamps(-20, -10, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable);

  // Query in between two values: return nothing.
  result = buffer.getImuDataBtwTimestamps(21, 24, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result, VIO::utils::ThreadsafeImuBuffer::QueryResult::
                        kTooFewMeasurementsAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 0);
  EXPECT_EQ(imu_measurements.cols(), 0);
  result = buffer.getImuDataBtwTimestamps(21, 24, &imu_timestamps,
                                          &imu_measurements, true);
  EXPECT_EQ(result, VIO::utils::ThreadsafeImuBuffer::QueryResult::
                        kTooFewMeasurementsAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 0);
  EXPECT_EQ(imu_measurements.cols(), 0);

  // Query right between two values: return nothing.
  result = buffer.getImuDataBtwTimestamps(20, 25, &imu_timestamps,
                                          &imu_measurements);
  EXPECT_EQ(result, VIO::utils::ThreadsafeImuBuffer::QueryResult::
                        kTooFewMeasurementsAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 0);
  EXPECT_EQ(imu_measurements.cols(), 0);

  // Query right between two values but ask for lower bound: return lower bound.
  result = buffer.getImuDataBtwTimestamps(20, 25, &imu_timestamps,
                                          &imu_measurements, true);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 1);
  EXPECT_EQ(imu_measurements.cols(), 1);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
}

TEST(ThreadsafeImuBuffer, getImuDataInterpolatedBorders) {
  VIO::utils::ThreadsafeImuBuffer buffer(-1);
  buffer.addMeasurement(10, ImuAccGyr::Constant(10.0));
  buffer.addMeasurement(15, ImuAccGyr::Constant(15.0));
  buffer.addMeasurement(20, ImuAccGyr::Constant(20.0));
  buffer.addMeasurement(25, ImuAccGyr::Constant(25.0));
  buffer.addMeasurement(30, ImuAccGyr::Constant(30.0));
  buffer.addMeasurement(40, ImuAccGyr::Constant(40.0));
  buffer.addMeasurement(50, ImuAccGyr::Constant(50.0));

  ImuStampS imu_timestamps;
  ImuAccGyrS imu_measurements;
  VIO::utils::ThreadsafeImuBuffer::QueryResult result;

  // Test aligned getter (no-interpolation, only border values).
  result = buffer.getImuDataInterpolatedBorders(20, 30, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 3);
  EXPECT_EQ(imu_measurements.cols(), 3);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 30);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 30.0);

  // Test aligned getter (no-interpolation).
  result = buffer.getImuDataInterpolatedBorders(20, 40, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 4);
  EXPECT_EQ(imu_measurements.cols(), 4);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 30);
  EXPECT_EQ(imu_timestamps(3), 40);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 30.0);
  EXPECT_EQ(imu_measurements.col(3)(0), 40.0);

  // Test unaligned getter (lower/upper-interpolation).
  result = buffer.getImuDataInterpolatedBorders(19, 21, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 3);
  EXPECT_EQ(imu_measurements.cols(), 3);
  EXPECT_EQ(imu_timestamps(0), 19);
  EXPECT_EQ(imu_timestamps(1), 20);
  EXPECT_EQ(imu_timestamps(2), 21);
  EXPECT_EQ(imu_measurements.col(0)(0), 19.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 21.0);

  // Fail: query out of upper bound.
  result = buffer.getImuDataInterpolatedBorders(40, 51, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
  result = buffer.getImuDataInterpolatedBorders(60, 61, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);

  // Fail: query out of lower bound.
  result = buffer.getImuDataInterpolatedBorders(-1, 20, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable);
  result = buffer.getImuDataInterpolatedBorders(-20, -10, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable);

  // Query between two values: return the border values.
  result = buffer.getImuDataInterpolatedBorders(21, 29, &imu_timestamps,
                                                &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 3);
  EXPECT_EQ(imu_measurements.cols(), 3);
  EXPECT_EQ(imu_timestamps(0), 21);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 29);
  EXPECT_EQ(imu_measurements.col(0)(0), 21.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 29.0);
}

TEST(ThreadsafeImuBuffer, getImuDataInterpolatedUpperBorder) {
  VIO::utils::ThreadsafeImuBuffer buffer(-1);
  buffer.addMeasurement(10, ImuAccGyr::Constant(10.0));
  buffer.addMeasurement(15, ImuAccGyr::Constant(15.0));
  buffer.addMeasurement(20, ImuAccGyr::Constant(20.0));
  buffer.addMeasurement(25, ImuAccGyr::Constant(25.0));
  buffer.addMeasurement(30, ImuAccGyr::Constant(30.0));
  buffer.addMeasurement(40, ImuAccGyr::Constant(40.0));
  buffer.addMeasurement(50, ImuAccGyr::Constant(50.0));

  ImuStampS imu_timestamps;
  ImuAccGyrS imu_measurements;
  VIO::utils::ThreadsafeImuBuffer::QueryResult result;

  // Test aligned getter (no-interpolation).
  result = buffer.getImuDataInterpolatedUpperBorder(20, 40, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 4);
  EXPECT_EQ(imu_measurements.cols(), 4);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 30);
  EXPECT_EQ(imu_timestamps(3), 40);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 30.0);
  EXPECT_EQ(imu_measurements.col(3)(0), 40.0);

  // Test unaligned getter (only upper-interpolation).
  result = buffer.getImuDataInterpolatedUpperBorder(19, 21, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 2);
  EXPECT_EQ(imu_measurements.cols(), 2);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 21);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 21.0);

  // Fail: query out of upper bound.
  result = buffer.getImuDataInterpolatedUpperBorder(40, 51, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);
  result = buffer.getImuDataInterpolatedUpperBorder(60, 61, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataNotYetAvailable);

  // Fail: query out of lower bound.
  result = buffer.getImuDataInterpolatedUpperBorder(9, 20, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable);
  result = buffer.getImuDataInterpolatedUpperBorder(-20, -10, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataNeverAvailable);

  // Query in between two values: return too few measurements available.
  // even if asked to interpolate, there are no measurements in between
  // given timestamps.
  result = buffer.getImuDataInterpolatedUpperBorder(21, 24, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result, VIO::utils::ThreadsafeImuBuffer::QueryResult::
                        kTooFewMeasurementsAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 0);
  EXPECT_EQ(imu_measurements.cols(), 0);

  // Query with only one value inside interval:
  // return the interpolated border value for upper border only,
  // and one measurement
  result = buffer.getImuDataInterpolatedUpperBorder(21, 29, &imu_timestamps,
                                                    &imu_measurements);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(imu_timestamps.cols(), 2);
  EXPECT_EQ(imu_measurements.cols(), 2);
  EXPECT_EQ(imu_timestamps(0), 25);
  EXPECT_EQ(imu_timestamps(1), 29);
  EXPECT_EQ(imu_measurements.col(0)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 29.0);
}

//// Disabled because CppUnitTests does not support Expect DEATH.
// TEST(ThreadsafeImuBuffer, DeathOnAddDataNotIncreasingTimestamp) {
//  VIO::utils::ThreadsafeImuBuffer buffer(-1);
//
//  ImuAccGyr imu_measurement;
//  imu_measurement.setRandom();
//
//  buffer.addMeasurement(0u, imu_measurement);
//  buffer.addMeasurement(10u, imu_measurement);
//  //EXPECT_DEATH(buffer.addMeasurement(9u, imu_measurement), "^");
//}

TEST(ThreadsafeImuBuffer, TestAddMeasurements) {
  const size_t kNumMeasurements = 10;
  VIO::utils::ThreadsafeImuBuffer buffer(-1);

  // Create IMU measurements and fill buffer.
  ImuStampS imu_timestamps_groundtruth(1, kNumMeasurements);
  ImuAccGyrS imu_measurements_groundtruth(6, kNumMeasurements);

  for (size_t idx = 0; idx < kNumMeasurements; ++idx) {
    Timestamp timestamp = static_cast<Timestamp>(idx * 10);
    ImuAccGyr imu_measurement;
    imu_measurement.setConstant(idx);
    imu_timestamps_groundtruth(idx) = timestamp;
    imu_measurements_groundtruth.col(idx) = imu_measurement;
  }
  buffer.addMeasurements(imu_timestamps_groundtruth,
                         imu_measurements_groundtruth);
}

}  // namespace VIO
