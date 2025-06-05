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

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/Gnss.h"
#include "kimera-vio/utils/ThreadsafeGnssBuffer.h"

namespace VIO {

TEST(ThreadsafeGnssBuffer, LinearInterpolate) {
  GnssPoint y;
  VIO::utils::ThreadsafeGnssBuffer::linearInterpolate(
      10, GnssPoint(10.0, 10.0, 10.0), 20, GnssPoint(50.0, 50.0, 50.0), 15, &y);
  EXPECT_EQ(y, GnssPoint(30.0, 30.0, 30.0));
}

TEST(ThreadsafeGnssBuffer, getNewestAndinterpolateGnssData) {
  VIO::utils::ThreadsafeGnssBuffer buffer(-1);

  GnssMeasurement meas;
  auto isOk = buffer.getNewestGnssMeasurement(&meas);
  EXPECT_EQ(isOk, false);

  buffer.addMeasurement(10, GnssPoint(10, 10, 10));
  isOk = buffer.getNewestGnssMeasurement(&meas);
  EXPECT_EQ(isOk, true);
  EXPECT_EQ(meas.point_, GnssPoint(10, 10, 10));
  EXPECT_EQ(meas.timestamp_, 10);
  buffer.addMeasurement(15, GnssPoint(15, 15, 15));
  buffer.addMeasurement(20, GnssPoint(20, 20, 20));
  buffer.addMeasurement(25, GnssPoint(25, 25, 25));
  buffer.addMeasurement(30, GnssPoint(30, 30, 30));
  buffer.addMeasurement(40, GnssPoint(40, 40, 40));
  buffer.addMeasurement(50, GnssPoint(50, 50, 50));

  isOk = buffer.getNewestGnssMeasurement(&meas);
  EXPECT_EQ(isOk, true);
  EXPECT_EQ(meas.point_, GnssPoint(50, 50, 50));
  EXPECT_EQ(meas.timestamp_, 50);

  Timestamp stamp;
  GnssPoint point;
  VIO::utils::ThreadsafeGnssBuffer::QueryResult result;
  // Test aligned getter.
  result = buffer.getInterpolatedValue(20, &stamp, &point);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeGnssBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(stamp, 20);
  EXPECT_EQ(point, GnssPoint(20, 20, 20));

  // Test aligned getter, but asking for lower bound
  result = buffer.getInterpolatedValue(35, &stamp, &point);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeGnssBuffer::QueryResult::kDataAvailable);
  EXPECT_EQ(stamp, 35);
  EXPECT_EQ(point, GnssPoint(35, 35, 35));

  // Fail: query out of lower bound.
  result = buffer.getInterpolatedValue(-10, &stamp, &point);
  EXPECT_EQ(result,
            VIO::utils::ThreadsafeGnssBuffer::QueryResult::kDataNeverAvailable);
}

}  // namespace VIO
