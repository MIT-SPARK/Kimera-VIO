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
#include "kimera-vio/utils/ThreadsafeOdometryBuffer.h"

#include <glog/logging.h>
#include <gtest/gtest.h>

namespace VIO {

using QueryResult = ThreadsafeOdometryBuffer::QueryResult;

TEST(ThreadsafeOdometryBuffer, getNearestOfEmpty) {
  ThreadsafeOdometryBuffer buffer(-1);
  gtsam::NavState navstate_result;
  QueryResult result = buffer.getNearest(0, &navstate_result);
  EXPECT_EQ(QueryResult::DataNotYetAvailable, result);
}

TEST(ThreadsafeOdometryBuffer, getNearestCorrect) {
  ThreadsafeOdometryBuffer buffer(-1);
  buffer.add(5,
             gtsam::NavState(gtsam::Rot3(),
                             (Eigen::Vector3d() << 0, 0, 5).finished(),
                             Eigen::Vector3d::Zero()));
  buffer.add(8,
             gtsam::NavState(gtsam::Rot3(),
                             (Eigen::Vector3d() << 0, 0, 8).finished(),
                             Eigen::Vector3d::Zero()));

  for (Timestamp i = 0; i < 10; ++i) {
    gtsam::NavState navstate_result;
    QueryResult result = buffer.getNearest(i, &navstate_result);
    if (i < 5) {
      // less than [5, 8]
      EXPECT_EQ(QueryResult::DataNeverAvailable, result)
          << "t=" << i << " failed";
    } else if (i <= 8) {
      // in [5, 8]
      EXPECT_EQ(QueryResult::DataAvailable, result) << "t=" << i << " failed";
      EXPECT_EQ((i < 7 ? 5 : 8), navstate_result.t()(2, 0));
    } else {
      // greater than [5, 8]
      EXPECT_EQ(QueryResult::DataNotYetAvailable, result)
          << "t=" << i << " failed";
    }
  }

  buffer.add(11,
             gtsam::NavState(gtsam::Rot3(),
                             (Eigen::Vector3d() << 0, 0, 11).finished(),
                             Eigen::Vector3d::Zero()));

  for (Timestamp i = 9; i < 10; ++i) {
    gtsam::NavState navstate_result;
    QueryResult result = buffer.getNearest(i, &navstate_result);
    // these should be valid now that our measurements are in the
    // range [5, 11]
    EXPECT_EQ(QueryResult::DataAvailable, result) << "t=" << i << "failed";
    EXPECT_EQ((i < 10 ? 8 : 11), navstate_result.t()(2, 0));
  }
}

}  // namespace VIO
