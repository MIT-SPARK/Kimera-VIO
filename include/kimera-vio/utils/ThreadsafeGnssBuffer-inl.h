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

#pragma once

#include <glog/logging.h>

#include "kimera-vio/common/vio_types.h"

namespace VIO {

namespace utils {

inline void ThreadsafeGnssBuffer::addMeasurement(
    const Timestamp& timestamp_nanoseconds,
    const GnssPoint& gnss_point) {
  // Enforce strict time-wise ordering.
  GnssMeasurement last_value;
  if (buffer_.getNewestValue(&last_value)) {
    if (timestamp_nanoseconds <= last_value.timestamp_) {
      LOG_FIRST_N(WARNING, 1) << "Gnss timestamps not strictly increasing";
      VLOG(10) << "Timestamps not strictly increasing";
      return;
    }
  }
  buffer_.addValue(
      timestamp_nanoseconds,
      GnssMeasurement(timestamp_nanoseconds, gnss_point));

  // Notify possibly waiting consumers.
  cv_new_measurement_.notify_all();
}

inline void ThreadsafeGnssBuffer::clear() {
  buffer_.clear();
}

inline size_t ThreadsafeGnssBuffer::size() const {
  return buffer_.size();
}

inline void ThreadsafeGnssBuffer::shutdown() {
  shutdown_ = true;
  cv_new_measurement_.notify_all();
}

}  // namespace utils

}  // namespace VIO
