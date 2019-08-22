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

/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Accumulator.h
 * @brief  For accumulating statistics.
 * @author Antoni Rosinol
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <glog/logging.h>

namespace VIO {

namespace utils {

static constexpr int kInfiniteWindowSize = std::numeric_limits<int>::max();

// If the window size is set to -1, the vector will grow infinitely, otherwise,
// the vector has a fixed size.
template <typename SampleType, typename SumType, int WindowSize>
class Accumulator {
 public:
  Accumulator()
      : sample_index_(0),
        total_samples_(0),
        sum_(0),
        window_sum_(0),
        min_(std::numeric_limits<SampleType>::max()),
        max_(std::numeric_limits<SampleType>::lowest()),
        most_recent_(0) {
    CHECK_GT(WindowSize, 0);
    if (WindowSize < kInfiniteWindowSize) {
      samples_.reserve(WindowSize);
    }
  }

  /* ------------------------------------------------------------------------ */
  void Add(SampleType sample) {
    most_recent_ = sample;
    if (sample_index_ < WindowSize) {
      samples_.push_back(sample);
      window_sum_ += sample;
      ++sample_index_;
    } else {
      SampleType& oldest = samples_.at(sample_index_++ % WindowSize);
      window_sum_ += sample - oldest;
      oldest = sample;
    }
    sum_ += sample;
    ++total_samples_;
    if (sample > max_) {
      max_ = sample;
    }
    if (sample < min_) {
      min_ = sample;
    }
  }

  /* ------------------------------------------------------------------------ */
  int total_samples() const { return total_samples_; }

  /* ------------------------------------------------------------------------ */
  SumType sum() const { return sum_; }

  /* ------------------------------------------------------------------------ */
  SumType Mean() const {
    return (total_samples_ < 1) ? 0.0 : sum_ / total_samples_;
  }

  /* ------------------------------------------------------------------------ */
  // Rolling mean is only used for fixed sized data for now. We don't need this
  // function for our infinite accumulator at this point.
  SumType RollingMean() const {
    if (WindowSize < kInfiniteWindowSize) {
      return window_sum_ / std::min(sample_index_, WindowSize);
    } else {
      return Mean();
    }
  }

  /* ------------------------------------------------------------------------ */
  SampleType GetMostRecent() const { return most_recent_; }

  /* ------------------------------------------------------------------------ */
  SumType max() const { return max_; }

  /* ------------------------------------------------------------------------ */
  SumType min() const { return min_; }

  /* ------------------------------------------------------------------------ */
  inline SumType median() const {
    CHECK_GT(samples_.size(), 0);
    return samples_.at(std::ceil(samples_.size() / 2) - 1);
  }

  /* ------------------------------------------------------------------------ */
  // First quartile.
  inline SumType q1() const {
    CHECK_GT(samples_.size(), 0);
    return samples_.at(std::ceil(samples_.size() / 4) - 1);
  }

  /* ------------------------------------------------------------------------ */
  // Third quartile.
  inline SumType q3() const {
    CHECK_GT(samples_.size(), 0);
    return samples_.at(std::ceil(samples_.size() * 3 / 4) - 1);
  }

  /* ------------------------------------------------------------------------ */
  SumType LazyVariance() const {
    if (samples_.size() < 2) {
      return 0.0;
    }

    SumType var = static_cast<SumType>(0.0);
    SumType mean = RollingMean();

    for (unsigned int i = 0; i < samples_.size(); ++i) {
      var += (samples_[i] - mean) * (samples_[i] - mean);
    }

    var /= samples_.size() - 1;
    return var;
  }

  /* ------------------------------------------------------------------------ */
  SumType StandardDeviation() const { return std::sqrt(LazyVariance()); }

  /* ------------------------------------------------------------------------ */
  const std::vector<SampleType> &GetAllSamples() const { return samples_; }

private:
  std::vector<SampleType> samples_;
  int sample_index_;
  int total_samples_;
  SumType sum_;
  SumType window_sum_;
  SampleType min_;
  SampleType max_;
  SampleType most_recent_;
};

typedef Accumulator<double, double, kInfiniteWindowSize> Accumulatord;

}  // namespace utils

}  // namespace VIO
