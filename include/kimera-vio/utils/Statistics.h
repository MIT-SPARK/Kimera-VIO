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
 * @file   Statistics.h
 * @brief  For logging statistics in a thread-safe manner.
 * @author Antoni Rosinol
 */
#pragma once

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "kimera-vio/utils/Accumulator.h"

///
// Example usage:
//
// #define ENABLE_STATISTICS 1 // Turn on/off the statistics calculation
// #include <utils/Statistics.h>
//
// double my_distance = measureDistance();
// utils::DebugStatsCollector distance_stat("Distance measurement");
// distance_stat.AddSample(my_distance);
//
// std::cout << utils::Statistics::Print();

namespace VIO {

namespace utils {

const double kNumSecondsPerNanosecond = 1.e-9;

struct StatisticsMapValue {
  static const int kWindowSize = 100;

  inline StatisticsMapValue() {
    time_last_called_ = std::chrono::system_clock::now();
  }

  inline void AddValue(double sample) {
    std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
    double dt = static_cast<double>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        now - time_last_called_)
                        .count()) *
                kNumSecondsPerNanosecond;
    time_last_called_ = now;

    values_.Add(sample);
    time_deltas_.Add(dt);
  }
  inline double GetLastDeltaTime() const {
    if (time_deltas_.total_samples()) {
      return time_deltas_.GetMostRecent();
    } else {
      return 0;
    }
  }
  inline double GetLastValue() const {
    if (values_.total_samples()) {
      return values_.GetMostRecent();
    } else {
      return 0;
    }
  }
  inline double Sum() const { return values_.sum(); }
  int TotalSamples() const { return values_.total_samples(); }
  double Mean() const { return values_.Mean(); }
  double RollingMean() const { return values_.RollingMean(); }
  double Max() const { return values_.max(); }
  double Min() const { return values_.min(); }
  double Median() const { return values_.median(); }
  double Q1() const { return values_.q1(); }
  double Q3() const { return values_.q3(); }
  double LazyVariance() const { return values_.LazyVariance(); }
  double MeanCallsPerSec() const {
    double mean_dt = time_deltas_.Mean();
    if (mean_dt != 0) {
      return 1.0 / mean_dt;
    } else {
      return -1.0;
    }
  }

  double MeanDeltaTime() const { return time_deltas_.Mean(); }
  double RollingMeanDeltaTime() const { return time_deltas_.RollingMean(); }
  double MaxDeltaTime() const { return time_deltas_.max(); }
  double MinDeltaTime() const { return time_deltas_.min(); }
  double LazyVarianceDeltaTime() const { return time_deltas_.LazyVariance(); }
  std::vector<double> GetAllValues() const { return values_.GetAllSamples(); }

private:
  // Create an accumulator with specified window size.
  Accumulator<double, double, kWindowSize> values_;
  Accumulator<double, double, kWindowSize> time_deltas_;
  std::chrono::time_point<std::chrono::system_clock> time_last_called_;
};

// A class that has the statistics interface but does nothing. Swapping this in
// in place of the Statistics class (say with a typedef) eliminates the function
// calls.
class DummyStatsCollector {
 public:
  explicit DummyStatsCollector(size_t /*handle*/) {}
  explicit DummyStatsCollector(std::string const& /*tag*/) {}
  void AddSample(double /*sample*/) const {}
  void IncrementOne() const {}
  size_t GetHandle() const { return 0u; }
};

class StatsCollectorImpl {
 public:
  explicit StatsCollectorImpl(size_t handle);
  explicit StatsCollectorImpl(std::string const& tag);
  ~StatsCollectorImpl() = default;

  void AddSample(double sample) const;
  void IncrementOne() const;
  size_t GetHandle() const;

 private:
  size_t handle_;
};

class Statistics {
 public:
  typedef std::map<std::string, size_t> map_t;
  friend class StatsCollectorImpl;
  // Definition of static functions to query the stats.
  static size_t GetHandle(std::string const& tag);
  static bool HasHandle(std::string const& tag);
  static std::string GetTag(size_t handle);
  static double GetLastValue(size_t handle);
  static double GetLastValue(std::string const& tag);
  static double GetTotal(size_t handle);
  static double GetTotal(std::string const& tag);
  static double GetMean(size_t handle);
  static double GetMean(std::string const& tag);
  static std::vector<double> GetAllSamples(size_t handle);
  static std::vector<double> GetAllSamples(std::string const &tag);
  static size_t GetNumSamples(size_t handle);
  static size_t GetNumSamples(std::string const& tag);
  static double GetVariance(size_t handle);
  static double GetVariance(std::string const& tag);
  static double GetMin(size_t handle);
  static double GetMin(std::string const& tag);
  static double GetMax(size_t handle);
  static double GetMax(std::string const& tag);
  static double GetMedian(size_t handle);
  static double GetMedian(std::string const &tag);
  static double GetQ1(size_t handle);
  static double GetQ1(std::string const &tag);
  static double GetQ3(size_t handle);
  static double GetQ3(std::string const &tag);
  static double GetHz(size_t handle);
  static double GetHz(std::string const& tag);

  static double GetMeanDeltaTime(std::string const& tag);
  static double GetMeanDeltaTime(size_t handle);
  static double GetMaxDeltaTime(std::string const& tag);
  static double GetMaxDeltaTime(size_t handle);
  static double GetMinDeltaTime(std::string const& tag);
  static double GetMinDeltaTime(size_t handle);
  static double GetLastDeltaTime(std::string const& tag);
  static double GetLastDeltaTime(size_t handle);
  static double GetVarianceDeltaTime(std::string const& tag);
  static double GetVarianceDeltaTime(size_t handle);

  // Writes a csv file, but transposed, each row first element represents the
  // columns headers, and the subsequent values are the data.
  static void WriteAllSamplesToCsvFile(const std::string &path);
  static void WriteToYamlFile(const std::string& path);
  static void Print(std::ostream& out);  // NOLINT
  static std::string Print();
  static std::string SecondsToTimeString(double seconds);
  static void Reset();
  static const map_t& GetStatsCollectors() { return Instance().tag_map_; }

 private:
  void AddSample(size_t handle, double sample);

  static Statistics& Instance();

  Statistics();
  ~Statistics();

  typedef std::vector<utils::StatisticsMapValue> list_t;

  list_t stats_collectors_;
  map_t tag_map_;
  size_t max_tag_length_;
  std::mutex mutex_;
};

// TODO make this a gflag?
#define ENABLE_STATISTICS 1
#if ENABLE_STATISTICS
typedef StatsCollectorImpl StatsCollector;
#else
typedef DummyStatsCollector StatsCollector;
#endif

}  // namespace utils

}  // namespace VIO
