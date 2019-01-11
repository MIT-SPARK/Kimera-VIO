/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontEnd.cpp
 * @brief  Class managing sequences of IMU measurements
 * @author Luca Carlone
 */
#include "ImuFrontEnd.h"

#include <glog/logging.h>

using namespace VIO;

/* --------------------------------------------------------------------------------------- */
std::tuple<int64_t, int64_t, bool> ImuFrontEnd::getOldestAndNewestStamp() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(buffer_.empty())
  {
    return std::make_tuple(-1, -1, false);
  }
  return std::make_tuple(buffer_.begin()->first, buffer_.rbegin()->first, true);
}
/* --------------------------------------------------------------------------------------- */
std::pair<ImuStamps, ImuAccGyr>
ImuFrontEnd::getBetweenValuesInterpolated(
        const int64_t& stamp_from,
        const int64_t& stamp_to,
        bool doInterpolate) {
  ImuStamps imu_stamps;
  ImuAccGyr imu_accgyr;

  if (!(stamp_from >= 0 and stamp_from < stamp_to)) {
    VLOG(10) << "WARNING: Timestamps out of order.";
    // Return empty means unsuccessful.
    return std::make_pair(imu_stamps, imu_accgyr);
  }
  VLOG_IF(10, stamp_from == 0) << "WARNING: required 0 stamp_from in imuBuffer.";

  std::lock_guard<std::mutex> lock(mutex_);
  if(buffer_.size() < 2) {
    VLOG(10) << "Buffer has less than 2 entries.";
    return std::make_pair(imu_stamps, imu_accgyr); // return empty means unsuccessful.
  }

  const int64_t oldest_stamp = buffer_.begin()->first;
  const int64_t newest_stamp = buffer_.rbegin()->first;
  if (stamp_from < oldest_stamp) {
    VLOG(10) << "Requests older timestamp than in buffer.";
    return std::make_pair(imu_stamps, imu_accgyr); // return empty means unsuccessful.
  }
  if(stamp_to > newest_stamp) {
    VLOG(10) << "Requests newer timestamp than in buffer.";
    return std::make_pair(imu_stamps, imu_accgyr); // return empty means unsuccessful.
  }

  auto it_from_before = iterator_equal_or_before(stamp_from);
  auto it_to_after = iterator_equal_or_after(stamp_to);
  if (it_from_before == buffer_.end()) {
    VLOG(10) << "it_from_before == buffer.end. (imu buffer warning).";
    std::make_pair(imu_stamps, imu_accgyr); // return empty means unsuccessful. // Why there is no return??
  }
  if (it_to_after == buffer_.end()) {
    VLOG(10) << "it_to_after == buffer.end. (imu buffer warning).";
    std::make_pair(imu_stamps, imu_accgyr); // return empty means unsuccessful. // Why there is no return??
  }
  // the following is a mystery
  auto it_from_after = it_from_before;
  ++it_from_after;
  auto it_to_before = it_to_after;
  --it_to_before;
  if (it_from_after == it_to_before) {
    VLOG(10) << "Not enough data for interpolation.";
    return std::make_pair(imu_stamps, imu_accgyr); // return empty means unsuccessful.
  }

  // Count number of measurements.
  size_t n = 0;
  auto it = it_from_after;
  while(it != it_to_after)
  {
    ++n;
    ++it;
  }
  n += 2;

  // Interpolate values at start and end and copy in output vector.
  imu_stamps.resize(n);
  imu_accgyr.resize(6, n);
  for(size_t i = 0; i < n; ++i)
  {
    if(i == 0) // first value
    {
      imu_stamps(i) = stamp_from;
      double w =
          static_cast<double>(stamp_from - it_from_before->first) /
          static_cast<double>(it_from_after->first - it_from_before->first);
      if(!doInterpolate)
        w = 0.0; // pick first value
      imu_accgyr.col(i) = (1.0 - w) * it_from_before->second + w * it_from_after->second;
    }
    else if(i == n-1) // last value
    {
      imu_stamps(i) = stamp_to;
      double w =
          static_cast<double>(stamp_to - it_to_before->first) /
          static_cast<double>(it_to_after->first - it_to_before->first);
      if(!doInterpolate)
        w = 0.0; // pick first value
      imu_accgyr.col(i) = (1.0 - w) * it_to_before->second + w * it_to_after->second;
    }
    else
    {
      imu_stamps(i) = it_from_after->first;
      imu_accgyr.col(i) = it_from_after->second;
      ++it_from_after;
    }
  }
  return std::make_pair(imu_stamps, imu_accgyr);
}
/* --------------------------------------------------------------------------------------- */
typename ImuFrontEnd::ImuData::iterator ImuFrontEnd::iterator_equal_or_before(int64_t stamp) {
  // if (!mutex_.try_lock()) { printf("Call lock() before accessing data.\n"); }
  auto it = buffer_.lower_bound(stamp);
  if(it->first == stamp)
  {
    return it; // Return iterator to key if exact key exists.
  }
  if(stamp > buffer_.rbegin()->first)
  {
    return (--buffer_.end()); // Pointer to last value.
  }
  if(it == buffer_.begin())
  {
    return buffer_.end(); // Invalid if data before first value.
  }
  --it;
  return it;
}
/* --------------------------------------------------------------------------------------- */
typename ImuFrontEnd::ImuData::iterator ImuFrontEnd::iterator_equal_or_after(int64_t stamp) {
  // if (!mutex_.try_lock()) { printf("Call lock() before accessing data.\n"); }
  return buffer_.lower_bound(stamp);
}
