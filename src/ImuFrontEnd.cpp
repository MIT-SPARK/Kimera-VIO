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
#ifdef IMU_BUFFER_DEBUG_COUT
    printf("WARNING: Timestamps out of order/n");
#endif
    // Return empty means unsuccessful.
    return std::make_pair(imu_stamps, imu_accgyr);
  }
#ifdef IMU_BUFFER_DEBUG_COUT
  if (stamp_from == 0)
    printf("WARNING: required 0 stamp_from in imuBuffer /n");
#endif

  std::lock_guard<std::mutex> lock(mutex_);
  if(buffer_.size() < 2) {
#ifdef IMU_BUFFER_DEBUG_COUT
    printf("Buffer has less than 2 entries.\n");
#endif
    return std::make_pair(imu_stamps, imu_accgyr); // return empty means unsuccessful.
  }

  const int64_t oldest_stamp = buffer_.begin()->first;
  const int64_t newest_stamp = buffer_.rbegin()->first;
  if (stamp_from < oldest_stamp) {
#ifdef IMU_BUFFER_DEBUG_COUT
    printf("Requests older timestamp than in buffer.\n");
#endif
    return std::make_pair(imu_stamps, imu_accgyr); // return empty means unsuccessful.
  }
  if(stamp_to > newest_stamp) {
#ifdef IMU_BUFFER_DEBUG_COUT
    printf("Requests newer timestamp than in buffer.\n");
#endif
    return std::make_pair(imu_stamps, imu_accgyr); // return empty means unsuccessful.
  }

  auto it_from_before = iterator_equal_or_before(stamp_from);
  auto it_to_after = iterator_equal_or_after(stamp_to);
  if (it_from_before == buffer_.end()) {
#ifdef IMU_BUFFER_DEBUG_COUT
    printf("it_from_before == buffer.end. (imu buffer warning) \n");
#endif
    std::make_pair(imu_stamps, imu_accgyr); // return empty means unsuccessful. // Why there is no return??
  }
  if (it_to_after == buffer_.end()) {
#ifdef IMU_BUFFER_DEBUG_COUT
    printf("it_to_after == buffer.end. (imu buffer warning) \n");
#endif
    std::make_pair(imu_stamps, imu_accgyr); // return empty means unsuccessful. // Why there is no return??
  }
  // the following is a mystery
  auto it_from_after = it_from_before;
  ++it_from_after;
  auto it_to_before = it_to_after;
  --it_to_before;
  if(it_from_after == it_to_before)
  {
#ifdef IMU_BUFFER_DEBUG_COUT
    printf("Not enough data for interpolation\n");
#endif
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
